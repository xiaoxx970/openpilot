import json
import os
import tempfile
import unittest

from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives.asymmetric.utils import encode_dss_signature
from cryptography.hazmat.primitives.ciphers.aead import AESGCM

from openpilot.tools.navrelay import webpush

ENDPOINT = "https://web.push.apple.com/QGuQyavXuto"


class Phone:
  """The browser's side of RFC 8291: its keys, and decrypting what the push service brings."""
  def __init__(self):
    self.key = ec.generate_private_key(ec.SECP256R1())
    self.auth = os.urandom(16)

  def subscription(self, endpoint=ENDPOINT):
    return {"endpoint": endpoint, "keys": {"p256dh": webpush.b64(webpush.point(self.key.public_key())), "auth": webpush.b64(self.auth)}}

  def decrypt(self, body: bytes) -> bytes:
    salt, rs, idlen = body[:16], int.from_bytes(body[16:20], "big"), body[20]
    as_public = body[21:21 + idlen]
    assert rs == webpush.RECORD_SIZE
    secret = self.key.exchange(ec.ECDH(), ec.EllipticCurvePublicKey.from_encoded_point(ec.SECP256R1(), as_public))
    ua_public = webpush.point(self.key.public_key())
    ikm = webpush._hkdf(self.auth, b"WebPush: info\x00" + ua_public + as_public, 32, secret)
    cek = webpush._hkdf(salt, b"Content-Encoding: aes128gcm\x00", 16, ikm)
    nonce = webpush._hkdf(salt, b"Content-Encoding: nonce\x00", 12, ikm)
    plain = AESGCM(cek).decrypt(nonce, body[21 + idlen:], None)
    assert plain.endswith(b"\x02")
    return plain[:-1]


class TestWebPush(unittest.TestCase):
  def setUp(self):
    self.dir = self.enterContext(tempfile.TemporaryDirectory())
    self.posts, self.status = [], 201

    def post(url, body, headers):
      self.posts.append((url, body, headers))
      return self.status
    self.push = webpush.Push(self.dir, "https://example.org", post=post, clock=lambda: 1000.)
    self.phone = Phone()

  def test_sent_encrypted_and_signed(self):
    self.assertEqual(self.push.subscribe(self.phone.subscription()), {"subscribed": True})
    self.assertEqual(self.push.send("已发到车上", "大行宫"), 1)
    url, body, headers = self.posts[0]
    self.assertEqual(url, ENDPOINT)
    self.assertEqual(json.loads(self.phone.decrypt(body)), {"title": "已发到车上", "body": "大行宫", "url": "/"})
    self.assertEqual(headers["Content-Encoding"], "aes128gcm")
    token, k = headers["Authorization"].removeprefix("vapid t=").split(", k=")
    self.assertEqual(k, self.push.public_key())
    head, claims, sig = token.split(".")
    self.assertEqual(json.loads(webpush.unb64(claims)), {"aud": "https://web.push.apple.com", "exp": 1000 + 12 * 3600, "sub": "https://example.org"})
    raw = webpush.unb64(sig)
    public = ec.EllipticCurvePublicKey.from_encoded_point(ec.SECP256R1(), webpush.unb64(k))
    public.verify(encode_dss_signature(int.from_bytes(raw[:32], "big"), int.from_bytes(raw[32:], "big")), f"{head}.{claims}".encode(),
                  ec.ECDSA(hashes.SHA256()))

  def test_key_and_subscriptions_kept(self):
    self.push.subscribe(self.phone.subscription())
    again = webpush.Push(self.dir, "https://example.org")
    self.assertEqual((again.public_key(), len(again.subscriptions)), (self.push.public_key(), 1))
    self.assertEqual(os.stat(os.path.join(self.dir, "vapid.pem")).st_mode & 0o777, 0o600)

  def test_only_push_services(self):
    self.assertTrue("error" in self.push.subscribe(self.phone.subscription("https://example.com/x")))
    self.assertTrue("error" in self.push.subscribe(self.phone.subscription("http://web.push.apple.com/x")))
    self.assertTrue("error" in self.push.subscribe({"endpoint": ENDPOINT, "keys": {"p256dh": "x", "auth": "y"}}))
    self.assertEqual(self.push.subscriptions, [])

  def test_gone_dropped(self):
    self.push.subscribe(self.phone.subscription())
    self.push.subscribe(self.phone.subscription())   # the same endpoint once
    self.assertEqual(len(self.push.subscriptions), 1)
    self.status = 410
    self.assertEqual(self.push.send("t", "b"), 0)
    self.assertEqual(self.push.subscriptions, [])


if __name__ == "__main__":
  unittest.main()
