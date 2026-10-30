"""Web Push (RFC 8030/8291/8292) to the page added to the phone's home screen (iOS 16.4+): a tap opens the page, not
Shortcuts. VAPID key and subscriptions are kept in the state directory. Needs the `cryptography` package
(python3-cryptography); without it navrelay runs without push.
"""
from __future__ import annotations

import base64
import json
import os
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives.asymmetric.utils import decode_dss_signature
from cryptography.hazmat.primitives.ciphers.aead import AESGCM
from cryptography.hazmat.primitives.kdf.hkdf import HKDF

PUSH_HOSTS = ("push.apple.com", "fcm.googleapis.com", "push.services.mozilla.com", "notify.windows.com")   # where
# subscriptions may point: the server posts to them
MAX_SUBSCRIPTIONS = 8
TTL = 3600            # s the push service keeps a message for a phone that is off
TIMEOUT = 10.
RECORD_SIZE = 4096


def b64(data: bytes) -> str:
  return base64.urlsafe_b64encode(data).rstrip(b"=").decode()


def unb64(text: str) -> bytes:
  return base64.urlsafe_b64decode(text + "=" * (-len(text) % 4))


def point(key: ec.EllipticCurvePublicKey) -> bytes:
  return key.public_bytes(serialization.Encoding.X962, serialization.PublicFormat.UncompressedPoint)


def _hkdf(salt: bytes, info: bytes, length: int, ikm: bytes) -> bytes:
  return HKDF(algorithm=hashes.SHA256(), length=length, salt=salt, info=info).derive(ikm)


def encrypt(payload: bytes, p256dh: str, auth: str, salt: bytes | None = None, sender: ec.EllipticCurvePrivateKey | None = None) -> bytes:
  """RFC 8291 aes128gcm: one record, the body of the push request."""
  ua_public = unb64(p256dh)
  sender = sender or ec.generate_private_key(ec.SECP256R1())
  as_public = point(sender.public_key())
  secret = sender.exchange(ec.ECDH(), ec.EllipticCurvePublicKey.from_encoded_point(ec.SECP256R1(), ua_public))
  ikm = _hkdf(unb64(auth), b"WebPush: info\x00" + ua_public + as_public, 32, secret)
  salt = salt or os.urandom(16)
  cek = _hkdf(salt, b"Content-Encoding: aes128gcm\x00", 16, ikm)
  nonce = _hkdf(salt, b"Content-Encoding: nonce\x00", 12, ikm)
  header = salt + RECORD_SIZE.to_bytes(4, "big") + bytes([len(as_public)]) + as_public
  return header + AESGCM(cek).encrypt(nonce, payload + b"\x02", None)


def vapid_jwt(key: ec.EllipticCurvePrivateKey, endpoint: str, subject: str, now: float) -> str:
  """RFC 8292: the push service's origin, signed ES256."""
  parts = urllib.parse.urlsplit(endpoint)
  claims = {"aud": f"{parts.scheme}://{parts.netloc}", "exp": int(now) + 12 * 3600, "sub": subject}
  signing = b64(json.dumps({"typ": "JWT", "alg": "ES256"}).encode()) + "." + b64(json.dumps(claims).encode())
  r, s = decode_dss_signature(key.sign(signing.encode(), ec.ECDSA(hashes.SHA256())))
  return signing + "." + b64(r.to_bytes(32, "big") + s.to_bytes(32, "big"))


def allowed_endpoint(endpoint: str) -> bool:
  parts = urllib.parse.urlsplit(endpoint)
  host = parts.hostname or ""
  return parts.scheme == "https" and any(host == h or host.endswith("." + h) for h in PUSH_HOSTS)


def _post(url: str, body: bytes, headers: dict) -> int:
  req = urllib.request.Request(url, body, headers, method="POST")
  try:
    with urllib.request.urlopen(req, timeout=TIMEOUT) as r:
      return r.status
  except urllib.error.HTTPError as e:
    return e.code


class Push:
  def __init__(self, state_dir: str | None, subject: str, post=_post, clock=time.time):  # noqa: TID251  # JWT expiry is wall time
    self.dir, self.subject, self.post, self.clock = state_dir, subject, post, clock
    self.lock = threading.Lock()
    self.key = self._load_key()
    self.subscriptions: list[dict] = self._load_subscriptions()

  def _path(self, name: str) -> str | None:
    return os.path.join(self.dir, name) if self.dir else None

  def _load_key(self) -> ec.EllipticCurvePrivateKey:
    path = self._path("vapid.pem")
    if path and os.path.exists(path):
      with open(path, "rb") as f:
        return serialization.load_pem_private_key(f.read(), None)
    key = ec.generate_private_key(ec.SECP256R1())
    if path:
      pem = key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8, serialization.NoEncryption())
      fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
      with os.fdopen(fd, "wb") as f:
        f.write(pem)
    return key

  def _load_subscriptions(self) -> list[dict]:
    path = self._path("push.json")
    try:
      with open(path or "", encoding="utf-8") as f:
        subs = json.load(f)
      return [s for s in subs if isinstance(s, dict)] if isinstance(subs, list) else []
    except (OSError, ValueError):
      return []

  def _save(self):
    path = self._path("push.json")
    if path:
      with open(path + ".tmp", "w", encoding="utf-8") as f:
        json.dump(self.subscriptions, f)
      os.replace(path + ".tmp", path)

  def public_key(self) -> str:
    return b64(point(self.key.public_key()))

  def subscribe(self, sub: dict) -> dict:
    endpoint = sub.get("endpoint")
    keys = sub.get("keys") if isinstance(sub.get("keys"), dict) else {}
    if not isinstance(endpoint, str) or not allowed_endpoint(endpoint):
      return {"error": "不认识的推送服务"}
    try:
      if len(unb64(str(keys.get("p256dh")))) != 65 or len(unb64(str(keys.get("auth")))) != 16:
        raise ValueError
    except ValueError:
      return {"error": "订阅不完整"}
    with self.lock:
      self.subscriptions = ([{"endpoint": endpoint, "keys": {"p256dh": keys["p256dh"], "auth": keys["auth"]}}]
                            + [s for s in self.subscriptions if s.get("endpoint") != endpoint])[:MAX_SUBSCRIPTIONS]
      self._save()
    return {"subscribed": True}

  def send(self, title: str, body: str, url: str = "/") -> int:
    """To every subscription, now; gone ones (404, 410) are dropped. The count delivered."""
    payload = json.dumps({"title": title, "body": body, "url": url}, ensure_ascii=False).encode()
    with self.lock:
      subs = list(self.subscriptions)
    sent, gone = 0, []
    for s in subs:
      try:
        body_bytes = encrypt(payload, s["keys"]["p256dh"], s["keys"]["auth"])
        token = vapid_jwt(self.key, s["endpoint"], self.subject, self.clock())
        status = self.post(s["endpoint"], body_bytes, {
          "TTL": str(TTL), "Urgency": "high", "Content-Encoding": "aes128gcm", "Content-Type": "application/octet-stream",
          "Authorization": f"vapid t={token}, k={self.public_key()}"})
      except (OSError, ValueError, KeyError):
        continue
      if status in (404, 410):
        gone.append(s["endpoint"])
      elif 200 <= status < 300:
        sent += 1
    if gone:
      with self.lock:
        self.subscriptions = [s for s in self.subscriptions if s.get("endpoint") not in gone]
        self._save()
    return sent

  def notify(self, title: str, body: str, url: str = "/"):
    """send() without holding up the caller."""
    threading.Thread(target=self.send, args=(title, body, url), daemon=True).start()
