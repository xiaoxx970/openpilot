#!/usr/bin/env bash
# Install or update navrelay on the server. Run as root (sudo bash install.sh <public ip>) with navrelay.py next to it.
# Keeps an existing key and certificates; prints the key and the phone's URLs at the end.
set -euo pipefail

IP="${1:?usage: install.sh <public ip> [port]}"
PORT="${2:-443}"
CAR="${NAVRELAY_CAR:-http://100.69.175.43:8082}"
HERE="$(cd "$(dirname "$0")" && pwd)"
ETC=/etc/navrelay

python3 -c 'import sys; assert sys.version_info >= (3, 8), sys.version'
id navrelay >/dev/null 2>&1 || useradd --system --no-create-home --shell /usr/sbin/nologin navrelay
install -d -m 755 /opt/navrelay
install -m 644 "$HERE/navrelay.py" /opt/navrelay/navrelay.py
install -d -m 750 -o root -g navrelay "$ETC"
install -d -m 700 -o navrelay -g navrelay /var/lib/navrelay

# a private CA the phone trusts once, and a server certificate for the IP under it (iOS: at most 825 days). Its key is shredded
# once the server certificate is signed, so root on this box can't mint certificates for other sites the phone visits (the
# name constraint only limits IP names, not DNS ones). A new certificate means a new CA, installed on the phone again.
if [ ! -f "$ETC/server.crt" ] || [ ! -f "$ETC/ca.crt" ] || ! openssl x509 -in "$ETC/server.crt" -noout -text | grep -q "IP Address:$IP"; then
  openssl req -x509 -newkey rsa:2048 -nodes -keyout "$ETC/ca.key" -out "$ETC/ca.crt" -days 3650 -subj "/CN=navrelay CA $IP" \
    -addext "basicConstraints=critical,CA:TRUE,pathlen:0" -addext "keyUsage=critical,keyCertSign,cRLSign" \
    -addext "nameConstraints=critical,permitted;IP:$IP/255.255.255.255"
  openssl req -newkey rsa:2048 -nodes -keyout "$ETC/server.key" -out "$ETC/server.csr" -subj "/CN=$IP"
  printf 'subjectAltName=IP:%s\nextendedKeyUsage=serverAuth\nbasicConstraints=CA:FALSE\nkeyUsage=critical,digitalSignature,keyEncipherment\n' "$IP" > "$ETC/server.ext"
  openssl x509 -req -in "$ETC/server.csr" -CA "$ETC/ca.crt" -CAkey "$ETC/ca.key" -set_serial "0x$(openssl rand -hex 16)" \
    -out "$ETC/server.crt" -days 800 -sha256 -extfile "$ETC/server.ext"
  shred -u "$ETC/ca.key"
  rm -f "$ETC/server.csr" "$ETC/server.ext"
  echo "NEW CA: install https://$IP/ca.crt on the phone again"
fi
chown root:navrelay "$ETC"/server.key "$ETC"/server.crt "$ETC"/ca.crt
chmod 640 "$ETC/server.key"; chmod 644 "$ETC/server.crt" "$ETC/ca.crt"

if [ ! -f "$ETC/navrelay.env" ]; then
  KEY="$(python3 -c 'import secrets; print(secrets.token_urlsafe(24))')"
  cat > "$ETC/navrelay.env" <<EOF
NAVRELAY_KEY=$KEY
NAVRELAY_CAR=$CAR
NAVRELAY_PORT=$PORT
NAVRELAY_CERT=$ETC/server.crt
NAVRELAY_CERT_KEY=$ETC/server.key
NAVRELAY_CA=$ETC/ca.crt
NAVRELAY_STATE=/var/lib/navrelay
EOF
fi
chown root:navrelay "$ETC/navrelay.env"; chmod 640 "$ETC/navrelay.env"

cat > /etc/systemd/system/navrelay.service <<EOF
[Unit]
Description=navrelay: navigation destinations for the car
After=network-online.target tailscaled.service
Wants=network-online.target

[Service]
User=navrelay
AmbientCapabilities=CAP_NET_BIND_SERVICE
EnvironmentFile=$ETC/navrelay.env
ExecStart=/usr/bin/python3 -I /opt/navrelay/navrelay.py
Restart=always
RestartSec=5
NoNewPrivileges=true
ProtectSystem=strict
ProtectHome=true
PrivateTmp=true
ReadWritePaths=/var/lib/navrelay

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl enable navrelay >/dev/null 2>&1
systemctl restart navrelay

if command -v ufw >/dev/null && ufw status | grep -q "Status: active"; then
  ufw allow "$PORT/tcp" >/dev/null
fi

sleep 2
systemctl is-active navrelay
. "$ETC/navrelay.env"
echo "car reachable from here: $(curl -s -o /dev/null -w '%{http_code}' --max-time 5 "$NAVRELAY_CAR/api/state" || echo no)"
echo "local https: $(curl -s -o /dev/null -w '%{http_code}' --cacert "$ETC/ca.crt" --max-time 5 -H "X-Key: $NAVRELAY_KEY" "https://$IP:$NAVRELAY_PORT/api/state" --connect-to "$IP:$NAVRELAY_PORT:127.0.0.1:$NAVRELAY_PORT" || echo no)"
echo "certificate for the phone: https://$IP:$NAVRELAY_PORT/ca.crt"
echo "page: https://$IP:$NAVRELAY_PORT/?k=$NAVRELAY_KEY"
