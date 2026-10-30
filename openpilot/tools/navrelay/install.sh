#!/usr/bin/env bash
# Install or update navrelay on the server, behind a reverse proxy that does https (1Panel's openresty).
# Run as root (sudo bash install.sh [port]) with navrelay.py next to it. Listens on 127.0.0.1 only, plain http.
# Keeps an existing key; prints it at the end.
set -euo pipefail

PORT="${1:-8089}"
CAR="${NAVRELAY_CAR:-http://100.69.175.43:8082}"
HERE="$(cd "$(dirname "$0")" && pwd)"
ETC=/etc/navrelay

python3 -c 'import sys; assert sys.version_info >= (3, 8), sys.version'
id navrelay >/dev/null 2>&1 || useradd --system --no-create-home --shell /usr/sbin/nologin navrelay
install -d -m 755 /opt/navrelay
install -m 644 "$HERE/navrelay.py" /opt/navrelay/navrelay.py
install -d -m 750 -o root -g navrelay "$ETC"
install -d -m 700 -o navrelay -g navrelay /var/lib/navrelay

if [ ! -f "$ETC/navrelay.env" ]; then
  echo "NAVRELAY_KEY=$(python3 -c 'import secrets; print(secrets.token_urlsafe(24))')" > "$ETC/navrelay.env"
fi
# everything but the key is rewritten, so an older https setup is dropped
KEY_LINE="$(grep '^NAVRELAY_KEY=' "$ETC/navrelay.env")"
cat > "$ETC/navrelay.env" <<EOF
$KEY_LINE
NAVRELAY_CAR=$CAR
NAVRELAY_BIND=127.0.0.1
NAVRELAY_PORT=$PORT
NAVRELAY_STATE=/var/lib/navrelay
EOF
chown root:navrelay "$ETC/navrelay.env"; chmod 640 "$ETC/navrelay.env"
rm -f "$ETC"/server.key "$ETC"/server.crt "$ETC"/ca.crt "$ETC"/ca.srl

cat > /etc/systemd/system/navrelay.service <<EOF
[Unit]
Description=navrelay: navigation destinations for the car
After=network-online.target
Wants=network-online.target

[Service]
User=navrelay
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

sleep 2
systemctl is-active navrelay
. "$ETC/navrelay.env"
echo "car reachable from here: $(curl -s -o /dev/null -w '%{http_code}' --max-time 5 "$NAVRELAY_CAR/api/state" || echo no)"
echo "relay: $(curl -s -o /dev/null -w '%{http_code}' --max-time 10 -H "X-Key: $NAVRELAY_KEY" "http://127.0.0.1:$PORT/api/state" || echo no)"
echo "key: $NAVRELAY_KEY"
