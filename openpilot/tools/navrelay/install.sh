#!/usr/bin/env bash
# Install or update navrelay on the server, behind a reverse proxy that does https (1Panel's openresty).
# Run as root (sudo bash install.sh [port]) with navrelay.py, webpush.py, destination_page.html, destinationd.py,
# baidu_route.py and baidu.py (road_events) next to it. Baidu's server AK/SK for search and previews: NAVRELAY_BAIDU_AK / NAVRELAY_BAIDU_SK in
# the env file, kept across runs like the key. Listens on 127.0.0.1 only, plain http.
# Keeps an existing key, which it never prints: read it in /etc/navrelay/navrelay.env.
set -euo pipefail

PORT="${1:-8089}"
CAR="${NAVRELAY_CAR:-http://100.69.175.43:8082}"
HERE="$(cd "$(dirname "$0")" && pwd)"
ETC=/etc/navrelay

python3 -c 'import sys; assert sys.version_info >= (3, 8), sys.version'
python3 -s -c 'import cryptography' 2>/dev/null || apt-get install -y python3-cryptography   # push; navrelay runs without
id navrelay >/dev/null 2>&1 || useradd --system --no-create-home --shell /usr/sbin/nologin navrelay
install -d -m 755 /opt/navrelay
install -m 644 "$HERE/navrelay.py" /opt/navrelay/navrelay.py
install -m 644 "$HERE/destination_page.html" /opt/navrelay/destination_page.html   # selfdrive/navigation/destination_page.html
# destinationd's own code (pure python), from the same commit: selfdrive/navigation/{destinationd,baidu_route}.py, selfdrive/road_events/baidu.py
LIB=/opt/navrelay/lib/openpilot
install -d -m 755 "$LIB/selfdrive/navigation" "$LIB/selfdrive/road_events" "$LIB/tools/navrelay"
for d in "$LIB" "$LIB/selfdrive" "$LIB/selfdrive/navigation" "$LIB/selfdrive/road_events" "$LIB/tools" "$LIB/tools/navrelay"; do
  : > "$d/__init__.py"
done
install -m 644 "$HERE/webpush.py" "$LIB/tools/navrelay/"
install -m 644 "$HERE/destinationd.py" "$HERE/baidu_route.py" "$LIB/selfdrive/navigation/"
install -m 644 "$HERE/baidu.py" "$LIB/selfdrive/road_events/"
install -m 644 "$HERE/destination_page.html" "$LIB/selfdrive/navigation/"   # destinationd reads it at import
install -d -m 750 -o root -g navrelay "$ETC"
install -d -m 700 -o navrelay -g navrelay /var/lib/navrelay

if [ ! -f "$ETC/navrelay.env" ]; then
  echo "NAVRELAY_KEY=$(python3 -c 'import secrets; print(secrets.token_urlsafe(24))')" > "$ETC/navrelay.env"
fi
# everything but the keys is rewritten, so an older https setup is dropped
KEY_LINE="$(grep '^NAVRELAY_KEY=' "$ETC/navrelay.env")"
JS_LINE="$(grep '^NAVRELAY_JS_AK=' "$ETC/navrelay.env" || echo "NAVRELAY_JS_AK=${NAVRELAY_JS_AK:-}")"
BAIDU_LINES="$(grep '^NAVRELAY_BAIDU_[AS]K=' "$ETC/navrelay.env" || true)"
cat > "$ETC/navrelay.env" <<EOF
$KEY_LINE
$JS_LINE
$BAIDU_LINES
NAVRELAY_CAR=$CAR
NAVRELAY_BIND=127.0.0.1
NAVRELAY_PORT=$PORT
NAVRELAY_STATE=/var/lib/navrelay
NAVRELAY_PAGE=/opt/navrelay/destination_page.html
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
Environment=PYTHONPATH=/opt/navrelay/lib
ExecStart=/usr/bin/python3 -s /opt/navrelay/navrelay.py
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
