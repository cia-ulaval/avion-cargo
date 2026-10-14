#!/usr/bin/env bash
set -Eeuo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u
/usr/local/bin/prepare-workspace.sh

/usr/local/bin/avion-web-proxy & WEB_PROXY_PID=$!

cleanup() {
    kill "$WEB_PROXY_PID" 2>/dev/null || true
    wait "$WEB_PROXY_PID" 2>/dev/null || true
}

trap cleanup EXIT TERM INT
echo "Development container ready."
echo "Autolander web proxy started (PID ${WEB_PROXY_PID}) on port 8086"
echo "Open another host terminal and run: docker compose exec dev bash"

exec sleep infinity
