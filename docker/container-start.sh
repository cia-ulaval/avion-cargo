#!/usr/bin/env bash
set -Eeuo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u
/usr/local/bin/prepare-workspace.sh

echo "Development container ready. Open another host terminal and run: docker compose exec dev bash"
exec sleep infinity
