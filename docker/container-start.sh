#!/usr/bin/env bash
set -Eeuo pipefail

# ROS setup scripts may reference unset variables; source with nounset disabled.
set +u
source /opt/ros/jazzy/setup.bash
set -u
/usr/local/bin/prepare-workspace.sh

# Keep one shared container running so all four simulation terminals share loopback/ROS DDS.
echo "Development container ready. Open another host terminal and run: docker compose exec dev bash"
exec sleep infinity
