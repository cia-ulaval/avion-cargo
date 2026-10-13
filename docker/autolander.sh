#!/usr/bin/env bash
set -Eeuo pipefail
# ROS setup scripts may reference unset variables; source with nounset disabled.
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd /workspace/avion-cargo
exec poetry run precision_landing simulation/config/autolanding_config.json --gz-simulation
