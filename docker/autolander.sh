#!/usr/bin/env bash
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd /workspace/avion-cargo
exec poetry run precision_landing simulation/config/autolanding_config.json --gz-simulation
