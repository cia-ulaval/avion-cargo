#!/usr/bin/env bash
set -Eeuo pipefail
ROOT=/workspace/avion-cargo
source /home/dev/venv-ardupilot/bin/activate
cd /home/dev/ardupilot
exec ./Tools/autotest/sim_vehicle.py -D -v ArduCopter -f JSON \
  --add-param-file="$ROOT/simulation/config/gazebo-iris-gimbal.parm"
