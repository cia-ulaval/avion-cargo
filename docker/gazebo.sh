#!/usr/bin/env bash
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u
ROOT=/workspace/avion-cargo
export GZ_SIM_SYSTEM_PLUGIN_PATH="/home/dev/ardupilot_gazebo/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export GZ_SIM_RESOURCE_PATH="$ROOT/simulation/models:$ROOT/simulation/worlds:/home/dev/ardupilot_gazebo/models:/home/dev/ardupilot_gazebo/worlds${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
cd "$ROOT"
exec gz sim -v4 -r simulation/worlds/iris_runway_aruco.sdf
