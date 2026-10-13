#!/usr/bin/env bash
set -Eeuo pipefail
# ROS setup scripts may reference unset variables; source with nounset disabled.
set +u
source /opt/ros/jazzy/setup.bash
set -u
exec ros2 run ros_gz_image image_bridge \
  /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
