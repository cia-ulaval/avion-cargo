# syntax=docker/dockerfile:1.7
# Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + ArduPilot SITL/plugin.
# Build context must be the root of cia-ulaval/avion-cargo.
FROM ubuntu:24.04

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    TZ=Etc/UTC \
    GZ_VERSION=harmonic \
    GZ_CONFIG_PATH=/usr/share/gz \
    ROS_DISTRO=jazzy \
    ROS_DOMAIN_ID=42 \
    ROS_LOCALHOST_ONLY=1 \
    HOME=/home/dev \
    POETRY_CACHE_DIR=/home/dev/.cache/pypoetry \
    USER=dev

# Enable Ubuntu Universe before installing tools such as debugpy and x11-apps.
RUN apt-get update \
    && apt-get install -y --no-install-recommends software-properties-common ca-certificates curl gnupg \
    && add-apt-repository -y universe \
    && rm -rf /var/lib/apt/lists/*

# General development tools, OpenGL/X11 runtime libraries and build dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends \
      adduser adwaita-icon-theme at-spi2-core build-essential ca-certificates cmake curl \
      dbus-x11 file fonts-dejavu-core fonts-liberation g++ gdb gdbserver git gnupg \
      gnome-themes-extra iproute2 iputils-ping libegl1 libgl1 libgl1-mesa-dri libglx-mesa0 \
      libglib2.0-0 libgtk-3-0 libopencv-dev libsm6 libx11-6 libxext6 libxkbcommon-x11-0 \
      libxrender1 locales lsb-release make mesa-utils nano net-tools ninja-build pipx \
      pkg-config procps python3 python3-debugpy python3-dev python3-pip python3-venv \
      rapidjson-dev sudo tzdata unzip vim wget x11-apps x11-utils x11-xserver-utils \
      xauth xdg-utils \
      libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Jazzy repository (Ubuntu Noble / 24.04), following the official ROS apt-source package.
RUN ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')" \
    && curl -fL --retry 4 --retry-delay 2 \
       -o /tmp/ros2-apt-source.deb \
       "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb

# Gazebo Harmonic packages for Ubuntu Noble.
RUN curl -fL --retry 4 --retry-delay 2 \
      https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       ros-jazzy-ros-base ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image \
       gz-harmonic libgz-sim8-dev \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu 24.04's official image already has an 'ubuntu' account/group at UID/GID 1000.
# Remove those defaults so 'dev' can use 1000:1000 and write to a host-owned bind mount.
# ArduPilot's prerequisite installer uses sudo, so grant passwordless sudo to this dev account.
# Pre-create the Poetry cache before switching to dev; fresh Compose volumes inherit its ownership.
RUN if id ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && if getent group ubuntu >/dev/null 2>&1; then groupdel ubuntu; fi \
    && groupadd --gid 1000 dev \
    && useradd --create-home --shell /bin/bash --uid 1000 --gid dev dev \
    && echo 'dev ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/dev \
    && chmod 0440 /etc/sudoers.d/dev \
    && mkdir -p /workspace/avion-cargo/.venv /home/dev/.cache/pypoetry/artifacts \
    && chown -R dev:dev /workspace /home/dev

# The scenario repository pins the upstream revisions used for its checked-in Gazebo world.
COPY simulation/versions.txt /tmp/versions.txt

USER dev
WORKDIR /home/dev
ENV PATH="/home/dev/.local/bin:/home/dev/venv-ardupilot/bin:${PATH}"
RUN pipx install 'poetry>=2.1.3'

# Install and build the exact ArduPilot revision documented by this scenario.
RUN set -eux; \
    ARDUPILOT_REF="$(sed -n 's/^ardupilot=//p' /tmp/versions.txt)"; \
    test -n "${ARDUPILOT_REF}"; \
    git clone --filter=blob:none --no-checkout https://github.com/ArduPilot/ardupilot.git /home/dev/ardupilot; \
    git -C /home/dev/ardupilot checkout --detach "${ARDUPILOT_REF}"; \
    git -C /home/dev/ardupilot submodule update --init --recursive; \
    cd /home/dev/ardupilot; \
    Tools/environment_install/install-prereqs-ubuntu.sh -y; \
    ./waf configure --board sitl --debug; \
    ./waf copter -j2

# Build the Gazebo Harmonic ArduPilot system plugin at the scenario's pinned revision.
RUN set -eux; \
    ARDUPILOT_GAZEBO_REF="$(sed -n 's/^ardupilot_gazebo=//p' /tmp/versions.txt)"; \
    test -n "${ARDUPILOT_GAZEBO_REF}"; \
    git clone --filter=blob:none --no-checkout https://github.com/ArduPilot/ardupilot_gazebo.git /home/dev/ardupilot_gazebo; \
    git -C /home/dev/ardupilot_gazebo checkout --detach "${ARDUPILOT_GAZEBO_REF}"; \
    git -C /home/dev/ardupilot_gazebo submodule update --init --recursive; \
    cd /home/dev/ardupilot_gazebo; \
    export GZ_VERSION=harmonic; \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo; \
    cmake --build build -j2

# Convenience setup for interactive shells inside the persistent container.
RUN printf '%s\n' \
      'source /opt/ros/jazzy/setup.bash' \
      'export GZ_SIM_SYSTEM_PLUGIN_PATH="/home/dev/ardupilot_gazebo/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"' \
      'export GZ_SIM_RESOURCE_PATH="/workspace/avion-cargo/simulation/models:/workspace/avion-cargo/simulation/worlds:/home/dev/ardupilot_gazebo/models:/home/dev/ardupilot_gazebo/worlds${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"' \
      'export PYTHONUNBUFFERED=1' \
      >> /home/dev/.bashrc

COPY --chown=root:root docker/prepare-workspace.sh /usr/local/bin/prepare-workspace.sh
COPY --chown=root:root docker/container-start.sh /usr/local/bin/container-start.sh
COPY --chown=root:root docker/fix-volume-permissions.sh /usr/local/bin/fix-volume-permissions.sh
RUN sudo chmod 0755 /usr/local/bin/prepare-workspace.sh /usr/local/bin/container-start.sh /usr/local/bin/fix-volume-permissions.sh

# Temporary compatibility pin: overrides the floating Poetry version late so Docker can reuse the expensive SITL/plugin build layers.
RUN pipx install --force 'poetry==2.4.3' \
    && poetry --version

# Add ROS workspace build tools in a late layer so this change does not invalidate
# the earlier, expensive ArduPilot SITL and Gazebo plugin build layers.
USER root
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       python3-colcon-common-extensions python3-rosdep python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Install image-resident launch helpers. These live outside /workspace so the source
# bind mount cannot hide them, and they invoke the actual scenario files under simulation/.
RUN <<'CREATE_LAUNCHERS'
set -eu
cat > /usr/local/bin/avion-sitl <<'SCRIPT'
#!/usr/bin/env bash
set -euo pipefail
cd /home/dev/ardupilot
exec ./Tools/autotest/sim_vehicle.py -D -v ArduCopter -f JSON \
  --add-param-file=/workspace/avion-cargo/simulation/config/gazebo-iris-gimbal.parm \
  --console --map
SCRIPT
cat > /usr/local/bin/avion-gazebo <<'SCRIPT'
#!/usr/bin/env bash
set -eo pipefail
# ROS setup.bash references variables that may be unset; disable nounset while sourcing.
set +u
source /opt/ros/jazzy/setup.bash
set -u
# gz-tools discovers commands such as `gz sim` from YAML files under this directory.
export GZ_CONFIG_PATH="${GZ_CONFIG_PATH:-/usr/share/gz}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/home/dev/ardupilot_gazebo/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export GZ_SIM_RESOURCE_PATH="/workspace/avion-cargo/simulation/models:/workspace/avion-cargo/simulation/worlds:/home/dev/ardupilot_gazebo/models:/home/dev/ardupilot_gazebo/worlds${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
exec gz sim -v4 -r /workspace/avion-cargo/simulation/worlds/iris_runway_aruco.sdf
SCRIPT
cat > /usr/local/bin/avion-image-bridge <<'SCRIPT'
#!/usr/bin/env bash
set -eo pipefail
# ROS setup.bash references variables that may be unset; disable nounset while sourcing.
set +u
source /opt/ros/jazzy/setup.bash
set -u
AUTOLANDER_IMAGE_TOPIC='/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image'
exec ros2 run ros_gz_bridge parameter_bridge \
  "${AUTOLANDER_IMAGE_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image"
SCRIPT
cat > /usr/local/bin/avion-autolander <<'SCRIPT'
#!/usr/bin/env bash
set -eo pipefail
# ROS setup.bash references variables that may be unset; disable nounset while sourcing.
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd /workspace/avion-cargo
exec poetry run precision_landing simulation/config/autolanding_config.json --gz-simulation
SCRIPT
# Bind mounts can overwrite executable mode bits from the image. Repair these
# after Compose mounts the repository, immediately before container startup.
cat > /usr/local/bin/avion-entrypoint <<'SCRIPT'
#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR=/workspace/avion-cargo/docker
if [[ -d "$SCRIPT_DIR" ]]; then
  while IFS= read -r -d '' script; do
    chmod 0755 "$script"
  done < <(find "$SCRIPT_DIR" -maxdepth 1 -type f -name '*.sh' -print0)
fi
if [[ $# -eq 0 ]]; then
  set -- /usr/local/bin/container-start.sh
fi
exec "$@"
SCRIPT
chmod 0755 /usr/local/bin/avion-sitl /usr/local/bin/avion-gazebo \
  /usr/local/bin/avion-image-bridge /usr/local/bin/avion-autolander \
  /usr/local/bin/avion-entrypoint
CREATE_LAUNCHERS
USER dev

# Run the entrypoint after Compose has attached the source bind mount so launch
# scripts get executable permissions at runtime, not just in the image layer.
ENTRYPOINT ["/usr/local/bin/avion-entrypoint"]
WORKDIR /workspace/avion-cargo
CMD ["/usr/local/bin/container-start.sh"]
