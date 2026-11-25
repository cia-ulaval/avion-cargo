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

RUN apt-get update \
    && apt-get install -y --no-install-recommends software-properties-common ca-certificates curl gnupg \
    && add-apt-repository -y universe \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
      adduser adwaita-icon-theme at-spi2-core build-essential ca-certificates cmake curl \
      dbus-x11 file fonts-dejavu-core fonts-liberation g++ gdb gdbserver git gnupg socat \
      gnome-themes-extra iproute2 iputils-ping libegl1 libgl1 libgl1-mesa-dri libglx-mesa0 \
      libglib2.0-0 libgtk-3-0 libopencv-dev libsm6 libx11-6 libxext6 libxkbcommon-x11-0 \
      libxrender1 libqt5svg5 locales lsb-release make mesa-utils nano net-tools ninja-build pipx \
      pkg-config procps python3 python3-debugpy python3-dev python3-pip python3-venv \
      rapidjson-dev sudo tzdata unzip vim wget lua5.4 x11-apps x11-utils x11-xserver-utils \
      xauth xdg-utils \
      libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN if id ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && if getent group ubuntu >/dev/null 2>&1; then groupdel ubuntu; fi \
    && groupadd --gid 1000 dev \
    && useradd --create-home --shell /bin/bash --uid 1000 --gid dev dev \
    && echo 'dev ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/dev \
    && chmod 0440 /etc/sudoers.d/dev \
    && mkdir -p /workspace/avion-cargo/.venv /home/dev/.cache/pypoetry/artifacts \
    && chown -R dev:dev /workspace /home/dev

COPY simulation/versions/ardupilot.version /tmp/versions/ardupilot.version

USER dev
WORKDIR /home/dev
ENV PATH="/home/dev/.local/bin:/home/dev/venv-ardupilot/bin:${PATH}"
RUN set -eux; \
    ARDUPILOT_REF="$(cat /tmp/versions/ardupilot.version)"; \
    test -n "${ARDUPILOT_REF}"; \
    git clone --filter=blob:none --no-checkout https://github.com/bertrand-awz/ardupilot.git /home/dev/ardupilot; \
    git -C /home/dev/ardupilot checkout --detach "${ARDUPILOT_REF}"; \
    git -C /home/dev/ardupilot submodule update --init --recursive; \
    cd /home/dev/ardupilot; \
    Tools/environment_install/install-prereqs-ubuntu.sh -y; \
    ./waf configure --board sitl --debug; \
    ./waf copter -j2; \
    ./waf plane -j2; \
    mkdir -p /home/dev/ardupilot/scripts; \
    ln -sfn \
      ../libraries/AP_Scripting/applets/plane_precland.lua \
      /home/dev/ardupilot/scripts/plane_precland.lua; \
    test -s /home/dev/ardupilot/scripts/plane_precland.lua

# Keep every dependency below the expensive ArduPilot build above as much as 
# possible. Docker reuses earlier layers when a later layer changes. A change 
# above the ArduPilot build will invalidate its cache and trigger the long 
# rebuild, so do it only if strictly necessary.
USER root

RUN curl -fsSL https://deb.nodesource.com/setup_22.x | bash - \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       nodejs libwebkit2gtk-4.1-dev libayatana-appindicator3-dev \
       librsvg2-dev libxdo-dev libssl-dev libgtk-3-dev patchelf \
    && node --version \
    && npm --version \
    && rm -rf /var/lib/apt/lists/*

RUN ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')" \
    && curl -fL --retry 4 --retry-delay 2 \
       -o /tmp/ros2-apt-source.deb \
       "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb

RUN curl -fL --retry 4 --retry-delay 2 \
      https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       ros-jazzy-ros-base ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image \
       gz-harmonic libgz-sim8-dev libgz-plugin2-dev python3-gz-transport13 \
    && rm -rf /var/lib/apt/lists/*

USER dev
WORKDIR /home/dev
ENV PATH="/home/dev/.cargo/bin:/home/dev/.local/bin:/home/dev/venv-ardupilot/bin:${PATH}"
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
    sh -s -- -y --profile minimal --default-toolchain stable \
    && rustc --version \
    && cargo --version

COPY simulation/versions/ardupilot_gazebo_plugin.version /tmp/versions/ardupilot_gazebo_plugin.version

RUN set -eux; \
    ARDUPILOT_GAZEBO_REF="$(cat /tmp/versions/ardupilot_gazebo_plugin.version)"; \
    test -n "${ARDUPILOT_GAZEBO_REF}"; \
    git clone --filter=blob:none --no-checkout https://github.com/ArduPilot/ardupilot_gazebo.git /home/dev/ardupilot_gazebo; \
    git -C /home/dev/ardupilot_gazebo checkout --detach "${ARDUPILOT_GAZEBO_REF}"; \
    git -C /home/dev/ardupilot_gazebo submodule update --init --recursive; \
    cd /home/dev/ardupilot_gazebo; \
    export GZ_VERSION=harmonic; \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo; \
    cmake --build build -j2

COPY simulation/versions/d3kble_gazebo_plugin.version /tmp/versions/d3kble_gazebo_plugin.version

RUN set -eux; \
    D3K_SIMULATOR_REF="$(cat /tmp/versions/d3kble_gazebo_plugin.version)"; \
    test -n "${D3K_SIMULATOR_REF}"; \
    mkdir -p /home/dev/dwm3000-esp32-interface; \
    curl -fL --retry 4 --retry-delay 2 \
      "https://codeload.github.com/PladsElsker/dwm3000-esp32-interface/tar.gz/${D3K_SIMULATOR_REF}" \
      -o /tmp/dwm3000-esp32-interface.tar.gz; \
    tar -xzf /tmp/dwm3000-esp32-interface.tar.gz \
      --strip-components=1 -C /home/dev/dwm3000-esp32-interface; \
    rm /tmp/dwm3000-esp32-interface.tar.gz; \
    cmake -S /home/dev/dwm3000-esp32-interface/simulation/gazebo \
      -B /home/dev/d3k-gazebo-build \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DCMAKE_INSTALL_PREFIX=/home/dev/d3k-gazebo-install; \
    cmake --build /home/dev/d3k-gazebo-build --parallel 2; \
    ctest --test-dir /home/dev/d3k-gazebo-build --output-on-failure; \
    cmake --install /home/dev/d3k-gazebo-build

RUN printf '%s\n' \
      'source /opt/ros/jazzy/setup.bash' \
      'export GZ_SIM_SYSTEM_PLUGIN_PATH="/home/dev/d3k-gazebo-install/lib:/home/dev/ardupilot_gazebo/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"' \
      'export GZ_SIM_RESOURCE_PATH="/workspace/avion-cargo/simulation/models:/workspace/avion-cargo/simulation/worlds:/home/dev/ardupilot_gazebo/models:/home/dev/ardupilot_gazebo/worlds${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"' \
      'export PYTHONUNBUFFERED=1' \
      >> /home/dev/.bashrc

COPY --chown=root:root docker/prepare-workspace.sh /usr/local/bin/prepare-workspace.sh
COPY --chown=root:root docker/container-start.sh /usr/local/bin/container-start.sh
COPY --chown=root:root docker/fix-volume-permissions.sh /usr/local/bin/fix-volume-permissions.sh
RUN sudo chmod 0755 /usr/local/bin/prepare-workspace.sh /usr/local/bin/container-start.sh /usr/local/bin/fix-volume-permissions.sh

RUN pipx install 'poetry==2.4.3' \
    && poetry --version

USER root
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       python3-colcon-common-extensions python3-rosdep python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN <<'CREATE_LAUNCHERS'
set -eu
cat > /usr/local/bin/avion-sitl <<'SCRIPT'
#!/usr/bin/env bash
set -Eeuo pipefail

ARDUPILOT_ROOT="${AVION_ARDUPILOT_ROOT:-/home/dev/ardupilot}"
cd "$ARDUPILOT_ROOT"

VEHICLE="${AVION_SITL_VEHICLE:-ArduCopter}"
FRAME="${AVION_SITL_FRAME:-JSON}"
MODEL="${AVION_SITL_MODEL:-}"
PARAM_FILE="${AVION_SITL_PARAM_FILE-/workspace/avion-cargo/simulation/config/gazebo_iris.parm}"

if [[ "$VEHICLE" == ArduPlane ]]; then
  source_script="$PWD/libraries/AP_Scripting/applets/plane_precland.lua"
  target_script="$PWD/scripts/plane_precland.lua"
  if [[ ! -f "$source_script" || ! -r "$source_script" || ! -s "$source_script" ]]; then
    echo "Cannot start ArduPlane: precision landing applet missing or unreadable: $source_script" >&2
    echo "Restore the ArduPilot revision in simulation/versions.txt or rebuild the Docker image." >&2
    exit 1
  fi

  mkdir -p scripts
  if [[ -e "$target_script" ]]; then
    if ! cmp -s -- "$source_script" "$target_script"; then
      echo "Cannot start ArduPlane: $target_script differs from the bundled precision landing applet." >&2
      echo "Inspect and move the custom script before retrying; it has not been overwritten." >&2
      exit 1
    fi
  else
    ln -sfn -- ../libraries/AP_Scripting/applets/plane_precland.lua "$target_script"
  fi
  if ! cmp -s -- "$source_script" "$target_script"; then
    echo "Cannot start ArduPlane: precision landing applet installation failed: $target_script" >&2
    exit 1
  fi
  echo "QuadPlane precision landing script ready: $target_script"
fi

if [[ "$VEHICLE" == ArduPlane ]]; then
  for applet in plane_precland qland_yaw_alignment; do
    description="precision landing"
    [[ "$applet" != qland_yaw_alignment ]] || description="QLAND yaw alignment"
    source_script="$PWD/libraries/AP_Scripting/applets/$applet.lua"
    target_script="$PWD/scripts/$applet.lua"
    if [[ ! -f "$source_script" || ! -r "$source_script" || ! -s "$source_script" ]]; then
      echo "Cannot start ArduPlane: $description applet missing or unreadable: $source_script" >&2
      echo "Check the patched ArduPilot revision in simulation/versions.txt, then rebuild the Docker image." >&2
      exit 1
    fi

    mkdir -p scripts
    if [[ -e "$target_script" ]]; then
      if ! cmp -s -- "$source_script" "$target_script"; then
        echo "Cannot start ArduPlane: $target_script differs from the bundled $description applet." >&2
        echo "Inspect and move the custom script before retrying; it has not been overwritten." >&2
        exit 1
      fi
    else
      ln -sfn -- "../libraries/AP_Scripting/applets/$applet.lua" "$target_script"
    fi
    if ! cmp -s -- "$source_script" "$target_script"; then
      echo "Cannot start ArduPlane: $description applet installation failed: $target_script" >&2
      exit 1
    fi
    echo "QuadPlane $description script ready: $target_script"
  done
fi

args=(-D -v "$VEHICLE" -f "$FRAME")
if [[ -n "$MODEL" ]]; then
  args+=(--model "$MODEL")
fi
if [[ -n "$PARAM_FILE" ]]; then
  args+=("--add-param-file=$PARAM_FILE")
fi
if [[ -n "${AVION_SITL_GCS_OUT:-}" ]]; then
  args+=(--out "$AVION_SITL_GCS_OUT")
fi

exec ./Tools/autotest/sim_vehicle.py "${args[@]}"
SCRIPT

cat > /usr/local/bin/avion-gazebo <<'SCRIPT'
#!/usr/bin/env bash
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u

export GZ_CONFIG_PATH="${GZ_CONFIG_PATH:-/usr/share/gz}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/home/dev/d3k-gazebo-install/lib:/home/dev/ardupilot_gazebo/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export GZ_SIM_RESOURCE_PATH="/workspace/avion-cargo/simulation/models:/workspace/avion-cargo/simulation/worlds:/home/dev/ardupilot_gazebo/models:/home/dev/ardupilot_gazebo/worlds${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

WORLD="${AVION_WORLD:-/workspace/avion-cargo/simulation/worlds/runway_iris_faruco.sdf}"

exec gz sim -v4 -r "$WORLD"
SCRIPT

cat > /usr/local/bin/avion-image-bridge <<'SCRIPT'
#!/usr/bin/env bash
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u

AUTOLANDER_IMAGE_TOPIC="${AVION_IMAGE_TOPIC:-/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image}"

exec ros2 run ros_gz_bridge parameter_bridge \
  "${AUTOLANDER_IMAGE_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image"
SCRIPT

cat > /usr/local/bin/avion-autolander <<'SCRIPT'
#!/usr/bin/env bash
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd /workspace/avion-cargo

CONFIG="${AVION_AUTOLANDER_CONFIG:-simulation/config/autolanding_config.json}"

exec poetry run precision_landing "$CONFIG" --gz-simulation
SCRIPT

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

cat > /usr/local/bin/avion-web-proxy <<'SCRIPT'
#!/usr/bin/env bash
set -euo pipefail
exec socat TCP-LISTEN:8086,bind=0.0.0.0,reuseaddr,fork TCP:127.0.0.1:8085
SCRIPT

cat > /usr/local/bin/avion-vtol <<'SCRIPT'
#!/usr/bin/env bash
set -Eeuo pipefail
cd /workspace/avion-cargo

CONFIG="${AVION_VTOL_CONFIG:-}"
if [[ -z "$CONFIG" ]]; then
  echo "AVION_VTOL_CONFIG must be set by the active simulation Compose file." >&2
  exit 2
fi
if [[ ! -f "$CONFIG" ]]; then
  echo "VTOL configuration does not exist: $CONFIG" >&2
  exit 2
fi
exec poetry run run_vtol --config "$CONFIG"
SCRIPT

cat > /usr/local/bin/avion-payload <<'SCRIPT'
#!/usr/bin/env bash
set -Eeuo pipefail
cd /workspace/avion-cargo

CONFIG="${AVION_PAYLOAD_CONFIG:-}"
if [[ -z "$CONFIG" ]]; then
  echo "AVION_PAYLOAD_CONFIG must be set by the active simulation Compose file." >&2
  exit 2
fi
if [[ ! -f "$CONFIG" ]]; then
  echo "Payload configuration does not exist: $CONFIG" >&2
  exit 2
fi
exec poetry run run_payload --config "$CONFIG"
SCRIPT

cat > /usr/local/bin/avion-payload-manual <<'SCRIPT'
#!/usr/bin/env bash
set -Eeuo pipefail
cd /workspace/avion-cargo

CONFIG="${AVION_MANUAL_PAYLOAD_CONFIG:-}"
UART_DEVICE="${AVION_MANUAL_PAYLOAD_UART_DEVICE:-}"
if [[ -z "$CONFIG" || ! -f "$CONFIG" ]]; then
  echo "AVION_MANUAL_PAYLOAD_CONFIG must point to an existing manual payload JSON configuration." >&2
  exit 2
fi
if [[ -n "$UART_DEVICE" ]]; then
  echo "Waiting for payload D3K PTY: $UART_DEVICE"
  for _ in {1..60}; do
    [[ -e "$UART_DEVICE" ]] && break
    sleep 1
  done
  if [[ ! -e "$UART_DEVICE" ]]; then
    echo "Payload D3K PTY did not appear: $UART_DEVICE" >&2
    exit 1
  fi
fi
exec poetry run run_payload_manual \
  --config "$CONFIG"
SCRIPT

cat > /usr/local/bin/avion-payload-manual-ui <<'SCRIPT'
#!/usr/bin/env bash
set -Eeuo pipefail
cd /workspace/avion-cargo/src/payload_manual/ui

if [[ ! -x node_modules/.bin/tauri ]]; then
  echo "Installing the Payload Manual Control UI dependencies..."
  npm install --no-audit --no-fund
fi

exec npm run tauri -- dev
SCRIPT

cat > /usr/local/bin/avion-d3k-diagnostics <<'SCRIPT'
#!/usr/bin/env bash
set -Eeuo pipefail
cd /workspace/avion-cargo

CONFIG="${AVION_D3K_DIAGNOSTICS_CONFIG:-}"
if [[ -z "$CONFIG" ]]; then
  echo "AVION_D3K_DIAGNOSTICS_CONFIG must be set by the active simulation Compose file." >&2
  exit 2
fi
if [[ ! -f "$CONFIG" ]]; then
  echo "D3K diagnostics configuration does not exist: $CONFIG" >&2
  exit 2
fi
export PYTHONPATH="/workspace/avion-cargo/src${PYTHONPATH:+:$PYTHONPATH}"
exec poetry run run_d3k_diagnostics --config "$CONFIG"
SCRIPT

chmod 0755 /usr/local/bin/avion-sitl /usr/local/bin/avion-gazebo \
  /usr/local/bin/avion-image-bridge /usr/local/bin/avion-autolander \
  /usr/local/bin/avion-entrypoint /usr/local/bin/avion-web-proxy \
  /usr/local/bin/avion-vtol /usr/local/bin/avion-payload \
  /usr/local/bin/avion-payload-manual /usr/local/bin/avion-payload-manual-ui \
  /usr/local/bin/avion-d3k-diagnostics
CREATE_LAUNCHERS

USER dev

ENTRYPOINT ["/usr/local/bin/avion-entrypoint"]
WORKDIR /workspace/avion-cargo
CMD ["/usr/local/bin/container-start.sh"]
