#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [[ -z "${DISPLAY:-}" ]]; then
  echo "ERROR: DISPLAY is not set. Run this from a terminal in your graphical Linux desktop session." >&2
  exit 2
fi
display_num="$(sed -nE 's/^.*:([0-9]+)(\.[0-9]+)?$/\1/p' <<< "$DISPLAY")"
if [[ -z "$display_num" || ! -S "/tmp/.X11-unix/X${display_num}" ]]; then
  echo "ERROR: No local X11 socket found for DISPLAY=$DISPLAY under /tmp/.X11-unix." >&2
  echo "This helper is for a local Linux desktop session (Xorg or XWayland), not SSH X11 forwarding." >&2
  exit 2
fi

if ! command -v xhost >/dev/null 2>&1; then
  echo "ERROR: xhost is missing on the host. Install x11-xserver-utils (Ubuntu/Debian)." >&2
  exit 2
fi

xhost +local:

if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
  echo "NVIDIA GPU detected; using compose.linux.nvidia.yaml."
  echo "This requires NVIDIA Container Toolkit configured for Docker and Compose >= 2.30."
  docker compose -f compose.yaml -f compose.linux.nvidia.yaml up -d --force-recreate dev
else
  if [[ ! -d /dev/dri ]]; then
    cat >&2 <<'EOF'
ERROR: /dev/dri is not present and no working nvidia-smi was detected.
Check the host graphics driver. This configuration will not silently fall back to TCP GLX.
EOF
    exit 2
  fi
  video_node="$(find /dev/dri -maxdepth 1 -type c -name 'card*' -print -quit || true)"
  render_node="$(find /dev/dri -maxdepth 1 -type c -name 'renderD*' -print -quit || true)"
  if [[ -z "$render_node" ]]; then
    cat >&2 <<'EOF'
ERROR: No /dev/dri/renderD* node was found. Check that the host graphics driver is loaded.
EOF
    exit 2
  fi
  export HOST_RENDER_GID="$(stat -c '%g' "$render_node")"
  if [[ -n "$video_node" ]]; then
    export HOST_VIDEO_GID="$(stat -c '%g' "$video_node")"
  else
    export HOST_VIDEO_GID="$HOST_RENDER_GID"
  fi
  echo "Using DISPLAY=$DISPLAY"
  echo "Using host DRM device groups: video=$HOST_VIDEO_GID render=$HOST_RENDER_GID"
  docker compose -f compose.yaml -f compose.linux.yaml up -d --force-recreate dev
fi
