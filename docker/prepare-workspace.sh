#!/usr/bin/env bash
set -Eeuo pipefail

ROOT=/workspace/avion-cargo
cd "$ROOT"

if [[ ! -f pyproject.toml || ! -f poetry.lock ]]; then
  echo "ERROR: $ROOT must be the avion-cargo repository root (pyproject.toml and poetry.lock are required)." >&2
  exit 2
fi

if [[ ! -f .venv/pyvenv.cfg ]]; then
  mkdir -p .venv
  /usr/bin/python3 -m venv --system-site-packages .venv
fi
if grep -q '^include-system-site-packages = false$' .venv/pyvenv.cfg; then
  sed -i 's/^include-system-site-packages = false$/include-system-site-packages = true/' .venv/pyvenv.cfg
fi

poetry env use "$ROOT/.venv/bin/python"
poetry install --with dev

poetry run python - <<'PY'
import cv2
import rclpy
from sensor_msgs.msg import Image

print(f"Python dependencies ready; OpenCV={cv2.__version__}; ArUco={hasattr(cv2, 'aruco')}")
print(f"ROS 2 image message available: {Image.__name__}")
PY

poetry run precision_landing --help >/dev/null
poetry run calibrate_camera --help >/dev/null
echo "Workspace ready at $ROOT (Python $(poetry run python --version 2>&1))."
