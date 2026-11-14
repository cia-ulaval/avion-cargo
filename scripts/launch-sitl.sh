#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ARDUPILOT_ROOT="${AVION_ARDUPILOT_ROOT:-/home/dev/ardupilot}"
cd "$ARDUPILOT_ROOT"

VEHICLE="${AVION_SITL_VEHICLE:-ArduCopter}"
FRAME="${AVION_SITL_FRAME:-JSON}"
MODEL="${AVION_SITL_MODEL:-}"
PARAM_FILE="${AVION_SITL_PARAM_FILE-$REPO_ROOT/simulation/config/gazebo-iris-gimbal.parm}"

if [[ "$VEHICLE" == ArduPlane ]]; then
  # QuadPlane needs this Lua applet to turn LANDING_TARGET measurements into
  # QLOITER/QLAND position corrections. Repair old images at every launch.
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

args=(-D -v "$VEHICLE" -f "$FRAME")
if [[ -n "$MODEL" ]]; then
  args+=(--model "$MODEL")
fi
if [[ -n "$PARAM_FILE" ]]; then
  args+=("--add-param-file=$PARAM_FILE")
fi

exec ./Tools/autotest/sim_vehicle.py "${args[@]}"
