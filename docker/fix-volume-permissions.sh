#!/usr/bin/env bash
set -Eeuo pipefail

# Compose runs this one-shot service as root before the dev service starts.
# The named volumes may outlive the image and can retain incorrect ownership.
fix_volume() {
  local path="$1"
  local marker="${path%/}/.docker-permissions-fixed"

  mkdir -p "$path"

  # Recursive chown is only needed once for each named volume. The marker lives
  # inside the volume, so it persists with that volume and is removed by `down -v`.
  if [[ ! -f "$marker" ]]; then
    echo "Initializing ownership for $path"
    chown -R 1000:1000 "$path"
    touch "$marker"
    chown 1000:1000 "$marker"
  else
    echo "Ownership already initialized for $path"
  fi
}

fix_volume /home/dev/.cache/pypoetry
fix_volume /workspace/avion-cargo/.venv
