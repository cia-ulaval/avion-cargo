#!/usr/bin/env bash
set -Eeuo pipefail

fix_volume() {
  local path="$1"
  local marker="${path%/}/.docker-permissions-fixed"

  mkdir -p "$path"

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
