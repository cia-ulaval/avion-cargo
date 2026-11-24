#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if ROOT_DIR="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null)"; then
  :
else
  ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
fi
cd "$ROOT_DIR"

if ! docker compose version >/dev/null 2>&1; then
  echo "Docker Compose is unavailable or this user cannot access Docker." >&2
  exit 1
fi

# Prepare the host X11 cookie and reuse the repository's existing auth overlay.
if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is unset; run this launcher from a graphical desktop session." >&2
  exit 1
fi
"$ROOT_DIR/scripts/prepare-x11.sh"

# Reuse the base container, existing X11 auth overlay and simulation launcher.
export COMPOSE_FILE="$ROOT_DIR/compose.yaml:$ROOT_DIR/compose-x11-auth.yaml:$ROOT_DIR/compose-payload-manual.yaml"
# The Tauri Linux dependencies and Rust/Node toolchains are part of this image.
docker compose up -d --build dev
exec "$SCRIPT_DIR/launch-simulation.sh"
