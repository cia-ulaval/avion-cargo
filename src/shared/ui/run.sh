#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
ENTRYPOINT="${1:-}"

case "$ENTRYPOINT" in
  payload_manual|payload_manual_trilateration) ;;
  *)
    echo "Usage: $0 {payload_manual|payload_manual_trilateration}" >&2
    exit 2
    ;;
esac

APP_UI_DIR="$ROOT/src/$ENTRYPOINT/ui"
APP_TAURI_DIR="$APP_UI_DIR/src-tauri"
if [[ ! -f "$SCRIPT_DIR/package.json" || ! -f "$SCRIPT_DIR/src-tauri/Cargo.toml" \
   || ! -f "$SCRIPT_DIR/src-tauri/tauri.base.conf.json" || ! -f "$SCRIPT_DIR/merge-tauri-config.py" ]]; then
  echo "The shared Avion UI build scaffold is incomplete." >&2
  exit 2
fi
if [[ ! -f "$APP_UI_DIR/index.html" || ! -f "$APP_UI_DIR/src/App.vue" \
   || ! -f "$APP_UI_DIR/src/main.ts" || ! -f "$APP_UI_DIR/src/style.css" \
   || ! -f "$APP_TAURI_DIR/tauri.identity.json" || ! -f "$APP_TAURI_DIR/Cargo.toml" \
   || ! -f "$APP_TAURI_DIR/src/lib.rs" ]]; then
  echo "UI source or app-specific Tauri project is missing for $ENTRYPOINT." >&2
  exit 2
fi

if [[ ! -x "$SCRIPT_DIR/node_modules/.bin/tauri" ]]; then
  echo "Installing shared Avion UI dependencies..."
  npm ci --prefix "$SCRIPT_DIR" --no-audit --no-fund
fi

# Compose the common Tauri build scaffold with this entry point's identity overlay.
python3 "$SCRIPT_DIR/merge-tauri-config.py" \
  "$APP_TAURI_DIR/tauri.identity.json" \
  "$APP_TAURI_DIR/tauri.conf.json"

# Vue source belongs to the selected entry point, while npm dependencies are installed once.
if [[ -L "$APP_UI_DIR/node_modules" ]]; then
  if [[ "$(readlink -f "$APP_UI_DIR/node_modules")" != "$(readlink -f "$SCRIPT_DIR/node_modules")" ]]; then
    rm "$APP_UI_DIR/node_modules"
  fi
elif [[ -e "$APP_UI_DIR/node_modules" ]]; then
  rm -rf "$APP_UI_DIR/node_modules"
fi
if [[ ! -e "$APP_UI_DIR/node_modules" ]]; then
  ln -s "$SCRIPT_DIR/node_modules" "$APP_UI_DIR/node_modules"
fi

cd "$APP_UI_DIR"
export AVION_UI_APP_DIR="$APP_UI_DIR"
exec "$SCRIPT_DIR/node_modules/.bin/tauri" dev
