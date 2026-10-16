#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is unset. Run this script from your graphical desktop session." >&2
  exit 1
fi
if ! command -v xauth >/dev/null 2>&1; then
  echo "Install xauth on the host first:" >&2
  echo "  Arch Linux: sudo pacman -S xorg-xauth" >&2
  echo "  Debian/Ubuntu: sudo apt install xauth" >&2
  exit 1
fi

entries="$(xauth nlist "$DISPLAY")"
if [[ -z "$entries" ]]; then
  echo "No X11 cookie found for DISPLAY=$DISPLAY. Check the host XAUTHORITY variable." >&2
  exit 1
fi

umask 077
auth_dir="$ROOT_DIR/.cache/x11"
mkdir -p "$auth_dir"
chmod 700 "$auth_dir"
auth_tmp="$(mktemp "$auth_dir/.Xauthority.XXXXXX")"
trap 'rm -f -- "$auth_tmp"' EXIT

printf '%s\n' "$entries" | sed 's/^..../ffff/' | xauth -f "$auth_tmp" nmerge -
mv -f -- "$auth_tmp" "$auth_dir/Xauthority"
echo "X11 authentication prepared for DISPLAY=$DISPLAY."
