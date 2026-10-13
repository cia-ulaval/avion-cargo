#!/usr/bin/env bash
set -Eeuo pipefail

X11_SOCKET="${1:-/tmp/.X11-unix/X1}"

if ! command -v xhost >/dev/null 2>&1; then
  echo "xhost is missing. Install x11-xserver-utils." >&2
  exit 1
fi

if ! command -v socat >/dev/null 2>&1; then
  echo "socat is missing. Install it with: sudo apt install socat" >&2
  exit 1
fi

if [[ ! -S "$X11_SOCKET" ]]; then
  echo "X11 socket not found: $X11_SOCKET" >&2
  exit 1
fi

cleanup() {
  xhost -local: >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

xhost +local:
echo "Forwarding TCP port 6000 to $X11_SOCKET."
echo "Keep this terminal open while using GUI apps. Press Ctrl+C to stop."

socat TCP-LISTEN:6000,reuseaddr,fork "UNIX-CLIENT:${X11_SOCKET}"