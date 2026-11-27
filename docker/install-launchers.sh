#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="${AVION_CARGO_ROOT:-/workspace/avion-cargo}"
LAUNCHER_DIR="$ROOT/docker/launchers"
BIN_DIR="${AVION_LAUNCHER_BIN_DIR:-/usr/local/bin}"

mkdir -p "$BIN_DIR"
ln -sfn "$LAUNCHER_DIR/avion-sitl" "$BIN_DIR/avion-sitl"
ln -sfn "$LAUNCHER_DIR/avion-gazebo" "$BIN_DIR/avion-gazebo"
ln -sfn "$LAUNCHER_DIR/avion-image-bridge" "$BIN_DIR/avion-image-bridge"
ln -sfn "$LAUNCHER_DIR/avion-autolander" "$BIN_DIR/avion-autolander"
ln -sfn "$LAUNCHER_DIR/avion-entrypoint" "$BIN_DIR/avion-entrypoint"
ln -sfn "$LAUNCHER_DIR/avion-web-proxy" "$BIN_DIR/avion-web-proxy"
ln -sfn "$LAUNCHER_DIR/avion-vtol" "$BIN_DIR/avion-vtol"
ln -sfn "$LAUNCHER_DIR/avion-payload" "$BIN_DIR/avion-payload"
ln -sfn "$LAUNCHER_DIR/avion-payload-manual" "$BIN_DIR/avion-payload-manual"
ln -sfn "$LAUNCHER_DIR/avion-payload-manual-ui" "$BIN_DIR/avion-payload-manual-ui"
ln -sfn "$LAUNCHER_DIR/avion-payload-manual-trilateration" "$BIN_DIR/avion-payload-manual-trilateration"
ln -sfn "$LAUNCHER_DIR/avion-payload-manual-trilateration-ui" "$BIN_DIR/avion-payload-manual-trilateration-ui"
ln -sfn "$LAUNCHER_DIR/avion-d3k-diagnostics" "$BIN_DIR/avion-d3k-diagnostics"
