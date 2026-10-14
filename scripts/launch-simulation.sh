#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if ROOT_DIR="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null)"; then
  :
else
  ROOT_DIR="$(pwd)"
fi
cd "$ROOT_DIR"

if ! docker compose version >/dev/null 2>&1; then
  echo "Docker Compose is unavailable or this user cannot access Docker." >&2
  exit 1
fi

TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
LOG_DIR="$ROOT_DIR/logs/simulation/$TIMESTAMP"
STATUS_DIR="$LOG_DIR/.status"
SESSION="avion-sim-$TIMESTAMP"
mkdir -p "$STATUS_DIR"

SERVICES=(avion-autolander avion-sitl avion-gazebo avion-image-bridge)
TERMINAL_PIDS=()
STOPPING=0

cleanup() {
  local original_status=$?
  (( STOPPING )) && return
  STOPPING=1
  trap - EXIT INT TERM

  echo
  echo "Stopping simulation terminal windows and attached exec sessions..."
  for pid in "${TERMINAL_PIDS[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "${TERMINAL_PIDS[@]}"; do
    kill -KILL "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
  echo "Logs saved in: $LOG_DIR"
  exit "$original_status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Ensuring the dev container is running..."
docker compose up -d dev
if [[ -z "$(docker compose ps --status running -q dev)" ]]; then
  echo "Error: dev container is not running." >&2
  docker compose logs --tail=100 dev >&2 || true
  exit 1
fi

launch_terminal() {
  local service="$1"
  local logfile="$LOG_DIR/$service.log"
  local statusfile="$STATUS_DIR/$service.exit"
  local q_root q_service q_log q_status terminal_cmd

  : > "$logfile"
  printf -v q_root '%q' "$ROOT_DIR"
  printf -v q_service '%q' "$service"
  printf -v q_log '%q' "$logfile"
  printf -v q_status '%q' "$statusfile"

  terminal_cmd="cd $q_root
printf 'Service: %s\\nLog: %s\\n\\n' $q_service $q_log
set +e
docker compose exec -it dev $q_service 2>&1 | tee -a $q_log
status=\${PIPESTATUS[0]}
printf '%s\\n' \"\$status\" > $q_status
printf '\\n%s exited with status %s.\\n' $q_service \"\$status\"
printf 'The launcher will close this window when the supervisor stops the other services.\\n'
exec bash -i"

  echo "Opening terminal: $service (log: $logfile)"
  konsole --separate --workdir "$ROOT_DIR" --title "$service" -e bash -lc "$terminal_cmd" &
  TERMINAL_PIDS+=("$!")
  sleep 0.25
}

for service in "${SERVICES[@]}"; do
  launch_terminal "$service"
done

echo
echo "Simulation terminals launched: ${SERVICES[*]}"
echo "Per-process logs: $LOG_DIR"
echo "Close a service window or press Ctrl+C here to stop the simulation session."
echo "Monitoring service lifecycles..."

while true; do
  for service in "${SERVICES[@]}"; do
    statusfile="$STATUS_DIR/$service.exit"
    if [[ -f "$statusfile" ]]; then
      status="$(<"$statusfile")"
      echo "Service $service exited with status $status; stopping the remaining services."
      exit 1
    fi
  done

  for i in "${!TERMINAL_PIDS[@]}"; do
    pid="${TERMINAL_PIDS[$i]}"
    service="${SERVICES[$i]}"
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "Terminal for $service was closed; stopping the remaining services."
      exit 1
    fi
  done
  sleep 1
done
