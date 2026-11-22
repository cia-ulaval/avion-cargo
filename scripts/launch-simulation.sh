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

SIMULATION_ROOT="$ROOT_DIR/logs/simulation"
mkdir -p "$SIMULATION_ROOT"

HOST_LOCK="$SIMULATION_ROOT/.launcher-lock"
if ! mkdir "$HOST_LOCK" 2>/dev/null; then
  old_pid="$(cat "$HOST_LOCK/pid" 2>/dev/null || true)"
  if [[ "$old_pid" =~ ^[0-9]+$ ]] && kill -0 "$old_pid" 2>/dev/null; then
    echo "A simulation launcher is already running (PID $old_pid)." >&2
    exit 1
  fi
  rm -rf -- "$HOST_LOCK"
  if ! mkdir "$HOST_LOCK" 2>/dev/null; then
    echo "Could not acquire simulation launcher lock: $HOST_LOCK" >&2
    exit 1
  fi
fi
printf '%s\n' "$$" > "$HOST_LOCK/pid"

TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
LOG_DIR="$SIMULATION_ROOT/$TIMESTAMP"
STATUS_DIR="$LOG_DIR/.status"
CONTAINER_LOG_DIR="/workspace/avion-cargo/logs/simulation/$TIMESTAMP"
CONTAINER_STATUS_DIR="$CONTAINER_LOG_DIR/.status"
mkdir -p "$STATUS_DIR"

SERVICES=(avion-gazebo avion-image-bridge avion-autolander)
SERVICE_PGIDS=()
CONTAINER_LOCK_HELD=0
STOPPING=0
MONITOR_PID=""
declare -A REPORTED=()

release_container_lock() {
  docker compose exec -T dev bash -s -- "$CONTAINER_STATUS_DIR" <<'REMOTE_RELEASE' >/dev/null 2>&1 || true
set -Eeuo pipefail
lock_dir=/tmp/avion-cargo-simulation.lock
expected_status_dir="$1"
if [[ -f "$lock_dir/status-dir" ]] &&
   [[ "$(cat "$lock_dir/status-dir")" == "$expected_status_dir" ]]; then
  rm -rf -- "$lock_dir"
fi
REMOTE_RELEASE
}

container_group_alive() {
  local pgid="$1"
  docker compose exec -T dev bash -c \
    'kill -0 -- "-$1" 2>/dev/null' _ "$pgid" >/dev/null 2>&1
}

signal_groups() {
  local signal="$1"
  local pgid
  for pgid in "${SERVICE_PGIDS[@]}"; do
    [[ "$pgid" =~ ^[0-9]+$ ]] || continue
    docker compose exec -T dev bash -c \
      'kill -"$1" -- "-$2" 2>/dev/null || true' _ "$signal" "$pgid" \
      >/dev/null 2>&1 || true
  done
}

stop_services() {
  local _round pgid any_alive
  ((${#SERVICE_PGIDS[@]})) || return 0

  echo "Sending SIGTERM to simulation process groups..."
  signal_groups TERM
  for _round in {1..5}; do
    any_alive=0
    for pgid in "${SERVICE_PGIDS[@]}"; do
      if container_group_alive "$pgid"; then
        any_alive=1
        break
      fi
    done
    (( any_alive == 0 )) && break
    sleep 1
  done

  any_alive=0
  for pgid in "${SERVICE_PGIDS[@]}"; do
    if container_group_alive "$pgid"; then
      any_alive=1
      break
    fi
  done
  if (( any_alive )); then
    echo "Some processes did not stop; sending SIGKILL to remaining groups." >&2
    signal_groups KILL
  fi
}

cleanup() {
  local original_status=$?
  (( STOPPING )) && return
  STOPPING=1
  trap - EXIT INT TERM

  if [[ -n "$MONITOR_PID" ]]; then
    kill "$MONITOR_PID" 2>/dev/null || true
    wait "$MONITOR_PID" 2>/dev/null || true
    MONITOR_PID=""
  fi

  echo
  stop_services
  if (( CONTAINER_LOCK_HELD )); then
    release_container_lock
  fi
  rm -rf -- "$HOST_LOCK"
  echo "Logs saved in: $LOG_DIR"
  exit "$original_status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_container_lock() {
  docker compose exec -T dev bash -s -- "$CONTAINER_STATUS_DIR" <<'REMOTE_LOCK'
set -Eeuo pipefail
new_status_dir="$1"
lock_dir=/tmp/avion-cargo-simulation.lock

if [[ -d "$lock_dir" ]]; then
  old_status_dir="$(cat "$lock_dir/status-dir" 2>/dev/null || true)"
  active=0
  if [[ -n "$old_status_dir" && -d "$old_status_dir" ]]; then
    shopt -s nullglob
    for pgid_file in "$old_status_dir"/*.pgid; do
      pgid="$(cat "$pgid_file" 2>/dev/null || true)"
      if [[ "$pgid" =~ ^[0-9]+$ ]] && kill -0 -- "-$pgid" 2>/dev/null; then
        active=1
        break
      fi
    done
  fi
  if (( active )); then
    echo "A simulation process group is already running in the container." >&2
    echo "Existing session status directory: ${old_status_dir:-unknown}" >&2
    exit 1
  fi
  rm -rf -- "$lock_dir"
fi
mkdir "$lock_dir"
printf '%s\n' "$new_status_dir" > "$lock_dir/status-dir"
REMOTE_LOCK
  CONTAINER_LOCK_HELD=1
}

launch_service() {
  local service="$1"
  local logfile="$LOG_DIR/$service.log"
  local statusfile="$STATUS_DIR/$service.exit"
  local pidfile="$STATUS_DIR/$service.pid"
  local pgidfile="$STATUS_DIR/$service.pgid"
  local c_logfile="$CONTAINER_LOG_DIR/$service.log"
  local c_statusfile="$CONTAINER_STATUS_DIR/$service.exit"
  local c_pidfile="$CONTAINER_STATUS_DIR/$service.pid"
  local c_pgidfile="$CONTAINER_STATUS_DIR/$service.pgid"
  local pgid remote_status

  : > "$logfile"
  rm -f -- "$statusfile" "$pidfile" "$pgidfile"

  echo "Starting $service (log: $logfile)"
  if docker compose exec -T dev bash -s -- \
    "$service" "$c_logfile" "$c_statusfile" "$c_pidfile" "$c_pgidfile" <<'REMOTE_START'
set -Eeuo pipefail
service="$1"
logfile="$2"
statusfile="$3"
pidfile="$4"
pgidfile="$5"

if [[ ! -x "/usr/local/bin/$service" ]]; then
  echo "Missing executable: /usr/local/bin/$service" >&2
  exit 127
fi
mkdir -p "$(dirname "$logfile")" "$(dirname "$statusfile")"
rm -f -- "$statusfile" "$pidfile" "$pgidfile"

setsid bash -c '
  service="$1"
  statusfile="$2"
  pidfile="$3"
  pgidfile="$4"
  printf "%s\\n" "$$" > "$pidfile"
  pgid="$(ps -o pgid= -p "$$" | tr -d "[:space:]")"
  printf "%s\\n" "$pgid" > "$pgidfile"
  printf "[%s] Starting %s (PID %s, PGID %s)\\n" "$(date --iso-8601=seconds)" "$service" "$$" "$pgid"
  set +e
  "/usr/local/bin/$service"
  rc=$?
  printf "[%s] %s exited with status %s\\n" "$(date --iso-8601=seconds)" "$service" "$rc"
  printf "%s\\n" "$rc" > "$statusfile"
  kill -TERM -- "-$$" 2>/dev/null || true
  exit "$rc"
' _ "$service" "$statusfile" "$pidfile" "$pgidfile" >> "$logfile" 2>&1 < /dev/null &

for _ in {1..50}; do
  if [[ -s "$pidfile" && -s "$pgidfile" ]]; then
    break
  fi
  sleep 0.1
done
if [[ ! -s "$pidfile" || ! -s "$pgidfile" ]]; then
  echo "Failed to initialize background process for $service." >&2
  exit 1
fi
pgid="$(cat "$pgidfile")"
pid="$(cat "$pidfile")"
if [[ ! "$pid" =~ ^[0-9]+$ || ! "$pgid" =~ ^[0-9]+$ || "$pgid" != "$pid" ]]; then
  echo "Invalid process group for $service (PID=$pid, PGID=$pgid)." >&2
  exit 1
fi
REMOTE_START
  then
    remote_status=0
  else
    remote_status=$?
  fi

  if [[ -s "$pgidfile" ]]; then
    IFS= read -r pgid < "$pgidfile"
    if [[ "$pgid" =~ ^[0-9]+$ ]]; then
      SERVICE_PGIDS+=("$pgid")
    fi
  fi
  return "$remote_status"
}

monitor_services() {
  trap - EXIT INT TERM
  local service statusfile pgidfile status pgid

  while true; do
    for service in "${SERVICES[@]}"; do
      statusfile="$STATUS_DIR/$service.exit"
      pgidfile="$STATUS_DIR/$service.pgid"

      if [[ -f "$statusfile" ]]; then
        if [[ -z "${REPORTED[$service]:-}" ]]; then
          status="$(<"$statusfile")"
          echo "Service $service exited with status $status. Other services will continue. Log: $LOG_DIR/$service.log"
          REPORTED[$service]=1
        fi
      elif [[ -s "$pgidfile" && -z "${REPORTED[$service]:-}" ]]; then
        pgid="$(<"$pgidfile")"
        if [[ "$pgid" =~ ^[0-9]+$ ]] && ! container_group_alive "$pgid"; then
          echo "Service $service stopped without writing an exit status (possibly killed by a signal). Other services will continue. Log: $LOG_DIR/$service.log"
          REPORTED[$service]=1
        fi
      fi
    done
    sleep 1
  done
}

if [[ -z "$(docker compose ps --status running -q dev)" ]]; then
  echo "Dev container is not running; starting it..."
  docker compose up -d dev
fi
if [[ -z "$(docker compose ps --status running -q dev)" ]]; then
  echo "Error: dev container is not running." >&2
  docker compose logs --tail=100 dev >&2 || true
  exit 1
fi

start_container_lock

echo "Starting supporting simulation processes in the background (logs forwarded to files)."
echo "Logs: $LOG_DIR"
for service in "${SERVICES[@]}"; do
  launch_service "$service"
  sleep "${AVION_SIM_STARTUP_DELAY:-1}"
done

echo
echo "Supporting simulation services have been launched."
echo "Per-service logs are under: $LOG_DIR"
echo "Starting avion-sitl interactively; press Ctrl+C to stop SITL and the supporting services."

monitor_services &
MONITOR_PID=$!

docker compose exec -it dev /usr/local/bin/avion-sitl
