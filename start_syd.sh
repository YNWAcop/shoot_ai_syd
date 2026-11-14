#!/usr/bin/env bash
# Full competition, including all three shooting stages. Enter 1 to start.
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
test -f "$WS/devel_syd/setup.bash" || { echo 'Missing devel_syd/setup.bash'; exit 1; }
source "$WS/env_syd.sh"

test "$(rospack find shoot_race)" = "$WS/src/shoot_race" || { echo 'Wrong shoot_race package path'; exit 1; }
exec 9>"$WS/.race.lock"
flock -n 9 || { echo 'This launcher is already running.'; exit 1; }
if timeout 5 rosnode list >/dev/null 2>&1; then
    echo 'ROS is already running. Stop the previous session before using this launcher.'
    exit 1
fi
mkdir -p "$WS/run_logs"
LOG_DIR=$(mktemp -d "$WS/run_logs/race_$(date +%Y%m%d_%H%M%S)_XXXX")
CORE_PID=''
BRINGUP_PID=''
stop_owned_process() {
    local pid="$1"
    [ -n "$pid" ] || return 0
    kill -INT "$pid" 2>/dev/null || true
    for n in {1..20}; do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.5
    done
    kill -TERM "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
}
cleanup() {
    echo 'Stopping this session...'
    stop_owned_process "$BRINGUP_PID"
    stop_owned_process "$CORE_PID"
    echo "Startup logs: $LOG_DIR"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
roscore >"$LOG_DIR/roscore.log" 2>&1 &
CORE_PID=$!
READY=0
for n in {1..15}; do
    kill -0 "$CORE_PID" 2>/dev/null || break
    if timeout 2 rosnode list >/dev/null 2>&1; then READY=1; break; fi
    sleep 1
done
if [ "$READY" != 1 ]; then
    cat "$LOG_DIR/roscore.log"
    exit 1
fi
roslaunch shoot_race bringup.launch >"$LOG_DIR/bringup.log" 2>&1 &
BRINGUP_PID=$!
echo "Starting sensors. Log: $LOG_DIR/bringup.log"
sleep 8
if ! kill -0 "$BRINGUP_PID" 2>/dev/null; then
    tail -n 60 "$LOG_DIR/bringup.log"
    exit 1
fi
echo 'FULL COMPETITION: movement and soft-projectile firing start after you enter 1.'
roslaunch shoot_race race.launch dry_fire:=false auto_start:=false force:=false set:="mission.rehearsal_pause_s=0" "$@"
