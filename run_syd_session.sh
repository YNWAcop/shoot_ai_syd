#!/usr/bin/env bash
# Full competition, including all three shooting stages. Enter 1 to start.
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
test -f "$WS/devel_syd/setup.bash" || { echo 'Missing devel_syd/setup.bash'; exit 1; }
source "$WS/env_syd.sh"
RVIZ_ENABLED="${SYD_RVIZ:-true}"
case "$RVIZ_ENABLED" in
    true|false) ;;
    *) echo 'SYD_RVIZ must be true or false.'; exit 1 ;;
esac

test "$(rospack find shoot_race)" = "$WS/src/shoot_race" || { echo 'Wrong shoot_race package path'; exit 1; }
exec 9>"$WS/.race.lock"
flock -n 9 || { echo 'This launcher is already running.'; exit 1; }
if timeout 5 rosnode list >/dev/null 2>&1; then
    echo 'ROS is already running. Stop the previous session before using this launcher.'
    exit 1
fi
mkdir -p "$WS/run_logs"
LOG_DIR="${SYD_SESSION_LOG_DIR:-}"
if [ -z "$LOG_DIR" ]; then
    LOG_DIR=$(mktemp -d "$WS/run_logs/race_$(date +%Y%m%d_%H%M%S)_XXXX")
fi
CORE_PID=''
BRINGUP_PID=''
MODULE_PIDS=()
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
    for ((i=${#MODULE_PIDS[@]}-1; i>=0; i--)); do
        stop_owned_process "${MODULE_PIDS[$i]}"
    done
    stop_owned_process "$BRINGUP_PID"
    stop_owned_process "$CORE_PID"
    touch "$LOG_DIR/finished"
    echo "Startup logs: $LOG_DIR"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
touch "$LOG_DIR/started"
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
if [ "${SYD_DESKTOP_MODULES:-false}" = true ]; then
    start_module() {
        local logfile="$1"
        shift
        roslaunch "$@" >"$LOG_DIR/$logfile.log" 2>&1 &
        MODULE_PIDS+=("$!")
    }
    start_module chassis abot_bringup robot_with_imu.launch
    start_module navigation robot_slam navigation.launch map_name:=sheji_lab.yaml
    start_module camera track_tag usb_cam_with_calibration.launch
    start_module ar shoot_race ar_visualization.launch rviz:="$RVIZ_ENABLED"
    start_module recognition find_object_2d find_object_2d.launch gui:=true image_topic:=/usb_cam/image_raw objects_path:="$WS/src/abot_find/object"
else
    roslaunch shoot_race bringup.launch rviz:="$RVIZ_ENABLED" >"$LOG_DIR/bringup.log" 2>&1 &
    BRINGUP_PID=$!
fi
echo "Starting sensors. Module logs: $LOG_DIR"
echo "Ubuntu desktop: DISPLAY=$DISPLAY, RViz=$RVIZ_ENABLED, image recognition GUI enabled."
sleep 8
if [ -n "$BRINGUP_PID" ] && ! kill -0 "$BRINGUP_PID" 2>/dev/null; then
    tail -n 60 "$LOG_DIR/bringup.log"
    exit 1
fi
for pid in "${MODULE_PIDS[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "A module exited during startup. Inspect the module tabs and $LOG_DIR."
        exit 1
    fi
done
echo 'FULL COMPETITION: movement and soft-projectile firing start after you enter 1.'
roslaunch shoot_race race.launch dry_fire:=false auto_start:=false force:=false set:="mission.rehearsal_pause_s=0" "$@" > >(tee "$LOG_DIR/race.log") 2>&1
