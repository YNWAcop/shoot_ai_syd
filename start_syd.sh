#!/usr/bin/env bash
# Ubuntu desktop startup, laid out like the original 748573.sh.
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
if [ "${1:-}" = '--terminal' ]; then
    shift
    exec bash "$WS/run_syd_session.sh" "$@"
fi
unset PYTHONPATH
source "$WS/env_syd.sh"
command -v gnome-terminal >/dev/null || { echo 'gnome-terminal is unavailable. Use --terminal.'; exit 1; }
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
if [ -z "${DBUS_SESSION_BUS_ADDRESS:-}" ] && [ -S "$XDG_RUNTIME_DIR/bus" ]; then
    export DBUS_SESSION_BUS_ADDRESS="unix:path=$XDG_RUNTIME_DIR/bus"
fi
if command -v xdpyinfo >/dev/null && ! xdpyinfo >/dev/null 2>&1; then
    echo 'Cannot access the Ubuntu desktop. Log in on the robot, or use --terminal.'
    exit 1
fi
mkdir -p "$WS/run_logs"
LOG_DIR=$(mktemp -d "$WS/run_logs/race_$(date +%Y%m%d_%H%M%S)_XXXX")
echo "Opening syd desktop tabs. Enter 1 in the Competition tab after IMU readiness checks."
echo "Logs: $LOG_DIR"
# Quote paths and race arguments once; list all eight tabs explicitly below.
DESKTOP_TOOL="$WS/src/shoot_race/scripts/desktop_tabs.py"
WATCH_COMMAND=$(/usr/bin/python2 "$DESKTOP_TOOL" command /usr/bin/python2 "$DESKTOP_TOOL" watch "$LOG_DIR")
RUN_COMMAND=$(/usr/bin/python2 "$DESKTOP_TOOL" command env "SYD_RVIZ=${SYD_RVIZ:-true}" /usr/bin/python2 "$DESKTOP_TOOL" run "$WS" "$LOG_DIR" "$@")

# Same window/tab layout as 748573.sh, using the syd environment and session.
if ! gnome-terminal --show-menubar --working-directory="$WS" \
--window --title='syd | ROS master' -e "$WATCH_COMMAND roscore" \
--tab --title='syd | Chassis + IMU' -e "$WATCH_COMMAND chassis" \
--tab --title='syd | Shooting log' -e "$WATCH_COMMAND race" \
--tab --title='syd | Navigation' -e "$WATCH_COMMAND navigation" \
--tab --title='syd | Camera' -e "$WATCH_COMMAND camera" \
--tab --title='syd | AR + RViz' -e "$WATCH_COMMAND ar" \
--tab --title='syd | Image recognition' -e "$WATCH_COMMAND recognition" \
--tab --active --title='syd | Competition - enter 1' -e "$RUN_COMMAND"; then
    touch "$LOG_DIR/finished"
    echo 'Unable to open the desktop terminal. You can use --terminal.'
    exit 1
fi
