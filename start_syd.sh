#!/usr/bin/env bash
# Open the original eight-tab Ubuntu layout using the syd workspace.
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
if [ "${1:-}" = '--terminal' ]; then
    shift
    exec bash "$WS/run_syd_session.sh" "$@"
fi
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
if ! /usr/bin/python2 "$WS/src/shoot_race/scripts/desktop_tabs.py" open "$WS" "$LOG_DIR" "$@"; then
    touch "$LOG_DIR/finished"
    echo 'Unable to open the desktop terminal. You can use --terminal.'
    exit 1
fi
