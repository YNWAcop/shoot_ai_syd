#!/usr/bin/env bash
SYD_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
unset PYTHONPATH
source /opt/ros/melodic/setup.bash
source "$SYD_WS/devel_syd/setup.bash"
export ROS_PACKAGE_PATH="$SYD_WS/src:/opt/ros/melodic/share"
export PYTHONIOENCODING=utf-8
export DISPLAY="${DISPLAY:-:0}"
# SSH sessions may not inherit Ubuntu GDM's desktop authorization file.
# Preserve an explicitly supplied XAUTHORITY (including forwarded displays).
if [ -z "${XAUTHORITY:-}" ] && [ "$DISPLAY" = ':0' ]; then
    SYD_XAUTH="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/gdm/Xauthority"
    if [ -r "$SYD_XAUTH" ]; then
        export XAUTHORITY="$SYD_XAUTH"
    elif [ -r "$HOME/.Xauthority" ]; then
        export XAUTHORITY="$HOME/.Xauthority"
    fi
    unset SYD_XAUTH
fi
