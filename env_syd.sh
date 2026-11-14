#!/usr/bin/env bash
SYD_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
unset PYTHONPATH
source /opt/ros/melodic/setup.bash
source "$SYD_WS/devel_syd/setup.bash"
export ROS_PACKAGE_PATH="$SYD_WS/src:/opt/ros/melodic/share"
export PYTHONIOENCODING=utf-8
export DISPLAY="${DISPLAY:-:0}"
