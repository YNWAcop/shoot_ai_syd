unset PYTHONPATH
XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u)} pactl unload-module module-suspend-on-idle 2>/dev/null  # 关闭声卡闲置休眠，防止播报吞字
source /opt/ros/melodic/setup.bash
###gmapping with abot###
export ROS_PACKAGE_PATH=/home/abot/748573:$ROS_PACKAGE_PATH
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/748573/devel/setup.bash; roslaunch abot_bringup robot_with_imu.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/748573/devel/setup.bash; roslaunch abot_bringup shoot.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/748573/devel/setup.bash; roslaunch robot_slam navigation.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/748573/devel/setup.bash; roslaunch track_tag usb_cam_with_calibration.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/748573/devel/setup.bash; roslaunch track_tag ar_track_camera.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/748573/devel/setup.bash; roslaunch find_object_2d find_object_2d.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; source ~/748573/devel/setup.bash; roslaunch shoot_race race.launch; exec bash"'
