# 本包全部是 Python 脚本，不需要编译。
# 放在 N8NNWC/src 下，roslaunch/rosrun 即可找到（ROS_PACKAGE_PATH 已包含工作空间）。
# 如果执行了 catkin_make，这里也能正常通过。
cmake_minimum_required(VERSION 2.8.3)
project(shoot_race)
find_package(catkin REQUIRED)
catkin_package()
