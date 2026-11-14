#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""把车随机挪开一点（代替人手推车），用于精定位重复性测试（菜单 13、14）。

每次随机选方向：先原地转 3~5 度，再横向平移 3~5 cm，再前后移 2~3 cm。之后由精定位把车拉回示教位置，
比较多次精定位后的靶心像素、墙距、朝向，就知道精定位的重复性。
  转向：看 /odom 的朝向（轮速计 + IMU），转够角度就停；底盘 0.3 rad/s 以下转不动，所以用 0.6 rad/s。
  平移：看激光侧墙距离（里程计横向高估约 3 倍），够距离就停；前后用里程计。
  每个动作最多 2.5 秒，超时就停（底盘没反应时不会一直发指令）。
车只在任务框里挪几厘米。人守在旁边，Ctrl+C 随时停止（停止时会发零速度）。

用法：rosrun shoot_race nudge.py        （需要先启动 bringup.launch）
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import math
import os
import random
import sys

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_DIR, 'src'))

from shoot_race.compat import drop_python3_paths  # noqa: E402
drop_python3_paths()   # 去掉 PYTHONPATH 里的 Python3 库目录，必须在 import rospy 之前

import rospy  # noqa: E402

from shoot_race.config import load_config  # noqa: E402
from shoot_race.geometry import fit_wall, wrap_angle  # noqa: E402
from shoot_race.motion import VelocityCommand  # noqa: E402
from shoot_race.ros_io import RosIO  # noqa: E402

MAX_MOVE_S = 2.5        # 每个动作最多多久
TURN_RATE = 0.6         # rad/s
SIDE_SPEED = 0.06       # m/s
FORWARD_SPEED = 0.05    # m/s


def side_wall(ros, cfg):
    """离车近的那面侧墙 (side, 距离)；都拟合不出来返回 (None, None)。"""
    points = ros.scan_points()
    if points is None:
        return None, None
    angles, xs, ys = points
    fit_cfg = cfg['system']['wall_fit']
    best = (None, None)
    for side in ('left', 'right'):
        fit = fit_wall(angles, xs, ys, side,
                       sector_half_angle=math.radians(fit_cfg['sector_half_angle_deg']),
                       expected_distance=None, distance_band=fit_cfg['distance_band'],
                       min_points=fit_cfg['min_points'], outlier_threshold=fit_cfg['outlier_threshold'],
                       max_yaw_offset=math.radians(fit_cfg['max_yaw_offset_deg']))
        if fit is not None and (best[1] is None or fit.distance < best[1]):
            best = (side, fit.distance)
    return best


def main():
    rospy.init_node('shoot_race_nudge', anonymous=True)
    cfg = load_config(os.path.join(PACKAGE_DIR, 'config'))
    ros = RosIO(cfg['system'])
    rospy.sleep(1.0)
    loop = rospy.Rate(20)
    rng = random.Random()

    def stop(seconds=0.5):
        t0 = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - t0 < seconds:
            ros.send_velocity(VelocityCommand(0.0, 0.0, 0.0))
            loop.sleep()

    def run_until(command, done):
        """一直发 command，直到 done() 为真或超时。返回用了多久。"""
        t0 = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - t0 < MAX_MOVE_S and not done():
            ros.send_velocity(command)
            loop.sleep()
        elapsed = rospy.get_time() - t0
        stop()
        return elapsed

    last_pose = [ros.odom_pose()]

    def odom():
        """最新的 /odom 位姿；偶尔拿不到时用上一次的。"""
        pose = ros.odom_pose()
        if pose is not None:
            last_pose[0] = pose
        return last_pose[0]

    start = odom()
    if start is None:
        print('拿不到 /odom，先启动 bringup.launch')
        return
    turn = math.radians(rng.uniform(3.0, 5.0)) * rng.choice([-1, 1])
    side_move = rng.uniform(0.03, 0.05) * rng.choice([-1, 1])     # 正 = 向左
    forward_move = rng.uniform(0.02, 0.03) * rng.choice([-1, 1])
    try:
        # 1. 原地转
        yaw0 = start[2]
        run_until(VelocityCommand(0.0, 0.0, math.copysign(TURN_RATE, turn)),
                  lambda: abs(wrap_angle(odom()[2] - yaw0)) >= abs(turn))
        turned = wrap_angle(odom()[2] - yaw0)
        # 2. 横向平移（看侧墙）
        side, d0 = side_wall(ros, cfg)
        moved_side = None
        if side is not None:
            def lateral_moved():
                _, d = side_wall(ros, cfg)
                if d is None:
                    return 0.0
                return (d0 - d) if side == 'left' else (d - d0)      # 向左为正
            run_until(VelocityCommand(0.0, math.copysign(SIDE_SPEED, side_move), 0.0),
                      lambda: abs(lateral_moved()) >= abs(side_move))
            moved_side = lateral_moved()
        else:
            print('看不到侧墙，跳过横向平移')
        # 3. 前后（看里程计）
        p0 = odom()

        def forward_moved():
            p = odom()
            return math.cos(p0[2]) * (p[0] - p0[0]) + math.sin(p0[2]) * (p[1] - p0[1])
        run_until(VelocityCommand(math.copysign(FORWARD_SPEED, forward_move), 0.0, 0.0),
                  lambda: abs(forward_moved()) >= abs(forward_move))
        print('已挪开：转 %+.1f 度（目标 %+.1f），横向 %s（目标 %+.1f cm，左为正），前后 %+.1f cm（目标 %+.1f）' % (
            math.degrees(turned), math.degrees(turn),
            '--' if moved_side is None else '%+.1f cm' % (100 * moved_side), 100 * side_move,
            100 * forward_moved(), 100 * forward_move))
    finally:
        stop(0.5)


if __name__ == '__main__':
    main()
