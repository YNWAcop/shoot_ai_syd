#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""横向平移阶跃测试：量出底盘对小速度平移指令的反应（启动延迟、实际速度），用来设置环形靶平移参数。

背景（2026-10-16 实测）：环形靶对准时程序发 0.05 m/s 的横向平移指令，0.6 秒内车几乎没动（约 3 mm），
被判定"卡住"。精定位时车能动，可能是底盘起步有延迟、或者小速度有死区。这个测试把它量清楚。

做法：每一档速度先向左平移 0.8 秒、停 1 秒，再向右平移 0.8 秒、停 1 秒（净位移接近 0，单次最多约 4 cm）。
每一次打印（注意 /wheel_odom 的横向速度恒为 0，所以全部用位移来算）：
  起步延迟  从发指令到里程计横向位移超过 2 mm 所用的时间（一直没超过显示 --）
  后段速度  平移后半段里程计位移的斜率（m/s）
  位移      里程计测到的横向位移（停稳后）
  激光      左墙或右墙距离的变化换算出的横向位移（最可信；两侧都没墙时显示 --）
车会左右小幅平移（每次几厘米）。人守在旁边，Ctrl+C 随时停止（停止时会发零速度）。

用法：rosrun shoot_race step_test.py [速度1 速度2 ...]   （需要先启动 bringup.launch）
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import math
import os
import sys

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_DIR, 'src'))

from shoot_race.compat import drop_python3_paths  # noqa: E402
drop_python3_paths()   # 去掉 PYTHONPATH 里的 Python3 库目录，必须在 import rospy 之前

import rospy  # noqa: E402

from shoot_race.config import load_config  # noqa: E402
from shoot_race.geometry import fit_wall  # noqa: E402
from shoot_race.motion import VelocityCommand  # noqa: E402
from shoot_race.ros_io import RosIO  # noqa: E402

SPEEDS = [0.02, 0.03, 0.05]   # 默认测试的横向速度（m/s）
MOVE_S = 0.8                  # 每次平移多久：最多 0.05 × 0.8 = 4 cm，任务框里不会撞挡板
REST_S = 1.0


def side_walls(ros, cfg):
    """左右两侧墙的距离 {side: 米}，拟合不出来的不给。"""
    points = ros.scan_points()
    if points is None:
        return {}
    angles, xs, ys = points
    fit_cfg = cfg['system']['wall_fit']
    result = {}
    for side in ('left', 'right'):
        fit = fit_wall(angles, xs, ys, side,
                       sector_half_angle=math.radians(fit_cfg['sector_half_angle_deg']),
                       expected_distance=None,
                       distance_band=fit_cfg['distance_band'],
                       min_points=fit_cfg['min_points'],
                       outlier_threshold=fit_cfg['outlier_threshold'],
                       max_yaw_offset=math.radians(fit_cfg['max_yaw_offset_deg']))
        if fit is not None:
            result[side] = fit.distance
    return result


def main():
    rospy.init_node('shoot_race_step_test')
    cfg = load_config(os.path.join(PACKAGE_DIR, 'config'))
    ros = RosIO(cfg['system'])
    rospy.sleep(1.5)
    loop = rospy.Rate(20)

    def lateral(start, pose):
        """从 start 到 pose，沿 start 时车身左方向走了多远（米）。"""
        return -math.sin(start[2]) * (pose[0] - start[0]) + math.cos(start[2]) * (pose[1] - start[1])

    def move_once(vy):
        """平移一次，返回 (起步延迟秒数或 None, 后段速度, 里程计横向位移, 激光横向位移或 None)。"""
        start = ros.odom_pose()
        walls_start = side_walls(ros, cfg)
        t0 = rospy.get_time()
        delay = None
        trace = []                      # (时间, 横向位移)
        while not rospy.is_shutdown() and rospy.get_time() - t0 < MOVE_S:
            ros.send_velocity(VelocityCommand(0.0, vy, 0.0))
            pose = ros.odom_pose()
            elapsed = rospy.get_time() - t0
            if pose is not None and start is not None:
                moved_now = lateral(start, pose)
                trace.append((elapsed, moved_now))
                if delay is None and abs(moved_now) > 0.002:
                    delay = elapsed
            loop.sleep()
        t1 = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - t1 < REST_S:
            ros.send_velocity(VelocityCommand(0.0, 0.0, 0.0))
            loop.sleep()
        end = ros.odom_pose()
        walls_end = side_walls(ros, cfg)
        moved = None if start is None or end is None else lateral(start, end)
        late = [p for p in trace if p[0] > MOVE_S / 2]
        speed = float('nan')
        if len(late) >= 2 and late[-1][0] > late[0][0]:
            speed = (late[-1][1] - late[0][1]) / (late[-1][0] - late[0][0])
        laser = None
        if 'left' in walls_start and 'left' in walls_end:
            laser = walls_start['left'] - walls_end['left']          # 往左走，左墙变近
        elif 'right' in walls_start and 'right' in walls_end:
            laser = walls_end['right'] - walls_start['right']        # 往左走，右墙变远
        return delay, speed, moved, laser

    if ros.odom_pose() is None:
        print('拿不到 odom -> base_link，先启动 bringup.launch')
        return
    speeds = [float(a) for a in sys.argv[1:] if not a.startswith('__')] or SPEEDS
    print('指令(m/s) 期望(mm) | 向左: 延迟(s) 后段速度 里程计(mm) 激光(mm) | 向右: 延迟(s) 后段速度 里程计(mm) 激光(mm)')
    try:
        for speed in speeds:
            expected = speed * MOVE_S * 1000

            def fmt(result):
                delay, speed, moved, laser = result
                delay_text = '  -- ' if delay is None else '%5.2f' % delay
                moved_text = '  --  ' if moved is None else '%+6.1f' % (moved * 1000)
                laser_text = '  --  ' if laser is None else '%+6.1f' % (laser * 1000)
                return '%s  %+6.3f  %s  %s' % (delay_text, speed, moved_text, laser_text)
            left = move_once(speed)
            right = move_once(-speed)
            print('  %.3f    %5.1f  |      %s |      %s' % (speed, expected, fmt(left), fmt(right)))
            if rospy.is_shutdown():
                break
    finally:
        for _ in range(10):
            ros.send_velocity(VelocityCommand(0.0, 0.0, 0.0))
            rospy.sleep(0.05)


if __name__ == '__main__':
    main()
