#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""原地转速测试：找出底盘真正能转起来的最小角速度，用来设置 shooting.yaml 的 ring.turn_min_rate。

背景（2026-10-16 实测）：环形靶对准时，程序要车再转 0.25 度，发的是最小转速 0.04 rad/s，
车一直没转（驱动把速度取整 + 电机死区），8 秒后超时。所以要先量出"多小的转速车还能动"。

做法：每一档转速先左转、停 0.8 秒，再右转、停 0.8 秒（净转角接近 0），同时用三种方法量实际转角：
  里程计  odom 朝向（轮速计 + IMU 融合）
  激光    车周围墙线的偏角变化（最可信：直接看车身相对墙转了多少；没检测到墙时显示 --）
  轮速    转动期间 /wheel_odom 角速度的平均值（看电机有没有转）
车会原地左右小幅转动。人守在旁边，Ctrl+C 随时停止（停止时会发零速度）。

用法：rosrun shoot_race spin_test.py [t=每次转动秒数] [转速1 转速2 ...]   （需要先启动 bringup.launch）
      不写转速时测默认的一组；不写 t= 时每次大约转 6 度（时间 = 6 度 / 转速，最长 1.5 秒）。
      例：rosrun shoot_race spin_test.py t=1.0 0.5 0.7 1.0
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
from shoot_race.geometry import fit_wall, wrap_angle  # noqa: E402
from shoot_race.motion import VelocityCommand  # noqa: E402
from shoot_race.ros_io import RosIO  # noqa: E402

RATES = [0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50]   # 默认要测的角速度（rad/s）
TARGET_ANGLE_DEG = 6.0   # 每次大约转多少度：转动时间 = 这个角度 / 转速（最长 1.5 秒）
REST_S = 0.8             # 每次转完停多久再读朝向（等车停稳、里程计更新）


def wall_yaw(ros, cfg):
    """用激光墙线测车身朝向：返回 {墙: 偏角弧度}，四面墙里能拟合出来的都给。"""
    points = ros.scan_points()
    if points is None:
        return {}
    angles, xs, ys = points
    fit_cfg = cfg['system']['wall_fit']
    result = {}
    for side in ('front', 'left', 'back', 'right'):
        fit = fit_wall(angles, xs, ys, side,
                       sector_half_angle=math.radians(fit_cfg['sector_half_angle_deg']),
                       expected_distance=None,
                       distance_band=fit_cfg['distance_band'],
                       min_points=fit_cfg['min_points'],
                       outlier_threshold=fit_cfg['outlier_threshold'],
                       max_yaw_offset=math.radians(fit_cfg['max_yaw_offset_deg']))
        if fit is not None:
            result[side] = fit.yaw_offset
    return result


def main():
    rospy.init_node('shoot_race_spin_test')
    cfg = load_config(os.path.join(PACKAGE_DIR, 'config'))
    ros = RosIO(cfg['system'])
    rospy.sleep(1.5)                   # 等订阅、发布连接建立
    loop = rospy.Rate(20)

    def hold(wz, seconds):
        """以角速度 wz 持续发指令 seconds 秒（底盘 250 ms 收不到指令会自己停，所以要一直发）。
        返回这段时间里轮速计角速度的平均值。"""
        end = rospy.get_time() + seconds
        wheel = []
        while not rospy.is_shutdown() and rospy.get_time() < end:
            ros.send_velocity(VelocityCommand(0.0, 0.0, wz))
            speed = ros.wheel_speed()
            if speed is not None:
                wheel.append(speed[2])
            loop.sleep()
        return sum(wheel) / len(wheel) if wheel else float('nan')

    def turn_once(wz, seconds):
        """转一次，返回 (里程计转角度数, 激光转角度数或 None, 轮速计平均角速度)。"""
        odom_start, laser_start = ros.odom_yaw(), wall_yaw(ros, cfg)
        wheel_avg = hold(wz, seconds)
        hold(0.0, REST_S)
        odom_end, laser_end = ros.odom_yaw(), wall_yaw(ros, cfg)
        odom_deg = None
        if odom_start is not None and odom_end is not None:
            odom_deg = math.degrees(wrap_angle(odom_end - odom_start))
        laser_deg = None
        common = set(laser_start) & set(laser_end)
        if common:
            side = sorted(common)[0]
            # fit_wall 的偏角 = 车身相对墙逆时针转了多少（和 WallAligner 用的约定一致），
            # 所以偏角的变化量就是车身转过的角度，左转为正
            laser_deg = math.degrees(wrap_angle(laser_end[side] - laser_start[side]))
        return odom_deg, laser_deg, wheel_avg

    if ros.odom_yaw() is None:
        print('拿不到 odom -> base_link，先启动 bringup.launch')
        return
    fixed_seconds = None
    rates = []
    for arg in sys.argv[1:]:
        if arg.startswith('__'):          # roslaunch / rosrun 自动加的参数
            continue
        if arg.startswith('t='):
            fixed_seconds = float(arg[2:])
        else:
            rates.append(float(arg))
    rates = rates or RATES
    print('转速     期望    左转: 里程计 / 激光 / 轮速计      右转: 里程计 / 激光 / 轮速计')
    print('rad/s    度          度      度    rad/s              度      度    rad/s')
    try:
        for rate in rates:
            seconds = fixed_seconds or min(1.5, math.radians(TARGET_ANGLE_DEG) / rate)
            expected = math.degrees(rate * seconds)
            left = turn_once(rate, seconds)
            right = turn_once(-rate, seconds)

            def fmt(result):
                odom_deg, laser_deg, wheel_avg = result
                odom_text = '  --  ' if odom_deg is None else '%+6.2f' % odom_deg
                laser_text = '  --  ' if laser_deg is None else '%+6.2f' % laser_deg
                return '%s  %s  %+6.3f' % (odom_text, laser_text, wheel_avg)
            print('%.2f   %5.2f     %s         %s' % (rate, expected, fmt(left), fmt(right)))
            if rospy.is_shutdown():
                break
    finally:
        for _ in range(10):
            ros.send_velocity(VelocityCommand(0.0, 0.0, 0.0))
            rospy.sleep(0.05)
    print('激光转角最可信。左右都能稳定转到期望值 70% 以上的最小那一档，就是 ring.turn_min_rate 该设的值。')


if __name__ == '__main__':
    main()
