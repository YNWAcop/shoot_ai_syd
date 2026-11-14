#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""车身参数测试：给安全检查用的两组数（system.yaml safety），原来都是估计值。

  size   车身尺寸：front_length / rear_length / half_width（车身中心往前、往后、往两侧多远，含枪管、屏幕等最外面的东西）。
         做法：用手把车推到某一面最外面的东西刚好贴住围栏（不要压），程序用激光量那面墙的距离，换算成尺寸。
         激光装在车身中心前方 laser_offset_x（0.06 m）：
             车头贴墙  front_length = 前墙距离 + 0.06
             车尾贴墙  rear_length  = 后墙距离 - 0.06
             侧面贴墙  half_width   = 侧墙距离（激光在中线上）——实测量不出来（贴墙那侧读数不对），车宽用尺子量
         每次读 1 秒（约 10 帧）取中位数；四个方向都打印，哪面贴着墙就看哪一行。车不会动。

  brake  急停滑行：以最高速度（默认 0.5 m/s）直线前进，到速后突然发零速度，量从松开到停住又走了多远。
         safety.brake_decel 按"停车距离 = 余量 + v²/(2·brake_decel) + v·sensor_latency_s"里刹车那一项反算。
         车头前方要有至少 1.5 米空地（激光量，不够不跑），开出去约 0.8~0.9 米；跑完把车推回来可以再测一次。
         途中前方空地不到 0.35 米会立即停车。人守在旁边，Ctrl+C 随时停止（停止时会发零速度）。

用法：rosrun shoot_race body_test.py size
      rosrun shoot_race body_test.py brake [速度 m/s]        （都需要先启动 bringup.launch）
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import math
import os
import sys

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_DIR, 'src'))

from shoot_race.compat import drop_python3_paths  # noqa: E402
drop_python3_paths()   # 去掉 PYTHONPATH 里的 Python3 库目录，必须在 import rospy 之前

import numpy as np  # noqa: E402
import rospy  # noqa: E402

from shoot_race.config import load_config  # noqa: E402
from shoot_race.geometry import fit_wall, points_in_base_frame  # noqa: E402
from shoot_race.motion import VelocityCommand  # noqa: E402
from shoot_race.ros_io import RosIO  # noqa: E402

BRAKE_SPEED = 0.5        # 默认测试速度（m/s），= 现在的最高速度
BRAKE_ACCEL = 1.5        # 起步加速度（m/s²），= 现在的加速度上限
BRAKE_HOLD_S = 1.0       # 加速段结束后再跑多久才松开（底盘死区 0.15~0.25 秒 + 惯性 0.3 秒，1 秒后才接近稳速）
MIN_FREE_M = 1.5         # 开跑前车头前方至少要有多少空地（开出去约 0.6 米 + 滑行约 0.3 米 + 余量）
ABORT_FREE_M = 0.35      # 途中车头前方空地小于这个就立即停


def wall_distances(ros, cfg):
    """四面墙的距离（激光到墙，米），拟合不出来的不给。"""
    points = ros.scan_points()
    if points is None:
        return {}
    angles, xs, ys = points
    fit_cfg = cfg['system']['wall_fit']
    result = {}
    for side in ('front', 'back', 'left', 'right'):
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


def front_free(ros, cfg):
    """车头前方（车身宽度 + 两侧各 5 cm 的走廊里）最近障碍离车头多远（米）；没有激光返回 None。
    车身轮廓里面的点（车自己的零件，见 safety.self_depth）不算。"""
    points = ros.scan_points()
    if points is None:
        return None
    _, xs, ys = points
    safety = cfg['system']['safety']
    bx, by = points_in_base_frame(xs, ys, cfg['system']['laser_offset_x'])
    front = float(safety['front_length'])
    ahead = (bx > front) & (np.abs(by) < float(safety['half_width']) + 0.05)
    if not ahead.any():
        return 9.0
    return float(bx[ahead].min()) - front


def run_size(ros, cfg):
    offset = float(cfg['system']['laser_offset_x'])
    safety = cfg['system']['safety']
    samples = {}
    end = rospy.get_time() + 1.0
    while not rospy.is_shutdown() and rospy.get_time() < end:
        for side, distance in wall_distances(ros, cfg).items():
            samples.setdefault(side, []).append(distance)
        rospy.sleep(0.1)
    if not samples:
        print('一面墙都没拟合出来：车离墙太远，或者激光没数据（先启动 bringup.launch）')
        return
    print('方向   墙距(激光到墙)   如果这一面正贴着墙，对应的参数          现在 system.yaml 里')
    rows = [('front', '车头', 'front_length', 1.0, offset),
            ('back', '车尾', 'rear_length', 1.0, -offset),
            ('left', '左侧', 'half_width', 1.0, 0.0),
            ('right', '右侧', 'half_width', 1.0, 0.0)]
    for side, name, key, sign, add in rows:
        if side not in samples:
            print('%s   --（没看到墙）' % name)
            continue
        distance = float(np.median(samples[side]))
        print('%s   %.3f m           %s = %.3f m                     %.3f' % (
            name, distance, key, sign * distance + add, float(safety[key])))
    print('只看贴着墙的那一行。推到贴住时别用力压（车身会变形、轮子会滑），车身要正对墙（偏 5 度以内）。')
    print('侧面贴墙时激光量不出来（2026-10-21：贴墙那侧读出 0.7~1.0 米这种不可能的数），车宽请用尺子量：')
    print('  一侧轮子贴围栏，量围栏到另一侧最外面的距离，half_width = 车宽 / 2。量完填进 system.yaml safety 一节。')


def run_brake(ros, cfg, speed):
    safety = cfg['system']['safety']
    free = front_free(ros, cfg)
    if free is None:
        print('没有激光数据，先启动 bringup.launch')
        return
    if free < MIN_FREE_M:
        print('车头前方只有 %.2f 米空地，至少要 %.1f 米。把车往后挪、车头对着空的方向再测。' % (free, MIN_FREE_M))
        return
    if ros.odom_pose() is None or ros.wheel_speed() is None:
        print('拿不到 odom 或 /wheel_odom，先启动 bringup.launch')
        return
    loop = rospy.Rate(20)
    log = []              # (时刻, 指令速度, 轮速计速度, odom x, odom y, 前方空地)

    def record(command_speed):
        pose = ros.odom_pose()
        wheel = ros.wheel_speed()
        log.append((rospy.get_time(), command_speed, wheel[0] if wheel else float('nan'),
                    pose[0] if pose else float('nan'), pose[1] if pose else float('nan'), front_free(ros, cfg)))

    print('急停测试：%.2f m/s 直线前进，到速后松开。车头前方空地 %.2f 米。3 秒后开始，Ctrl+C 随时停止。' % (speed, free))
    rospy.sleep(3.0)
    aborted = False
    release_time = None
    try:
        start = rospy.get_time()
        ramp_s = speed / BRAKE_ACCEL
        while not rospy.is_shutdown():
            now = rospy.get_time() - start
            if now >= ramp_s + BRAKE_HOLD_S:
                break
            command = min(speed, BRAKE_ACCEL * now)
            ros.send_velocity(VelocityCommand(command, 0.0, 0.0))
            record(command)
            if log[-1][5] is not None and log[-1][5] < ABORT_FREE_M:
                aborted = True
                break
            loop.sleep()
        release_time = rospy.get_time()
        end = release_time + 2.0
        while not rospy.is_shutdown() and rospy.get_time() < end:
            ros.send_velocity(VelocityCommand(0.0, 0.0, 0.0))
            record(0.0)
            loop.sleep()
    finally:
        for _ in range(10):
            ros.send_velocity(VelocityCommand(0.0, 0.0, 0.0))
            rospy.sleep(0.05)
    if aborted:
        print('途中前方空地不到 %.2f 米，已停车，这次结果不算。' % ABORT_FREE_M)
        return
    if release_time is None or len(log) < 10:
        print('数据太少，没有结果')
        return
    data = np.array([(t, c, w, x, y, f if f is not None else np.nan) for t, c, w, x, y, f in log], dtype=float)
    t = data[:, 0] - release_time
    before = (t < 0) & (t > -0.25)
    v_release = float(np.nanmean(data[before, 2])) if before.any() else float('nan')
    i0 = int(np.argmin(np.abs(t)))
    moving_after = np.where((t > 0) & (np.abs(data[:, 2]) > 0.01))[0]
    i_stop = int(moving_after[-1]) + 1 if len(moving_after) else i0
    i_stop = min(i_stop, len(data) - 1)
    slide_odom = math.hypot(data[-1, 3] - data[i0, 3], data[-1, 4] - data[i0, 4])
    slide_laser = data[i0, 5] - data[-1, 5] if not (np.isnan(data[i0, 5]) or np.isnan(data[-1, 5])) else float('nan')
    stop_time = t[i_stop]
    slow = np.where((t > 0) & (data[:, 2] < 0.9 * v_release))[0]
    react_time = t[slow[0]] if len(slow) else float('nan')
    latency = float(safety['sensor_latency_s'])
    slide = slide_laser if not math.isnan(slide_laser) else slide_odom
    print('')
    print('松开时轮速计速度   %.3f m/s（指令 %.2f）' % (v_release, speed))
    print('松开后多久开始减速  %.2f 秒（底盘死区）' % react_time)
    print('松开后多久停住      %.2f 秒' % stop_time)
    print('松开后又走了        激光 %.3f m / 里程计 %.3f m' % (slide_laser, slide_odom))
    if v_release > 0.05 and slide - v_release * latency > 0.005:
        decel = v_release ** 2 / (2.0 * (slide - v_release * latency))
        model = float(safety['stop_distance']) + v_release ** 2 / (2.0 * float(safety['brake_decel'])) + v_release * latency
        print('换算 brake_decel    %.2f m/s²（现在填的是 %.2f）' % (decel, float(safety['brake_decel'])))
        print('现在的停车距离按 %.2f m 算（含固定余量 %.2f m），实测滑行 %.3f m' % (
            model, float(safety['stop_distance']), slide))
        print('测 2~3 次，brake_decel 取最小的那次（滑得最远的），填进 system.yaml safety.brake_decel。')
    else:
        print('速度太低或滑行太短，算不出 brake_decel')


def main():
    args = [arg for arg in sys.argv[1:] if not arg.startswith('__')]   # 去掉 rosrun 自动加的参数
    if not args or args[0] not in ('size', 'brake'):
        print(__doc__)
        return
    rospy.init_node('shoot_race_body_test')
    cfg = load_config(os.path.join(PACKAGE_DIR, 'config'))
    ros = RosIO(cfg['system'])
    rospy.sleep(1.5)                   # 等订阅、发布连接建立
    if args[0] == 'size':
        run_size(ros, cfg)
    else:
        speed = float(args[1]) if len(args) > 1 else BRAKE_SPEED
        run_brake(ros, cfg, max(0.1, min(0.5, speed)))


if __name__ == '__main__':
    main()
