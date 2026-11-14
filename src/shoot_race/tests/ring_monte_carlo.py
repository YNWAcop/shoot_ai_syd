# -*- coding: utf-8 -*-
"""环形靶对准的蒙特卡洛：按 2026-10-16 录包实测的识别特性，看当前 shooting.yaml 参数下的用时、超时率、弹着偏差。

模拟的内容（都来自录包，见 README 4.5）：
  find_object 约 2.6 帧/秒、延迟 0.2~0.4 秒、单帧噪声标准差 2.9 像素；
  车平移时识别像素只变化理论值的 TRUE_GAIN 倍；底盘从静止起步要 0.45 秒、速度按整数 cm/s 截断；
  每次平移车身朝向偏 YAW_MEAN ± YAW_STD 度（画面偏 YAW_MEAS 倍，弹着点按真实角度偏）。
用法：python3 tests/ring_monte_carlo.py [真实倍数 0.55] [次数 300] [朝向偏差均值 -0.3] [朝向偏差标准差 0.15]
      环境变量 RING_NOISE（单帧噪声像素，默认 2.9）、RING_FPS（帧率，默认 2.6）。2026-10-19 新靶：0.71 / 3.6 / 4.5
结果只用来比较参数好坏，不代表真实命中率。
"""
from __future__ import absolute_import, division, print_function

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))

import numpy as np  # noqa: E402

from shoot_race.aiming import RingAimer  # noqa: E402
from shoot_race.config import load_config  # noqa: E402

cfg = load_config(os.path.join(HERE, '..', 'config'))
RING = cfg['shooting']['ring']
SHIFT = cfg['system']['lateral_shift']
FX = cfg['system']['camera']['fx']
TRUE_GAIN = float(sys.argv[1]) if len(sys.argv) > 1 else 0.55
RUNS = int(sys.argv[2]) if len(sys.argv) > 2 else 300
YAW_MEAN = float(sys.argv[3]) if len(sys.argv) > 3 else -0.3
YAW_STD = float(sys.argv[4]) if len(sys.argv) > 4 else 0.15
YAW_MEAS = 0.7            # 朝向变化在识别像素里的倍数（录包回归约 0.7）
NOISE_PX = float(os.environ.get('RING_NOISE', 2.9))    # 2026-10-19 新靶实测 3.6
FPS = float(os.environ.get('RING_FPS', 2.6))           # 2026-10-19 新靶实测 4.5 帧/秒
K = FX / RING['target_distance_m']     # 真实像素 / 米
DT = 0.05
TIMEOUT = float(os.environ.get('RING_TIMEOUT', 8.0))   # mission.yaml shoot_ring 的 timeout_s


def run(seed):
    rng = np.random.RandomState(seed)
    aimer = RingAimer(RING, SHIFT, FX)
    true_off = rng.uniform(-10, 10) / TRUE_GAIN     # 初始偏差：识别像素 ±10（录包里 2.8~10）
    lat = vel = yaw_meas = yaw_true = 0.0
    cmd_since = still_since = None
    moving = False
    frames, pending = [], []
    period = 1.0 / FPS
    next_frame = rng.uniform(0, period)
    t = 0.0
    while t < TIMEOUT:
        if t >= next_frame:
            meas = TRUE_GAIN * (true_off + K * lat) + yaw_meas + rng.normal(0, NOISE_PX)
            pending.append((t + rng.uniform(0.2, 0.4), RING['aim_x_px'] + meas))
            next_frame += rng.uniform(0.87 * period, 1.13 * period)
        pending.sort()
        while pending and pending[0][0] <= t:
            frames.append(pending.pop(0))
        obs = [f for f in frames if f[0] >= t - RING['max_obs_age_s']]
        if vel == 0.0 and still_since is None:
            still_since = t
        robot_still = still_since is not None and t - still_since >= RING['settle_s']
        vy, ready, info = aimer.update(t, lat + rng.normal(0, 0.001), obs, robot_still)
        if ready:
            return t, true_off + K * lat + yaw_true, aimer.shifts, False
        vcmd = math.copysign(math.floor(abs(vy) * 100) / 100.0, vy)
        if vcmd == 0.0:
            vel, cmd_since, moving = 0.0, None, False
        elif vel == 0.0:
            cmd_since = t if cmd_since is None else cmd_since
            if t - cmd_since >= 0.45:                       # 起步延迟
                vel = vcmd
                if not moving:
                    d = FX * math.radians(rng.normal(YAW_MEAN, YAW_STD))
                    yaw_true += d
                    yaw_meas += YAW_MEAS * d
                moving = True
        else:
            vel = vcmd
        if vel != 0.0:
            still_since = None
        lat += vel * DT
        t += DT
    return TIMEOUT + 0.3, true_off + K * lat + yaw_true, aimer.shifts, True   # 超时：照当前位置开火


if __name__ == '__main__':
    results = [run(s) for s in range(RUNS)]
    times = np.array([r[0] for r in results])
    miss_cm = np.array([abs(r[1]) / K * 100 for r in results])
    print('真实倍数 %.2f，朝向偏差 %.2f±%.2f 度，%d 次：' % (TRUE_GAIN, YAW_MEAN, YAW_STD, RUNS))
    print('  用时 中位 %.1f s、90%% %.1f s；超时 %.1f%%；平均平移 %.1f 次' % (
        np.median(times), np.percentile(times, 90), 100 * np.mean([r[3] for r in results]),
        np.mean([r[2] for r in results])))
    print('  对准偏差（不含枪的散布）中位 %.2f cm、90%% %.2f cm；≤1 cm 的占 %.0f%%' % (
        np.median(miss_cm), np.percentile(miss_cm, 90), 100 * np.mean(miss_cm <= 1.0)))
