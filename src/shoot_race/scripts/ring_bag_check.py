#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""环形靶录包分析：find_object 帧率、车静止时的像素噪声、车平移时像素的变化倍数（image_gain）。

为什么要有这个：环形靶对准（aiming.RingAimer）用到的几个数都是 2026-10-16 在 2025 场地录包量出来的
（帧率约 2.6 帧/秒、单帧噪声约 3 像素、image_gain 约 0.55）。换了场地、换了靶子模板或相机，都可能变，
不能写死，要用今年的录包复核。

录包：菜单 4（环形靶演练）、5（实弹）会自动录到 ~/race_logs/shoot_ring_<时分秒>.bag。
      至少要有一次"平移过"的录包（对准时车动过）才能算 image_gain；车没动只能看噪声和帧率。
用法：python2 ring_bag_check.py ~/race_logs/shoot_ring_*.bag
      （菜单 18 = 分析最近 10 个环形靶录包）
      python2 ring_bag_check.py --last ~/race_logs/align_p1_*.bag
      （--last：每个录包只看最后一个静止段 = 精定位做完后停着的那几秒，比较各次之间靶心像素、墙距、朝向差多少，
        就是精定位的重复性。菜单 13 自动调用）
输出：每个录包的帧率、每个静止段的像素中位数和横向位置，最后给出建议的
      shooting.yaml ring.image_gain 和 ring.frame_noise_px。

不开火、车不动，不需要底层节点在运行（只读文件）。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import glob
import math
import os
import sys

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_DIR, 'src'))

from shoot_race.compat import drop_python3_paths  # noqa: E402
drop_python3_paths()

import numpy as np  # noqa: E402

from shoot_race.config import load_config  # noqa: E402
from shoot_race.geometry import fit_wall, scan_to_points  # noqa: E402

STILL_SPEED = 0.003      # 轮速计速度低于这个算静止（m/s、rad/s 用 0.01）
MIN_SEGMENT_S = 1.2      # 静止段至少这么长才用
SKIP_AFTER_STOP_S = 0.4  # 停下后这么久内的帧不用（识别延迟）
MIN_FRAMES = 3           # 一个静止段至少几帧才用


def read_bag(path, topics):
    """读录包，返回 (环形靶 [(t, 像素, 模板号)], 激光 [(t, angle_min, angle_inc, ranges)], 轮速计 [(t, vx, vy, wz)])。"""
    import rosbag
    ring, scans, wheel = [], [], []
    with rosbag.Bag(path) as bag:
        for topic, msg, stamp in bag.read_messages(topics=[topics['ring'], topics['scan'], topics['wheel_odom']]):
            t = stamp.to_sec()
            if topic == topics['ring']:
                ring.append((t, msg.x, int(round(msg.z))))
            elif topic == topics['scan']:
                scans.append((t, msg.angle_min, msg.angle_increment, list(msg.ranges)))
            else:
                v = msg.twist.twist
                wheel.append((t, v.linear.x, v.linear.y, v.angular.z))
    return ring, scans, wheel


def still_segments(wheel):
    """轮速计静止段 [(开始, 结束)]。"""
    segments, start = [], None
    for t, vx, vy, wz in wheel:
        still = abs(vx) < STILL_SPEED and abs(vy) < STILL_SPEED and abs(wz) < 0.01
        if still and start is None:
            start = t
        elif not still and start is not None:
            if t - start >= MIN_SEGMENT_S:
                segments.append((start, t))
            start = None
    if start is not None and wheel and wheel[-1][0] - start >= MIN_SEGMENT_S:
        segments.append((start, wheel[-1][0]))
    return segments


def side_walls(scans, wall_cfg):
    """每帧激光拟合左右墙：[(t, {'left': (距离, 偏角度), 'right': ...})]。"""
    out = []
    for t, angle_min, angle_inc, ranges in scans:
        angles, xs, ys = scan_to_points(angle_min, angle_inc, ranges)
        row = {}
        for side in ('left', 'right', 'back'):
            fit = fit_wall(angles, xs, ys, side,
                           sector_half_angle=math.radians(wall_cfg['sector_half_angle_deg']),
                           expected_distance=None, distance_band=wall_cfg['distance_band'],
                           min_points=wall_cfg['min_points'], outlier_threshold=wall_cfg['outlier_threshold'],
                           max_yaw_offset=math.radians(wall_cfg['max_yaw_offset_deg']))
            if fit is not None:
                row[side] = (fit.distance, math.degrees(fit.yaw_offset))
        out.append((t, row))
    return out


def analyze_bag(ring, scans, wheel, wall_cfg, accept_ids, min_frames=MIN_FRAMES):
    """一个录包：帧率、各静止段（像素帧、左右墙距离/偏角）。"""
    frames = [(t, x) for t, x, obj in ring if obj in accept_ids and x > 0]
    result = {'frames': len(frames), 'lost': sum(1 for _, _, obj in ring if obj not in accept_ids),
              'rate_hz': None, 'segments': []}
    if len(frames) >= 2:
        gaps = np.diff([t for t, _ in frames])
        result['rate_hz'] = 1.0 / float(np.median(gaps))
    walls = side_walls(scans, wall_cfg)
    for a, b in still_segments(wheel):
        u = [x for t, x in frames if a + SKIP_AFTER_STOP_S <= t <= b]
        if len(u) < min_frames:
            continue
        segment = {'start': a, 'end': b, 'u': u}
        for side in ('left', 'right', 'back'):
            rows = [row[side] for t, row in walls if a <= t <= b and side in row]
            if len(rows) >= 5:
                segment[side] = (float(np.median([r[0] for r in rows])), float(np.median([r[1] for r in rows])))
        result['segments'].append(segment)
    return result


def lateral_of(segment, side):
    """横向位置（米，左为正）：离左墙越近越靠左 → -左墙距离；离右墙越远越靠左 → +右墙距离。"""
    distance = segment[side][0]
    return -distance if side == 'left' else distance


def repeatability(rows):
    """--last：各录包最后一个静止段之间的差异（精定位重复性）。"""
    print('\n==================== 精定位重复性（每次精定位后静止的那段）====================')
    print('  录包                      帧数  靶心像素   侧墙距离 mm  侧墙偏角   后墙距离 mm  后墙偏角')
    for name, seg, side in rows:
        u = np.median(seg['u']) if seg['u'] else float('nan')
        side_text = ('%8.1f  %+6.2f°' % (1000 * seg[side][0], seg[side][1])) if side in seg else '     -        -  '
        back_text = ('%8.1f  %+6.2f°' % (1000 * seg['back'][0], seg['back'][1])) if 'back' in seg else '     -        -  '
        print('  %-24s %4d  %7.1f    %s    %s' % (name[:24], len(seg['u']), u, side_text, back_text))
    if len(rows) < 2:
        print('  （至少 2 个录包才能比较）')
        return
    u = np.array([np.median(seg['u']) for _, seg, _ in rows if len(seg['u']) >= 3])
    if len(u) >= 2:
        print('  靶心像素：极差 %.1f 像素、标准差 %.1f 像素（按 1.94 像素/cm ≈ 标准差 %.1f cm；每段中位数自身约有 ±1 像素噪声）' % (
            u.max() - u.min(), u.std(), u.std() / 1.94))
    for key, label in ((rows[0][2], '侧墙'), ('back', '后墙')):
        values = [seg[key] for _, seg, _ in rows if key in seg]
        if len(values) >= 2:
            d = np.array([v[0] for v in values]) * 1000
            y = np.array([v[1] for v in values])
            print('  %s：距离标准差 %.1f mm，偏角标准差 %.2f 度（1.9 m 外 0.1 度 ≈ 0.33 cm）' % (label, d.std(), y.std()))


def main():
    last_only = '--last' in sys.argv
    paths = []
    for arg in sys.argv[1:]:
        if arg == '--last':
            continue
        paths.extend(sorted(glob.glob(os.path.expanduser(arg))))
    if not paths:
        print(__doc__)
        return 1
    cfg = load_config(os.path.join(PACKAGE_DIR, 'config'))
    ring_cfg = cfg['shooting']['ring']
    fx = cfg['system']['camera']['fx']
    k_theory = fx / ring_cfg['target_distance_m']        # 理论：车平移 1 米，像素变化多少
    residuals, pairs, last_rows = [], [], []
    for path in paths:
        try:
            ring, scans, wheel = read_bag(path, cfg['system']['topics'])
        except Exception as error:                        # 录包损坏（断电）等：跳过这个
            print('%s：读不了（%s）' % (os.path.basename(path), error))
            continue
        res = analyze_bag(ring, scans, wheel, cfg['system']['wall_fit'], ring_cfg['accept_ids'],
                          min_frames=0 if last_only else MIN_FRAMES)   # 2、3 号点看不到环形靶，也要看墙
        print('\n%s：%d 帧（丢 %d）  帧率 %s' % (os.path.basename(path), res['frames'], res['lost'],
                                            '%.1f 帧/秒' % res['rate_hz'] if res['rate_hz'] else '-'))
        segments = res['segments']
        # 用两侧里离车近、每段都看得到的那面墙
        sides = [s for s in ('left', 'right') if segments and all(s in seg for seg in segments)]
        side = min(sides, key=lambda s: np.median([seg[s][0] for seg in segments])) if sides else None
        for index, seg in enumerate(segments):
            u = np.array(seg['u'])
            if len(u) >= 6:
                residuals.extend((u - np.median(u)).tolist())
            wall = ('%s墙 %.1f mm 偏角 %+.2f°' % ('左' if side == 'left' else '右', 1000 * seg[side][0], seg[side][1])
                    if side else '看不到侧墙')
            pixel = ('像素中位数 %.1f（抖动 %.1f）' % (np.median(u), np.std(u))) if len(u) else '没看到环形靶'
            print('  静止段 %d：%.1f 秒  %d 帧  %s  %s' % (index, seg['end'] - seg['start'], len(u), pixel, wall))
        if segments:
            last_rows.append((os.path.basename(path), segments[-1], side or 'left'))
        if side:
            for seg in segments[1:]:
                moved_px = k_theory * (lateral_of(seg, side) - lateral_of(segments[0], side))
                pairs.append((float(np.median(seg['u']) - np.median(segments[0]['u'])), moved_px))

    if last_only:
        repeatability(last_rows)
        return 0
    print('\n==================== 汇总 ====================')
    if residuals:
        r = np.array(residuals)
        print('车静止时单帧像素噪声：标准差 %.2f 像素（%d 帧）→ 建议 ring.frame_noise_px: %.1f（现在 %.1f）' % (
            r.std(), len(r), r.std(), ring_cfg.get('frame_noise_px', 3.0)))
    big = [(du, mp) for du, mp in pairs if abs(mp) >= 5.0]      # 平移太小的噪声占比大，不用
    if len(big) >= 2:
        du = np.array([p[0] for p in big])
        mp = np.array([p[1] for p in big])
        gain = float(np.dot(du, mp) / np.dot(mp, mp))
        spread = float(np.std(du - gain * mp))
        print('平移 %d 次：像素实际变化 / 理论变化 = %.2f（残差 %.1f 像素）→ 建议 ring.image_gain: %.2f（现在 %.2f）' % (
            len(big), gain, spread, gain, ring_cfg.get('image_gain', 1.0)))
        for d, m in big:
            print('    理论 %+6.1f 像素  实际 %+6.1f 像素' % (m, d))
        if not 0.2 <= gain <= 1.5:
            print('  ！结果超出 0.2~1.5，多半是侧墙看错或录包里车被人推过，不要直接用')
    else:
        print('平移数据不够（需要至少 2 次平移 ≥ 5 像素的录包），image_gain 先保持 %.2f' % ring_cfg.get('image_gain', 1.0))
    return 0


if __name__ == '__main__':
    sys.exit(main())
