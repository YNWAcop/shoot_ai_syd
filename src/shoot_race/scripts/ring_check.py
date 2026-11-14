#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""环形靶找靶心检查（菜单 23）：车不动、不开火，连续看几秒相机画面，打印每帧找到的靶心和处理用时。

用途：
  1. 确认车上真实画面里能找到同心圆（得分要明显高于 min_score）；
  2. 看车上 CPU 处理一帧要多久（比赛时每秒最多处理 max_rate_hz 帧）；
  3. 看现在靶心在画面里的横坐标，和弹着点 ring.rings.aim_x_px 差多少（像素、厘米）。
最后存一张画面（绿点 = 找到的靶心，红竖线 = 弹着点）到 ~/race_logs/ringcheck_<时分秒>.jpg。

用法（需要先启动底层节点）：python2 ring_check.py [秒数]   默认 5 秒
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import os
import sys
import time

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_DIR, 'src'))

from shoot_race.compat import drop_python3_paths  # noqa: E402
drop_python3_paths()

import numpy as np  # noqa: E402
import rospy  # noqa: E402

from shoot_race.config import load_config  # noqa: E402
from shoot_race.ring_finder import RingFinder, effective_ring_params  # noqa: E402
from shoot_race.ros_io import RosIO  # noqa: E402


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('__')]
    seconds = float(args[0]) if args else 5.0
    rospy.init_node('shoot_race_ring_check', anonymous=True)
    cfg = load_config(os.path.join(PACKAGE_DIR, 'config'))
    ring_cfg = cfg['shooting']['ring']
    params = effective_ring_params(ring_cfg)
    finder = RingFinder((ring_cfg.get('rings') or {}).get('finder', {}))
    aim = params['aim_x_px']
    px_per_cm = cfg['system']['camera']['fx'] * params.get('image_gain', 1.0) / params['target_distance_m'] / 100.0
    ros = RosIO(cfg['system'])
    rospy.sleep(1.5)
    print('找靶心办法：%s（比赛用这个）。弹着点 u=%.1f，1 cm ≈ %.2f 像素（%.1f 米处）' % (
        ring_cfg.get('detector', 'find_object'), aim, px_per_cm, params['target_distance_m']))
    if ring_cfg.get('detector', 'find_object') != 'rings':
        print('注意：shooting.yaml ring.detector 不是 rings，比赛不会用找同心圆；这里照样测一下')
    results, costs, misses = [], [], 0
    last_stamp, hint = None, None
    end = rospy.get_time() + seconds
    image_kept = None
    while not rospy.is_shutdown() and rospy.get_time() < end:
        frames = ros.camera_frames((last_stamp + 1e-4) if last_stamp is not None else rospy.get_time() - 0.5)
        if not frames:
            rospy.sleep(0.05)
            continue
        stamp, image = frames[-1]
        last_stamp = stamp
        t0 = time.time()
        found = finder.find(image, hint)
        cost = time.time() - t0
        costs.append(cost)
        if found is None:
            misses += 1
            hint = None
            print('  没找到靶（%.0f ms）' % (cost * 1000))
            continue
        hint = (found['u'], found['v'])
        results.append(found)
        image_kept = (image, found)
        err = found['u'] - aim
        print('  靶心 (%.2f, %.2f) 得分 %.0f  %.0f ms   靶心在弹着点%s %.1f 像素（约 %.1f cm）' % (
            found['u'], found['v'], found['score'], cost * 1000, '右边' if err > 0 else '左边', abs(err),
            abs(err) / px_per_cm))
    if not costs:
        print('没收到相机画面（/usb_cam/image_raw），先启动底层节点')
        return 1
    print('')
    print('处理 %d 帧：找到 %d 帧，没找到 %d 帧；每帧平均 %.0f ms、最长 %.0f ms' % (
        len(costs), len(results), misses, 1000 * np.mean(costs), 1000 * np.max(costs)))
    if results:
        us = np.array([r['u'] for r in results])
        vs = np.array([r['v'] for r in results])
        scores = [r['score'] for r in results]
        print('靶心 u = %.2f（标准差 %.2f）  v = %.2f（标准差 %.2f）  得分 %.0f~%.0f（低于 %.1f 算没找到）' % (
            us.mean(), us.std(), vs.mean(), vs.std(), min(scores), max(scores), finder.min_score))
        err = us.mean() - aim
        shift_cm = err / px_per_cm
        print('比赛时车会向%s平移约 %.1f cm 再开火（靶心在弹着点%s %.1f 像素）' % (
            '右' if err > 0 else '左', abs(shift_cm), '右边' if err > 0 else '左边', abs(err)))
    if image_kept is not None:
        try:
            import cv2
            image, found = image_kept
            image = image.copy()
            u = int(round(aim))
            cv2.line(image, (u, 0), (u, image.shape[0] - 1), (0, 0, 255), 1)
            cv2.circle(image, (int(round(found['u'])), int(round(found['v']))), 2, (0, 255, 0), -1)
            cv2.putText(image, 'ring u=%.1f aim=%.1f' % (found['u'], aim), (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            path = os.path.expanduser('~/race_logs/ringcheck_%s.jpg' % time.strftime('%H%M%S'))
            cv2.imwrite(path, image)
            print('画面存在 %s' % path)
        except Exception as error:  # 存图失败不影响结果
            print('存图失败：%s' % error)
    return 0


if __name__ == '__main__':
    sys.exit(main())
