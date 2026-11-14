#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""弹着点标定（旋转靶、移动靶）：用开火记录算出 aim_u_px / aim_v_px。

做法：
  1. 菜单 17（shot_log.py）连开几枪，同时手机录像（或先停掉转轮、在码周围贴纸，看水印）；
     shots.csv 里记着每枪弹丸到靶时刻各个码在画面里的像素位置。
  2. 本脚本逐枪问（菜单 17 打完记了每枪出弹数的话，没出弹的枪自动跳过）：打在几号码附近、弹孔在码中心右边几 cm、下边几 cm。
     弹着点像素 = 那个码的像素 + 偏移 × fx / (100 × 距离)（1.5 m 处 1 cm ≈ 4.5 像素）。
  3. 各枪取平均，确认后写进 config/shooting_taught.yaml（覆盖 shooting.yaml，更新程序时保留）。
  参考 RoboMaster 开源火控的实弹标定流程：每打一枪标一次弹着点，用实测落点修正瞄准偏置。

用法：python2 aim_calib.py [开火记录目录]      不写目录时用 ~/race_logs 下最新的 shotlog_* / aimtest_*
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import glob
import io
import math
import os
import sys

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_DIR, 'src'))

from shoot_race.compat import drop_python3_paths, to_text  # noqa: E402
drop_python3_paths()

import yaml  # noqa: E402

from shoot_race.config import SHOOTING_TAUGHT_FILE, deep_merge, load_config  # noqa: E402

try:
    read_line = raw_input        # Python2
except NameError:                # Python3
    read_line = input


def ask(prompt):
    print(prompt, end='')
    sys.stdout.flush()
    return to_text(read_line()).strip()


def read_shots(path):
    """shots.csv → [{'shot': 1, 'ids': {码号: (u, v)}}]，没看到的码不放进去。"""
    rows = []
    with io.open(path, encoding='utf-8') as handle:
        header = handle.readline().strip().split(',')
        for line in handle:
            values = line.strip().split(',')
            if len(values) != len(header):
                continue
            row = dict(zip(header, values))
            ids = {}
            for key in header:
                if key.startswith('id') and key.endswith('_u') and row[key]:
                    marker_id = int(key[2:-2])
                    ids[marker_id] = (float(row[key]), float(row['id%d_v' % marker_id]))
            beads = row.get('beads', '')
            rows.append({'shot': int(row['shot']), 'ids': ids,
                         'beads': int(beads) if beads.isdigit() else None})
    return rows


def is_ring_log(folder):
    """环形靶的开火记录（info.txt 里 target: ring）：没有码，本脚本用不了（环形靶在菜单 22 打完时直接标定）。"""
    try:
        with io.open(os.path.join(folder, 'info.txt'), encoding='utf-8') as handle:
            return 'target: ring' in handle.read()
    except (IOError, OSError):
        return False


def target_of(rows):
    ids = set()
    for row in rows:
        ids.update(row['ids'])
    return 'moving' if ids and min(ids) >= 6 else 'rotating'


def save(config_dir, target, aim_u, aim_v):
    path = os.path.join(config_dir, SHOOTING_TAUGHT_FILE)
    data = {}
    if os.path.exists(path):
        with open(path) as handle:
            data = yaml.safe_load(handle) or {}
    data = deep_merge(data, {target: {'aim_u_px': round(aim_u, 1), 'aim_v_px': round(aim_v, 1)}})
    temp = path + '.tmp'
    with io.open(temp, 'w', encoding='utf-8') as handle:
        handle.write('# 由 aim_calib.py 生成：弹着点标定，覆盖 shooting.yaml 的同名项。删掉某项恢复 shooting.yaml 的值。\n')
        handle.write(to_text(yaml.safe_dump(data, allow_unicode=True, default_flow_style=False)))
    os.rename(temp, path)
    print('已保存到 %s' % path)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('__')]
    if args:
        folder = os.path.expanduser(args[0])
    else:
        # 菜单 17 的 shotlog_* 和菜单 22 弹道测试的 aimtest_*，取最新的一个（按修改时间）
        folders = [d for pattern in ('shotlog_*', 'aimtest_*')
                   for d in glob.glob(os.path.expanduser('~/race_logs/' + pattern))
                   if os.path.isdir(d) and not is_ring_log(d)]
        folders.sort(key=os.path.getmtime)
        if not folders:
            print('没有开火记录（~/race_logs/shotlog_* 或 aimtest_*），先用菜单 17 或 22 打几枪')
            return 1
        folder = folders[-1]
    if is_ring_log(folder):
        print('%s 是环形靶的记录，这里只标旋转靶 / 移动靶（环形靶在菜单 22 打完时输入弹孔偏移）' % folder)
        return 1
    rows = read_shots(os.path.join(folder, 'shots.csv'))
    if not rows:
        print('%s 里没有开火记录' % folder)
        return 1
    config_dir = os.path.join(PACKAGE_DIR, 'config')
    cfg = load_config(config_dir)
    target = target_of(rows)
    params = cfg['shooting'][target]
    px_per_cm = cfg['system']['camera']['fx'] / (100.0 * params['target_distance_m'])
    print('开火记录 %s：%s，%d 枪。现在的弹着点 (%.1f, %.1f)，1 cm ≈ %.1f 像素' % (
        folder, target, len(rows), params['aim_u_px'], params['aim_v_px'], px_per_cm))
    print('每枪输入：打在几号码附近、弹孔在码中心右边几 cm（左为负）、下边几 cm（上为负）。没看清直接回车跳过。')
    points = []
    for row in rows:
        if row.get('beads') == 0:
            print('第 %d 枪没出弹（shots.csv 的 beads = 0），跳过' % row['shot'])
            continue
        seen = ' '.join('%d' % k for k in sorted(row['ids']))
        marker = ask('第 %d 枪（画面里有码 %s）：打在几号码附近：' % (row['shot'], seen or '无'))
        if not marker:
            continue
        try:
            marker_id = int(marker)
            right = float(ask('  弹孔在 %d 号码中心右边几 cm：' % marker_id) or 0)
            down = float(ask('  弹孔在 %d 号码中心下边几 cm：' % marker_id) or 0)
        except ValueError:
            print('  输入不是数字，跳过这一枪')
            continue
        if marker_id not in row['ids']:
            print('  这一枪开火时没看到 %d 号码，跳过' % marker_id)
            continue
        u, v = row['ids'][marker_id]
        points.append((u + right * px_per_cm, v + down * px_per_cm))
        print('  → 这一枪的弹着点像素 (%.1f, %.1f)' % points[-1])
    if not points:
        print('没有可用的记录')
        return 1
    mean_u = sum(p[0] for p in points) / len(points)
    mean_v = sum(p[1] for p in points) / len(points)
    spread = math.sqrt(sum((p[0] - mean_u) ** 2 + (p[1] - mean_v) ** 2 for p in points) / len(points))
    print('%d 枪平均弹着点 (%.1f, %.1f)，散布 %.1f 像素（%.1f cm）；现在的值 (%.1f, %.1f)' % (
        len(points), mean_u, mean_v, spread, spread / px_per_cm, params['aim_u_px'], params['aim_v_px']))
    if len(points) < 3:
        print('少于 3 枪，结果可能不稳，建议多打几枪再标')
    if ask('写入 %s 的 %s.aim_u_px / aim_v_px？输入 y 确认：' % (SHOOTING_TAUGHT_FILE, target)) == 'y':
        save(config_dir, target, mean_u, mean_v)
    return 0


if __name__ == '__main__':
    sys.exit(main())
