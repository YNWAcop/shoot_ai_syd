#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""开火记录：车不动、不瞄准，按固定间隔连开几枪，每一枪记下"开火时刻各个码在画面里的位置"，
并保存弹丸到靶时刻的相机画面（画出程序现在设定的弹着点十字和各码编号）。

用途：标定旋转靶 / 移动靶的弹着点（shooting.yaml 的 aim_u_px / aim_v_px）。
  配合手机高速录像看弹道：录像里看水弹落在哪个码旁边、偏多少厘米，
  再对照这里记下的"那一刻那个码在相机画面里的像素位置"，就能算出弹着点在相机画面里的真实位置。
  环形靶（ring）没有码，记开火时刻的靶心横坐标（ring_u 列）和画面。靶心来自 shooting.yaml ring.detector：
  rings = 相机画面找同心圆（默认），find_object = 旧做法。

用法（需要底层节点在运行；菜单 17 会自动调用并录包）：
    python2 shot_log.py [靶 rotating/moving/ring] [枪数] [间隔秒]
    默认：rotating 5 4
结果：~/race_logs/shotlog_<时分秒>/  shots.csv（每枪一行）+ shot<N>.jpg（每枪的画面）+ info.txt（参数和电压）
开火前问射击模块电量表读数，打完问每枪实际出了几颗（都可以直接回车跳过）：
  出弹数写进 shots.csv 的 beads 列，菜单 20 标弹着点时自动跳过没出弹的枪。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import io
import os
import sys
import time

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_DIR, 'src'))

from shoot_race.compat import drop_python3_paths, to_text  # noqa: E402
drop_python3_paths()

import rospy  # noqa: E402

from shoot_race.config import load_config  # noqa: E402
from shoot_race.ring_finder import RingSource, effective_ring_params  # noqa: E402
from shoot_race.ros_io import RosIO  # noqa: E402
from shoot_race.shooter import Shooter  # noqa: E402
from shoot_race.shotfile import add_beads_column, ask_voltage, beads_summary, format_voltage, parse_beads  # noqa: E402

try:
    read_line = raw_input        # Python2
except NameError:                # Python3
    read_line = input

MARKERS = {'rotating': [1, 2, 3, 4, 5], 'moving': [6, 7, 8], 'ring': []}


def ask(prompt):
    print(prompt, end='')
    sys.stdout.flush()
    return read_line().strip()


def nearest(samples, t):
    """samples [(t, u, v)] 里离 t 最近的一个；没有返回 None。"""
    if not samples:
        return None
    return min(samples, key=lambda s: abs(s[0] - t))


def save_frame(ros, path, t_hit, aim, positions):
    """保存最接近弹丸到靶时刻的相机画面，画出弹着点十字和各码位置。失败返回 False（不影响开火记录）。"""
    try:
        import cv2
        frames = ros.camera_frames(t_hit - 0.5)
        if not frames:
            return False
        stamp, image = min(frames, key=lambda f: abs(f[0] - t_hit))
        height = image.shape[0]
        if aim[0] is not None:
            u = int(round(aim[0]))
            if aim[1] is not None:
                v = int(round(aim[1]))
                cv2.line(image, (u - 15, v), (u + 15, v), (0, 0, 255), 1)
                cv2.line(image, (u, v - 15), (u, v + 15), (0, 0, 255), 1)
            else:                                       # 环形靶只标定了横坐标：画一条竖线
                cv2.line(image, (u, 0), (u, height - 1), (0, 0, 255), 1)
        for marker_id, (mu, mv) in positions.items():
            if mv is None:                              # 环形靶 find_object 只给横坐标：绿色虚竖线
                x = int(round(mu))
                for y in range(0, height, 12):
                    cv2.line(image, (x, y), (x, min(height - 1, y + 6)), (0, 255, 0), 1)
                cv2.putText(image, str(marker_id), (x + 4, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                continue
            cv2.circle(image, (int(round(mu)), int(round(mv))), 4, (0, 255, 0), 1)
            cv2.putText(image, str(marker_id), (int(round(mu)) + 6, int(round(mv)) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(image, 'dt=%+.3fs' % (stamp - t_hit), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.imwrite(path, image)
        return True
    except Exception as error:
        print('  （保存画面失败：%s）' % error)
        return False


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('__')]
    target = args[0] if args else 'rotating'
    count = int(args[1]) if len(args) > 1 else 5
    interval = float(args[2]) if len(args) > 2 else 4.0
    if target not in MARKERS:
        print('靶只能是 rotating / moving / ring')
        return 1
    interval = max(interval, 1.5)                  # 发射器两次之间至少要 1.2 秒恢复

    rospy.init_node('shoot_race_shot_log', anonymous=True)
    cfg = load_config(os.path.join(PACKAGE_DIR, 'config'))
    params = cfg['shooting'][target]
    if target == 'ring':
        params = effective_ring_params(params)
    aim = (params.get('aim_u_px', params.get('aim_x_px')), params.get('aim_v_px'))
    latency = params.get('latency_s', 0.15)
    burst = params['fire']['burst_s']
    ros = RosIO(cfg['system'])
    rospy.sleep(1.5)

    shooter_cfg = cfg['shooting']['shooter']
    shooter = Shooter(shooter_cfg['port'], shooter_cfg['baudrate'], shooter_cfg['start_hex'], shooter_cfg['stop_hex'])
    ok, text = shooter.open()
    print(text)
    if not ok:
        return 1
    print('靶：%s  枪数：%d  间隔：%.1f 秒  发射时长：%.2f 秒  设定弹着点像素：%s' % (target, count, interval, burst, aim))
    voltage = ask_voltage(ask)
    if ask('会真的开火！确认前方安全、录像已开始，输入 yes 继续：') != 'yes':
        shooter.safe_stop()
        return 0

    out_dir, csv_path = new_log_dir('shotlog', target, count, interval, burst, aim, voltage)
    ring_source = RingSource(cfg['shooting']['ring']) if target == 'ring' else None
    try:
        fire_series(ros, shooter, target, count, interval, aim, burst, latency, out_dir, csv_path, ring_source)
    finally:
        shooter.safe_stop()
    record_beads(csv_path, out_dir)
    print('记录在 %s' % out_dir)
    return 0


def new_log_dir(prefix, target, count, interval, burst, aim, voltage, extra=''):
    """建 ~/race_logs/<prefix>_<时分秒>/，写 info.txt 和 shots.csv 表头。返回 (目录, shots.csv 路径)。

    shots.csv 格式固定（菜单 20 aim_calib.py 按表头读），弹道测试 aim_test.py 也用它。
    """
    out_dir = os.path.expanduser('~/race_logs/%s_%s' % (prefix, time.strftime('%H%M%S')))
    os.makedirs(out_dir)
    csv_path = os.path.join(out_dir, 'shots.csv')
    with io.open(os.path.join(out_dir, 'info.txt'), 'w', encoding='utf-8') as handle:
        handle.write(to_text('target: %s\ncount: %d\ninterval_s: %.1f\nburst_s: %.3f\naim_px: %s\nvoltage_v: %s\nstart: %s\n%s' % (
            target, count, interval, burst, aim, format_voltage(voltage) or '未记录',
            time.strftime('%Y-%m-%d %H:%M:%S'), extra)))
    columns = ['id%d_u,id%d_v' % (k, k) for k in MARKERS[target]] if target != 'ring' else ['ring_u']
    with io.open(csv_path, 'w', encoding='utf-8') as handle:
        handle.write(to_text('shot,clock,ros_time,aim_u,aim_v,' + ','.join(columns) + '\n'))
    return out_dir, csv_path


def ring_at(ros, ring_source, t_cmd):
    """开火前后的环形靶靶心横坐标（中位数）；没看到返回 None。

    find_object 约 2.6 帧/秒、结果晚 0.1~0.4 秒，单帧噪声约 3 像素；找同心圆每次处理最新一帧（开火前、开火后各一次）。
    车开火时是静止的，所以取开火前 1.2 秒到开火后 0.45 秒之间的几帧的中位数。
    """
    xs = sorted(x for (t, x) in ring_source.observations(ros, 3.0) if t_cmd - 1.2 <= t <= t_cmd + 0.45)
    return xs[len(xs) // 2] if xs else None


def fire_series(ros, shooter, target, count, interval, aim, burst, latency, out_dir, csv_path, ring_source=None):
    """倒数 3 秒后按固定间隔开 count 枪；每枪记下弹丸到靶时刻各码的像素位置（环形靶：靶心横坐标），存一张画面。

    返回每枪的位置 [{码号或 'ring': (u, v)}]（环形靶 v 为 None），没看到的不放进去。
    """
    shots = []
    for left in (3, 2, 1):
        print('%d...' % left)
        rospy.sleep(1.0)
    for shot in range(1, count + 1):
        if rospy.is_shutdown():
            break
        if ring_source is not None:
            ring_source.observations(ros, 3.0)          # 开火前先看一帧（找同心圆只处理最新一帧）
        t_cmd = ros.now()
        clock = time.strftime('%H:%M:%S') + ('.%03d' % int((time.time() % 1) * 1000))
        sent = shooter.fire_burst(burst)
        t_hit = t_cmd + latency                         # 弹丸大约到靶的时刻
        rospy.sleep(0.4)                                # 等到靶时刻之后的 AR 帧进来
        pixels = ros.ar_pixels(MARKERS[target], 2.0)
        positions = {}
        for marker_id in MARKERS[target]:
            s = nearest(pixels.get(marker_id, []), t_hit)
            if s is not None and abs(s[0] - t_hit) < 0.2:
                positions[marker_id] = (s[1], s[2])
        row = [str(shot), clock, '%.3f' % t_cmd, '%s' % aim[0], '%s' % aim[1]]
        if target == 'ring':
            ring_u = ring_at(ros, ring_source, t_cmd) if ring_source is not None else None
            if ring_u is not None:
                positions['ring'] = (ring_u, None)
            row.append('%.1f' % ring_u if ring_u is not None else '')
        for marker_id in MARKERS[target]:
            if marker_id in positions:
                row += ['%.1f' % positions[marker_id][0], '%.1f' % positions[marker_id][1]]
            else:
                row += ['', '']
        shots.append(positions)
        with io.open(csv_path, 'a', encoding='utf-8') as handle:
            handle.write(to_text(','.join(row) + '\n'))
            handle.flush()
            os.fsync(handle.fileno())
        image_ok = save_frame(ros, os.path.join(out_dir, 'shot%d.jpg' % shot), t_hit, aim, positions)
        if target == 'ring':
            text = ('靶心 x=%.1f' % positions['ring'][0]) if 'ring' in positions else ''
            missing = '没看到环形靶'
        else:
            text = '  '.join('码%d (%.0f,%.0f)' % (k, u, v) for k, (u, v) in sorted(positions.items()))
            missing = '没看到码'
        print('第 %d 枪 %s %s：%s%s' % (shot, clock, '已发' if sent else '没发出去！', text or missing,
                                     '' if image_ok else '（没存到画面）'))
        rest = interval - (ros.now() - t_cmd)
        if shot < count and rest > 0:
            rospy.sleep(rest)
    return shots


def record_beads(csv_path, out_dir):
    """打完问每枪实际出了几颗，写进 shots.csv 的 beads 列。输错了重问，直接回车 = 不记录。"""
    for _ in range(3):
        try:
            text = ask('每枪实际出了几颗？按顺序输入，空格隔开（如 1 1 0 2 1，0 = 没出），直接回车 = 不记录：')
        except (EOFError, KeyboardInterrupt):
            return
        try:
            beads = parse_beads(text)
            if not beads:
                return
            add_beads_column(csv_path, beads)
        except (ValueError, IOError, OSError) as error:
            print('  %s，请重新输入' % error)
            continue
        summary = beads_summary(beads)
        print(summary)
        with io.open(os.path.join(out_dir, 'info.txt'), 'a', encoding='utf-8') as handle:
            handle.write(to_text('beads: %s\n%s\n' % (' '.join(str(b) for b in beads), summary)))
        return


if __name__ == '__main__':
    sys.exit(main())
