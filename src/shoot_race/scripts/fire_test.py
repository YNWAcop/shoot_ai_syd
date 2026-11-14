#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""发射时长测试：车不动、不瞄准，只按指定的时长开火，找"每次稳定出 1 颗"的 burst_s。

背景：发射器只有"开始""停止"两条串口指令（55 01 12… / 55 01 11…），没有"单发"指令。
程序发"开始"、等 burst_s 秒、再发"停止"，电机转多久决定出几颗。
2026-10-16 实测：0.03 秒 4 次只出了 2 次（另 2 次卡弹）；0.07 秒连发 2 次出弹太多。

用法（不需要启动 ROS，但不能同时运行比赛程序，串口会被占用）：
    python2 fire_test.py 0.05        # 用 0.05 秒测 5 次
    python2 fire_test.py 0.05 8      # 测 8 次
开测前输入射击模块电量表的读数（伏，回车 = 不记录）：射击模块自带电池，电量低了电机转得慢，
同样的时长推不出弹（2026-10-18 晚 0.07 秒 5 发只出 1~3 发），记下电压才知道定出来的时长对应什么电量。
每次按回车开火（输入 q 退出），开火后输入这次出了几颗（0 = 卡弹/没出），直接回车 = 不记录。
最后打印统计，并追加到 ~/race_logs/fire_test.csv（时间,时长,每次出弹数,电压）。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import os
import sys
import time

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_DIR, 'src'))

from shoot_race.compat import drop_python3_paths  # noqa: E402
drop_python3_paths()

from shoot_race.config import load_config  # noqa: E402
from shoot_race.shooter import Shooter  # noqa: E402
from shoot_race.shotfile import ask_voltage, format_voltage  # noqa: E402

try:
    read_line = raw_input        # Python2
except NameError:                # Python3（电脑上测试用）
    read_line = input

MIN_BURST, MAX_BURST = 0.01, 0.30   # 防止手误输入过大的时长导致连发一长串


def ask(prompt):
    """先用 print 输出中文提示（Python2 的 raw_input 直接带中文提示会报编码错误），再读一行。"""
    print(prompt, end='')
    sys.stdout.flush()
    return read_line().strip()


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('__')]
    if not args:
        print(__doc__)
        return 1
    burst = float(args[0])
    count = int(args[1]) if len(args) > 1 else 5
    if not MIN_BURST <= burst <= MAX_BURST:
        print('时长 %.3f 秒超出范围 %.2f~%.2f' % (burst, MIN_BURST, MAX_BURST))
        return 1

    cfg = load_config(os.path.join(PACKAGE_DIR, 'config'))['shooting']['shooter']
    shooter = Shooter(cfg['port'], cfg['baudrate'], cfg['start_hex'], cfg['stop_hex'])
    ok, text = shooter.open()
    print(text)
    if not ok:
        print('（比赛程序或旧的 shoot.launch 在运行时串口会被占用）')
        return 1
    voltage = ask_voltage(ask)
    if ask('会真的开火！确认前方安全、没有人，输入 yes 继续：') != 'yes':
        shooter.safe_stop()
        return 0

    results = []
    try:
        for index in range(1, count + 1):
            if ask('第 %d/%d 次（%.3f 秒）：按回车开火，输入 q 退出：' % (index, count, burst)).lower() == 'q':
                break
            sent = shooter.fire_burst(burst)
            if not sent:
                print('串口发送失败：%s' % shooter.last_error)
                continue
            time.sleep(0.3)
            answer = ask('  这次出了几颗？（0 = 卡弹或没出，直接回车 = 不记录）：')
            if answer.isdigit():
                results.append(int(answer))
    finally:
        shooter.safe_stop()

    if results:
        print('\n时长 %.3f 秒，电压 %s，记录 %d 次：' % (burst, format_voltage(voltage) or '未记录', len(results)))
        for beads in sorted(set(results)):
            print('  出 %d 颗：%d 次' % (beads, results.count(beads)))
        print('  平均每次 %.2f 颗；没出（卡弹）%d 次' % (sum(results) / float(len(results)), results.count(0)))
        log_dir = os.path.expanduser('~/race_logs')
        if not os.path.isdir(log_dir):
            os.makedirs(log_dir)
        with open(os.path.join(log_dir, 'fire_test.csv'), 'a') as handle:
            handle.write('%s,%.3f,%s,%s\n' % (time.strftime('%Y-%m-%d %H:%M:%S'), burst,
                                             ' '.join(str(r) for r in results), format_voltage(voltage)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
