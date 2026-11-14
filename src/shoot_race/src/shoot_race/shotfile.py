# -*- coding: utf-8 -*-
"""开火测试记录的小工具（菜单 16 fire_test.py、菜单 17 shot_log.py、菜单 20 aim_calib.py 共用）。

为什么要记电压和出弹数（2026-10-18）：
  射击模块自带锂电池（比赛器材要求不低于 8.4V 4400mAh，模块上有电量表），和车体电池分开。
  发射器只有"开始/停止"两条指令，一次出几颗取决于电机在 burst_s 秒里转了多少——电压低了电机转得慢，
  同样的 burst_s 推不出弹。10-16 定 burst_s = 0.07 时 5 次都出弹，10-18 晚同样 0.07 只出 1~3 发，
  但当时没记电压，没法判断是不是电量问题。所以开测前记下电量表读数，打完记下每枪实际出了几颗。
  没出弹的枪在 aim_calib 标弹着点时自动跳过（那一枪根本没有落点）。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import io
import os

from shoot_race.compat import to_text

VOLTAGE_RANGE = (5.0, 9.5)     # 2 节锂电池：满电 8.4 V，低于 6 V 基本没电；超出这个范围多半是输错了
MAX_BEADS = 9                  # 一次开火最多记几颗（再多肯定是输错了）


def parse_voltage(text):
    """电量表读数 → 浮点数伏；空着返回 None；不是数字或不在合理范围抛 ValueError。"""
    text = to_text(text).strip().upper().rstrip('V').strip()
    if not text:
        return None
    try:
        value = float(text)
    except ValueError:
        raise ValueError('"%s" 不是数字' % text)
    if not VOLTAGE_RANGE[0] <= value <= VOLTAGE_RANGE[1]:
        raise ValueError('电压 %.2f 不在 %.1f~%.1f 伏之间' % (value, VOLTAGE_RANGE[0], VOLTAGE_RANGE[1]))
    return value


def ask_voltage(ask, tries=3):
    """在终端问电量表读数（ask = 打印提示并读一行的函数）。输错了重问，最多 tries 次；回车 = 不记录。"""
    for _ in range(tries):
        text = ask('射击模块电量表读数是多少伏？（看模块上的电量显示，如 8.1；直接回车 = 不记录）：')
        try:
            return parse_voltage(text)
        except ValueError as error:
            print('  %s，请重新输入' % error)
    return None


def format_voltage(value):
    return '' if value is None else '%.2f' % value


def parse_beads(text):
    """'1 1 0 2 1'（空格或逗号隔开）→ [1, 1, 0, 2, 1]；空着返回 []；有非法项抛 ValueError。"""
    items = to_text(text).replace(',', ' ').replace('，', ' ').split()
    try:
        beads = [int(item) for item in items]
    except ValueError:
        raise ValueError('只能输入整数，用空格隔开')
    if any(b < 0 or b > MAX_BEADS for b in beads):
        raise ValueError('每枪出弹数应在 0~%d 之间' % MAX_BEADS)
    return beads


def add_beads_column(csv_path, beads):
    """给 shot_log 的 shots.csv 末尾加一列 beads（每枪实际出了几颗）。

    beads 的个数必须和 csv 的数据行数相同，否则不改文件、抛 ValueError（防止错位对到别的枪上）。
    """
    with io.open(csv_path, encoding='utf-8') as handle:
        lines = [line.rstrip('\n') for line in handle if line.strip()]
    if not lines:
        raise ValueError('%s 是空的' % csv_path)
    header, rows = lines[0], lines[1:]
    if 'beads' in header.split(','):
        raise ValueError('%s 已经有 beads 列' % csv_path)
    if len(beads) != len(rows):
        raise ValueError('输入了 %d 个数，但记录里有 %d 枪' % (len(beads), len(rows)))
    out = [header + ',beads'] + ['%s,%d' % (row, b) for row, b in zip(rows, beads)]
    temp = csv_path + '.tmp'
    with io.open(temp, 'w', encoding='utf-8') as handle:
        handle.write(to_text('\n'.join(out) + '\n'))
    os.rename(temp, csv_path)


def beads_summary(beads):
    """[1, 0, 2, 1] → '出弹 3/4 枪（共 4 颗），空发 1 枪'。"""
    if not beads:
        return ''
    fired = sum(1 for b in beads if b > 0)
    empty = len(beads) - fired
    return '出弹 %d/%d 枪（共 %d 颗），空发 %d 枪' % (fired, len(beads), sum(beads), empty)


def parse_rings(text):
    """环形靶报靶：'8 10 0 9'（空格或逗号隔开，0 = 脱靶）→ [8, 10, 0, 9]；空着返回 []；不在 0~10 抛 ValueError。"""
    items = to_text(text).replace(',', ' ').replace('，', ' ').split()
    try:
        rings = [int(item) for item in items]
    except ValueError:
        raise ValueError('只能输入 0~10 的整数，用空格隔开')
    if any(r < 0 or r > 10 for r in rings):
        raise ValueError('环数应在 0~10 之间（0 = 脱靶）')
    return rings


def rings_summary(rings):
    """[8, 10, 0, 9] → '上靶 3/4 枪，上靶平均 9.0 环（共 27 环）'。"""
    if not rings:
        return ''
    hits = [r for r in rings if r > 0]
    text = '上靶 %d/%d 枪' % (len(hits), len(rings))
    if hits:
        text += '，上靶平均 %.1f 环（共 %d 环）' % (sum(hits) / len(hits), sum(hits))
    return text
