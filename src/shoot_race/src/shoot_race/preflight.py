# -*- coding: utf-8 -*-
"""开局自检：开赛前逐项检查，有关键项不合格就不让开始（可以用 force 参数强制）。

旧程序启动时不检查任何东西，比如串口没打开只打一条警告，然后"空枪"跑完全程；
相机或识别节点没起来，要到对应环节才发现。这里在输入 1 之前就把问题全部列出来。
"""
from __future__ import absolute_import, division, print_function, unicode_literals



def find_executable(name):
    """在 PATH 里找可执行文件（Python3 用 shutil.which，Python2 用 distutils）。"""
    try:
        from shutil import which
    except ImportError:
        from distutils.spawn import find_executable as which
    return which(name)


class CheckResult(object):
    def __init__(self, name, ok, critical, detail):
        self.name = name            # 检查项名称
        self.ok = ok                # 是否通过
        self.critical = critical    # 不通过时是否禁止开赛
        self.detail = detail        # 说明


def run_preflight(ctx, config_errors):
    """返回 CheckResult 列表。"""
    cfg = ctx.cfg
    io = ctx.io
    results = []

    # 1. 配置文件
    results.append(CheckResult('配置文件', not config_errors, True,
                               '无错误' if not config_errors else '；'.join(config_errors)))

    # 2. 发射器串口
    if ctx.dry_fire:
        results.append(CheckResult('发射器串口', True, False, '演练模式，不使用串口'))
    else:
        results.append(CheckResult('发射器串口', ctx.shooter.is_ready(), True, ctx.shooter.port))

    # 3. 传感器话题是否在持续更新
    step_types = set(spec['type'] for spec in cfg['mission']['steps'])
    max_age = cfg['system']['preflight']['max_topic_age_s']
    needed = [('scan', '激光雷达', True), ('wheel_odom', '轮速里程计', True)]
    if 'recognize' in step_types:
        needed.append(('camera', '相机图像', True))
    if 'shoot_ar' in step_types:
        needed.append(('ar', 'AR码识别', True))
    if 'shoot_ring' in step_types:
        if cfg['shooting']['ring'].get('detector', 'find_object') == 'rings':
            if 'recognize' not in step_types:
                needed.append(('camera', '相机图像（环形靶找同心圆）', True))
        else:
            needed.append(('ring', '环形靶识别(find_object)', True))
    for key, label, critical in needed:
        age = io.topic_age(key)
        ok = age is not None and age <= max_age
        detail = '从未收到' if age is None else '%.1f 秒前' % age
        results.append(CheckResult(label, ok, critical, detail))

    # 4. 定位
    pose = io.robot_pose()
    results.append(CheckResult('AMCL 定位(TF map→base_link)', pose is not None, True,
                               '无' if pose is None else 'x=%.2f y=%.2f' % (pose[0], pose[1])))

    # 5. move_base（只有用到时才检查）
    uses_move_base = any(spec['type'] == 'drive' and
                         spec.get('mode', cfg['system']['drive_mode']) == 'move_base'
                         for spec in cfg['mission']['steps'])
    if uses_move_base:
        results.append(CheckResult('move_base', io.move_base_ready(3.0), True, ''))

    # 6. 播报音频和播放器
    all_files = []
    for files in cfg['voice']['messages'].values():
        for name in files:
            if '{rotating}' in name:
                all_files += [name.format(rotating=i, moving=0) for i in range(1, 6)]
            elif '{moving}' in name:
                all_files += [name.format(rotating=0, moving=i) for i in range(6, 9)]
            else:
                all_files.append(name)
    missing = ctx.voice.missing_files(all_files)
    results.append(CheckResult('播报音频', not missing, False,
                               '齐全' if not missing else '缺少：' + ', '.join(missing)))
    player = cfg['voice']['player_command'][0]
    results.append(CheckResult('播放器 ' + player, find_executable(player) is not None,
                               False, ''))

    # 7. 识别
    recognizer = ctx.recognizer
    results.append(CheckResult('离线卡片模板', recognizer.matcher is not None, False,
                               '已加载' if recognizer.matcher is not None else '未加载'))
    vlm_ok = recognizer.vlm is not None and recognizer.vlm.is_configured()
    results.append(CheckResult('豆包 API', vlm_ok, False,
                               '已配置' if vlm_ok else '没有 API Key 或 model，识别将只用离线方法'))
    return results


def format_report(results):
    """把检查结果排成几行文字。"""
    lines = ['========== 开局自检 ==========']
    for r in results:
        mark = '通过' if r.ok else ('不通过[关键]' if r.critical else '警告')
        lines.append('[%s] %s  %s' % (mark, r.name, r.detail))
    lines.append('==============================')
    return lines


def has_critical_failure(results):
    return any(r.critical and not r.ok for r in results)
