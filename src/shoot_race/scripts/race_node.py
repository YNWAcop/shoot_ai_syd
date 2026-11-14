#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""比赛主程序入口。

启动顺序：
    1. 读取配置并检查；
    2. 创建各模块（RosIO、发射器、播报、识别器、日志）；
    3. 等传感器数据进来，做开局自检，打印结果；
    4. 设置 AMCL 初始位姿（起点框）；
    5. 等你在这个终端里输入 1 并回车 → 比赛开始；
    6. MissionRunner 按 mission.yaml 依次执行；
    7. 结束或按 Ctrl+C：停车、关发射器。

ROS 参数（在 race.launch 里设置，也可以命令行传）：
    ~config_dir   配置目录，默认本包的 config/
    ~from_step    从哪一步开始（步骤 id），单步测试用，例如 shoot_ring
    ~to_step      到哪一步结束（含）
    ~dry_fire     true：演练模式，不真正发射
    ~force        true：自检有关键项不通过也允许开始（只用于调试）
    ~auto_start   true：不等输入 1，直接开始（只用于调试）
    ~set          临时改参数，例如 "system.path_follower.max_speed=0.2; shooting.ring.mode=taught"（见 config.apply_overrides）
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import os
import select
import sys

# 让 Python 能找到本包 src/ 下的 shoot_race 模块（不需要 catkin_make）
PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_DIR, 'src'))

from shoot_race.compat import drop_python3_paths  # noqa: E402
drop_python3_paths()   # 去掉 PYTHONPATH 里的 Python3 库目录，必须在 import rospy/yaml 之前

import rospy  # noqa: E402

from shoot_race.config import apply_overrides, check_config, load_config, waypoint  # noqa: E402
from shoot_race.mission import Context, MissionRunner  # noqa: E402
from shoot_race.preflight import format_report, has_critical_failure, run_preflight  # noqa: E402
from shoot_race.recognition import CardMatcher, CardRecognizer, VlmClient  # noqa: E402
from shoot_race.ros_io import RosIO  # noqa: E402
from shoot_race.runlog import RunLog  # noqa: E402
from shoot_race.shooter import Shooter  # noqa: E402
from shoot_race.voice import VoicePlayer  # noqa: E402


def wait_for_start_key(log):
    """在终端等待输入 1 + 回车。返回 True 表示开始。

    roslaunch 启动的节点也能读到所在终端的输入（stdin 继承自终端）。
    用 select 每 0.2 秒检查一次，这样 Ctrl+C 能及时退出。
    """
    stream = sys.stdin
    if not stream.isatty():
        try:
            stream = open('/dev/tty')
        except (IOError, OSError):
            log.info('没有可用的终端输入，无法用键盘启动；请在终端里运行，或设置 ~auto_start')
            return False
    log.info('>>> 准备就绪：输入 1 并回车开始比赛 <<<')
    while not rospy.is_shutdown():
        readable, _, _ = select.select([stream], [], [], 0.2)
        if not readable:
            continue
        line = stream.readline()
        if line == '':
            return False
        if line.strip() == '1':
            return True
        log.info('输入的是 "%s"，请输入 1' % line.strip())
    return False


def build_recognizer(cfg, log):
    """创建识别器；离线模板或豆包任何一个创建失败都不影响另一个。"""
    rec = cfg['recognition']
    cards_dir = os.path.join(cfg['config_dir'], rec['offline']['cards_dir'])
    matcher = None
    try:
        matcher = CardMatcher(rec['cards'], cards_dir, rec['offline'])
    except Exception as error:
        log.info('离线卡片模板加载失败：%s' % error)
    vlm = VlmClient(rec['vlm'])
    return CardRecognizer(rec, cards_dir, matcher=matcher, vlm=vlm)


def main():
    rospy.init_node('shoot_race')
    config_dir = rospy.get_param('~config_dir', os.path.join(PACKAGE_DIR, 'config'))
    from_step = rospy.get_param('~from_step', '') or None
    to_step = rospy.get_param('~to_step', '') or None
    dry_fire = bool(rospy.get_param('~dry_fire', False))
    force = bool(rospy.get_param('~force', False))
    auto_start = bool(rospy.get_param('~auto_start', False))

    cfg = load_config(config_dir)
    overrides = rospy.get_param('~set', '') or ''
    try:
        changes = apply_overrides(cfg, overrides)
    except ValueError as error:
        rospy.logerr('%s' % error)
        return
    config_errors = check_config(cfg)
    log = RunLog(cfg['system']['log_dir'], printer=rospy.loginfo, clock=rospy.get_time)
    log.save_config(cfg)
    log.info('日志目录：%s' % log.folder)
    for path, old_value, new_value in changes:
        log.event('config', 'override', '%s: %s -> %s' % (path, old_value, new_value))

    io = RosIO(cfg['system'])
    shooter_cfg = cfg['shooting']['shooter']
    shooter = Shooter(shooter_cfg['port'], shooter_cfg['baudrate'],
                      shooter_cfg['start_hex'], shooter_cfg['stop_hex'])
    if not dry_fire:
        ok, message = shooter.open()
        log.info(message)
    voice = VoicePlayer(cfg['voice']['audio_dir'], cfg['voice']['player_command'])
    recognizer = build_recognizer(cfg, log)
    ctx = Context(cfg, io, shooter, voice, recognizer, log, dry_fire=dry_fire)
    runner = MissionRunner(ctx)
    try:
        runner.select_steps(from_step, to_step)        # 步骤 id 写错的话，现在就报出来
    except ValueError as error:
        log.info('%s' % error)
        return

    # 等传感器数据进来，再自检
    rospy.sleep(cfg['system']['preflight']['warmup_s'])
    results = run_preflight(ctx, config_errors)
    for line in format_report(results):
        log.info(line)
    if has_critical_failure(results) and not force:
        log.info('自检有关键项不通过，程序退出。修好后重新启动（调试时可加 force:=true 强制开始）。')
        return

    # 告诉 AMCL：车现在在起点框里。
    # 从中间某一步开始（单步练习）时车不在起点，不能这样设置，需要先在 rviz 里用"2D Pose Estimate"给定位置。
    if cfg['system']['set_initial_pose'] and not from_step:
        x, y, yaw = waypoint(cfg, 'start')
        io.set_initial_pose(x, y, yaw)
        log.info('已设置初始位姿：起点 (%.2f, %.2f)' % (x, y))
    elif from_step:
        log.info('从步骤 %s 开始：不重置定位，请确认 AMCL 位置正确' % from_step)

    if not auto_start and not wait_for_start_key(log):
        return

    try:
        runner.run(from_step=from_step, to_step=to_step)
    finally:
        ctx.stop_robot()
        shooter.safe_stop()
        voice.stop()
        log.info('程序结束，记录保存在 %s' % log.folder)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
