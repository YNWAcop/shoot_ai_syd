#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""示教工具：把车摆到位，用传感器实测值代替手填数字。

结果写进 config/field_taught.yaml（会覆盖 field.yaml 里的同名项；field.yaml 本身不改，注释不会丢）。
运行前需要底盘、雷达、定位都已启动（bringup.launch）。

用法：
    rosrun shoot_race teach.py show              # 打印当前位姿和四面墙的测量值，不保存
    rosrun shoot_race teach.py watch             # 每秒刷新四面墙的测量值（核对激光方向用），Ctrl+C 退出
    rosrun shoot_race teach.py pose p1           # 记录当前位姿为路径点 p1
    rosrun shoot_race teach.py box p1            # 记录任务框 p1 的贴墙基准（墙距 + 朝向偏角），同时把当前位姿记为路径点 p1
    rosrun shoot_race teach.py box p1 right back yaw=right
                                                 # 同上，但指定用哪几面墙、朝向以哪面墙为准（写进 field_taught.yaml）。
                                                 # 2026-10-16 录包：1 号点右墙偏角最稳（每帧抖动 0.04 度），
                                                 # 后墙会被站在车后的人挡住，偏角跳 2~6 度
    rosrun shoot_race teach.py init start        # 告诉 AMCL 车现在在路径点 start（核对定位用），然后打印位姿
    rosrun shoot_race teach.py adopt [局数|记录目录]
                                                 # 把最近几局（默认 3 局）里各任务框"精定位完成时的位姿"取中位数，写成同名路径点，
                                                 # 下次行驶直接停到精定位位置附近（精定位只剩 1~2 cm）。不需要 ROS。
                                                 # 只用"那一局的任务框示教值和现在一样"的记录（box/turn/shift 改过之后，旧记录自动不用）
    rosrun shoot_race teach.py shift p1 2.5      # 把任务框 p1 的示教位置向左挪 2.5 cm（负数向右）。
                                                 # 环形靶按示教位置直接开火时，弹孔偏右 H cm 就 shift p1 H
    rosrun shoot_race teach.py turn p2 -1.5      # 车在任务框 p2 精定位后、以及开到路径点 p2 时，朝向多转 -1.5 度（负数 = 向右转），墙距不变。
                                                 # 用来改子弹落在靶的哪个位置：车向右转 1 度，画面里靶左移约 11.7 像素，不出框

推荐流程（场地布置好后，先 teach.py init start 让 AMCL 知道车在起点）：
    1. 任务框 p1 p2 p3 end：把车摆进框里正中、朝向调好，teach.py box <框名>
       （贴墙基准 + 同名路径点一次记下：行驶停到这里附近，精定位负责最后 1~2 cm）；
    2. 其他路径点（识别区 recog、通道点 lane_*）：把车摆到位，teach.py pose <名字>；
    3. 跑两三局演练后 teach.py adopt，用"精定位完成时的位姿"微调路径点。
       2026-10-18 22:19 录包：AMCL 坐标里各框真实位置和示教路径点差 4~22 cm、朝向差 1~6 度
       （识别区 13 cm、终点 22 cm），行驶停在"路径点"，剩下的全靠精定位慢慢挪；而两局之间同一个框只差约 2 cm，所以 adopt 很管用。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import io
import math
import os
import sys

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_DIR, 'src'))

from shoot_race.compat import drop_python3_paths  # noqa: E402
drop_python3_paths()   # 去掉 PYTHONPATH 里的 Python3 库目录，必须在 import yaml/rospy 之前

import yaml  # noqa: E402
import rospy  # noqa: E402

from shoot_race.compat import to_text  # noqa: E402
from shoot_race.config import TAUGHT_FILE, deep_merge, load_config, turn_update, waypoint  # noqa: E402
from shoot_race.geometry import fit_wall  # noqa: E402
from shoot_race.ros_io import RosIO  # noqa: E402


def measure_walls(ros, cfg, sides, expected=None, samples=10):
    """连续测 samples 次取平均，减小噪声。返回 {side: (距离, 偏角弧度)}。"""
    fit_cfg = cfg['system']['wall_fit']
    sums = {side: [0.0, 0.0, 0] for side in sides}
    rate = rospy.Rate(10)
    for _ in range(samples):
        points = ros.scan_points()
        if points is not None:
            angles, xs, ys = points
            for side in sides:
                fit = fit_wall(angles, xs, ys, side,
                               sector_half_angle=math.radians(fit_cfg['sector_half_angle_deg']),
                               expected_distance=(expected or {}).get(side),
                               distance_band=fit_cfg['distance_band'],
                               min_points=fit_cfg['min_points'],
                               outlier_threshold=fit_cfg['outlier_threshold'],
                               max_yaw_offset=math.radians(fit_cfg['max_yaw_offset_deg']))
                if fit is not None:
                    sums[side][0] += fit.distance
                    sums[side][1] += fit.yaw_offset
                    sums[side][2] += 1
        rate.sleep()
    return {side: (s[0] / s[2], s[1] / s[2]) for side, s in sums.items() if s[2] > 0}


def save_taught(config_dir, update):
    """把 update 合并进 field_taught.yaml。"""
    path = os.path.join(config_dir, TAUGHT_FILE)
    data = {}
    if os.path.exists(path):
        with open(path) as handle:
            data = yaml.safe_load(handle) or {}
    data = deep_merge(data, update)
    text = to_text(yaml.safe_dump(data, allow_unicode=True, default_flow_style=None))
    # 先写临时文件，成功后再替换，避免中途出错把已有的示教数据清空
    temp_path = path + '.tmp'
    with io.open(temp_path, 'w', encoding='utf-8') as handle:
        handle.write('# 由 teach.py 自动生成，覆盖 field.yaml 中的同名项。可以手动删除某一项恢复 field.yaml 的值。\n')
        handle.write(text)
    os.rename(temp_path, path)
    print('已保存到 %s' % path)


def print_nearest_waypoint(cfg, pose):
    """打印离当前位姿最近的路径点和偏差（推车到各点核对定位用）。"""
    best = None
    for name in cfg['field']['waypoints']:
        x, y, yaw = waypoint(cfg, name)
        distance = math.hypot(pose[0] - x, pose[1] - y)
        if best is None or distance < best[1]:
            best = (name, distance, x, y, yaw)
    name, distance, x, y, yaw = best
    dyaw = math.degrees(math.atan2(math.sin(pose[2] - yaw), math.cos(pose[2] - yaw)))
    print('最近的路径点 %s：差 %.1f cm（x %+.1f cm，y %+.1f cm），朝向差 %+.1f 度' % (
        name, 100 * distance, 100 * (pose[0] - x), 100 * (pose[1] - y), dyaw))


def shift_box(config_dir, name, left_cm):
    """把任务框的示教位置横向挪 left_cm 厘米（正数向左）：改侧墙的目标距离，其它不动。"""
    cfg = load_config(config_dir)
    box = cfg['field']['boxes'].get(name)
    if box is None:
        print('没有任务框 %s' % name)
        return
    walls = []
    changed = None
    for wall in box['walls']:
        wall = dict(wall)
        if wall['side'] in ('left', 'right') and changed is None:
            old = wall['distance']
            # 车向左挪：离左墙变近、离右墙变远
            wall['distance'] = round(old - left_cm / 100.0 if wall['side'] == 'left' else old + left_cm / 100.0, 4)
            changed = (wall['side'], old, wall['distance'])
        walls.append(wall)
    if changed is None:
        print('任务框 %s 没有左/右墙基准，没法横向挪' % name)
        return
    save_taught(config_dir, {'boxes': {name: {'walls': walls}}})
    print('任务框 %s：%s 墙目标距离 %.4f → %.4f m（车向%s挪 %.1f cm）' % (
        name, changed[0], changed[1], changed[2], '左' if left_cm >= 0 else '右', abs(left_cm)))


def turn_box(config_dir, name, left_deg):
    """把任务框精定位后的朝向、以及同名路径点的朝向都转 left_deg 度（正数向左转/逆时针，负数向右转），墙距不动。

    用途（2026-10-18）：改"子弹落在靶的哪个位置"。弹着点在画面里的位置由枪决定、不会变，车头转了，靶在画面里就平移：
    1.5 米外转 1 度，画面里约 11.7 像素（fx × tan 1°）。车向右转（负数）→ 靶在画面里左移。
    左右平移受任务框限制（框 38×32 cm，车宽 29 cm），改朝向不出框。具体算法见 config.turn_update。
    """
    cfg = load_config(config_dir)
    if name not in cfg['field']['boxes']:
        print('没有任务框 %s' % name)
        return
    update, notes = turn_update(cfg, name, left_deg)
    save_taught(config_dir, update)
    for note in notes:
        print(note)


def adopt_run(config_dir, arg=None, default_runs=3):
    """把最近几局里精定位正常完成（ok=1）时的位姿取中位数，写成同名路径点。

    arg：不写 = 最近 default_runs 局；数字 = 最近这么多局；目录 = 只用这一局。
    每个任务框只用"那一局的示教值（墙距、朝向基准）和现在一样"的记录：box / turn / shift 改过以后，
    旧记录里精定位停的是旧位置，不能用（靠每局记录目录里的 config_snapshot.yaml 判断）。
    """
    import glob
    cfg = load_config(config_dir)
    if arg is not None and not arg.isdigit():
        runs, wanted = [arg], 1
    else:
        log_dir = os.path.expanduser(cfg['system']['log_dir'])
        runs = sorted((d for d in glob.glob(os.path.join(log_dir, '2*_*')) if os.path.isdir(d)), reverse=True)
        wanted = int(arg) if arg else default_runs
    samples = {}             # 框名 → [(x, y, yaw), ...]
    sources = {}
    for run in runs:
        poses = read_align_poses(os.path.join(run, 'events.csv'))
        if not poses:
            continue
        snapshot = read_snapshot_boxes(run)
        for name, values in poses.items():
            if len(samples.get(name, [])) >= wanted:
                continue
            if snapshot is not None and not same_box(snapshot.get(name), cfg['field']['boxes'].get(name)):
                continue           # 那一局之后这个框重新示教过
            samples.setdefault(name, []).append(values)
            sources.setdefault(name, []).append(os.path.basename(run))
        if samples and all(len(v) >= wanted for v in samples.values()) and len(samples) >= len(cfg['field']['boxes']):
            break
    if not samples:
        print('没有找到精定位正常完成、且任务框示教值和现在一致的记录')
        return
    update = {}
    for name in sorted(samples):
        values = samples[name]
        median = [round(_median([v[i] for v in values]), 3 if i < 2 else 1) for i in range(3)]
        spread = max(100 * math.hypot(v[0] - median[0], v[1] - median[1]) for v in values)
        old = cfg['field']['waypoints'].get(name)
        moved = '' if old is None else '（挪了 %.1f cm，朝向 %+.1f 度）' % (
            100 * math.hypot(median[0] - old[0], median[1] - old[1]), median[2] - float(old[2]))
        print('路径点 %s：%s → %s%s  用了 %d 局，各局离中位数最远 %.1f cm%s' % (
            name, old, median, moved, len(values), spread,
            '  ← 各局差得多（AMCL 不稳），建议多跑一局再 adopt' if spread > 5.0 else ''))
        update[name] = median
    save_taught(config_dir, {'waypoints': update})
    print('来自 %s' % ', '.join(sorted(set(s for v in sources.values() for s in v))))


def _median(values):
    values = sorted(values)
    n = len(values)
    return values[n // 2] if n % 2 else 0.5 * (values[n // 2 - 1] + values[n // 2])


def read_snapshot_boxes(run):
    """那一局用的任务框配置（config_snapshot.yaml 里的 field.boxes）；没有快照返回 None（不检查）。"""
    path = os.path.join(run, 'config_snapshot.yaml')
    if not os.path.exists(path):
        return None
    try:
        with io.open(path, encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        return data.get('field', {}).get('boxes', {})
    except Exception:          # 快照读不了就不检查，照常使用
        return None


def same_box(a, b):
    """两份任务框配置的墙距、朝向基准是否一样（差 0.5 mm / 0.05 度以内）。"""
    if not a or not b:
        return False
    if a.get('yaw_side') != b.get('yaw_side') or abs(float(a.get('yaw_ref_deg', 0)) - float(b.get('yaw_ref_deg', 0))) > 0.05:
        return False
    walls_a = dict((w['side'], float(w['distance'])) for w in a.get('walls', []))
    walls_b = dict((w['side'], float(w['distance'])) for w in b.get('walls', []))
    return set(walls_a) == set(walls_b) and all(abs(walls_a[k] - walls_b[k]) < 0.0005 for k in walls_a)


def read_align_poses(path):
    """events.csv 里 align 步骤的 pose 事件：{框名: [x, y, 朝向度]}，只要 ok=1 的。"""
    poses = {}
    if not os.path.exists(path):
        return poses
    with io.open(path, encoding='utf-8') as handle:            # 按文本读，Python2 下中文也不会出错
        for line in handle:
            parts = line.rstrip('\n').split(',', 3)            # match_time,step,event,detail
            if len(parts) < 4 or parts[2] != 'pose' or 'box=' not in parts[3]:
                continue
            fields = dict(item.split('=', 1) for item in parts[3].split() if '=' in item)
            if fields.get('ok') != '1':
                continue
            poses[fields['box']] = [round(float(fields['x']), 3), round(float(fields['y']), 3),
                                    round(float(fields['yaw'].replace('deg', '')), 1)]
    return poses


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('show', 'watch', 'pose', 'box', 'init', 'shift', 'turn', 'adopt'):
        print(__doc__)
        return
    command = sys.argv[1]
    if command == 'adopt':
        adopt_run(os.path.join(PACKAGE_DIR, 'config'), sys.argv[2] if len(sys.argv) > 2 else None)
        return
    if command == 'shift':
        # 只改文件、不需要 ROS
        shift_box(os.path.join(PACKAGE_DIR, 'config'), sys.argv[2], float(sys.argv[3]))
        return
    if command == 'turn':
        turn_box(os.path.join(PACKAGE_DIR, 'config'), sys.argv[2], float(sys.argv[3]))
        return
    rospy.init_node('shoot_race_teach', anonymous=True)
    config_dir = os.path.join(PACKAGE_DIR, 'config')
    cfg = load_config(config_dir)
    ros = RosIO(cfg['system'])          # 和 ROS 交互的对象（变量名不用 io，避免和 io 模块重名）
    rospy.sleep(1.5)                     # 等数据进来

    if command == 'init':
        name = sys.argv[2] if len(sys.argv) > 2 else 'start'
        x, y, yaw = waypoint(cfg, name)
        for _ in range(3):                                  # 发几次，防止 AMCL 刚启动时漏收
            ros.set_initial_pose(x, y, yaw)
            rospy.sleep(0.4)
        print('已告诉 AMCL：车在 %s (%.2f, %.2f, %.0f 度)' % (name, x, y, math.degrees(yaw)))
        rospy.sleep(1.5)
        command = 'show'

    pose = ros.robot_pose()
    if command == 'show':
        if pose is not None:
            print('位姿：x=%.3f y=%.3f yaw=%.2f 度' % (pose[0], pose[1], math.degrees(pose[2])))
            print_nearest_waypoint(cfg, pose)
        else:
            print('位姿：拿不到 TF（AMCL 没起来？先 teach.py init start 告诉它车在起点）')
        for side, (distance, yaw) in sorted(measure_walls(ros, cfg, ['front', 'left', 'back', 'right']).items()):
            print('%-5s 墙距 %.3f m  偏角 %+.2f 度' % (side, distance, math.degrees(yaw)))
        return

    if command == 'watch':
        # 核对激光方向：在车的前/左/右/后放纸箱，对应方向的墙距应该变成纸箱的距离（从雷达中心量）
        print('每秒刷新一次四面墙的测量值（米 / 偏角度数），Ctrl+C 退出')
        sides = ['front', 'left', 'back', 'right']
        while not rospy.is_shutdown():
            walls = measure_walls(ros, cfg, sides, samples=3)
            parts = []
            for side in sides:
                if side in walls:
                    parts.append('%s %.3f %+.1f' % (side, walls[side][0], math.degrees(walls[side][1])))
                else:
                    parts.append('%s  ----' % side)
            print('   '.join(parts))
            rospy.sleep(0.7)
        return

    name = sys.argv[2]
    if command == 'pose':
        if pose is None:
            print('拿不到位姿，无法记录')
            return
        values = [round(pose[0], 3), round(pose[1], 3), round(math.degrees(pose[2]), 1)]
        print('路径点 %s = %s' % (name, values))
        save_taught(config_dir, {'waypoints': {name: values}})
        return

    # command == 'box'
    box = cfg['field']['boxes'].get(name)
    if box is None:
        print('field.yaml 里没有任务框 %s' % name)
        return
    extra = sys.argv[3:]
    sides = [a for a in extra if a in ('left', 'right', 'front', 'back')] or [w['side'] for w in box['walls']]
    yaw_side = box['yaw_side']
    for a in extra:
        if a.startswith('yaw='):
            yaw_side = a[4:]
    if yaw_side not in sides:
        print('朝向基准墙 %s 不在要用的墙 %s 里' % (yaw_side, sides))
        return
    # 示教时不用 field.yaml 里的旧墙距过滤（那些值可能来自旧场地），只靠扇区 + RANSAC 找墙
    measured = measure_walls(ros, cfg, sides)
    if set(measured) != set(sides):
        print('有墙没测到：需要 %s，测到 %s。检查车的位置，或用 teach.py show 看四面墙的实测值。' % (sides, list(measured)))
        return
    walls = [{'side': side, 'distance': round(measured[side][0], 4)} for side in sides]
    yaw_ref_deg = round(math.degrees(measured[yaw_side][1]), 2)
    wall_text = '，'.join('%s 墙 %.4f m' % (w['side'], w['distance']) for w in walls)
    print('任务框 %s：%s，朝向偏角 %.2f 度（以 %s 墙为准）' % (name, wall_text, yaw_ref_deg, yaw_side))
    update = {'boxes': {name: {'walls': walls, 'yaw_side': yaw_side, 'yaw_ref_deg': yaw_ref_deg}}}
    pose = ros.robot_pose()
    if pose is not None:
        values = [round(pose[0], 3), round(pose[1], 3), round(math.degrees(pose[2]), 1)]
        update['waypoints'] = {name: values}
        print('路径点 %s = %s' % (name, values))
    else:
        print('拿不到位姿（AMCL 没起来？），只记了任务框；路径点 %s 之后用 teach.py pose %s 补' % (name, name))
    save_taught(config_dir, update)


if __name__ == '__main__':
    main()
