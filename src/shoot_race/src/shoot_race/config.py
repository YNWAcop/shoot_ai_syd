# -*- coding: utf-8 -*-
"""读取 config/ 目录下的配置文件，合并成一个字典，并做基本检查。

配置文件分工（每次调参只改其中一个）：
    system.yaml       话题名、串口、速度上限、主循环频率、日志目录等"系统"参数
    field.yaml        场地：路径点、路径、各任务框的贴墙基准（人工填写的初始值）
    field_taught.yaml 示教工具 teach.py 自动写入的实测值，会覆盖 field.yaml 中的同名项
    shooting_taught.yaml 弹着点标定 aim_calib.py 写入，会覆盖 shooting.yaml 中的同名项
    mission.yaml      比赛流程：步骤顺序、每步时间上限、超时后怎么办
    shooting.yaml     射击：各靶瞄准点、容差、延迟、开火参数
    recognition.yaml  识别：卡片答案表、水果对应的移动靶号、豆包接口
    voice.yaml        播报：每条播报对应的音频文件

合并后的结构：cfg['system'] / cfg['field'] / cfg['mission'] / ...
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import copy
import math
import os

import yaml

from .compat import to_text

CONFIG_FILES = ['system', 'field', 'mission', 'shooting', 'recognition', 'voice']
TAUGHT_FILE = 'field_taught.yaml'
SHOOTING_TAUGHT_FILE = 'shooting_taught.yaml'   # 弹着点标定结果（scripts/aim_calib.py 写入），覆盖 shooting.yaml 的同名项

# mission.yaml 中允许出现的步骤类型（具体含义见 steps.py）
STEP_TYPES = ['say', 'drive', 'align', 'recognize', 'shoot_ring', 'shoot_ar', 'pause']


def deep_merge(base, override):
    """把 override 合并进 base 的副本（字典递归合并，其它类型直接覆盖）。"""
    result = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _read_yaml(path):
    with open(path, 'r') as handle:
        data = yaml.safe_load(handle)
    return data or {}


def load_config(config_dir):
    """读取全部配置。缺少必需文件时直接抛出异常（宁可启动失败，也不带着错误配置上场）。"""
    cfg = {'config_dir': os.path.abspath(config_dir)}
    for name in CONFIG_FILES:
        cfg[name] = _read_yaml(os.path.join(config_dir, name + '.yaml'))
    taught_path = os.path.join(config_dir, TAUGHT_FILE)
    if os.path.exists(taught_path):
        cfg['field'] = deep_merge(cfg['field'], _read_yaml(taught_path))
    shooting_taught = os.path.join(config_dir, SHOOTING_TAUGHT_FILE)
    if os.path.exists(shooting_taught):
        cfg['shooting'] = deep_merge(cfg['shooting'], _read_yaml(shooting_taught))
    apply_surface(cfg)
    return cfg


def apply_surface(cfg):
    """按 system.surface 选场地地面：把 system.surfaces.<地面> 里的参数合并进 system 的同名项。

    比赛有两种地面材质，转向打滑差很多（2026-10-21：垫子上边走边转只转出指令的 0.36，硬地面 0.9），
    底盘增益、精定位最小转速要跟着换。返回用的地面名；没设 surface 或名字不存在时不改（check_config 报错）。"""
    system = cfg.get('system') or {}
    name = system.get('surface')
    surfaces = system.get('surfaces') or {}
    if not name or name not in surfaces:
        return None
    for section, values in (surfaces[name] or {}).items():
        if isinstance(values, dict):
            system[section] = deep_merge(system.get(section) or {}, values)
        else:
            system[section] = copy.deepcopy(values)
    return name


def apply_overrides(cfg, text):
    """临时改几个参数（场地测试用，不用改 yaml 文件）。

    text 形如 "system.path_follower.max_speed=0.2; shooting.ring.mode=taught"：
    用点号写出参数在合并后配置里的位置，等号后面按 yaml 解析（0.2 是数字，true 是布尔，[1,2] 是列表）。
    只允许改已经存在的参数，写错名字直接报错（防止以为改了其实没改）。
    返回 [(参数名, 旧值, 新值), ...]。
    """
    changes = []
    items = [item.strip() for item in to_text(text or '').replace('；', ';').split(';')]   # Python2 下 ROS 参数是字节串
    # 先换地面（system.surface），再改单项：同一个 set 里单独写的参数不会被地面的整组参数盖掉
    items = sorted([item for item in items if item], key=lambda item: item.split('=', 1)[0].strip() != 'system.surface')
    for item in items:
        if '=' not in item:
            raise ValueError('参数覆盖写法错误（缺少 =）：%s' % item)
        path, value_text = [part.strip() for part in item.split('=', 1)]
        keys = path.split('.')
        node = cfg
        for key in keys[:-1]:
            if not isinstance(node, dict) or key not in node:
                raise ValueError('参数覆盖：配置里没有 %s' % path)
            node = node[key]
        if not isinstance(node, dict) or keys[-1] not in node:
            raise ValueError('参数覆盖：配置里没有 %s' % path)
        value = yaml.safe_load(value_text)
        if path == 'system.surface' and value not in (cfg['system'].get('surfaces') or {}):
            raise ValueError('参数覆盖：system.surfaces 里没有地面 %s' % value)
        changes.append((path, node[keys[-1]], value))
        node[keys[-1]] = value
        if path == 'system.surface':
            apply_surface(cfg)
    return changes


def waypoint(cfg, name):
    """取一个路径点，返回 (x, y, yaw_rad)。field.yaml 里写的是 [x, y, 朝向(度)]。"""
    x, y, yaw_deg = cfg['field']['waypoints'][name]
    return float(x), float(y), math.radians(float(yaw_deg))


def turn_update(cfg, name, left_deg):
    """teach.py turn 用：任务框 name 精定位后的朝向、以及同名路径点的朝向，都多转 left_deg 度（正数向左/逆时针）。

    返回 (要合并进 field_taught.yaml 的内容, 说明文字列表)。没有这个任务框时抛 KeyError。
    精定位把 yaw_side 那面墙的偏角（WallFit.yaw_offset，车向左转为正）对到 yaw_ref_deg，所以加 left_deg；
    同名路径点的朝向也加 left_deg，车开到这里时就已经转好，精定位不用在垫子上原地转太多（原地转增益只有 0.15~0.3）。
    """
    box = cfg['field']['boxes'][name]
    old_ref = float(box.get('yaw_ref_deg', 0.0))
    new_ref = round(old_ref + left_deg, 2)
    update = {'boxes': {name: {'yaw_ref_deg': new_ref}}}
    notes = ['任务框 %s：朝向基准（%s 墙偏角）%.2f → %.2f 度' % (name, box.get('yaw_side', '?'), old_ref, new_ref)]
    point = cfg['field'].get('waypoints', {}).get(name)
    if point is not None:
        x, y, yaw_deg = point
        new_yaw = round(float(yaw_deg) + left_deg, 2)
        update['waypoints'] = {name: [float(x), float(y), new_yaw]}
        notes.append('路径点 %s：朝向 %.2f → %.2f 度' % (name, float(yaw_deg), new_yaw))
    notes.append('车停稳后多向%s转 %.2f 度' % ('左' if left_deg >= 0 else '右', abs(left_deg)))
    return update, notes


def path_item_name(item):
    """路径里的一项：直接写路径点名，或 {to: 路径点名, profile: 档位}（2026-10-20 加，见 path_segments）。"""
    return item['to'] if isinstance(item, dict) else item


def path_waypoints(cfg, path_name):
    """取一条路径上的全部路径点，返回 [(x, y, yaw_rad), ...]。

    路径项写成 {to: 名字, offset: [dx, dy]} 时，目标 = 这个路径点再挪 (dx, dy) 米（地图坐标）。
    用于"开到终点前面一点、剩下的交给精定位"：终点框贴着围栏角，teach.py adopt 只更新 end，前面那个点跟着变。"""
    result = []
    for item in cfg['field']['paths'][path_name]:
        x, y, yaw = waypoint(cfg, path_item_name(item))
        if isinstance(item, dict) and item.get('offset'):
            dx, dy = item['offset']
            x, y = x + float(dx), y + float(dy)
        result.append((x, y, yaw))
    return result


def path_segments(cfg, path_name):
    """路径上每一段（开到第 i 个路径点的那一段）用的运动参数，返回 [dict, ...]，和 path_waypoints 一一对应。

    不写 profile 的段 = 常规档：system.yaml path_follower 的 max_speed / decel / corner_speed、accel_limits.linear。
    写了 profile 的段：system.yaml path_follower.profiles.<档位> 里写的项覆盖常规档。
    每段的 dict：max_speed、accel、decel、end_speed（开到这一段终点时的速度，None = 按拐角角度自动定）、wheel_limit、
    settle（true = 到这一段终点先等车完全停稳再开下一段）、track（true = 贴着这一段的直线走，横向偏了优先纠偏）。
    """
    system = cfg['system']
    p = system['path_follower']
    base = {'max_speed': float(p['max_speed']), 'accel': float(system['accel_limits']['linear']),
            'decel': float(p['decel']), 'end_speed': None, 'wheel_limit': p.get('wheel_limit'), 'settle': False,
            'profile': 'normal'}
    use_profiles = p.get('profiles_enabled', True)          # false：路径里写的档位全部按常规档走（对比测试用）
    segments = []
    for item in cfg['field']['paths'][path_name]:
        segment = dict(base)
        name = item.get('profile') if isinstance(item, dict) and use_profiles else None
        if name:
            profile = p.get('profiles', {})[name]
            segment.update((k, v) for k, v in profile.items()
                           if k in ('max_speed', 'accel', 'decel', 'end_speed', 'wheel_limit', 'settle', 'settle_speed',
                                    'track'))
            segment['profile'] = name
            if 'wheel_limit' not in profile and segment['wheel_limit'] is not None:
                # 单轮速度上限要比这一段的最高速度高一点，否则直线到不了设定速度
                segment['wheel_limit'] = max(float(segment['wheel_limit']), float(segment['max_speed']) + 0.05)
        segments.append(segment)
    return segments


def _keeps_moving(cfg, step):
    """这一步等待时会不会定期动一下（目前只有旋转靶：keepalive 前后小幅挪动）。"""
    if step.get('type') != 'shoot_ar' or step.get('target') != 'rotating':
        return False
    params = cfg.get('shooting', {}).get('rotating', {})
    return params.get('keepalive_nudge_s', 0) > 0 and 0 < params.get('keepalive_still_s', 99) < 18


def _flat_keys(values, prefix=''):
    keys = set()
    for key, value in (values or {}).items():
        name = prefix + key
        if isinstance(value, dict):
            keys |= _flat_keys(value, name + '.')
        else:
            keys.add(name)
    return keys


def _check_surfaces(system):
    """场地地面：surface 要在 surfaces 里；每种地面写的参数名要一样（漏写一项，换地面时那项会留着上一种地面的值）。"""
    errors = []
    surfaces = system.get('surfaces')
    if not surfaces:
        return errors
    name = system.get('surface')
    if name not in surfaces:
        errors.append('system.yaml surface: %s 不在 surfaces 里（有：%s）' % (name, ', '.join(sorted(surfaces))))
    key_sets = dict((surface, _flat_keys(values)) for surface, values in surfaces.items())
    union = set().union(*key_sets.values())
    for surface, keys in sorted(key_sets.items()):
        missing = union - keys
        if missing:
            errors.append('system.yaml surfaces.%s 少了 %s（每种地面要写同一套参数）' % (surface, ', '.join(sorted(missing))))
    return errors


def check_config(cfg):
    """检查配置之间的引用关系。返回错误信息列表，空列表表示没问题。"""
    errors = []
    field = cfg['field']
    waypoints = field.get('waypoints', {})
    paths = field.get('paths', {})
    boxes = field.get('boxes', {})

    errors.extend(_check_surfaces(cfg['system']))
    profiles = cfg['system'].get('path_follower', {}).get('profiles', {})
    for path_name, items in paths.items():
        for item in items:
            name = path_item_name(item)
            if name not in waypoints:
                errors.append('路径 %s 用到了不存在的路径点 %s' % (path_name, name))
            if isinstance(item, dict) and item.get('profile') and item['profile'] not in profiles:
                errors.append('路径 %s 用到了不存在的运动档位 %s（system.yaml path_follower.profiles）' % (path_name, item['profile']))
            if isinstance(item, dict) and item.get('offset') is not None:
                offset = item['offset']
                if not isinstance(offset, (list, tuple)) or len(offset) != 2 or max(abs(float(v)) for v in offset) > 0.5:
                    errors.append('路径 %s 的 offset 要写成 [dx, dy]（米，每项不超过 0.5）' % path_name)

    # 示教坐标核对（2026-10-20 15:46：示教时定位没跟上，p1/p2/p3/识别区全记在起点旁边 20 cm 内，整局只在起点原地转）。
    # 图纸上起点框到 1 号框约 1 m，1/2/3 号框之间各隔 1.2 m
    start = waypoints.get('start')
    named = [n for n in ('recog', 'p1', 'p2', 'p3', 'end') if n in waypoints]
    if start is not None:
        for name in named:
            gap = math.hypot(float(waypoints[name][0]) - float(start[0]), float(waypoints[name][1]) - float(start[1]))
            if gap < 0.25:
                errors.append('路径点 %s 离起点只有 %.0f cm：多半是示教时定位没跟上（车被搬过去、AMCL 没跟着走），'
                              '用遥控把车开到位后重新示教，或跑一局后用菜单 12 采用精定位位置' % (name, gap * 100))
    for a, b in (('p1', 'p2'), ('p2', 'p3')):
        if a in waypoints and b in waypoints:
            gap = math.hypot(float(waypoints[a][0]) - float(waypoints[b][0]), float(waypoints[a][1]) - float(waypoints[b][1]))
            if gap < 0.6:
                errors.append('路径点 %s 和 %s 只隔 %.0f cm（图纸上两个任务框隔 1.2 m）：示教坐标不对，重新示教' % (a, b, gap * 100))

    lane_wall = field.get('lane_wall')
    if lane_wall:
        if lane_wall.get('side') not in ('back', 'front', 'left', 'right'):
            errors.append('field.yaml lane_wall.side 只能是 back / front / left / right')
        for name in lane_wall.get('waypoints', []):
            if name not in waypoints:
                errors.append('field.yaml lane_wall 用到了不存在的路径点 %s' % name)

    for box_name, box in boxes.items():
        sides = [w['side'] for w in box.get('walls', [])]
        if box.get('yaw_side') not in sides:
            errors.append('任务框 %s 的 yaw_side 必须是它 walls 里的一面墙' % box_name)

    step_ids = set()
    for step in cfg['mission'].get('steps', []):
        step_id = step.get('id')
        kind = step.get('type')
        if not step_id:
            errors.append('mission.yaml 有步骤缺少 id')
            continue
        if step_id in step_ids:
            errors.append('步骤 id 重复：%s' % step_id)
        step_ids.add(step_id)
        if kind not in STEP_TYPES:
            errors.append('步骤 %s 的类型 %s 不认识' % (step_id, kind))
        if kind == 'drive' and step.get('path') not in paths:
            errors.append('步骤 %s 用到了不存在的路径 %s' % (step_id, step.get('path')))
        if kind == 'align' and step.get('box') not in boxes:
            errors.append('步骤 %s 用到了不存在的任务框 %s' % (step_id, step.get('box')))
        if kind == 'say' and step.get('message') not in cfg['voice'].get('messages', {}):
            errors.append('步骤 %s 用到了不存在的播报 %s' % (step_id, step.get('message')))
        if kind == 'shoot_ar' and step.get('target') not in ('rotating', 'moving'):
            errors.append('步骤 %s 的 target 只能是 rotating 或 moving' % step_id)
        if 'timeout_s' not in step:
            errors.append('步骤 %s 没有 timeout_s（每一步都必须有时间上限）' % step_id)
        elif step['timeout_s'] >= 20 and not _keeps_moving(cfg, step):
            errors.append('步骤 %s 的 timeout_s 不能 ≥ 20 秒（规则：20 秒状态不变会被判结束）' % step_id)

    rotating = cfg.get('shooting', {}).get('rotating', {})
    if rotating and rotating.get('fire_side', 'any') not in ('any', 'left', 'right'):
        errors.append('shooting.yaml rotating.fire_side 只能是 any / left / right')
    if rotating and rotating.get('correction', 'shift') not in ('shift', 'twist', 'servo'):
        errors.append('shooting.yaml rotating.correction 只能是 servo / twist / shift')
    if rotating and rotating.get('correction') == 'servo':
        servo = rotating.get('servo') or {}
        if not 0.0 < rotating.get('max_twist_deg', 6.0) <= 12.0:
            errors.append('shooting.yaml rotating.max_twist_deg 应在 0~12 度（再大车身就出框了）')
        if not 0.5 <= servo.get('tolerance_px', 2.5) < rotating['hit_tolerance_px']:
            errors.append('shooting.yaml rotating.servo.tolerance_px 应在 0.5 ~ hit_tolerance_px 之间')
        if not 0.1 <= servo.get('yaw_min_rate', 0.25) <= servo.get('yaw_max_rate', 0.4) <= 0.8:
            errors.append('shooting.yaml rotating.servo 原地转速应 0.1 ≤ yaw_min_rate ≤ yaw_max_rate ≤ 0.8 rad/s')
        if not 0.01 <= servo.get('lateral_min_speed', 0.03) <= servo.get('lateral_max_speed', 0.05) <= 0.1:
            errors.append('shooting.yaml rotating.servo 横移速度应 0.01 ≤ lateral_min_speed ≤ lateral_max_speed ≤ 0.1 m/s')
        if servo.get('window_s', 6.0) > 20.0:
            errors.append('shooting.yaml rotating.servo.window_s 不能超过 20 秒（AR 缓存只存约 24 秒）')
    if rotating and rotating.get('correction', 'shift') == 'twist':
        if not cfg['system'].get('yaw_twist'):
            errors.append('rotating.correction 是 twist 时 system.yaml 要有 yaw_twist 一节')
        if not 0.0 < rotating.get('max_twist_deg', 6.0) <= 12.0:
            errors.append('shooting.yaml rotating.max_twist_deg 应在 0~12 度（再大车身就出框了）')
        if not 4.0 <= rotating.get('twist_px_per_deg', 8.8) <= 20.0:
            errors.append('shooting.yaml rotating.twist_px_per_deg 应在 4~20（理论约 8.8）')
    if rotating and rotating.get('history_s', 0) < rotating.get('fit_window_s', 0):
        errors.append('shooting.yaml rotating.history_s 必须 ≥ fit_window_s')
    if rotating and rotating.get('give_up_observe_s', 0) > rotating.get('fit_window_s', 0):
        errors.append('shooting.yaml rotating.give_up_observe_s 必须 ≤ fit_window_s（拟合最多只用这么长的数据）')

    ring = cfg.get('shooting', {}).get('ring', {})
    if ring:
        if ring.get('mode', 'vision') not in ('vision', 'taught', 'taught_check'):
            errors.append('shooting.yaml ring.mode 只能是 vision / taught / taught_check')
        if not 0.2 <= ring.get('image_gain', 1.0) <= 1.5:
            errors.append('shooting.yaml ring.image_gain 应在 0.2~1.5 之间（实测约 0.55）')
        if ring.get('look_more_px', 0) and ring['look_more_px'] < ring.get('tolerance_px', 0):
            errors.append('shooting.yaml ring.look_more_px 必须 ≥ tolerance_px')
        detector = ring.get('detector', 'find_object')
        if detector not in ('rings', 'find_object'):
            errors.append('shooting.yaml ring.detector 只能是 rings / find_object')
        if detector == 'find_object' and ring.get('max_obs_age_s', 0) < 0.8:
            errors.append('shooting.yaml ring.max_obs_age_s 太小：find_object 约 2.6 帧/秒，至少 0.8 秒')
        if detector == 'rings':
            rings = ring.get('rings') or {}
            if 'aim_x_px' not in rings:
                errors.append('shooting.yaml ring.detector 是 rings 时要写 ring.rings.aim_x_px'
                              '（找同心圆的像素和 find_object 的不通用，不能沿用 ring.aim_x_px）')
            effective = dict(ring)
            effective.update(dict((k, v) for k, v in rings.items() if k != 'finder'))
            if effective.get('look_more_px', 0) and effective['look_more_px'] < effective.get('tolerance_px', 0):
                errors.append('shooting.yaml ring.rings.look_more_px 必须 ≥ tolerance_px')
            finder = rings.get('finder') or {}
            for key in finder:
                if key not in ('search_u', 'search_v', 'r_min_px', 'r_max_px', 'track_px', 'track_keep_s',
                               'candidates', 'min_score', 'max_rate_hz'):
                    errors.append('shooting.yaml ring.rings.finder 里没有 %s 这一项' % key)

    moving = cfg.get('shooting', {}).get('moving', {})
    fuse = moving.get('fuse_codes') if moving else None
    if fuse:
        if not isinstance(fuse, list) or not all(isinstance(i, int) for i in fuse):
            errors.append('shooting.yaml moving.fuse_codes 要写成码号列表，比如 [6, 7, 8]（[] = 只用目标码）')
        if int(moving.get('fuse_min_pairs', 3)) < 2:
            errors.append('shooting.yaml moving.fuse_min_pairs 至少 2（偏移要靠几帧取中位数，太少会被认串的码带偏）')
    if moving and not 20.0 <= moving.get('max_speed_px_s', 40.0) <= 200.0:
        errors.append('shooting.yaml moving.max_speed_px_s 应在 20~200（今年滑块实测 14~18 像素/秒）')

    cards = cfg['recognition'].get('cards', [])
    fruit_map = cfg['recognition'].get('fruit_to_moving_id', {})
    for card in cards:
        if card.get('fruit') not in fruit_map:
            errors.append('卡片 %s 的水果 %s 没有在 fruit_to_moving_id 中给出靶号'
                          % (card.get('file'), card.get('fruit')))
    return errors
