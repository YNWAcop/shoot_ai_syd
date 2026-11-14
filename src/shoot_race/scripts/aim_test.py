#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""弹道测试（菜单 22）：车先把目标对准弹着点，再连开几枪，方便看弹道散布。两种靶：

  环形靶（默认，2026-10-19 起）：移动靶是倒靶，打一枪就倒，看不了连发散布，改用环形靶。
    车放在 1 号框（离环形靶约 1.9 米）。靶心横坐标和比赛一样来自 ring.detector（rings = 相机画面找同心圆，
    find_object = 旧做法），用和比赛同一套 RingAimer 对准（多帧加权估计、只差一点先多看几帧再平移，见 aiming.RingAimer），
    对到弹着点 ± tolerance_px（detector 为 rings 时用 ring.rings 里的 aim_x_px、tolerance_px）；
    平移时用侧墙偏角稳住车头。打完可以输入每枪报靶几环、弹孔平均偏多少厘米，程序算出新的弹着点，确认后写入
    shooting_taught.yaml（rings：ring.rings.aim_x_px；find_object：ring.aim_x_px）。
  移动靶的码（6/7/8）：滑块停住，用 AR 码的像素位置对准 moving.aim_u_px（PixelAligner，按实测更新换算系数）。

流程：
  1. 看目标在画面里的横坐标，和弹着点比；
  2. 粗调：差超过 COARSE_PX 像素时原地转车头（转 1 度画面里动约 11.7 像素，不受平移距离限制），
     用侧墙偏角做反馈，转到差 COARSE_PX 以内；
  3. 细调：左右平移（激光侧墙测横向位置），停稳再看，直到对准；
  4. 对准后倒数 3 秒，按间隔连开 N 枪：每枪记下目标的像素位置、存一张画面（同菜单 17），
     打完输入每枪出了几颗。
  高低方向车没法调。

用法（需要底层节点在运行；菜单 22 会自动调用并录包）：
    python2 aim_test.py [ring 或 码号] [枪数] [间隔秒]      默认：ring 5 2.0
结果：~/race_logs/aimtest_<时分秒>/  shots.csv + shot<N>.jpg + info.txt（格式同菜单 17）
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PACKAGE_DIR, 'src'))
sys.path.insert(0, HERE)

from shoot_race.compat import drop_python3_paths  # noqa: E402
drop_python3_paths()

import rospy  # noqa: E402

import io  # noqa: E402

import yaml  # noqa: E402

from shoot_race.aiming import LateralShifter, PixelAligner, RingAimer  # noqa: E402
from shoot_race.compat import to_text  # noqa: E402
from shoot_race.config import SHOOTING_TAUGHT_FILE, deep_merge, load_config  # noqa: E402
from shoot_race.geometry import fit_wall  # noqa: E402
from shoot_race.motion import VelocityCommand  # noqa: E402
from shoot_race.ring_finder import RingSource, effective_ring_params  # noqa: E402
from shoot_race.ros_io import RosIO  # noqa: E402
from shoot_race.shooter import Shooter  # noqa: E402
from shoot_race.shotfile import ask_voltage, parse_rings, rings_summary  # noqa: E402
from shot_log import ask, fire_series, new_log_dir, record_beads  # noqa: E402

TOL_PX = 3.0            # 横向对准容差（像素，1.5 米处约 0.7 cm）。底盘最小一步约 1.5 cm（7 像素），再小做不到
COARSE_PX = 30.0        # 差超过这么多像素先转车头粗调（约 2.5 度）；再小的转动垫子上会转过头（最小一下约 3~4 度）
TURN_TOL_DEG = 0.3      # 粗调转车头的到位容差（度）
TURN_MIN_RATE = 0.25    # rad/s。垫子上原地转增益只有 0.15~0.3，太小转不动
TURN_MAX_RATE = 0.5
TURN_TIMEOUT_S = 6.0
TURN_COAST_S = 0.6      # 松开后还会接着转：约 转速 × 0.6 秒
MAX_TURNS = 4
MAX_ITER = 8            # 最多平移几次
FINE_STEP_M = 0.02      # 平移量小于这个用慢速
FINE_SPEED = 0.03       # m/s
MAX_TOTAL_M = 0.15      # 相对开始位置最多平移多远（测试用，不受任务框限制）
SETTLE_S = 1.5          # 每次平移后停稳多久再看（底盘死区约 0.35 秒 + 松油门后还滑，0.8 秒时还在动）
LOOK_S = 0.6            # 看码用最近多少秒的数据（取中位数）
MIN_SAMPLES = 5
SHIFT_TIMEOUT_S = 8.0
YAW_HOLD_GAIN = 1.5     # 平移时稳车头：角速度 = -增益 × 车头偏了多少（弧度）
YAW_HOLD_MAX = 0.2      # rad/s
RING_LOOK_FRAMES = 4    # 环形靶粗调时看几帧取中位数（find_object 约 2.6 帧/秒，单帧噪声约 3 像素）
RING_LOOK_TIMEOUT_S = 5.0
RING_ALIGN_TIMEOUT_S = 60.0
# 弹道测试要对得比比赛更准（比赛要省时间）：每轮多看几帧再定，估计误差从约 ±2.6 降到约 ±1.8 像素
RING_TEST_FRAMES = {'confirm_frames': 6, 'recheck_frames': 5, 'max_look_frames': 10}
STOP = VelocityCommand(0.0, 0.0, 0.0)


def side_fit(ros, cfg, side):
    """拟合一侧墙，返回 WallFit 或 None。"""
    points = ros.scan_points()
    if points is None:
        return None
    angles, xs, ys = points
    fit_cfg = cfg['system']['wall_fit']
    return fit_wall(angles, xs, ys, side,
                    sector_half_angle=math.radians(fit_cfg['sector_half_angle_deg']),
                    expected_distance=None,
                    distance_band=fit_cfg['distance_band'],
                    min_points=fit_cfg['min_points'],
                    outlier_threshold=fit_cfg['outlier_threshold'],
                    max_yaw_offset=math.radians(fit_cfg['max_yaw_offset_deg']))


def lateral_of(fit, side):
    """横向位置（米，左为正，原点任意）：左墙变近 = 往左；右墙变远 = 往左（同 steps.LateralTracker）。"""
    return -fit.distance if side == 'left' else fit.distance


def look(ros, marker_id):
    """目标码最近 LOOK_S 秒的像素位置中位数 (u, v)；数据太少返回 None。"""
    samples = ros.ar_pixels([marker_id], LOOK_S).get(marker_id, [])
    if len(samples) < MIN_SAMPLES:
        return None
    us = sorted(s[1] for s in samples)
    vs = sorted(s[2] for s in samples)
    return us[len(us) // 2], vs[len(vs) // 2]


def hold(ros, seconds):
    """发零速度并等 seconds 秒（车停稳）。"""
    end = rospy.get_time() + seconds
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and rospy.get_time() < end:
        ros.send_velocity(STOP)
        rate.sleep()


def shift_to(ros, cfg, side, shifter, target):
    """横移到目标横向位置（激光侧墙反馈），同时用这面墙的偏角稳住车头。返回 LateralShifter 的最终状态。"""
    first = side_fit(ros, cfg, side)
    yaw0 = first.yaw_offset if first is not None else None
    shifter.start(target)
    rate = rospy.Rate(20)
    t0 = rospy.get_time()
    status = 'moving'
    while not rospy.is_shutdown() and rospy.get_time() - t0 < SHIFT_TIMEOUT_S:
        fit = side_fit(ros, cfg, side)
        lateral = lateral_of(fit, side) if fit is not None else None
        vy, status, _ = shifter.update(rospy.get_time(), lateral)
        wz = yaw_hold_rate(fit, yaw0)
        ros.send_velocity(VelocityCommand(0.0, vy, wz))
        if status != 'moving':
            break
        rate.sleep()
    ros.send_velocity(STOP)
    return status


def turn_by(ros, cfg, side, left_rad):
    """原地转车头 left_rad（左为正），用侧墙偏角（WallFit.yaw_offset，车向左转为正）做反馈。返回实际转了多少（弧度）。

    松开后车还会接着转一段（底盘死区约 0.35 秒 + 惯性约 0.3 秒），所以按"当前转速 × TURN_COAST_S"提前停。
    """
    first = side_fit(ros, cfg, side)
    if first is None:
        return 0.0
    start = first.yaw_offset
    target = start + left_rad
    rate = rospy.Rate(20)
    t0 = rospy.get_time()
    history = [(t0, start)]
    while not rospy.is_shutdown() and rospy.get_time() - t0 < TURN_TIMEOUT_S:
        fit = side_fit(ros, cfg, side)
        now = rospy.get_time()
        if fit is not None:
            history.append((now, fit.yaw_offset))
        while len(history) > 2 and now - history[1][0] >= 0.3:
            history.pop(0)
        yaw_now = history[-1][1]
        span = history[-1][0] - history[0][0]
        speed = (history[-1][1] - history[0][1]) / span if span >= 0.15 else 0.0   # 时间太短算出来全是噪声
        error = target - yaw_now
        coast = speed * TURN_COAST_S                     # 现在松开还会再转多少
        if abs(error) <= math.radians(TURN_TOL_DEG) or (error - coast) * left_rad <= 0:
            break
        wz = math.copysign(max(TURN_MIN_RATE, min(TURN_MAX_RATE, 2.0 * abs(error))), error)
        ros.send_velocity(VelocityCommand(0.0, 0.0, wz))
        rate.sleep()
    hold(ros, SETTLE_S)                                  # 停稳后再读一次，算实际转了多少
    last = side_fit(ros, cfg, side)
    return (last.yaw_offset if last is not None else history[-1][1]) - start


def look_ring(ros, params, source, frames=RING_LOOK_FRAMES):
    """车停着时新看到的环形靶靶心，够 frames 帧后返回中位数 (u, None)；RING_LOOK_TIMEOUT_S 内看不够返回 None。"""
    t0 = rospy.get_time()
    while not rospy.is_shutdown() and rospy.get_time() - t0 < RING_LOOK_TIMEOUT_S:
        fresh = sorted(x for (t, x) in source.observations(ros, RING_LOOK_TIMEOUT_S + 1.0)
                       if t >= t0 + params['vision_latency_s'])
        if len(fresh) >= frames:
            return fresh[len(fresh) // 2], None
        hold(ros, 0.1)
    return None


def yaw_hold_rate(fit, yaw0):
    """平移时稳车头：车头相对 yaw0 偏了多少就反向转回来（rad/s）。"""
    if fit is None or yaw0 is None:
        return 0.0
    drift = fit.yaw_offset - yaw0                       # 正 = 车头偏左了
    if abs(drift) <= math.radians(0.15):
        return 0.0
    return max(-YAW_HOLD_MAX, min(YAW_HOLD_MAX, -YAW_HOLD_GAIN * drift))


def ring_align(ros, cfg, params, side, px_per_cm, source):
    """环形靶细调：和比赛同一套 RingAimer，平移上限放宽到 MAX_TOTAL_M、每轮多看几帧（RING_TEST_FRAMES），平移时稳车头。

    "停稳"按比赛的判断（轮速计低于 still_speed、连续 settle_s 秒），另外要求离上一次动车至少 SETTLE_S 秒。
    返回 (状态, info)：done（对准了）/ max_shift（平移到上限还差一点）/ timeout。
    """
    test_params = dict(params)
    if source is None or source.detector != 'rings':
        test_params.update(RING_TEST_FRAMES)            # 找同心圆噪声只有 0.3 像素，不用多看
    test_params['max_shift_m'] = MAX_TOTAL_M
    aimer = RingAimer(test_params, cfg['system']['lateral_shift'], cfg['system']['camera']['fx'])
    limits = cfg['system']['still_speed']
    rate = rospy.Rate(20)
    t0 = rospy.get_time()
    last_move = t0 - SETTLE_S                         # 粗调最后已经停稳过
    still_since, yaw0, last_line, info = None, None, None, {}
    start_lateral = [None]

    def finish(state):
        """返回前记下相对开始一共横移了多少（米，左为正）。"""
        fit = side_fit(ros, cfg, side)
        if fit is not None and start_lateral[0] is not None:
            info['total_shift_m'] = lateral_of(fit, side) - start_lateral[0]
        return state, info
    try:
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if now - t0 > RING_ALIGN_TIMEOUT_S:
                return finish('timeout')
            fit = side_fit(ros, cfg, side)
            lateral = lateral_of(fit, side) if fit is not None else None
            if start_lateral[0] is None and lateral is not None:
                start_lateral[0] = lateral
            wheel = ros.wheel_speed()
            wheel_still = wheel is None or (abs(wheel[0]) < limits['linear'] and abs(wheel[1]) < limits['linear']
                                            and abs(wheel[2]) < limits['angular'])
            if wheel_still and now - last_move >= SETTLE_S:
                still_since = now if still_since is None else still_since
            else:
                still_since = None
            still = still_since is not None and now - still_since >= params['settle_s']
            observations = source.observations(ros, params['max_obs_age_s'], process=still)
            vy, ready, info = aimer.update(now, lateral, observations, still)
            line = describe_ring(info, px_per_cm)
            if line and line != last_line:
                print(line)
                last_line = line
            if ready:
                return finish('max_shift' if info.get('reason') == 'max_shift_reached' else 'done')
            if aimer.state == 'shift':
                if yaw0 is None and fit is not None:
                    yaw0 = fit.yaw_offset                 # 平移开始时（车是停稳的）的车头
                wz = yaw_hold_rate(fit, yaw0)
            else:
                yaw0, wz = None, 0.0
            if vy != 0.0 or wz != 0.0:
                last_move = now
            ros.send_velocity(VelocityCommand(0.0, vy, wz))
            rate.sleep()
    finally:
        ros.send_velocity(STOP)
    return finish('timeout')


def describe_ring(info, px_per_cm):
    """RingAimer 的 info → 一行说明（只在算出偏差、或要平移时打印）。"""
    if 'err_px' not in info:
        return ''
    err = info['err_px']
    text = '  看了 %d 帧（合并 %d 轮）：靶心在弹着点%s %.1f 像素（约 %.1f cm，估计误差 ±%.1f 像素）' % (
        info['frames'], info['looks'], '右边' if err > 0 else '左边', abs(err), abs(err) / px_per_cm, info['sigma_px'])
    if 'reset_px' in info:
        text += '；和平移前对不上 %+.1f 像素，之前的不算' % info['reset_px']
    if 'shift_mm' in info:
        text += ' → 车向%s平移 %.0f mm' % ('左' if info['shift_mm'] > 0 else '右', abs(info['shift_mm']))
    elif info.get('reason') == 'look_more':
        text += ' → 只差一点，多看几帧'
    elif info.get('reason') == 'max_shift_reached':
        text += ' → 已平移到上限 %d cm' % int(MAX_TOTAL_M * 100)
    return text


def ring_aim_key(detector):
    """当前找靶心办法对应的弹着点配置项。"""
    return 'ring.rings.aim_x_px' if detector == 'rings' else 'ring.aim_x_px'


def save_ring_aim(config_dir, aim_x, detector='find_object'):
    """新的弹着点写进 shooting_taught.yaml（覆盖 shooting.yaml，更新程序时保留；同 aim_calib.save）。
    detector 为 rings 时写 ring.rings.aim_x_px，否则写 ring.aim_x_px（两种像素不通用）。"""
    path = os.path.join(config_dir, SHOOTING_TAUGHT_FILE)
    data = {}
    if os.path.exists(path):
        with open(path) as handle:
            data = yaml.safe_load(handle) or {}
    value = round(aim_x, 1)
    patch = {'ring': {'rings': {'aim_x_px': value}}} if detector == 'rings' else {'ring': {'aim_x_px': value}}
    data = deep_merge(data, patch)
    temp = path + '.tmp'
    with io.open(temp, 'w', encoding='utf-8') as handle:
        handle.write('# 由 aim_calib.py / aim_test.py 生成：弹着点标定，覆盖 shooting.yaml 的同名项。删掉某项恢复 shooting.yaml 的值。\n')
        handle.write(to_text(yaml.safe_dump(data, allow_unicode=True, default_flow_style=False)))
    os.rename(temp, path)
    print('已保存到 %s' % path)


def ring_report(shots, aim_x, px_per_cm, out_dir, config_dir, detector='find_object'):
    """环形靶打完：开火时靶心平均横坐标、每枪报靶几环、弹孔平均偏多少 → 建议的 aim_x_px。"""
    xs = [p['ring'][0] for p in shots if 'ring' in p]
    lines = []
    if xs:
        mean_x = sum(xs) / len(xs)
        lines.append('开火时靶心平均 x=%.1f（%d 枪看到），弹着点设定 %.1f：理论上弹孔偏%s %.1f cm' % (
            mean_x, len(xs), aim_x, '右' if aim_x > mean_x else '左', abs(aim_x - mean_x) / px_per_cm))
        print(lines[-1])
    for _ in range(3):
        try:
            rings = parse_rings(ask('每枪报靶几环？按顺序，空格隔开（0 = 脱靶），直接回车 = 不记录：'))
            break
        except ValueError as error:
            print('  %s，请重新输入' % error)
    else:
        rings = []
    if rings:
        lines.append('报靶：%s（%s）' % (' '.join(str(r) for r in rings), rings_summary(rings)))
        print(lines[-1])
    if xs:
        try:
            right = ask('弹孔整体在靶心右边几 cm（左为负；看不出来直接回车）：')
            right = float(right) if right else None
        except ValueError:
            print('  不是数字，不算建议值')
            right = None
        if right is not None:
            new_x = mean_x + right * px_per_cm
            lines.append('弹孔平均偏右 %+.1f cm → 建议 %s = %.1f + %+.1f × %.2f = %.1f（现在 %.1f）' % (
                right, ring_aim_key(detector), mean_x, right, px_per_cm, new_x, aim_x))
            print(lines[-1])
            if ask('写入 %s 的 %s？输入 y 确认：' % (SHOOTING_TAUGHT_FILE, ring_aim_key(detector))) == 'y':
                save_ring_aim(config_dir, new_x, detector)
    if lines:
        with io.open(os.path.join(out_dir, 'info.txt'), 'a', encoding='utf-8') as handle:
            handle.write(to_text('\n'.join(lines) + '\n'))


def parse_target(arg):
    """'ring' / 'r' / 空 → 'ring'；'6' → 6。其他抛 ValueError。"""
    text = (arg or 'ring').strip().lower()
    if text in ('ring', 'r', '0'):
        return 'ring'
    marker_id = int(text)
    if not 1 <= marker_id <= 8:
        raise ValueError('码号只能是 1~8')
    return marker_id


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('__')]
    try:
        target = parse_target(args[0] if args else '')
    except ValueError:
        print('第一个参数是 ring（环形靶）或码号 1~8，收到：%s' % args[0])
        return 1
    count = int(args[1]) if len(args) > 1 else 5
    interval = max(float(args[2]) if len(args) > 2 else 2.0, 1.5)   # 发射器两次之间至少要 1.2 秒恢复
    ring = target == 'ring'
    kind = 'ring' if ring else ('moving' if 6 <= target <= 8 else 'rotating')

    rospy.init_node('shoot_race_aim_test', anonymous=True)
    config_dir = os.path.join(PACKAGE_DIR, 'config')
    cfg = load_config(config_dir)
    params = cfg['shooting'][kind]
    fx = cfg['system']['camera']['fx']
    source = None
    detector = None
    if ring:
        detector = params.get('detector', 'find_object')
        source = RingSource(params)
        params = effective_ring_params(params)
        aim = (params['aim_x_px'], None)
        # 像素和弹孔：1 cm ≈ fx × image_gain / (100 × 距离) ≈ 2.47 像素（1.9 米，见 shooting.yaml ring）
        px_per_m = fx * params.get('image_gain', 1.0) / params['target_distance_m']
        tol_px = params['tolerance_px']
        name = '环形靶'
    else:
        aim = (params['aim_u_px'], params['aim_v_px'])
        px_per_m = fx / params.get('target_distance_m', 1.5)
        tol_px = TOL_PX
        name = '%d 号码' % target
    burst = params['fire']['burst_s']
    latency = params.get('latency_s', 0.15)
    ros = RosIO(cfg['system'])
    rospy.sleep(1.5)

    shooter_cfg = cfg['shooting']['shooter']
    shooter = Shooter(shooter_cfg['port'], shooter_cfg['baudrate'], shooter_cfg['start_hex'], shooter_cfg['stop_hex'])
    ok, text = shooter.open()
    print(text)
    if not ok:
        return 1
    print('目标：%s（%s 的参数）  弹着点像素 %s  发射时长 %.2f 秒  %d 枪、间隔 %.1f 秒' % (
        name, kind, ('x=%.1f' % aim[0]) if ring else '(%.1f, %.1f)' % aim, burst, count, interval))
    if ring:
        print('车放在 1 号框（离环形靶约 %.1f 米），车头大致对靶。' % params['target_distance_m'])
    voltage = ask_voltage(ask)
    if ask('车会先转车头 / 左右平移对准（平移最多 %d cm），然后连开 %d 枪。确认前方安全、录像已开始，输入 yes：' % (
            int(MAX_TOTAL_M * 100), count)) != 'yes':
        shooter.safe_stop()
        return 0

    fits = dict((side, side_fit(ros, cfg, side)) for side in ('left', 'right'))
    seen = [(fit.distance, side) for side, fit in fits.items() if fit is not None]
    if not seen:
        print('左右两侧都看不到墙，没法平移')
        shooter.safe_stop()
        return 1
    side = min(seen)[1]
    print('横向位置用 %s 墙测量' % side)

    def see():
        return look_ring(ros, params, source) if ring else look(ros, target)

    # ---- 粗调：转车头 ----
    for turn in range(MAX_TURNS):
        hold(ros, SETTLE_S)
        seen_uv = see()
        if seen_uv is None:
            break
        err = aim[0] - seen_uv[0]                 # 目标要在画面里右移多少像素 → 车头向左转
        if abs(err) <= COARSE_PX:
            break
        want = math.atan2(err, fx)
        print('  粗调第 %d 次：目标 u=%.1f，差 %+.1f 像素 → 车头向%s转 %.1f 度' % (
            turn + 1, seen_uv[0], err, '左' if want > 0 else '右', abs(math.degrees(want))))
        done = turn_by(ros, cfg, side, want)
        print('    实际转了 %+.1f 度' % math.degrees(done))

    # ---- 细调：平移 ----
    if ring:
        state, info = ring_align(ros, cfg, params, side, px_per_m / 100.0, source)
        err_px = -info['err_px'] if 'err_px' in info else None   # 统一成"目标要右移多少像素"
        if err_px is None:
            print('一直没看清环形靶（%s），停止' % ('找同心圆' if detector == 'rings'
                                                  else 'find_object 模板 %s' % params['accept_ids']))
            state = 'lost'
        else:
            print('对准结果：靶心在弹着点%s %.1f 像素（约 %.1f cm，估计误差 ±%.1f 像素）%s' % (
                '左边' if err_px > 0 else '右边', abs(err_px), abs(err_px) / px_per_m * 100.0, info['sigma_px'],
                '，对准了' if state == 'done' else '，没完全对准（%s）' % state))
        moved = info.get('total_shift_m')
        if moved is not None:
            race_max = params['max_shift_m']
            # 横移 Δ 和车头转 Δ/距离（弧度）对弹着点的作用一样；比赛时框里只能平移 race_max，多出来的要靠示教车头
            print('对准一共向%s平移 %.1f cm（比赛时最多 %.0f cm）%s' % (
                '左' if moved > 0 else '右', abs(moved) * 100.0, race_max * 100.0,
                '' if abs(moved) <= race_max else '：比赛时在框里打不到这个位置。如果开始时车停在 1 号点的示教位置，'
                '要把 1 号点车头向%s转约 %.1f 度（teach.py turn p1 %+.1f）' % (
                    '左' if moved > 0 else '右', math.degrees(abs(moved) / params['target_distance_m']),
                    math.degrees(moved / params['target_distance_m']))))
    else:
        state, err_px = align_marker(ros, cfg, target, aim, px_per_m, side)
    if state != 'done':
        if ask('没有对准到 ±%.0f 像素以内，仍然开火？输入 yes：' % tol_px) != 'yes':
            shooter.safe_stop()
            return 0

    extra = 'target_id: %s\nalign_state: %s\nalign_err_px: %s\n' % (
        target, state, '%.1f' % err_px if err_px is not None else '?')
    if ring and info.get('total_shift_m') is not None:
        extra += 'align_shift_m: %+.3f\n' % info['total_shift_m']
    out_dir, csv_path = new_log_dir('aimtest', kind, count, interval, burst, aim, voltage, extra)
    try:
        shots = fire_series(ros, shooter, kind, count, interval, aim, burst, latency, out_dir, csv_path, source)
    finally:
        shooter.safe_stop()
    record_beads(csv_path, out_dir)
    if ring:
        ring_report(shots, aim[0], px_per_m / 100.0, out_dir, config_dir, detector)
        print('记录在 %s' % out_dir)
    else:
        print('记录在 %s。看完弹道后用菜单 20 逐枪输入落点（相对 %d 号码中心），算平均落点和散布。' % (out_dir, target))
    return 0


def align_marker(ros, cfg, marker_id, aim, px_per_m, side):
    """AR 码细调（移动靶 6/7/8）：PixelAligner 反复"看码 → 平移 → 停稳再看"。返回 (状态, 目标还要右移多少像素)。"""
    aligner = PixelAligner(aim[0], px_per_m, TOL_PX, MAX_TOTAL_M, MAX_ITER, max_flips=4)
    shifter = LateralShifter(cfg['system']['lateral_shift'])
    # 小步（< FINE_STEP_M）用慢速平移：0.05 m/s 松油门后还滑约 11 mm，一步最少走 1.5 cm 左右；
    # 0.03 m/s 起步慢（约 0.7 秒，2026-10-16 实测）但滑得少，适合最后几毫米
    fine_params = dict(cfg['system']['lateral_shift'], max_speed=FINE_SPEED, min_speed=FINE_SPEED, stall_s=3.0)
    fine_shifter = LateralShifter(fine_params)
    state, seen_uv, info = None, None, {}
    looks = 0
    try:
        while not rospy.is_shutdown():
            hold(ros, SETTLE_S)
            looks += 1
            seen_uv = look(ros, marker_id)
            fit = side_fit(ros, cfg, side)
            if seen_uv is None or fit is None:
                print(('看不到 %d 号码，停止对准' % marker_id) if seen_uv is None else '看不到侧墙，停止对准')
                state = 'lost'
                break
            state, target, info = aligner.plan(seen_uv[0], lateral_of(fit, side))
            print('  第 %d 次看：码 (%.1f, %.1f)，横向还差 %+.1f 像素（%+.1f cm），换算 %.1f 像素/cm%s' % (
                looks, seen_uv[0], seen_uv[1], info['err_px'],
                info['err_px'] / px_per_m * 100.0, info['gain_px_per_cm'],
                '，平移 %+.0f mm' % info['shift_mm'] if state == 'shift' else ''))
            if state != 'shift':
                break
            small = abs(info.get('shift_mm', 0.0)) < FINE_STEP_M * 1000.0
            status = shift_to(ros, cfg, side, fine_shifter if small else shifter, target)
            if status not in ('done', 'stalled'):
                print('  平移结束：%s' % status)
    finally:
        ros.send_velocity(STOP)
    if seen_uv is None:
        return state, None
    err = aim[0] - seen_uv[0]
    high = seen_uv[1] - aim[1]
    print('对准结果：横向差 %+.1f 像素（%+.1f cm，%s）；弹着点比码中心%s %.1f 像素（约 %.1f cm），高低车调不了' % (
        err, err / px_per_m * 100.0, '对准了' if state == 'done' else '没完全对准',
        '高' if high > 0 else '低', abs(high), abs(high) / px_per_m * 100.0))
    return state, err


if __name__ == '__main__':
    sys.exit(main())
