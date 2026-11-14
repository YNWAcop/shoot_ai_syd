# -*- coding: utf-8 -*-
"""比赛中的每一种步骤。

mission.yaml 里的每一项对应这里的一个类（type 字段 → 类）：
    say        SayStep        停车 → 播放一段播报
    drive      DriveStep      沿示教好的路径开过去（自写的路径跟随，或 move_base）
    align      AlignStep      贴墙精定位，把车停进任务框
    recognize  RecognizeStep  拍卡片 → 识别出两个靶号
    shoot_ring ShootRingStep  环形靶：视觉对准 → 停稳确认 → 开火
    shoot_ar   target: rotating → ShootRotatingStep  旋转靶：拟合转轮 → 平移对准交点 → 到时刻开火
               target: moving   → ShootMovingStep    移动靶：车不动，预测到达时刻 → 开火（避开换向）
    pause      PauseStep      原地等待（调试用）

每个步骤都有同样的四个方法，由 mission.MissionRunner 按顺序调用：
    start(ctx)       进入这一步时调用一次
    update(ctx)      每个控制周期调用一次，返回 RUNNING 或 DONE；
                     需要车动时，把想要的速度写到 ctx.command（不直接发布）
    on_timeout(ctx)  超过这一步的 timeout_s 时调用一次（比如"没瞄准也开火"）
    finish(ctx)      离开这一步时调用一次

ctx（Context，定义在 mission.py）里放着所有共享对象：配置、RosIO、发射器、播报、识别器、日志、
以及跨步骤共享的数据 ctx.targets（识别出的靶号）。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import collections
import math

from .aiming import (MovingInterceptor, MovingPredictor, RingAimer, RotatingPlanner, RotatingServo, YawTwister,
                     fuse_slider_codes)
from .config import path_item_name, path_segments, path_waypoints
from .geometry import SIDE_DIRECTION, WallFit, fit_wall, points_in_base_frame, wrap_angle
from .motion import STOP, PathFollower, VelocityCommand, WallAligner, hold_heading, obstacle_in_path
from .ring_finder import RingFinder, RingSource, effective_ring_params

RUNNING = 'running'
DONE = 'done'


class Step(object):
    """所有步骤的基类：保存 mission.yaml 里这一项的通用字段。"""

    def __init__(self, spec):
        self.spec = spec
        self.id = spec['id']                                # 英文 id，命令行指定起止步骤时用
        self.name = spec.get('name', self.id)               # 中文名，日志里显示
        self.timeout_s = float(spec['timeout_s'])           # 这一步最多用多少秒
        self.skip_if_less_than_s = spec.get('skip_if_less_than_s')  # 比赛剩余时间不足就跳过

    def start(self, ctx):
        pass

    def update(self, ctx):
        return DONE

    def on_timeout(self, ctx):
        pass

    def finish(self, ctx):
        ctx.command = STOP


# ------------------------------------------------------------
# 公共小工具
# ------------------------------------------------------------
def fire_shots(ctx, step_id, target_name, trigger, shot_cfg, detail=''):
    """开火 shot_cfg['shots'] 次，每次之间隔 interval_s 秒。

    先直接发零速度（绕过平滑器）确保车停着；整个过程是阻塞的，期间车静止，最多一两秒。
    dry_fire（演练模式）时只记录不发串口。
    """
    ctx.io.send_velocity(STOP)
    shots = int(shot_cfg['shots'])
    fired_at = []
    for index in range(shots):
        if index > 0:
            ctx.io.sleep(shot_cfg['interval_s'])
        fired_at.append(ctx.io.now())
        if ctx.dry_fire:
            sent = True
        else:
            sent = ctx.shooter.fire_burst(shot_cfg['burst_s'])
        ctx.log.shot(step_id, target_name, trigger,
                     '%s shot=%d/%d sent=%s' % (detail, index + 1, shots, sent))
    ctx.io.sleep(shot_cfg['post_fire_wait_s'])
    for index, t_fire in enumerate(fired_at):
        save_shot_photo(ctx, step_id, target_name, t_fire, index + 1)


SHOT_MARKERS = {'rotating': [1, 2, 3, 4, 5], 'moving': [6, 7, 8], 'ring': []}


def save_shot_photo(ctx, step_id, target_name, t_fire, index):
    """整局里每开一枪存一张"弹丸到靶时刻"的相机画面（2026-10-18：以前只有菜单 17 存图，整局打偏了看不到画面）。

    画出弹着点（红十字；环形靶只有横坐标，画竖线）和当时识别到的码（绿圈，目标码加粗）；
    环形靶另外在这张画面上找一次同心圆，画出靶心（绿点）并写上横坐标。
    文件：记录目录下 shot_<步骤>_<第几枪>.jpg。出任何错都只记一笔，不影响比赛流程。
    """
    try:
        import cv2
        params = ctx.cfg['shooting'][target_name]
        if target_name == 'ring':
            params = effective_ring_params(params)
        t_hit = t_fire + params.get('latency_s', 0.15)
        frames = ctx.io.camera_frames(t_fire - 0.3)
        if not frames:
            ctx.log.event(step_id, 'photo', '没有相机画面')
            return
        stamp, image = min(frames, key=lambda f: abs(f[0] - t_hit))
        image = image.copy()
        aim_u = params.get('aim_u_px', params.get('aim_x_px'))
        aim_v = params.get('aim_v_px')
        if aim_u is not None:
            u = int(round(aim_u))
            if aim_v is not None:
                v = int(round(aim_v))
                cv2.line(image, (u - 15, v), (u + 15, v), (0, 0, 255), 1)
                cv2.line(image, (u, v - 15), (u, v + 15), (0, 0, 255), 1)
            else:
                cv2.line(image, (u, 0), (u, image.shape[0] - 1), (0, 0, 255), 1)
        if target_name == 'ring':
            rings = ctx.cfg['shooting']['ring'].get('rings') or {}
            found = RingFinder(rings.get('finder', {})).find(image)
            if found is not None:
                center = (int(round(found['u'])), int(round(found['v'])))
                cv2.circle(image, center, 2, (0, 255, 0), -1)
                cv2.putText(image, 'ring u=%.1f' % found['u'], (center[0] + 8, center[1] - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        target_id = getattr(ctx, 'targets', {}).get(target_name)
        ids = SHOT_MARKERS.get(target_name, [])
        pixels = ctx.io.ar_pixels(ids, 1.0) if ids else {}
        for marker_id, samples in pixels.items():
            if not samples:
                continue
            t, mu, mv = min(samples, key=lambda s: abs(s[0] - stamp))
            if abs(t - stamp) > 0.2:
                continue
            thick = 2 if marker_id == target_id else 1
            cv2.circle(image, (int(round(mu)), int(round(mv))), 5, (0, 255, 0), thick)
            cv2.putText(image, str(marker_id), (int(round(mu)) + 6, int(round(mv)) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), thick)
        cv2.putText(image, '%s #%d dt=%+.2fs target=%s' % (step_id, index, stamp - t_hit, target_id),
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        ctx.log.save_image('shot_%s_%d.jpg' % (step_id, index), image)
    except Exception as error:  # 存图失败不能影响比赛
        ctx.log.event(step_id, 'photo', '存图失败：%s' % error)


def stop_and_settle(ctx, settle_s):
    """立即停车（不经过平滑），再等 settle_s 秒让车身停稳。超时兜底开火前调用。"""
    ctx.stop_robot()
    ctx.io.sleep(settle_s)


def scan_wall_fits(ctx, wall_refs):
    """对 wall_refs 里每面墙做一次拟合，返回 {side: WallFit 或 None}。"""
    points = ctx.io.scan_points()
    fits = {}
    if points is None:
        return fits
    angles, xs, ys = points
    fit_cfg = ctx.cfg['system']['wall_fit']
    for ref in wall_refs:
        fits[ref['side']] = fit_wall(
            angles, xs, ys, ref['side'],
            sector_half_angle=math.radians(fit_cfg['sector_half_angle_deg']),
            expected_distance=ref['distance'],
            distance_band=fit_cfg['distance_band'],
            min_points=fit_cfg['min_points'],
            outlier_threshold=fit_cfg['outlier_threshold'],
            max_yaw_offset=math.radians(fit_cfg['max_yaw_offset_deg']))
    return fits


class LateralTracker(object):
    """用激光侧墙距离测车的横向位置（米，左为正，原点任意，只用差值）。

    左墙距离变小 = 往左走；右墙距离变大 = 往左走。
    第一次调用时选离得近的那面侧墙，之后固定用它（中途换墙会让位置跳变）。拿不到返回 None。
    """

    def __init__(self, step_id):
        self.step_id = step_id
        self.side = None
        self.yaw = None           # 同一面墙最近一次拟合的偏角（弧度，车向左转为正），横移时稳车头用
        self.yaw0 = None          # 这次横移开始时的偏角
        self.recent_yaw = collections.deque(maxlen=6)   # 最近几次偏角（单次拟合有约 ±0.2 度噪声，取平均再用）

    def position(self, ctx):
        fits = scan_wall_fits(ctx, [{'side': 'left', 'distance': None}, {'side': 'right', 'distance': None}])
        if self.side is None:
            seen = [(fit.distance, side) for side, fit in fits.items() if fit is not None]
            if not seen:
                return None
            self.side = min(seen)[1]
            ctx.log.event(self.step_id, 'aim', '横向位置用 %s 墙测量' % self.side)
        fit = fits.get(self.side)
        if fit is None:
            self.yaw = None
            return None
        self.yaw = fit.yaw_offset
        self.recent_yaw.append(fit.yaw_offset)
        return -fit.distance if self.side == 'left' else fit.distance

    def hold_rate(self, shifting, enabled=True):
        """横移时稳车头的角速度：刚开始横移时（车是停稳的）记下车头，横移过程中偏了多少就转回来；不横移时返回 0。"""
        if not shifting or not enabled or not self.recent_yaw:
            self.yaw0 = None
            return 0.0
        yaw = sum(self.recent_yaw) / len(self.recent_yaw)
        if self.yaw0 is None:
            self.yaw0 = yaw                      # 车刚开始横移、还是停稳的，最近几次的平均就是起始车头
            return 0.0
        return hold_heading(yaw - self.yaw0, deadband_rad=math.radians(0.3))


def wall_heading(ctx, tracker, max_deg=25.0):
    """车头相对侧墙的朝向（弧度，左转为正），激光侧墙拟合；拿不到或大得不像话（多半拟合错墙）返回 None。"""
    if tracker.position(ctx) is None or tracker.yaw is None:
        return None
    return tracker.yaw if abs(tracker.yaw) <= math.radians(max_deg) else None


class TransitionLogger(object):
    """瞄准过程的状态/原因变了才记一笔（调参用），避免每个周期都写日志。"""

    def __init__(self, step_id):
        self.step_id = step_id
        self.last = None

    def update(self, ctx, info):
        key = (info.get('state'), info.get('reason'))
        if key != self.last:
            ctx.log.event(self.step_id, 'aim', format_info(info))
            self.last = key


# ------------------------------------------------------------
# say：播报
# ------------------------------------------------------------
class SayStep(Step):
    """先停车，确认车停稳后播放音频（规则：播报期间车必须静止）。

    voice.yaml 里音频文件名可以带 {rotating} {moving} 占位符，播放前用识别结果替换，
    例如 'x{rotating}.mp3' → 'x3.mp3'。
    """

    def start(self, ctx):
        self.message = self.spec['message']
        self.playing = False
        ctx.command = STOP

    def update(self, ctx):
        if not self.playing:
            if not ctx.robot_still_for(ctx.cfg['system']['still_hold_s']):
                return RUNNING
            files = [name.format(**ctx.targets) for name in ctx.cfg['voice']['messages'][self.message]]
            ctx.log.event(self.id, 'play', ' '.join(files))
            ctx.voice.start(files)
            self.playing = True
            return RUNNING
        return RUNNING if ctx.voice.is_playing() else DONE

    def on_timeout(self, ctx):
        ctx.voice.stop()


# ------------------------------------------------------------
# drive：行驶
# ------------------------------------------------------------
class DriveStep(Step):
    """沿 field.yaml 里的一条路径行驶。

    mode = direct（默认）：本程序自己的路径跟随器（motion.PathFollower）。
        麦轮可以边走边转、侧向平移，按底盘实测响应预测停车点、拐角走圆弧，比 move_base 规划 + DWA 快得多。
        带激光安全检查：运动方向前方有障碍就停车等待。
    mode = move_base：依次把路径点发给 move_base（旧程序的方式，作为备用）。
    """

    def start(self, ctx):
        system = ctx.cfg['system']
        self.mode = self.spec.get('mode', system['drive_mode'])
        self.waypoints = path_waypoints(ctx.cfg, self.spec['path'])
        self.blocked_logged = False
        if self.mode == 'direct':
            p = system['path_follower']
            speed_scale = float(self.spec.get('speed_scale', 1.0))   # 某一段想开慢一点时用
            segments = path_segments(ctx.cfg, self.spec['path'])
            for segment in segments:
                segment['max_speed'] *= speed_scale
            self.default_accel = ctx.smoother.linear_accel
            self.follower = PathFollower(
                self.waypoints,
                max_speed=p['max_speed'] * speed_scale,
                corner_speed=p['corner_speed'],
                decel=p['decel'],
                max_yaw_rate=p['max_yaw_rate'],
                yaw_gain=p['yaw_gain'],
                goal_tolerance=p['goal_tolerance'],
                yaw_tolerance=math.radians(p['yaw_tolerance_deg']),
                lookahead_m=p['lookahead_m'],
                lookahead_s=p['lookahead_s'],
                wheel_limit=p['wheel_limit'],
                wheel_k=p['wheel_k'],
                min_turn_rate=p['min_turn_rate'],
                close_finish_m=p['close_finish_m'],
                sharp_corner_deg=p['sharp_corner_deg'],
                corner_reach_m=p['corner_reach_m'],
                turn_hold_m=p['turn_hold_m'],
                turn_release_m=p['turn_release_m'],
                handoff_yaw=math.radians(p['handoff_yaw_deg']) if p.get('handoff_yaw_deg') is not None else None,
                segments=segments,
                cross_gain=float(p.get('cross_gain', 2.5)),
                cross_max=float(p.get('cross_max', 0.12)),
                burst_switch_m=float(p.get('burst_switch_m', 0.015)),
                yaw_sync=float(p.get('yaw_sync', 1.0)),
                yaw_bandwidth=float(p['yaw_bandwidth']) if p.get('yaw_bandwidth') else None)
            self.logged_profile = None
            self._setup_lane_wall(ctx)
        else:
            ctx.external_control = True        # move_base 控制底盘时，本程序不发速度
            self.goal_index = 0
            if not ctx.io.move_base_ready(2.0):
                ctx.log.event(self.id, 'error', 'move_base 没有就绪')
            else:
                ctx.io.move_base_go(*self.waypoints[0])

    def update(self, ctx):
        if self.mode == 'direct':
            return self._update_direct(ctx)
        return self._update_move_base(ctx)

    def _setup_lane_wall(self, ctx):
        """通道点按激光量墙定位（field.yaml lane_wall）：找出这条路径里哪几个点是通道点。

        2026-10-20 16:19 录包：同一张地图，2 号挡板端头在 AMCL 坐标里比 10-18 那局近了 3~4 cm，
        通道点 x 按 AMCL 定死时，车头离挡板端头最近只有 0.6 cm。后围栏是一整面直墙，横移全程都在车后面，
        按"激光到后围栏的距离 = lane_wall.distance"定通道的前后位置，和 AMCL 偏多少无关。"""
        self.lane = None
        cfg = ctx.cfg['field'].get('lane_wall')
        if not cfg or not cfg.get('enabled', True):
            return
        names = [path_item_name(item) for item in ctx.cfg['field']['paths'][self.spec['path']]]
        indices = [i for i, name in enumerate(names) if name in cfg.get('waypoints', [])]
        if not indices:
            return
        self.lane = {'side': cfg.get('side', 'back'), 'distance': float(cfg['distance']), 'indices': indices,
                     'base': dict((i, self.waypoints[i]) for i in indices),
                     'max_shift': float(cfg.get('max_shift', 0.15)), 'smooth': float(cfg.get('smooth', 0.4)),
                     'shift': None, 'logged': False}

    def _update_lane_wall(self, ctx, pose):
        """每个周期量一次墙，把这条路径上的通道点挪到"激光到墙 = 目标距离"的位置（只挪垂直于墙的那个坐标）。
        车开过最后一个通道点之后（往任务框里开）就不再挪。"""
        lane = self.lane
        if lane is None or self.follower.segment > max(lane['indices']):
            return
        points = ctx.io.scan_points()
        if points is None:
            return
        x, y, yaw = pose
        side = lane['side']
        dir_x, dir_y = SIDE_DIRECTION[side]
        # 墙的方向在地图里（车头朝 +x 时 back = -x）；通道点沿这个方向挪
        map_dx = dir_x * math.cos(yaw) - dir_y * math.sin(yaw)
        map_dy = dir_x * math.sin(yaw) + dir_y * math.cos(yaw)
        along_x = abs(map_dx) >= abs(map_dy)
        first = lane['indices'][0]
        base = lane['base'][first]
        current = base[0] + (lane['shift'] or 0.0) if along_x else base[1] + (lane['shift'] or 0.0)
        here = x if along_x else y
        unit = map_dx if along_x else map_dy
        tilt_ok = abs(unit) >= 0.9               # 车身相对墙斜得太厉害（> 25 度）时不用
        unit = 1.0 if unit > 0 else -1.0         # 墙垂直于地图 x（或 y）轴：车沿这个轴挪多少，墙距就变多少
        # 预计墙距 = 目标距离 + 车离通道线还有多远（朝墙方向为正）
        expected = lane['distance'] + (here - current) * (-unit)
        angles, xs, ys = points
        fit_cfg = ctx.cfg['system']['wall_fit']
        fit = fit_wall(angles, xs, ys, side,
                       sector_half_angle=math.radians(fit_cfg['sector_half_angle_deg']),
                       expected_distance=max(0.1, expected),
                       distance_band=fit_cfg['distance_band'],
                       min_points=fit_cfg['min_points'],
                       outlier_threshold=fit_cfg['outlier_threshold'],
                       max_yaw_offset=math.radians(fit_cfg['max_yaw_offset_deg']))
        if fit is None or not tilt_ok:
            return                       # 没看到墙，或车身斜得太厉害：沿用上一次的位置
        target = here + (fit.distance - lane['distance']) * unit      # 墙距正好等于目标距离的位置
        shift = target - (base[0] if along_x else base[1])
        shift = max(-lane['max_shift'], min(lane['max_shift'], shift))
        if lane['shift'] is None:
            lane['shift'] = shift
        else:
            lane['shift'] += (shift - lane['shift']) * lane['smooth']
        for i in lane['indices']:
            bx, by, byaw = lane['base'][i]
            if along_x:
                self.follower.set_waypoint_x(i, bx + lane['shift'])
            else:
                self.follower.set_waypoint_y(i, by + lane['shift'])
        if not lane['logged']:
            ctx.log.event(self.id, 'lane', '通道按激光量%s墙定位：墙距 %.3f 米，通道点挪 %+.1f cm' % (
                {'back': '后', 'front': '前', 'left': '左', 'right': '右'}[side], fit.distance, lane['shift'] * 100))
            lane['logged'] = True

    def _update_direct(self, ctx):
        pose = ctx.io.robot_pose()
        if pose is None:
            ctx.command = STOP            # 暂时拿不到定位，原地等
            return RUNNING
        self._update_lane_wall(ctx, pose)
        command, finished, distance = self.follower.update(pose, ctx.chassis)
        segment = self.follower.current_segment()
        if segment.get('accel'):
            ctx.smoother.linear_accel = float(segment['accel'])      # 这一段的加速度上限（爆发段更大）
        if segment['profile'] != self.logged_profile:
            ctx.log.event(self.id, 'profile', '第 %d 段：%s（最高 %.2f m/s）' % (
                self.follower.segment + 1, segment['profile'], segment['max_speed']))
            self.logged_profile = segment['profile']
        if finished:
            ctx.command = STOP
            return DONE
        safety = ctx.cfg['system']['safety']
        points = ctx.io.scan_points()
        if points is not None and safety['enabled']:
            _, xs, ys = points
            base_xs, base_ys = points_in_base_frame(xs, ys, ctx.cfg['system']['laser_offset_x'])
            # 停车距离 = 固定余量 + 刹车距离 v²/(2a) + 激光延迟期间走过的距离
            speed = math.hypot(ctx.smoother.current.vx, ctx.smoother.current.vy)
            stop_distance = (safety['stop_distance'] + speed ** 2 / (2.0 * safety['brake_decel'])
                             + speed * safety['sensor_latency_s'])
            if obstacle_in_path(base_xs, base_ys, command, safety['front_length'], safety['rear_length'],
                                safety['half_width'], safety['side_margin'], stop_distance,
                                float(safety.get('self_depth', 0.02))):
                if self.follower.is_last() and distance < safety['finish_if_blocked_within']:
                    # 已经很接近终点（比如终点框紧贴围栏）：剩下的交给贴墙精定位慢慢靠过去。
                    # 朝向还没转完就先原地转完（只停平移，安全检查只管平移方向）。2026-10-17 去识别区只转到 56 度就结束了
                    wz, turned = self.follower.turn_in_place(pose, ctx.chassis)
                    if not turned:
                        ctx.stop_translation()
                        ctx.command = VelocityCommand(0.0, 0.0, wz)
                        return RUNNING
                    ctx.stop_robot()
                    ctx.log.event(self.id, 'near_goal', '离终点 %.2f 米处被墙挡住，交给精定位' % distance)
                    return DONE
                ctx.stop_robot()          # 立即停车，不走加速度平滑
                if not self.blocked_logged:
                    ctx.log.event(self.id, 'blocked', '前方有障碍，停车等待')
                    self.blocked_logged = True
                return RUNNING
        self.blocked_logged = False
        ctx.command = command
        return RUNNING

    def _update_move_base(self, ctx):
        if ctx.io.move_base_client is None:
            return DONE
        state = ctx.io.move_base_state()
        if state == 'active':
            return RUNNING
        if state == 'failed':
            ctx.log.event(self.id, 'warning', 'move_base 未到达第 %d 个点' % self.goal_index)
        self.goal_index += 1
        if self.goal_index >= len(self.waypoints):
            return DONE
        ctx.io.move_base_go(*self.waypoints[self.goal_index])
        return RUNNING

    def on_timeout(self, ctx):
        if self.mode == 'move_base':
            ctx.io.move_base_cancel()

    def finish(self, ctx):
        ctx.external_control = False
        ctx.command = STOP
        if getattr(self, 'default_accel', None) is not None:
            ctx.smoother.linear_accel = self.default_accel          # 恢复常规加速度上限
        pose = ctx.io.robot_pose()
        if pose is not None:
            ctx.log.event(self.id, 'pose', 'x=%.3f y=%.3f yaw=%.1fdeg' % (pose[0], pose[1], math.degrees(pose[2])))


# ------------------------------------------------------------
# align：贴墙精定位
# ------------------------------------------------------------
class AlignStep(Step):
    """用激光测墙，把车停到示教位置（任务框正中、朝向正对靶）。

    field.yaml 的 boxes.<box> 里写着：要看哪几面墙、每面墙的目标距离、用哪面墙定朝向、目标偏角。
    连续 ok_cycles 个周期误差都在容差内才算完成，避免抖动时误判。

    墙线核对（2026-10-17 终点框：车在墙角，后方扇区同时扫到两面墙，拟合出一条斜 16 度的"墙"，
    精定位照着它把车扭了 17 度）：任务框有同名路径点时，用 AMCL 朝向估计"定朝向那面墙"应该的偏角，
    差超过 wall_align.misfit_deg 就不按它转向（距离照用）。
    需要的墙连续 missing_finish_s 秒看不到、看得到的都已到位 → 提前结束，不干等到超时。
    完成时把地图位姿记进 events.csv（pose 事件），teach.py adopt 用它更新同名路径点。

    2026-10-20 按 22:19 整局录包（激光逐帧复盘）加的三条，目的都是不在最后几毫米上干耗时间：
      1. 每个框可以单独设容差（field.yaml boxes.<框>.distance_tolerance / yaw_tolerance_deg），
         识别区只要相机拍得到卡片，放宽到 2 cm / 2 度（原来 0.8 cm：横向停在 1.1 cm，发 0.022 m/s 车不动，干等 2.3 秒超时）。
      2. 差一点也算到：墙距都在 容差 × near_factor、朝向在 容差 × near_factor_yaw 以内，
         near_hold_s 秒内几乎不变（车已经不动或只在慢慢蹭）→ 等车停稳后完成，
         记 near 事件（2 号框：后墙 0.8~0.9 cm，容差 0.8 cm，发 0.020 m/s 车不动，耗 1.1 秒；
         1 号框：朝向 -0.8 → -0.5 度靠 0.04 rad/s 慢慢蹭了 2 秒）。
      3. 发了最小速度车不动：误差超出容差、墙距 stall_s 秒内几乎不变 → 最小速度 ×1.5（最多 boost_max 倍），记 boost 事件。
    """

    def start(self, ctx):
        box = ctx.cfg['field']['boxes'][self.spec['box']]
        self.box_name = self.spec['box']
        self.wall_refs = [{'side': w['side'], 'distance': float(w['distance'])} for w in box['walls']]
        params = dict(ctx.cfg['system']['wall_align'])
        if box.get('distance_tolerance') is not None:
            params['distance_tolerance'] = float(box['distance_tolerance'])     # 这个框单独的容差
        if box.get('yaw_tolerance_deg') is not None:
            params['yaw_tolerance_deg'] = float(box['yaw_tolerance_deg'])
        params['yaw_tolerance'] = math.radians(params['yaw_tolerance_deg'])
        self.params = params
        self.boost = 1.0
        self.history = collections.deque()       # [(时刻, errors)]：判断车是不是已经不动了
        self.near_since = None
        self.stopping = False                    # 已判"差一点也算到"：只发零速度，等车真正停稳再结束
        self.zero_since = None                   # 从什么时候起一直发零速度（_still 用）
        self.start_time = None
        self.yaw_ref = math.radians(float(box.get('yaw_ref_deg', 0.0)))
        self.aligner = WallAligner(self.wall_refs, box['yaw_side'], self.yaw_ref, params)
        self.ok_cycles_needed = int(params['ok_cycles'])
        self.ok_cycles = 0
        self.last_errors = {}
        self.missing_since = None
        self.rejected_logged = set()
        waypoints = ctx.cfg['field'].get('waypoints', {})
        self.box_yaw = math.radians(float(waypoints[self.box_name][2])) if self.box_name in waypoints else None
        self.finished_ok = False

    def _check_fits(self, ctx, fits):
        """用 AMCL 朝向核对"定朝向那面墙"的拟合偏角，差太多就只是不按它转向（距离照用）。

        2026-10-17 22:06 实测：AMCL 朝向本身会差约 4 度，所以只拦明显离谱的（终点墙角那种 20 度），
        而且只影响转向——之前把整面墙丢掉，1 号框横向没对齐，环形靶偏了 23 像素。
        """
        misfit = math.radians(float(self.params.get('misfit_deg', 0.0)))
        side = self.aligner.yaw_side
        fit = fits.get(side)
        if misfit <= 0 or self.box_yaw is None or fit is None:
            return fits
        pose = ctx.io.robot_pose()
        if pose is None:
            return fits
        expected = self.yaw_ref + wrap_angle(pose[2] - self.box_yaw)   # 车比示教朝向多逆时针转了多少，偏角就多多少
        if abs(wrap_angle(fit.yaw_offset - expected)) > misfit:
            if side not in self.rejected_logged:
                ctx.log.event(self.id, 'wall_rejected', '%s 墙拟合偏角 %.1f 度，按定位应约 %.1f 度，这面墙只用距离、不按它转向' % (
                    side, math.degrees(fit.yaw_offset), math.degrees(expected)))
                self.rejected_logged.add(side)
            checked = dict(fits)
            checked[side] = WallFit(fit.side, fit.distance, self.yaw_ref, fit.num_points)   # 朝向误差当作 0：不转
            return checked
        return fits

    def _settled(self, now, window_s):
        """最近 window_s 秒车没在动、或只在很慢地蹭：窗口前一半和后一半的平均误差差得很少
        （墙距 < settle_m、朝向 < settle_deg）。用平均值比较，单帧拟合的噪声（墙距约 1 mm、偏角约 0.1 度）不影响判断。"""
        p = self.params
        if not self.history or now - self.history[0][0] < window_s - 0.06:
            return False
        recent = [(t, e) for t, e in self.history if t >= now - window_s]
        keys = set(recent[-1][1].keys())
        if not keys or any(set(e.keys()) != keys for t, e in recent):
            return False
        middle = now - window_s / 2.0
        first = [e for t, e in recent if t < middle]
        second = [e for t, e in recent if t >= middle]
        if not first or not second:
            return False
        for k in keys:
            drift = abs(sum(e[k] for e in second) / len(second) - sum(e[k] for e in first) / len(first))
            limit = math.radians(float(p.get('settle_deg', 0.1))) if k == 'yaw' else float(p.get('settle_m', 0.002))
            if drift > limit:
                return False
        return True

    def _within(self, errors, factor, yaw_factor=None):
        p = self.params
        yaw_factor = factor if yaw_factor is None else yaw_factor
        return bool(errors) and all(
            abs(e) <= (yaw_factor * p['yaw_tolerance'] if k == 'yaw' else factor * p['distance_tolerance'])
            for k, e in errors.items())

    def _record(self, now, errors):
        p = self.params
        self.history.append((now, dict(errors)))
        keep_s = max(float(p.get('near_hold_s', 0.6)), float(p.get('stall_s', 0.6))) + 0.2
        while self.history and self.history[0][0] < now - keep_s:
            self.history.popleft()

    def _still(self, ctx, now):
        """车停稳了：底盘模型说停了；或者零速度已经发了超过一个死区、激光量的墙距和朝向最近 0.3 秒都没变。
        2026-10-20 16:19 录包：3 号框判"差一点"时车其实早就不动了（最后发的 0.045 m/s 横移根本没推动），
        模型却按"在动"算，又等了 0.9 秒才结束。"""
        if ctx.chassis.is_stopped():
            return True
        return (self.zero_since is not None and now - self.zero_since >= ctx.chassis.dead + 0.1
                and self._settled(now, 0.3))

    def update(self, ctx):
        now = ctx.io.now()
        if self.stopping:
            # 和正常完成一样，等底盘真正停稳（死区里还有刚发的小指令，车还会蹭一点）才交给下一步瞄准。
            # 仿真：不等就结束，环形靶看靶时车还在慢慢转，弹着点差 2.8 cm
            ctx.command = STOP
            self.zero_since = now if self.zero_since is None else self.zero_since
            errors = self.aligner.update(self._check_fits(ctx, scan_wall_fits(ctx, self.wall_refs)), None)[1]
            if errors:
                self._record(now, errors)
            self.ok_cycles = self.ok_cycles + 1 if self._still(ctx, now) else 0
            if self.ok_cycles >= self.ok_cycles_needed:
                self.last_errors = errors or self.last_errors
                return DONE
            return RUNNING
        fits = self._check_fits(ctx, scan_wall_fits(ctx, self.wall_refs))
        command, errors, aligned = self.aligner.update(fits, ctx.chassis, self.boost)
        self.last_errors = errors
        p = self.params
        self.start_time = now if self.start_time is None else self.start_time
        self._record(now, errors)
        all_seen = all(fits.get(ref['side']) is not None for ref in self.wall_refs) and 'yaw' in errors
        near_factor = float(p.get('near_factor', 0.0))
        in_near = all_seen and near_factor > 1.0 and self._within(
            errors, near_factor, float(p.get('near_factor_yaw', near_factor)))
        # 计时只在误差跑出"差一点"范围时才清零：误差正好压在容差线上时，噪声会让它一会儿算到位、一会儿不算，
        # 两种完成条件都被反复打断（仿真里这样耗到超时）
        self.near_since = (now if self.near_since is None else self.near_since) if in_near else None
        if in_near:
            # 差一点也算到（说明第 2 条）
            hold = float(p.get('near_hold_s', 0.6))
            if now - self.near_since >= hold and self._settled(now, hold):
                if not aligned:
                    ctx.log.event(self.id, 'near', '误差都在"差一点"范围内（墙距 %.1f 倍、朝向 %.1f 倍容差）、车已不动，算到位' % (
                        near_factor, float(p.get('near_factor_yaw', near_factor))))
                ctx.command = STOP
                self.finished_ok = True
                self.stopping = True
                self.ok_cycles = 0
                return RUNNING
        elif not aligned and all_seen:
            # 发了最小速度车不动（说明第 3 条）
            stall = float(p.get('stall_s', 0.0))
            # 刚开始的 1 秒不算：底盘死区约 0.4 秒，起步本来就慢（仿真：一开始就误判"推不动"，加大最小速度后来回冲过头）
            if stall > 0 and now - self.start_time >= 1.0 and self.boost < float(p.get('boost_max', 1.0)) \
                    and self._settled(now, stall):
                self.boost = min(self.boost * 1.5, float(p.get('boost_max', 1.0)))
                ctx.log.event(self.id, 'boost', '车不动，最小速度加大到 %.1f 倍' % self.boost)
                self.history.clear()
        if aligned:
            self.missing_since = None
            ctx.command = STOP
            self.zero_since = now if self.zero_since is None else self.zero_since
            # 车停稳后还在容差内才算数：底盘有 0.3 秒死区，刚发零速度时车还在滑（审查仿真：DONE 后又滑 1~15 mm）
            self.ok_cycles = self.ok_cycles + 1 if self._still(ctx, now) else 0
            if self.ok_cycles >= self.ok_cycles_needed:
                self.finished_ok = True
                return DONE
            return RUNNING
        self.ok_cycles = 0
        missing = any(fits.get(ref['side']) is None for ref in self.wall_refs)
        if missing:
            self.missing_since = now if self.missing_since is None else self.missing_since
            p = self.params
            # 至少看到一面墙的距离才能"提前结束"（一面墙都看不到时 errors 为空，不能算到位）
            seen_ok = any(k != 'yaw' for k in errors) and all(
                abs(e) <= (p['yaw_tolerance'] if k == 'yaw' else p['distance_tolerance']) for k, e in errors.items())
            if seen_ok and ctx.chassis.is_stopped() and now - self.missing_since >= float(p['missing_finish_s']):
                ctx.log.event(self.id, 'partial', '有墙看不到，看得到的已到位，提前结束')
                ctx.command = STOP
                return DONE
        else:
            self.missing_since = None
        ctx.command = command
        if command.is_zero():
            self.zero_since = now if self.zero_since is None else self.zero_since
        else:
            self.zero_since = None
        return RUNNING

    def finish(self, ctx):
        ctx.command = STOP
        text = ' '.join('%s=%.1f%s' % (k, math.degrees(v) if k == 'yaw' else v * 100.0,
                                        'deg' if k == 'yaw' else 'cm')
                        for k, v in sorted(self.last_errors.items()))
        ctx.log.event(self.id, 'align_error', text)
        pose = ctx.io.robot_pose()
        if pose is not None:
            # teach.py adopt 用它把"精定位完的位置"写成路径点，下次行驶直接停到这里附近
            ctx.log.event(self.id, 'pose', 'box=%s ok=%d x=%.3f y=%.3f yaw=%.1fdeg' % (
                self.box_name, 1 if self.finished_ok else 0, pose[0], pose[1], math.degrees(pose[2])))


# ------------------------------------------------------------
# recognize：识别区卡片识别
# ------------------------------------------------------------
class RecognizeStep(Step):
    """停稳 → 采集 capture_s 秒的图像 → 选最清晰的一张 → 识别 → 写入 ctx.targets。"""

    def start(self, ctx):
        self.phase = 'settle'
        self.capture_start = None
        self.image = None
        ctx.command = STOP

    def update(self, ctx):
        rec = ctx.cfg['recognition']
        now = ctx.io.now()
        if self.phase == 'settle':
            if ctx.robot_still_for(ctx.cfg['system']['still_hold_s']):
                self.phase = 'capture'
                self.capture_start = now
            return RUNNING

        if self.phase == 'capture':
            if now - self.capture_start < rec['capture_s']:
                return RUNNING
            frames = ctx.io.camera_frames(since=self.capture_start)
            if not frames:
                ctx.log.event(self.id, 'error', '没有收到相机图像，使用默认靶号')
                self._use_targets(ctx, ctx.recognizer.fallback())
                return DONE
            self.image = pick_sharpest(frames)
            ctx.log.save_image('recognition.jpg', self.image)
            ctx.recognizer.start(self.image, now)
            self.phase = 'recognize'
            return RUNNING

        result = ctx.recognizer.poll(now)
        if result is None:
            return RUNNING
        self._use_targets(ctx, result)
        return DONE

    def _use_targets(self, ctx, result):
        ctx.targets['rotating'] = result['rotating']
        ctx.targets['moving'] = result['moving']
        ctx.log.event(self.id, 'targets', 'rotating=%d moving=%d source=%s' % (
            result['rotating'], result['moving'], result['source']))
        summary = ctx.recognizer.summary()
        summary['final'] = result
        ctx.log.save_json('recognition.json', summary)

    def on_timeout(self, ctx):
        # 超时：有图就按"兜底"规则给一个结果，保证后面流程有靶号可用
        self._use_targets(ctx, ctx.recognizer.fallback())


def pick_sharpest(frames):
    """从 [(时间, 图像), ...] 中选最清晰的一张（拉普拉斯方差最大，模糊的照片方差小）。"""
    import cv2
    best_image, best_score = None, -1.0
    for _, image in frames:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        score = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        if score > best_score:
            best_image, best_score = image, score
    return best_image


# ------------------------------------------------------------
# shoot_ring：环形靶
# ------------------------------------------------------------
class ShootRingStep(Step):
    """环形靶（静止）。靶心横坐标来自 ring.detector：rings = 相机画面找同心圆（ring_finder.py，2026-10-22 起默认），
    find_object = 旧做法。detector 为 rings 时 ring.rings 里的 aim_x_px、tolerance_px 等覆盖上层同名项。

    三种打法（shooting.yaml ring.mode）：

      vision        "停稳看 → 横向平移过去 → 停稳再看"，偏差够小就开火（详见 aiming.RingAimer）。
      taught        精定位到位、停稳就开火，不看靶。前提：示教的 1 号框位置就是"打得准的位置"，
                    而且精定位的重复性比识别准（精定位实测 0.1~0.3 cm；find_object 单帧约 1.5 cm）。
      taught_check  停稳后看 check_frames 帧核对：偏差 ≤ check_px 就开火；偏差太大（精定位出错、车被碰过）
                    转入 vision 对准；check_wait_s 内一帧都没看到也照样开火。
    哪种好要看场地测试（菜单 13 精定位重复性），默认 vision。

    每收到一帧新的检测结果、或状态变化，都记一笔 aim 事件（像素偏差、用了几帧、要平移多少），调参用。
    超时（一直没对准或没看到靶）时按 on_timeout：fire = 先停稳再照当前位置开火；skip = 不打。
    """

    def start(self, ctx):
        ring_cfg = ctx.cfg['shooting']['ring']
        self.params = effective_ring_params(ring_cfg)
        self.source = RingSource(ring_cfg)
        system = ctx.cfg['system']
        self.aimer = RingAimer(self.params, system['lateral_shift'], system['camera']['fx'])
        self.mode = self.params.get('mode', 'vision')
        self.check_from = None        # taught_check：停稳后从什么时刻开始核对
        self.last_info = {}
        self.last_key = None
        self.tracker = LateralTracker(self.id)
        ctx.command = STOP

    def _taught(self, ctx, now, observations):
        """taught / taught_check 两种打法。返回 DONE / RUNNING；返回 None 表示核对没通过、转入 vision。"""
        ctx.command = STOP
        if not ctx.robot_still_for(self.params['settle_s']):
            self.check_from = None
            return RUNNING
        if self.check_from is None:
            self.check_from = now
        fresh = [x for (t, x) in observations if t >= self.check_from + self.params['vision_latency_s']]
        info = {'mode': self.mode, 'view_frames': len(fresh)}
        if fresh:
            fresh_sorted = sorted(fresh)
            info['view_err_px'] = fresh_sorted[len(fresh_sorted) // 2] - self.params['aim_x_px']
        self.last_info = info
        if self.mode == 'taught':
            fire_shots(ctx, self.id, 'ring', 'taught', self.params['fire'], format_info(info))
            return DONE
        if len(fresh) >= int(self.params.get('check_frames', 2)):
            if abs(info['view_err_px']) <= self.params.get('check_px', 8.0):
                fire_shots(ctx, self.id, 'ring', 'taught_checked', self.params['fire'], format_info(info))
                return DONE
            ctx.log.event(self.id, 'check_failed', '核对偏差太大，转入视觉对准：' + format_info(info))
            self.mode = 'vision'
            return None
        if now - self.check_from > self.params.get('check_wait_s', 1.5):
            fire_shots(ctx, self.id, 'ring', 'taught_noview', self.params['fire'], format_info(info))
            return DONE
        return RUNNING

    def update(self, ctx):
        now = ctx.io.now()
        still = ctx.robot_still_for(self.params['settle_s'])
        # 找同心圆每帧要几十到一两百毫秒：只在车停稳（要"看"）的时候处理新画面；车在平移时不处理
        observations = self.source.observations(ctx.io, self.params['max_obs_age_s'], process=still)
        if self.mode != 'vision':
            result = self._taught(ctx, now, observations)
            if result is not None:
                return result
        lateral = self.tracker.position(ctx)
        vy, ready, info = self.aimer.update(now, lateral, observations, still)
        self.last_info = info
        key = (info.get('state'), info.get('reason'), info.get('frames'), info.get('fresh'))
        if key != self.last_key:
            # 状态/原因变化、或收到新帧时记一笔：像素偏差、用了几帧、要移多少、移到位还是卡住（调参用）
            ctx.log.event(self.id, 'aim', format_info(info))
            self.last_key = key
        wz = self.tracker.hold_rate(self.aimer.state == 'shift', self.params.get('hold_yaw_on_shift', True))
        ctx.command = VelocityCommand(0.0, vy, wz)
        if ready:
            trigger = 'max_shift' if info.get('reason') == 'max_shift_reached' else 'aimed'
            fire_shots(ctx, self.id, 'ring', trigger, self.params['fire'], format_info(info))
            return DONE
        return RUNNING

    def on_timeout(self, ctx):
        if self.spec.get('on_timeout', 'fire') == 'fire':
            stop_and_settle(ctx, self.params['settle_s'])
            fire_shots(ctx, self.id, 'ring', 'timeout', self.params['fire'], format_info(self.last_info))


# ------------------------------------------------------------
# shoot_ar：旋转靶
# ------------------------------------------------------------
class ShootRotatingStep(Step):
    """旋转靶：拟合转轮 → 选交点 → 扭车头 / 平移对准 → 到时刻开火。
    shooting.yaml rotating.correction 选做法：
      servo（2026-10-23 v30，默认）：边看边对（aiming.RotatingServo），车动着也一直看画面，不用每动一次停下来重新观察；
             扭车头 / 平移的上限用里程计（车头朝向、横向位移）量，看不到侧墙也能用；
      twist / shift（v29 及以前，aiming.RotatingPlanner）：停稳观察 → 扭车头 / 平移 → 停稳再观察。

    靶号来自识别结果 ctx.targets['rotating']；参数来自 shooting.yaml 的 rotating 段。
    最晚开火时刻 = min(开始 + max_wait_s, 比赛结束 - reserve_after_s)：等不到就放弃，把时间留给后面的任务。
    一直看不到转轮（blind_timeout_s 秒）→ 按 on_timeout 处理（fire = 照当前位置开一枪），不干等。
    """

    BLIND_REASONS = ('few_markers', 'bad_circle', 'short_observation', 'target_not_seen', 'stale')

    def start(self, ctx):
        self.params = ctx.cfg['shooting']['rotating']
        system = ctx.cfg['system']
        self.marker_id = int(ctx.targets['rotating'])
        self.servo = None
        self.planner = None
        if self.params.get('correction') == 'servo':
            self.servo = RotatingServo(self.params, self.params.get('servo'), self.marker_id, 1.0 / system['loop_hz'])
            self.history_s = max(self.params['history_s'], (self.params.get('servo') or {}).get('window_s', 6.0))
            self.odom0 = None                            # 这一步开始时的里程计位姿（横移上限从这里算）
            self.heading_set = False                     # 扭的上限按"车头正对侧墙"算（绝对朝向），第一次 update 时定
        else:
            self.planner = RotatingPlanner(self.params, system['lateral_shift'], system['camera']['fx'],
                                           self.marker_id, 1.0 / system['loop_hz'], system.get('yaw_twist'))
        self.tracker = LateralTracker(self.id)
        self.logger = TransitionLogger(self.id)
        now = ctx.io.now()
        self.deadline = min(now + self.params['max_wait_s'], ctx.match_end() - self.params['reserve_after_s'])
        self.blind_since = None
        self.last_info = {}
        ctx.command = STOP
        ctx.log.event(self.id, 'marker', 'id=%d 最多等 %.1f 秒' % (self.marker_id, self.deadline - now))

    def _odom_offsets(self, ctx):
        """里程计量的 (横向位移 米 左为正, 车头 弧度 左转为正)，都相对这一步开始；拿不到里程计时退回激光侧墙。"""
        pose = ctx.io.odom_pose()
        if pose is not None:
            if self.odom0 is None:
                self.odom0 = pose
            x0, y0, yaw0 = self.odom0
            dx, dy = pose[0] - x0, pose[1] - y0
            return -dx * math.sin(yaw0) + dy * math.cos(yaw0), yaw0 + wrap_angle(pose[2] - yaw0)
        lateral = self.tracker.position(ctx)
        return lateral, (self.tracker.yaw if lateral is not None else None)

    def update(self, ctx):
        now = ctx.io.now()
        if self.servo is not None:
            samples = ctx.io.ar_pixels(self.params['wheel_ids'], self.history_s)
            lateral, yaw = self._odom_offsets(ctx)
            if not self.heading_set and yaw is not None:
                self.heading_set = True
                heading = wall_heading(ctx, self.tracker)
                if heading is not None:
                    self.servo.set_heading_reference(yaw - heading)
                ctx.log.event(self.id, 'heading', '车头相对侧墙 %s' % (
                    '%.1f 度（扭的上限按正对侧墙算）' % math.degrees(heading) if heading is not None
                    else '测不到（扭的上限按这一步开始时的车头算）'))
            vx, vy, wz, fire, info = self.servo.update(now, lateral, yaw, samples, ctx.still_duration(), self.deadline)
            self.last_info = info
            self.logger.update(ctx, info)
            ctx.command = VelocityCommand(vx, vy, wz)
            return self._after(ctx, now, fire, info)
        samples = ctx.io.ar_pixels(self.params['wheel_ids'], self.params['history_s'])
        lateral = self.tracker.position(ctx)
        yaw = self.tracker.yaw if lateral is not None else None
        vx, vy, fire, info = self.planner.update(now, lateral, samples, ctx.robot_still_for(self.params['settle_s']),
                                                 ctx.still_duration(), self.deadline, yaw)
        self.last_info = info
        self.logger.update(ctx, info)
        if self.planner.state == 'twist':
            wz = self.planner.wz                     # 扭车头（左右交点哪边先到打哪边，见 aiming.YawTwister）
        else:
            wz = self.tracker.hold_rate(abs(vy) > 1e-6, self.params.get('hold_yaw_on_shift', True))
        ctx.command = VelocityCommand(vx, vy, wz)
        return self._after(ctx, now, fire, info)

    def _after(self, ctx, now, fire, info):
        if fire:
            fire_shots(ctx, self.id, 'rotating', 'predicted', self.params['fire'], format_info(info))
            return DONE
        if info.get('give_up'):
            ctx.log.event(self.id, 'give_up', '要等 %.1f 秒，超过最晚开火时刻，放弃' % info.get('wait_s', 0.0))
            return DONE
        if info.get('reason') in self.BLIND_REASONS:
            if self.blind_since is None:
                self.blind_since = now
            elif now - self.blind_since > self.params['blind_timeout_s']:
                ctx.log.event(self.id, 'blind', '%.1f 秒看不清转轮（%s）' % (now - self.blind_since, info['reason']))
                self.on_timeout(ctx)
                return DONE
        else:
            self.blind_since = None
        return RUNNING

    def on_timeout(self, ctx):
        if self.spec.get('on_timeout', 'fire') == 'fire':
            stop_and_settle(ctx, self.params['settle_s'])
            fire_shots(ctx, self.id, 'rotating', 'timeout', self.params['fire'], format_info(self.last_info))


# ------------------------------------------------------------
# shoot_ar：移动靶
# ------------------------------------------------------------
class ShootMovingStep(Step):
    """移动靶：车停着，等目标码"预计到达枪口"时开火，避开换向（详见 aiming.MovingPredictor）。
    2026-10-23 起（moving.intercept）：码要很久才回到弹着点时，先原地扭车头把弹着点挪到码更早经过的地方（aiming.MovingInterceptor），
    扭完停稳再照常预测开火。开火时车是停着的。

    靶号来自识别结果 ctx.targets['moving']；参数来自 shooting.yaml 的 moving 段。
    只用车停稳以后拍的帧（车动的时候画面里的位置也在变，会把速度和行程端点算错）。
    """

    def start(self, ctx):
        self.params = ctx.cfg['shooting']['moving']
        system = ctx.cfg['system']
        self.marker_id = int(ctx.targets['moving'])
        self.period = 1.0 / system['loop_hz']
        self.predictor = MovingPredictor(self.params, self.period)
        self.interceptor = MovingInterceptor(self.params, self.marker_id)
        intercept = self.params.get('intercept') or {}
        self.twister = None
        if self.interceptor.enabled and system.get('yaw_twist'):
            twist_params = dict(system['yaw_twist'])
            twist_params['min_rate'] = intercept.get('twist_min_rate', twist_params['min_rate'])
            twist_params['timeout_s'] = intercept.get('twist_timeout_s', twist_params.get('timeout_s', 4.0))
            self.twister = YawTwister(twist_params)
        self.max_twists = int(intercept.get('max_twists', 2))
        self.twists = 0
        self.twist_from = None       # 这次扭开始的 (时刻, 车头)
        self.started_at = ctx.io.now()
        self.state = 'watch'
        self.tracker = LateralTracker(self.id)
        self.yaw_ref = None          # 车头正对侧墙时的里程计朝向（测不到侧墙时 = 这一步开始时的车头）
        self.wall_ref = False
        self.yaw_start = None
        self.planned_since = None    # 这次停稳以后已经算过截击（停稳时刻）
        self.last_reason = None
        self.logger = TransitionLogger(self.id)
        self.last_info = {}
        ctx.command = STOP
        ctx.log.event(self.id, 'marker', 'id=%d' % self.marker_id)

    def _samples(self, ctx, still_since):
        """目标码车停稳以后的像素位置 [(t, u, v), ...]。
        moving.fuse_codes 写了几个码（2026-10-25 起 [6, 7, 8]）：目标码没认出来的帧用同一滑块上另外的码补
        （偏移按同一帧两个码都认出来的帧实测，帧数不够时用 fuse_layout_px 先验，见 aiming.fuse_slider_codes）；
        空列表 = 只用目标码（以前的做法）。"""
        ids = [int(i) for i in (self.params.get('fuse_codes') or [])]
        history = self.params['history_s']
        if len(ids) < 2:
            samples = ctx.io.ar_pixels([self.marker_id], history)[self.marker_id]
            return [s for s in samples if s[0] >= still_since]
        if self.marker_id not in ids:
            ids.append(self.marker_id)
        # 只用车停稳以后的帧（偏移也只用这些帧量：车在开过来、精定位时离靶远近不同，码之间的像素间距跟着变）
        by_id = ctx.io.ar_pixels(ids, history)
        by_id = dict((i, [s for s in by_id.get(i, []) if s[0] >= still_since]) for i in ids)
        return fuse_slider_codes(by_id, self.marker_id, min_pairs=int(self.params.get('fuse_min_pairs', 3)),
                                 prior_px=self.params.get('fuse_layout_px'))

    def _heading(self, yaw):
        """车头相对侧墙的朝向（弧度）。测不到侧墙时假定这一步开始时的朝向 = 标定行程时的朝向。"""
        if yaw is None or self.yaw_ref is None:
            return None
        if self.wall_ref:
            return wrap_angle(yaw - self.yaw_ref)
        return self.interceptor.prior_heading + wrap_angle(yaw - self.yaw_start)

    def update(self, ctx):
        now = ctx.io.now()
        if self.state == 'twist':
            yaw = ctx.io.odom_yaw()
            wz, status, remaining = self.twister.update(now, yaw)
            if status == 'moving':
                ctx.command = VelocityCommand(0.0, 0.0, wz)
                return RUNNING
            ctx.command = STOP
            heading = self._heading(yaw)
            if yaw is not None and self.twist_from is not None:
                self.interceptor.learn(wrap_angle(yaw - self.twist_from[1]), now - self.twist_from[0])
            ctx.log.event(self.id, 'twist_' + status, '差 %.1f 度，车头相对侧墙 %s 度，用时 %.1f 秒' % (
                math.degrees(remaining or 0.0), '%.1f' % math.degrees(heading) if heading is not None else '?',
                now - self.twist_from[0] if self.twist_from else -1.0))
            self.state = 'watch'
            self.predictor = MovingPredictor(self.params, self.period)   # 扭过以后画面整体平移了，旧的历史不能用
            return RUNNING
        ctx.command = STOP
        if self.yaw_ref is None and self.twister is not None:
            yaw = ctx.io.odom_yaw()
            if yaw is not None:
                heading = wall_heading(ctx, self.tracker)
                self.yaw_start = yaw
                self.wall_ref = heading is not None
                self.yaw_ref = yaw - heading if heading is not None else yaw
                ctx.log.event(self.id, 'heading', '车头相对侧墙 %s' % (
                    '%.1f 度' % math.degrees(heading) if heading is not None else '测不到（按标定时的朝向算）'))
        if not ctx.robot_still_for(self.params['settle_s']):
            return RUNNING
        still_since = now - ctx.still_duration()
        samples = self._samples(ctx, still_since)
        fire, info = self.predictor.check(now, samples)
        self.last_info = info
        self.logger.update(ctx, info)
        if fire:
            if info.get('delay_s', 0.0) > 0.0:
                ctx.io.sleep(info['delay_s'])        # 精确到开火时刻（不在 20 Hz 控制周期上取整）
            fire_shots(ctx, self.id, 'moving', 'predicted', self.params['fire'], format_info(info))
            return DONE
        reason = info.get('reason')
        if self.twister is not None and self.yaw_ref is not None and self.twists < self.max_twists:
            new_still = self.planned_since is None or abs(self.planned_since - still_since) > 0.05
            just_passed = reason == 'missed' and self.last_reason != 'missed'      # 码刚走过弹着点：重新算一次
            if new_still or just_passed:
                yaw = ctx.io.odom_yaw()
                time_left = self.timeout_s - (now - self.started_at) - 1.0
                target, plan = self.interceptor.plan(now, samples, yaw, self.yaw_ref, self._heading(yaw), time_left)
                if plan.get('intercept') not in ('short', 'few', 'slow', 'no_data'):
                    self.planned_since = still_since
                    ctx.log.event(self.id, 'intercept', format_info(plan))
                if target is not None:
                    self.twister.start(target, now)
                    self.twist_from = (now, yaw)
                    self.state = 'twist'
                    self.twists += 1
        self.last_reason = reason
        return RUNNING

    def on_timeout(self, ctx):
        if self.spec.get('on_timeout', 'fire') == 'fire':
            stop_and_settle(ctx, self.params['settle_s'])
            fire_shots(ctx, self.id, 'moving', 'timeout', self.params['fire'], format_info(self.last_info))


def format_info(info):
    parts = []
    for key in sorted(info):
        value = info[key]
        parts.append('%s=%.3f' % (key, value) if isinstance(value, float) else '%s=%s' % (key, value))
    return ' '.join(parts)


# ------------------------------------------------------------
# pause：原地等待
# ------------------------------------------------------------
class PauseStep(Step):
    def start(self, ctx):
        self.end_time = ctx.io.now() + float(self.spec.get('seconds', 1.0))
        ctx.command = STOP

    def update(self, ctx):
        return DONE if ctx.io.now() >= self.end_time else RUNNING


STEP_CLASSES = {
    'say': SayStep,
    'drive': DriveStep,
    'align': AlignStep,
    'recognize': RecognizeStep,
    'shoot_ring': ShootRingStep,
    'pause': PauseStep,
}


def build_step(spec):
    """根据 mission.yaml 里的一项创建对应的步骤对象（shoot_ar 按 target 分成旋转靶、移动靶两种）。"""
    if spec['type'] == 'shoot_ar':
        return ShootRotatingStep(spec) if spec['target'] == 'rotating' else ShootMovingStep(spec)
    return STEP_CLASSES[spec['type']](spec)
