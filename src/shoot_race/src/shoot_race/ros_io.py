# -*- coding: utf-8 -*-
"""RosIO：整个程序里唯一直接和 ROS 打交道的类。

职责：
  - 订阅传感器话题，只把最新数据（带时间戳）存起来，回调里不做任何决策；
  - 提供查询接口给上层（steps、preflight）；
  - 速度指令的唯一出口 send_velocity()；
  - 设置 AMCL 初始位姿；需要时调用 move_base。

上层代码只通过下面这些方法使用 ROS，因此离线测试时可以换成一个仿真版本（tests/sim_world.py 的 SimIO），
不需要真的 ROS 环境。

  时间      now(), wait_next_tick(), sleep(), is_shutdown()
  传感器    scan_points(), robot_pose(), wheel_speed(), ring_observations(),
            ar_pixels(), camera_frames(), topic_age()
  输出      send_velocity(), set_initial_pose()
  move_base move_base_ready(), move_base_go(), move_base_state(), move_base_cancel()
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import collections
import math
import threading

import numpy as np
import rospy
import tf
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan

from .geometry import scan_to_points


class RosIO(object):
    def __init__(self, sys_cfg):
        """sys_cfg：system.yaml 的内容。"""
        topics = sys_cfg['topics']
        self.frames = sys_cfg['frames']
        self.camera = sys_cfg['camera']                         # 相机内参 fx fy cx cy（AR 位置 → 像素）
        self.lock = threading.Lock()
        self.rate = rospy.Rate(sys_cfg['loop_hz'])

        # ---- 数据缓存（回调只往这里写）----
        self.last_receive = {}                                  # 话题 → 最后收到的时刻
        self.latest_scan = None
        self.latest_wheel_odom = None
        self.ring_buffer = collections.deque(maxlen=60)        # (接收时刻, 像素x, 模板编号)
        # 每个码最多存 600 帧（AR 约 25 Hz → 约 24 秒，移动靶找行程端点要看一个来回 13.5 秒）
        self.ar_buffer = collections.defaultdict(lambda: collections.deque(maxlen=600))
        self.image_buffer = collections.deque(maxlen=int(sys_cfg['image_buffer_size']))

        # ---- 订阅 ----
        rospy.Subscriber(topics['scan'], LaserScan, self._on_scan, queue_size=1)
        rospy.Subscriber(topics['wheel_odom'], Odometry, self._on_wheel_odom, queue_size=1)
        rospy.Subscriber(topics['ring'], Point, self._on_ring, queue_size=10)
        rospy.Subscriber(topics['ar'], AlvarMarkers, self._on_ar, queue_size=5)
        rospy.Subscriber(topics['camera'], Image, self._on_image, queue_size=1, buff_size=2 ** 24)

        # ---- 发布 ----
        self.cmd_pub = rospy.Publisher(topics['cmd_vel'], Twist, queue_size=1)
        self.initial_pose_pub = rospy.Publisher(topics['initial_pose'], PoseWithCovarianceStamped,
                                                queue_size=1, latch=True)
        self.tf_listener = tf.TransformListener()
        self.move_base_client = None      # 第一次用到 move_base 时才连接

    # ============================================================
    # 回调：只存数据
    # ============================================================
    def _mark(self, key):
        self.last_receive[key] = self.now()

    def _on_scan(self, msg):
        self.latest_scan = msg
        self._mark('scan')

    def _on_wheel_odom(self, msg):
        self.latest_wheel_odom = msg
        self._mark('wheel_odom')

    def _on_ring(self, msg):
        with self.lock:
            self.ring_buffer.append((self.now(), msg.x, int(round(msg.z))))
        self._mark('ring')

    def _on_ar(self, msg):
        # 时间戳优先级：每个码自己的 header（上游 ar_track_alvar 只设这个）→ 整条消息的 header → 接收时刻
        # 同一条消息里的码是同一帧拍的，时间戳要一样（旋转靶 WheelTracker 按时间戳把同一帧的码放在一起定圆心）
        array_stamp = msg.header.stamp.to_sec()
        fallback = array_stamp if array_stamp > 0 else self.now()
        with self.lock:
            for marker in msg.markers:
                stamp = marker.header.stamp.to_sec()
                if stamp <= 0:
                    stamp = fallback
                position = marker.pose.pose.position
                self.ar_buffer[marker.id].append((stamp, position.x, position.y, position.z))
        self._mark('ar')

    def _on_image(self, msg):
        with self.lock:
            self.image_buffer.append(msg)
        self._mark('camera')

    # ============================================================
    # 时间
    # ============================================================
    def now(self):
        return rospy.get_time()

    def wait_next_tick(self):
        """等到下一个控制周期（主循环频率由 system.yaml 的 loop_hz 决定）。"""
        self.rate.sleep()

    def sleep(self, seconds):
        """阻塞等待若干秒（开火间隔等少数地方用）。"""
        rospy.sleep(seconds)

    def is_shutdown(self):
        return rospy.is_shutdown()

    # ============================================================
    # 传感器查询
    # ============================================================
    def topic_age(self, key):
        """距离最后一次收到该话题过了多少秒；从没收到过返回 None。"""
        if key not in self.last_receive:
            return None
        return self.now() - self.last_receive[key]

    def scan_points(self):
        """返回 (angles, xs, ys)（激光坐标系），没有数据返回 None。"""
        scan = self.latest_scan
        if scan is None:
            return None
        return scan_to_points(scan.angle_min, scan.angle_increment, scan.ranges,
                              max(scan.range_min, 0.05), scan.range_max)

    def robot_pose(self):
        """机器人在地图中的位姿 (x, y, yaw)，来自 AMCL 发布的 TF；拿不到返回 None。"""
        try:
            trans, rot = self.tf_listener.lookupTransform(
                self.frames['map'], self.frames['base'], rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        yaw = tf.transformations.euler_from_quaternion(rot)[2]
        return trans[0], trans[1], yaw

    def odom_pose(self):
        """里程计坐标系下的位姿 (x, y, yaw)，来自 robot_pose_ekf（轮速计 + IMU）；拿不到返回 None。

        和 robot_pose() 的区别：AMCL 的 map 坐标会在定位更新时跳一下；
        环形靶的毫米级平移、转速测试这类短时间的相对运动，要用连续的 odom 位姿。
        """
        try:
            trans, rot = self.tf_listener.lookupTransform(
                self.frames.get('odom', 'odom'), self.frames['base'], rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        return trans[0], trans[1], tf.transformations.euler_from_quaternion(rot)[2]

    def odom_yaw(self):
        """里程计坐标系下的车身朝向（弧度）；拿不到返回 None。"""
        pose = self.odom_pose()
        return None if pose is None else pose[2]

    def wheel_speed(self):
        """轮速里程计的速度 (vx, vy, wz)。

        注意：/odom（robot_pose_ekf 融合后）只有位置没有速度，判断"车是否停稳"必须用 /wheel_odom。
        """
        msg = self.latest_wheel_odom
        if msg is None:
            return None
        twist = msg.twist.twist
        return twist.linear.x, twist.linear.y, twist.angular.z

    def ring_observations(self, max_age, accept_ids):
        """最近 max_age 秒内、模板编号在 accept_ids 里的靶心观测 [(t, 像素x), ...]。"""
        oldest = self.now() - max_age
        with self.lock:
            return [(t, x) for (t, x, obj_id) in self.ring_buffer
                    if t >= oldest and obj_id in accept_ids]

    def ar_pixels(self, marker_ids, window_s):
        """若干 AR 码最近 window_s 秒的像素位置 {码号: [(图像时间戳, u, v), ...]}，按时间排序。

        u = fx·x/z + cx，v = fy·y/z + cy（去畸变后的像素坐标）。
        不直接用 ar_track_alvar 给的"米"：车上 marker_size 参数和码的实际尺寸不一致，
        米制坐标整体放大约 2.9 倍（2026-10-16 实测深度读 4.3 m，实际约 1.5 m），但 x/z、y/z 不受影响。
        """
        oldest = self.now() - window_s
        cam = self.camera
        result = {}
        with self.lock:
            for marker_id in marker_ids:
                result[marker_id] = [(t, cam['fx'] * x / z + cam['cx'], cam['fy'] * y / z + cam['cy'])
                                     for (t, x, y, z) in self.ar_buffer[marker_id] if t >= oldest and z > 1e-6]
        return result

    def camera_frames(self, since):
        """since 时刻之后收到的全部图像，转换成 [(时间戳, BGR numpy 图像), ...]。"""
        with self.lock:
            messages = [m for m in self.image_buffer if m.header.stamp.to_sec() >= since]
        frames = []
        for msg in messages:
            image = image_msg_to_bgr(msg)
            if image is not None:
                frames.append((msg.header.stamp.to_sec(), image))
        return frames

    # ============================================================
    # 输出
    # ============================================================
    def send_velocity(self, command):
        """全程序唯一的底盘速度出口。"""
        twist = Twist()
        twist.linear.x = command.vx
        twist.linear.y = command.vy
        twist.angular.z = command.wz
        self.cmd_pub.publish(twist)

    def set_initial_pose(self, x, y, yaw, std_xy=0.05, std_yaw=math.radians(3.0)):
        """告诉 AMCL 机器人现在在哪（开局放在起点框里时调用）。"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frames['map']
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]
        covariance = [0.0] * 36
        covariance[0] = covariance[7] = std_xy ** 2
        covariance[35] = std_yaw ** 2
        msg.pose.covariance = covariance
        self.initial_pose_pub.publish(msg)

    # ============================================================
    # move_base（备用导航方式）
    # ============================================================
    def move_base_ready(self, timeout_s):
        import actionlib
        from move_base_msgs.msg import MoveBaseAction
        if self.move_base_client is None:
            self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        return self.move_base_client.wait_for_server(rospy.Duration(timeout_s))

    def move_base_go(self, x, y, yaw):
        from move_base_msgs.msg import MoveBaseGoal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frames['map']
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        self.move_base_client.send_goal(goal)

    def move_base_state(self):
        """'active' 进行中 / 'succeeded' 到达 / 'failed' 失败或被取消。"""
        from actionlib_msgs.msg import GoalStatus
        state = self.move_base_client.get_state()
        if state in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            return 'active'
        if state == GoalStatus.SUCCEEDED:
            return 'succeeded'
        return 'failed'

    def move_base_cancel(self):
        if self.move_base_client is not None:
            self.move_base_client.cancel_all_goals()


def image_msg_to_bgr(msg):
    """sensor_msgs/Image → BGR numpy 图像。直接按字节解析，不依赖 cv_bridge（Python2/3 都可用）。"""
    channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(msg.encoding)
    if channels is None:
        return None
    data = np.frombuffer(msg.data, dtype=np.uint8)
    image = data.reshape(msg.height, msg.step)[:, :msg.width * channels]
    image = image.reshape(msg.height, msg.width, channels)
    if msg.encoding == 'rgb8':
        return image[:, :, ::-1].copy()
    if msg.encoding == 'mono8':
        return np.repeat(image, 3, axis=2)
    return image.copy()
