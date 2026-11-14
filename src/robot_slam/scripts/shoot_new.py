#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Robot Navigation and Shooting Control System
机器人导航和射击控制系统

This module provides navigation and shooting control for a ROS-based robot.
It integrates AR marker tracking, circular target detection, and precision shooting.
"""

import rospy
import math
import actionlib
import serial
import time
import sys
import os
from std_msgs.msg import String
from numbers import Integral
from race_config import validate_tuning
from keyboard_start import wait_for_start
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Twist
from tf_conversions import transformations
from math import pi
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from playsound import playsound
from sensor_msgs.msg import LaserScan
import datetime

# Note: Python 2 UTF-8 string handling
# This code uses byte strings (default in Python 2) for ASCII/UTF-8 data.
# If you need to handle unicode strings, use the u"" prefix (e.g., u"中文").
# The unsafe reload(sys).setdefaultencoding('utf-8') pattern is avoided.

# ==================== Configuration Constants ====================
TUNING = validate_tuning({})

# Serial port configuration
SERIAL_PORT = "/dev/shoot"
BAUD_RATE = 9600
"""------------------全局变量区---------------"""
music00_path = "/home/abot/LWJQCX/src/robot_slam/mp3/00.mp3"
music11_path = "/home/abot/LWJQCX/src/robot_slam/mp3/11.mp3"
music22_path = "/home/abot/LWJQCX/src/robot_slam/mp3/22.mp3"
music33_path = "/home/abot/LWJQCX/src/robot_slam/mp3/33.mp3"
music99_path = "/home/abot/LWJQCX/src/robot_slam/mp3/99.mp3"
musicst_path = "/home/abot/LWJQCX/src/robot_slam/mp3/33.mp3"
musiced_path = "/home/abot/LWJQCX/src/robot_slam/mp3/99.mp3"
# AR marker IDs for different areas
AR_MARKER_ID_AREA2 = 1  # Default for target_id_rotating (stage 2)
AR_MARKER_ID_AREA3 = 6  # Default for target_id_moving (stage 3)

# Alignment thresholds
YAW_THRESHOLD_AREA2 = 0.030# Yaw threshold for area 2
YAW_THRESHOLD_AREA3 = 0.02   # Yaw threshold for area 3

# AR marker Y range for valid shooting position
AR_Y_MIN = -0.480
AR_Y_MAX = AR_Y_MIN + 0.010

# Circular target alignment threshold
CIRCULAR_TARGET_ALIGNMENT_THRESHOLD = 0.5

# Image center X coordinate for circular target tracking
IMAGE_CENTER_X = 340


# Movement parameters
MOVE_SPEED = 0.13
POSITION_ADJUSTMENT_THRESHOLD = 0.02

# Shooting stabilization frames
SHOOT_FRAME_THRESHOLD_AREA2 = 0
SHOOT_FRAME_THRESHOLD_AREA3 = 3

# Position targets (laser distance based)
POSITION_CONFIGS = {
    0: {'left_target': 0.6, 'back_target': 1.282},
    1: {'right_target': 0.618, 'back_target': 1.282},
    2: {'right_target': 0.618,'back_target': 1.281},
    3: {'right_target': 0.218, 'back_target': 0.266},
    4: {'back_target': 0.40}
}

# Navigation goals (waypoints)
NAVIGATION_GOALS = [
    [1.00, -0.50, 0.0],   # Goal 0
    [0.17, -0.55, 0.0],   # Goal 1
    [0.91, -1.45, 0.0],   # Goal 2
    [0.17, -1.65, 0.0],   # Goal 3
    [0.98, -2.85, 0.0],   # Goal 4
    [0.40, -2.85, 0.0],   # Goal 5
    [0.25, -1.60, 0.0],   # Goal 6
    [0.25, -2.65, 0.0]    # Goal 7
]

# Serial commands
SHOOT_START_CMD = b'\x55\x01\x12\x00\x00\x00\x01\x69'
SHOOT_STOP_CMD = b'\x55\x01\x11\x00\x00\x00\x01\x68'

# PI 控制器单步 dt 上限（秒），防止长时间没收到帧后积分项一下顶满
MAX_CONTROL_DT = 0.2

# find_object 没检测到任何物体时发布的 z 值
NO_OBJECT_ID = 255

# ==================== Global Variables ====================

# Serial connection (will be initialized in main after rospy init)
serial_connection = None

# State machine
current_stage = -1

target_id_rotating = AR_MARKER_ID_AREA2
target_id_moving = AR_MARKER_ID_AREA3

# Laser scan distances
left_distance = float('inf')
front_distance = float('inf')
right_distance = float('inf')
back_distance = float('inf')

# Timing
last_target_time = 0

# Flags
circular_target_shot = False
shoot_permission_flag = 0


class PIController:
    """PI Controller with anti-windup for angle control."""
    
    def __init__(self, kp, ki, min_output, max_output):
        """
        Initialize PI controller.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            min_output: Minimum output limit
            max_output: Maximum output limit
        """
        self.kp = kp
        self.ki = ki
        self.min_output = min_output
        self.max_output = max_output
        self.integral = 0.0

    def update(self, error, dt):
        """
        Calculate control output based on error.
        
        Args:
            error: Current error value
            dt: Time step since last update
            
        Returns:
            Control output (clamped to min/max)
        """
        # Calculate unsaturated output
        unsat_output = self.kp * error + self.ki * self.integral

        # Clamp output
        output = max(self.min_output, min(self.max_output, unsat_output))

        # Anti-windup: only integrate if not saturated or error helps desaturate
        pushing_up = (output >= self.max_output) and (error > 0)
        pushing_down = (output <= self.min_output) and (error < 0)

        if self.ki > 0 and not (pushing_up or pushing_down):
            self.integral += error * dt
            # Clamp integral to prevent excessive accumulation
            self.integral = max(
                self.min_output / self.ki,
                min(self.max_output / self.ki, self.integral)
            )

        return output
    
    def reset(self):
        """Reset the integral term to zero."""
        self.integral = 0


class NavigationDemo:
    """Main navigation and shooting control class."""
    
    def __init__(self):
        """Initialize the navigation demo system."""
        # Callback execution flags
        self.find_cb_executed = False
        
        # Publishers
        self.set_pose_pub = rospy.Publisher(
            '/initialpose',
            PoseWithCovarianceStamped,
            queue_size=5
        )
        self.arrive_pub = rospy.Publisher(
            '/voiceWords',
            String,
            queue_size=10
        )
        self.cmd_vel_pub = rospy.Publisher(
            "/cmd_vel",
            Twist,
            queue_size=1000
        )
        
        # 进入 AR 阶段的时刻，早于这个时刻拍到的 AR 帧直接丢弃
        self.stage_start_stamp = rospy.Time(0)

        # Subscribers
        # queue_size=1：回调里有导航等长时间阻塞，只保留最新一帧，避免积压旧数据
        self.find_sub = rospy.Subscriber(
            '/object_position',
            Point,
            self.circular_target_callback,
            queue_size=1
        )
        self.ar_sub = rospy.Subscriber(
            '/ar_pose_marker',
            AlvarMarkers,
            self.ar_marker_callback,
            queue_size=1
        )
        self.scan_sub = rospy.Subscriber(
            '/scan',
            LaserScan,
            self.laser_scan_callback,
            queue_size=1
        )
        # Target IDs are fixed ROS parameters, not voice topic subscribers.
        # Move base client
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(TUNING['navigation']['server_timeout_s']))
        
        # Timing
        self.last_time = rospy.get_time()
        self.start_time = rospy.get_time()
        
        # PI Controllers
        self.angle_controller_area2 = PIController(
            kp=TUNING['ar']['stage2']['kp'],
            ki=TUNING['ar']['stage2']['ki'],
            min_output=-TUNING['ar']['stage2']['max_angular_rps'],
            max_output=TUNING['ar']['stage2']['max_angular_rps']
        )
        self.angle_controller_area3 = PIController(
            kp=TUNING['ar']['stage3']['kp'],
            ki=TUNING['ar']['stage3']['ki'],
            min_output=-TUNING['ar']['stage3']['max_angular_rps'],
            max_output=TUNING['ar']['stage3']['max_angular_rps']
        )
        
        # AR marker alignment parameters
        self.ar_offset_x = TUNING['ar']['offset_x_m']
        self.consecutive_aligned_frames = 0
    def update_time(self):
        """
        Update timing information.
        
        Returns:
            Tuple of (current_time, delta_time)
        """
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        self.last_time = current_time
        # 两帧间隔可能很长（导航途中、靶子出画面），不限幅的话积分项一次就会顶满
        dt = max(0.0, min(dt, MAX_CONTROL_DT))
        return current_time, dt

    def reset_controllers(self):
        """Reset all PI controller integral terms."""
        self.angle_controller_area2.reset()
        self.angle_controller_area3.reset()

    def enter_ar_stage(self, stage):
        """进入 AR 靶阶段：清空积分和计数，记录起始时刻，再切换阶段。"""
        global current_stage
        self.stage_start_stamp = rospy.Time.now()
        self.reset_controllers()
        self.consecutive_aligned_frames = 0
        self.last_time = rospy.get_time()
        current_stage = stage
        rospy.loginfo("Enter stage %d", stage)
    
    def laser_scan_callback(self, data):
        """
        Process laser scan data to calculate obstacle distances.
        
        Args:
            data: LaserScan message
        """
        global left_distance, front_distance, right_distance, back_distance
        
        # Left: average of 2nd to 6th smallest values
        left_ranges = sorted(data.ranges[520:561])
        left_distance = sum(left_ranges[1:6]) / 5
        
        # Front: average of 2nd to 6th smallest values
        front_ranges = sorted(data.ranges[350:371])
        front_distance = sum(front_ranges[1:6]) / 5
        
        # Right: average of 2nd to 6th smallest values
        right_ranges = sorted(data.ranges[150:220])
        right_distance = sum(right_ranges[1:6]) / 5
        
        # Back: average of 7th to 11th smallest values (wider filter)
        back_ranges = sorted(data.ranges[709:719] + data.ranges[0:19])
        back_distance = sum(back_ranges[6:11]) / 5
    
    def move_to_position(self, position_id):
        """
        Fine-tune robot position using laser scan data.
        
        Args:
            position_id: Position configuration ID (0-4)
        """
        if position_id not in POSITION_CONFIGS:
            rospy.logwarn("Invalid position_id: %d", position_id)
            return
        
        config = POSITION_CONFIGS[position_id]
        iteration_count = 0
        max_iterations = TUNING['positioning']['max_iterations']
        
        msg = Twist()
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        while iteration_count < max_iterations:
            # Calculate position errors
            if position_id in [0]:  # Left-based positioning
                delta_y = left_distance - config['left_target']
                delta_x = config['back_target'] - back_distance
            elif position_id in [1, 2, 3]:  # Right-based positioning
                delta_y = -(right_distance - config['right_target'])
                delta_x = config['back_target'] - back_distance
            elif position_id == 4:  # Back-only positioning
                delta_y = 0
                delta_x = config['back_target'] - back_distance
            else:
                break
            
            # Set movement speeds based on errors
            if abs(delta_x) < POSITION_ADJUSTMENT_THRESHOLD:
                msg.linear.x = 0
            else:
                # Determine speed based on position
                if position_id == 3:
                    speed = TUNING['positioning']['terminal_speed_mps']
                elif position_id == 4:
                    speed = TUNING['positioning']['back_only_speed_mps']
                else:
                    speed = MOVE_SPEED
                msg.linear.x = speed if delta_x > 0 else -speed
            
            if position_id != 4:  # Only adjust Y if not position 4
                if abs(delta_y) < POSITION_ADJUSTMENT_THRESHOLD:
                    msg.linear.y = 0
                else:
                    # Determine Y-axis speed
                    if position_id == 3:
                        y_speed = TUNING['positioning']['terminal_speed_mps']
                    else:
                        y_speed = MOVE_SPEED
                    msg.linear.y = y_speed if delta_y > 0 else -y_speed
            else:
                msg.linear.y = 0
            
            # Stop if both axes are aligned
            if msg.linear.x == 0 and msg.linear.y == 0:
                break
            
            self.cmd_vel_pub.publish(msg)
            rospy.sleep(TUNING['positioning']['interval_s'])
            iteration_count += 1
    
    def fire_shot(self, delay=0.08):
        """
        Execute shooting sequence.
        
        Args:
            delay: Delay between start and stop commands (seconds)
        """
        if serial_connection and serial_connection.is_open:
            serial_connection.write(SHOOT_START_CMD)
            rospy.loginfo("Shooting started")
            time.sleep(delay)
            serial_connection.write(SHOOT_STOP_CMD)
            rospy.loginfo("Shooting stopped")
        else:
            rospy.logwarn("Serial connection not available for shooting")
    
    def ar_marker_callback(self, data):
        """
        Handle AR marker detection and alignment.
        
        Args:
            data: AlvarMarkers message
        """
        global current_stage, last_target_time

        # 丢弃进入当前阶段之前拍到的帧（导航途中的旧画面）
        stamp = data.header.stamp
        if not stamp.is_zero() and stamp < self.stage_start_stamp:
            return

        stage_for_frame = current_stage
        for marker in data.markers:
            # A blocking callback may finish a stage; the same image must not
            # start processing targets belonging to the newly entered stage.
            if current_stage != stage_for_frame:
                return
            # Area 2: Red target
            if marker.id == target_id_rotating and current_stage == 2:
                self._process_ar_area2(marker)
            
            # Area 3: Rotating target
            elif marker.id == target_id_moving and current_stage == 3:
                self._process_ar_area3(marker)
    
    def _process_ar_area2(self, marker):
        """Process AR marker for area 2 (red target)."""
        global current_stage, last_target_time
        
        ar_x = marker.pose.pose.position.x
        ar_y = marker.pose.pose.position.y

        rospy.loginfo("【红靶锁定中】当前 X(左右偏置): %.3f 米, 当前 Y(绝对高度): %.3f 米", ar_x, ar_y)

       #ar_x_error = abs(ar_x)
        ar_x_error = abs(ar_x - self.ar_offset_x)
        
        current_time, dt = self.update_time()
        
        # Check if alignment is needed
        if ar_x_error >= YAW_THRESHOLD_AREA2:
            self.consecutive_aligned_frames = 0
            
            angular_z = self.angle_controller_area2.update(
                ar_x - self.ar_offset_x,
                dt
            )
            
            msg = Twist()
            msg.angular.z = -angular_z
            self.cmd_vel_pub.publish(msg)
        else:
            # Check if in valid Y range
            if AR_Y_MAX >= ar_y >= AR_Y_MIN:
                self.consecutive_aligned_frames += 1
                rospy.loginfo(
                    "Area2 aligned (%d/%d frames)",
                    self.consecutive_aligned_frames,
                    SHOOT_FRAME_THRESHOLD_AREA2 + 1
                )
                
                if self.consecutive_aligned_frames >= SHOOT_FRAME_THRESHOLD_AREA2 + 1:
                    rospy.loginfo("Area2: Firing!")
                   #self.fire_shot(delay=0.08)
                    # 第一发
                    self.fire_shot(delay=0.08)
                    rospy.loginfo("红靶第一发连射！")
                    
                    # ⚠️ 注意：这里的休眠时间比圆靶短，原因看下面分析
                    # rospy.sleep(1.1)  
                    
                    # # 第二发
                    # self.fire_shot(delay=0.08)
                    # rospy.loginfo("红靶第二发连射！")


                    rospy.sleep(2)
                    
                    # Navigate to next positions
                    self.goto(NAVIGATION_GOALS[3])
                    self.move_to_position(4)
                    self.goto(NAVIGATION_GOALS[7])
                    self.goto(NAVIGATION_GOALS[4])
                    self.move_to_position(2)

                    last_target_time = time.time()
                    time.sleep(TUNING['timing']['post_position_settle_s'])
                    # 到位并停稳后才进入移动靶阶段
                    self.enter_ar_stage(3)
    
    def _process_ar_area3(self, marker):
        """Process AR marker for area 3 (rotating target)."""
        global current_stage
        
        ar_x = marker.pose.pose.position.x
        ar_y = marker.pose.pose.position.y
        ar_x_error = abs(ar_x - self.ar_offset_x)
        
        current_time, dt = self.update_time()
        
        # Check if alignment is needed
        if ar_x_error >= YAW_THRESHOLD_AREA3:
            self.consecutive_aligned_frames = 0
            
            angular_z = self.angle_controller_area3.update(
                ar_x - self.ar_offset_x,
                dt
            )
            
            msg = Twist()
            msg.angular.z = -angular_z
            self.cmd_vel_pub.publish(msg)
        else:
            # Aligned - accumulate frames
            self.consecutive_aligned_frames += 1
            rospy.loginfo(
                "Area3 aligned (%d/%d frames)",
                self.consecutive_aligned_frames,
                SHOOT_FRAME_THRESHOLD_AREA3
            )
            
            if self.consecutive_aligned_frames >= SHOOT_FRAME_THRESHOLD_AREA3:
                rospy.loginfo("Area3: Firing!")
                self.fire_shot(delay=0.1)
                # rospy.sleep(1.1)  
                    
                #     # 第二发
                # self.fire_shot(delay=0.1)
                # Reset controllers after shooting
                self.reset_controllers()
                rospy.sleep(2)
                
                # Navigate to final position
                self.goto(NAVIGATION_GOALS[5])
                self.move_to_position(3)
                rospy.sleep(TUNING['timing']['post_position_settle_s'])
                current_stage = 4
    
    def circular_target_callback(self, data):
        """
        Handle circular target detection and shooting.

        Args:
            data: Point message with target position
        """

        global current_stage
        global last_target_time
        global shoot_permission_flag
        global circular_target_shot

        if self.find_cb_executed:
            return

        # Calculate offset from image center
        target_offset = data.x - IMAGE_CENTER_X
        offset_magnitude = abs(target_offset)

        current_time = time.time()
        time_since_last_target = current_time - last_target_time

        # 只有到达环形靶位置、允许射击之后才处理；
        # 启动前和开往第一个点的路上都不发任何速度指令，避免和 move_base 抢 /cmd_vel
        if (current_stage != 0 or shoot_permission_flag != 1 or
                circular_target_shot):
            return

        # Check if target detected
        object_id = int(round(data.z))
        target_detected = (object_id == TUNING['circular']['template_id'])

        #rospy.loginfo("======== 圆形靶模式匹配调试 ========")
        #rospy.loginfo("data.x = %.1f , data.z = %.1f", data.x, data.z)
        #rospy.loginfo("target_detected = %s", target_detected)
        #rospy.loginfo("current_stage = %d", current_stage)
        #rospy.loginfo("offset = %.2f", target_offset)
        #rospy.loginfo("time_since_last_target = %.2f", time_since_last_target)

        # =========================================================
        # Timeout fallback: 长时间锁不定就直接打，保证后面的流程能继续
        # =========================================================
        fallback_s = TUNING['circular']['fallback_fire_s']
        if fallback_s > 0 and time_since_last_target > fallback_s:
            rospy.logwarn("Circular target not locked within %.1f s, firing anyway",
                          fallback_s)
            self._shoot_circular_and_leave()
            return

        # =========================================================
        # Target lost / other template
        # =========================================================
        if not target_detected:
            if object_id == NO_OBJECT_ID:
                # 画面里没有任何模板：停止转动
                self.cmd_vel_pub.publish(Twist())
                rospy.loginfo("Target lost, stop rotating")
            # 匹配到的是别的模板编号：忽略这一条，不发停车
            return

        # =========================================================
        # Alignment phase
        # =========================================================
        if offset_magnitude > CIRCULAR_TARGET_ALIGNMENT_THRESHOLD:

            msg = Twist()

            # P control
            angular_speed = -TUNING['circular']['kp_rps_per_px'] * target_offset

            # Limit angular velocity
            limit = TUNING['circular']['max_angular_rps']
            angular_speed = max(-limit, min(limit, angular_speed))

            msg.angular.z = angular_speed

            rospy.loginfo(
                "TRACKING TARGET: offset=%.2f angular_z=%.2f",
                target_offset,
                angular_speed
            )

            self.cmd_vel_pub.publish(msg)

        # =========================================================
        # Shooting phase
        # =========================================================
        elif time_since_last_target > TUNING['circular']['lock_delay_s']:

            rospy.loginfo("TARGET LOCKED - FIRE!")
            self._shoot_circular_and_leave()

    def _shoot_circular_and_leave(self):
        """环形靶：停车、打两发，然后开到旋转靶位置并进入 stage 2。"""
        global current_stage
        global last_target_time
        global circular_target_shot

        # 先置标志，保证这一段只执行一次
        self.find_cb_executed = True
        circular_target_shot = True
        current_stage = 1

        # Stop before firing
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        rospy.sleep(0.2)

        # Fire
        self.fire_shot(delay=0.07)      # 第一发
        rospy.loginfo("第一发连射完成！")

        rospy.sleep(1.2)                # ⚠️ 极其关键的“硬件恢复时间”

        self.fire_shot(delay=0.07)      # 第二发
        rospy.loginfo("第二发连射完成！")

        # Navigate to next positions
        self.goto(NAVIGATION_GOALS[1])

        self.move_to_position(4)

        self.goto(NAVIGATION_GOALS[6])

        self.goto(NAVIGATION_GOALS[2])

        self.move_to_position(1)

        rospy.sleep(TUNING['timing']['post_position_settle_s'])

        last_target_time = time.time()

        # 到位并停稳后才进入旋转靶阶段
        self.enter_ar_stage(2)

    def set_pose(self, p):
        """
        Set robot initial pose.
        
        Args:
            p: Tuple of (x, y, theta_degrees)
            
        Returns:
            True if successful
        """
        if self.move_base is None:
            return False
        
        x, y, th = p
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        
        q = transformations.quaternion_from_euler(0.0, 0.0, th / 180.0 * pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]
        
        self.set_pose_pub.publish(pose)
        return True
    
    def _done_callback(self, status, result):
        """Navigation completion callback."""
        rospy.loginfo("Navigation done! status:%d result:%s", status, result)
    
    def _active_callback(self):
        """Navigation activation callback."""
        rospy.loginfo("[Navi] Navigation has been activated")
    
    def _feedback_callback(self, feedback):
        """Navigation feedback callback."""
        pass
    
    def goto(self, p):
        """
        Navigate to a goal position.
        
        Args:
            p: List [x, y, theta_degrees]
            
        Returns:
            True if navigation succeeded
        """
        rospy.loginfo("[Navi] Going to %s", p)
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2] / 180.0 * pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        
        self.move_base.send_goal(
            goal,
            self._done_callback,
            self._active_callback,
            self._feedback_callback
        )
        
        result = self.move_base.wait_for_result(rospy.Duration(TUNING['navigation']['goal_timeout_s']))
        
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Reached goal %s successfully!", p)
        
        return True
    
    def cancel(self):
        """Cancel all navigation goals."""
        self.move_base.cancel_all_goals()
        return True


def initialize_serial_connection():
    """
    Initialize serial connection for shooting control.
    
    Returns:
        Serial connection object or None if failed
    """
    global serial_connection
    
    try:
        serial_connection = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            parity="N",
            bytesize=8,
            stopbits=1
        )
        rospy.loginfo("Serial connection established to %s at %d baud", SERIAL_PORT, BAUD_RATE)
        return serial_connection
    except serial.SerialException as e:
        rospy.logerr("Serial port error: %s. Check if %s is available and accessible.", str(e), SERIAL_PORT)
        serial_connection = None
        return None
    except OSError as e:
        rospy.logerr("OS error accessing serial port: %s. Check device permissions.", str(e))
        serial_connection = None
        return None
    except Exception as e:
        rospy.logerr("Unexpected error initializing serial connection: %s", str(e))
        serial_connection = None
        return None


def load_tuning_config():
    """Load common tuning settings once, before constructing hardware nodes."""
    global TUNING, NAVIGATION_GOALS, POSITION_CONFIGS, MOVE_SPEED
    global POSITION_ADJUSTMENT_THRESHOLD, YAW_THRESHOLD_AREA2, YAW_THRESHOLD_AREA3
    global AR_Y_MIN, AR_Y_MAX, SHOOT_FRAME_THRESHOLD_AREA2, SHOOT_FRAME_THRESHOLD_AREA3
    global IMAGE_CENTER_X, CIRCULAR_TARGET_ALIGNMENT_THRESHOLD
    settings = validate_tuning(rospy.get_param('~tuning', {}))
    TUNING = settings
    NAVIGATION_GOALS = settings['navigation']['goals']
    POSITION_CONFIGS = dict(enumerate(settings['positioning']['configs']))
    MOVE_SPEED = settings['positioning']['move_speed_mps']
    POSITION_ADJUSTMENT_THRESHOLD = settings['positioning']['tolerance_m']
    YAW_THRESHOLD_AREA2 = settings['ar']['stage2']['x_tolerance_m']
    YAW_THRESHOLD_AREA3 = settings['ar']['stage3']['x_tolerance_m']
    AR_Y_MIN = settings['ar']['stage2']['y_min_m']
    AR_Y_MAX = settings['ar']['stage2']['y_max_m']
    SHOOT_FRAME_THRESHOLD_AREA2 = settings['ar']['stage2']['frames_required'] - 1
    SHOOT_FRAME_THRESHOLD_AREA3 = settings['ar']['stage3']['frames_required']
    IMAGE_CENTER_X = settings['circular']['image_center_x_px']
    CIRCULAR_TARGET_ALIGNMENT_THRESHOLD = settings['circular']['alignment_tolerance_px']
    rospy.loginfo('Tuning configuration loaded from ~tuning')


def load_target_ids():
    """Read fixed target IDs from private ROS parameters before using hardware."""
    global target_id_rotating, target_id_moving

    rotating = rospy.get_param('~target_id_rotating', AR_MARKER_ID_AREA2)
    moving = rospy.get_param('~target_id_moving', AR_MARKER_ID_AREA3)
    if (isinstance(rotating, bool) or not isinstance(rotating, Integral)
            or rotating not in (1, 2, 3, 4, 5)):
        raise ValueError('target_id_rotating must be an integer from 1 to 5')
    if (isinstance(moving, bool) or not isinstance(moving, Integral)
            or moving not in (6, 7, 8)):
        raise ValueError('target_id_moving must be an integer from 6 to 8')

    target_id_rotating = int(rotating)
    target_id_moving = int(moving)
    rospy.loginfo('Configured target IDs: rotating=%d moving=%d',
                  target_id_rotating, target_id_moving)




def main():
    """Main entry point."""
    global current_stage, last_target_time, shoot_permission_flag
    
    rospy.init_node('navigation_demo', anonymous=True)

    try:
        load_target_ids()
        load_tuning_config()
    except ValueError as error:
        rospy.logerr('Invalid competition configuration: %s', str(error))
        return

    initialize_serial_connection()

    r = rospy.Rate(1)

# 先创建 navi（会等 move_base 就绪）；current_stage=-1，等待键盘输入 1。
    navi = NavigationDemo()

    rospy.loginfo(
        "Target ids ready: rotating=%d moving=%d",
        target_id_rotating,
        target_id_moving
    )

    if not wait_for_start(rospy.is_shutdown, rospy.loginfo, rospy.logerr):
        return

    # 输入 1 之后的等待时间，沿用原来语音启动后的延时参数
    rospy.sleep(TUNING['timing']['start_delay_s'])
    if rospy.is_shutdown():
        return

# 开始导航
    navi.goto(NAVIGATION_GOALS[0])
    navi.move_to_position(0)
    rospy.sleep(TUNING['timing']['post_position_settle_s'])
    if rospy.is_shutdown():
        return

# 到达环形靶位置并停稳后，才允许圆盘识别/射击流程工作
    last_target_time = time.time()
    shoot_permission_flag = 1
    current_stage = 0

    # Keep node running
    while not rospy.is_shutdown():
        r.sleep()


if __name__ == "__main__":
    main()
