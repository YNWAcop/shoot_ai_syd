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
from std_msgs.msg import String, Int32
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

# Serial port configuration
SERIAL_PORT = "/dev/shoot"
BAUD_RATE = 9600

# AR marker IDs for different areas
AR_MARKER_ID_AREA2 = 5  # Red target
AR_MARKER_ID_AREA3 = 6 # Rotating target

# Alignment thresholds
YAW_THRESHOLD_AREA2 = 0.025  # Yaw threshold for area 2
YAW_THRESHOLD_AREA3 = 0.02   # Yaw threshold for area 3

# AR marker Y range for valid shooting position
AR_Y_MIN = -0.50
AR_Y_MAX = AR_Y_MIN + 0.10

# Circular target alignment threshold
CIRCULAR_TARGET_ALIGNMENT_THRESHOLD = 0.5

# Image center X coordinate for circular target tracking
IMAGE_CENTER_X = 337


# Movement parameters
MOVE_SPEED = 0.10
POSITION_ADJUSTMENT_THRESHOLD = 0.02

# Shooting stabilization frames
SHOOT_FRAME_THRESHOLD_AREA2 = 3
SHOOT_FRAME_THRESHOLD_AREA3 = 3

# Position targets (laser distance based)
POSITION_CONFIGS = {
    0: {'left_target': 0.584, 'back_target': 1.257},
    1: {'right_target': 0.59, 'back_target': 1.283},
    2: {'right_target': 0.59, 'back_target': 1.330},
    3: {'right_target': 0.297, 'back_target': 0.320},
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

# ==================== Global Variables ====================

# Serial connection (will be initialized in main after rospy init)
serial_connection = None

# State machine
current_stage = 0  # 0: circular target, 1: transition, 2: AR area2, 3: AR area3, 4: complete

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

        if not (pushing_up or pushing_down):
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
        
        # Subscribers
        self.find_sub = rospy.Subscriber(
            '/object_position',
            Point,
            self.circular_target_callback
        )
        self.ar_sub = rospy.Subscriber(
            '/ar_pose_marker',
            AlvarMarkers,
            self.ar_marker_callback
        )
        self.scan_sub = rospy.Subscriber(
            '/scan',
            LaserScan,
            self.laser_scan_callback
        )
        
        # Move base client
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        
        # Timing
        self.last_time = rospy.get_time()
        self.start_time = rospy.get_time()
        
        # PI Controllers
        self.angle_controller_area2 = PIController(
            kp=1.40,
            ki=0.5,
            min_output=-1.5,
            max_output=1.5
        )
        self.angle_controller_area3 = PIController(
            kp=1.40,
            ki=0.5,
            min_output=-1.0,
            max_output=1.0
        )
        
        # AR marker alignment parameters
        self.ar_offset_x = 0.024
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
        return current_time, dt
    
    def reset_controllers(self):
        """Reset all PI controller integral terms."""
        self.angle_controller_area2.reset()
        self.angle_controller_area3.reset()
    
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
        max_iterations = 100
        
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
                    speed = 0.13
                elif position_id == 4:
                    speed = 0.18
                else:
                    speed = MOVE_SPEED
                msg.linear.x = speed if delta_x > 0 else -speed
            
            if position_id != 4:  # Only adjust Y if not position 4
                if abs(delta_y) < POSITION_ADJUSTMENT_THRESHOLD:
                    msg.linear.y = 0
                else:
                    # Determine Y-axis speed
                    if position_id == 3:
                        y_speed = 0.13
                    else:
                        y_speed = MOVE_SPEED
                    msg.linear.y = y_speed if delta_y > 0 else -y_speed
            else:
                msg.linear.y = 0
            
            # Stop if both axes are aligned
            if msg.linear.x == 0 and msg.linear.y == 0:
                break
            
            self.cmd_vel_pub.publish(msg)
            rospy.sleep(0.1)
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
        
        for marker in data.markers:
            # Area 2: Red target
            if marker.id == AR_MARKER_ID_AREA2 and current_stage == 2:
                self._process_ar_area2(marker)
            
            # Area 3: Rotating target
            elif marker.id == AR_MARKER_ID_AREA3 and current_stage == 3:
                self._process_ar_area3(marker)
    
    def _process_ar_area2(self, marker):
        """Process AR marker for area 2 (red target)."""
        global current_stage, last_target_time
        
        ar_x = marker.pose.pose.position.x
        ar_y = marker.pose.pose.position.y
        ar_x_error = abs(ar_x)
        
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
                    self.fire_shot(delay=0.08)
                    rospy.sleep(2)
                    
                    # Navigate to next positions
                    self.goto(NAVIGATION_GOALS[3])
                    self.move_to_position(4)
                    self.goto(NAVIGATION_GOALS[7])
                    current_stage = 3
                    self.goto(NAVIGATION_GOALS[4])
                    self.move_to_position(2)
                    
                    last_target_time = time.time()
                    time.sleep(1)
    
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
                
                # Reset controllers after shooting
                self.reset_controllers()
                rospy.sleep(2)
                
                # Navigate to final position
                self.goto(NAVIGATION_GOALS[5])
                self.move_to_position(3)
                rospy.sleep(1)
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

        # Check if target detected
        target_detected = (data.z == 31)

        rospy.loginfo("======== 圆形靶模式匹配调试 ========")
        rospy.loginfo("data.x = %.1f , data.z = %.1f", data.x, data.z)
        rospy.loginfo("target_detected = %s", target_detected)
        rospy.loginfo("current_stage = %d", current_stage)
        rospy.loginfo("offset = %.2f", target_offset)
        rospy.loginfo("time_since_last_target = %.2f", time_since_last_target)

        # =========================================================
        # Alignment phase
        # =========================================================
        if (offset_magnitude > CIRCULAR_TARGET_ALIGNMENT_THRESHOLD and
            target_detected and
            not circular_target_shot and
            current_stage == 0):

            msg = Twist()

            # P control
            angular_speed = -0.01 * target_offset

            # Limit angular velocity
            angular_speed = max(-0.3, min(0.3, angular_speed))

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
        elif ((offset_magnitude <= CIRCULAR_TARGET_ALIGNMENT_THRESHOLD and
               target_detected and
               not circular_target_shot and
               time_since_last_target > 2) and
              shoot_permission_flag == 1):

            rospy.loginfo("TARGET LOCKED - FIRE!")

            # Stop before firing
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)

            rospy.sleep(0.2)

            # Fire
            self.fire_shot(delay=0.07)

            rospy.loginfo("SHOT FIRED!")

            # Navigate to next positions
            self.goto(NAVIGATION_GOALS[1])

            self.move_to_position(4)

            self.goto(NAVIGATION_GOALS[6])

            current_stage = 1
            circular_target_shot = True

            self.goto(NAVIGATION_GOALS[2])

            self.move_to_position(1)

            rospy.sleep(1)

            last_target_time = time.time()

            current_stage = 2

            self.find_cb_executed = True

        # =========================================================
        # Target lost -> stop rotation
        # =========================================================
        elif not target_detected:

            msg = Twist()

            self.cmd_vel_pub.publish(msg)

            rospy.loginfo("Target lost, stop rotating")
    
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
        
        result = self.move_base.wait_for_result(rospy.Duration(60))
        
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


def main():
    """Main entry point."""
    global current_stage, last_target_time, shoot_permission_flag
    
    rospy.init_node('navigation_demo', anonymous=True)
    
    # Initialize serial connection after rospy init
    initialize_serial_connection()
    
    # Wait for user input to start
    print('Please enter 1 to continue: ')
    raw_input()
    
    # Control rate
    r = rospy.Rate(1)
    r.sleep()
    
    # Create navigation instance
    navi = NavigationDemo()
    rospy.loginfo("Competition started")
    rospy.sleep(1)
    
    # Navigate to first position
    navi.goto(NAVIGATION_GOALS[0])
    navi.move_to_position(0)
    rospy.sleep(1)
    
    # Initialize state
    last_target_time = time.time()
    shoot_permission_flag = 1
    current_stage = 0
    
    # Keep node running
    while not rospy.is_shutdown():
        r.sleep()


if __name__ == "__main__":
    main()