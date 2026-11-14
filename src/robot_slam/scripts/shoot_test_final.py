#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import math
import actionlib
import serial
import time
from std_msgs.msg import String
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Twist
from tf_conversions import transformations
from math import pi
from std_msgs.msg import Int32
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import os
from playsound import playsound
from sensor_msgs.msg import LaserScan
import datetime

time2=0
# 设置串口和波特率
serialPort = "/dev/shoot"
baudRate = 9600
# 打开串口
ser = serial.Serial(port=serialPort, baudrate=baudRate, parity="N", bytesize=8, stopbits=1)
id = 255


# 圆盘中心点与图像中心的偏差
flog0 = 255
# 上述误差的绝对值
flog1 = 255
# 完成第一次射击，减为254
flog2 = 255
#
flog3 = 255
#
flog4 = 255
#
count = 0
#
# time = 0
#
move_flog = 0
# 二维码中心误差阈值
Yaw_th = 0.013
#Yaw_th3 = 0.0018 #05 18  23
Yaw_th3 = 0.013

ar_id_area2 = 2
ar_id_area3 = 6
# 二维码中心y方向范围
Min_y = -0.65  #-0。6
Max_y =-0.55 #-0。5
#
ar_flog = 255
#
case = 255
#
case1 = 255
case2 = 255
case3 = 255
#target_id_rotating = 255
#target_id_moving = 255

left_distance=float('inf')
front_distance=float('inf')
right_distance=float('inf')
back_distance=float('inf')
flag = 0
openspeed=0.10
adjustdistence=0.02
last_target_time = 0

class PIController:
    def __init__(self, kp, ki, min_output, max_output):
        self.kp = kp  
        self.ki = ki  
        self.min_output = min_output  
        self.max_output = max_output  
        self.integral = 0.0  

    def update(self, error, dt):
        # 计算当前未限幅的输出
        unsat_output = self.kp * error + self.ki * self.integral

        # 限幅输出
        output = max(self.min_output, min(self.max_output, unsat_output))

        # 判断是否饱和且误差方向推动饱和
        pushing_up = (output >= self.max_output) and (error > 0)
        pushing_down = (output <= self.min_output) and (error < 0)

        # 条件积分：只有在未饱和或误差有助于脱离饱和时才积分
        if not (pushing_up or pushing_down):
            self.integral += error * dt
            # 积分限幅（防止积分项过大）
            self.integral = max(self.min_output / self.ki, min(self.max_output / self.ki, self.integral))

        return output
    
    def reset(self):
        # 重置积分项为0
        self.integral = 0

class navigation_demo:
    def __init__(self):
        # 发布初始位置
        self.find_cb_executed = False
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        # 发布到达目标点的信息
        self.arrive_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
        # 发布速度指令
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)
        # 订阅圆盘模式识别结果 ，回调函数为find_cb
        self.find_sub = rospy.Subscriber('/object_position', Point, self.find_cb);
        # 订阅AR二维码消息，回调函数为ar_cb
        self.ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_cb);
        # move_base客户端，用于导航
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # 等待动作服务器60s内启动
        self.move_base.wait_for_server(rospy.Duration(60))
        #self.target_id_rotating_sub = rospy.Subscriber('target_id_rotating', Int32, self.target_id_rotating_callback)
        #self.target_id_moving_sub = rospy.Subscriber('target_id_moving', Int32, self.target_id_moving_callback)
        # 创建一个发布者，用于发布Twist类型的消息到/cmd_vel话题
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)  # 订阅激光雷达数据
        
        
        self.last_time = rospy.get_time()  # 初始化last_time
        self.start_time = rospy.get_time()  # 初始化last_target_time
        self.angle_controller      = PIController(kp=1.20, ki=0.5,  min_output=-3.0, max_output=3.0)
        self.angle_controller2     = PIController(kp=1.7, ki=0,  min_output=-1, max_output=1)
        self.ar_offset_x = 0.06

            # 时间更新方法
    def update_time(self):
        current_time = rospy.get_time()
        dt = current_time - self.last_time  # 计算时间差
        self.last_time = current_time  # 更新last_time
        return current_time, dt
    
    # 在任务切换或者完成后调用reset()来清空积分项
    def reset_integrals(self):
        self.angle_controller.reset()  
        #self.angle_controller_find.reset()
        self.angle_controller2.reset()

    #使机器人以给定的速度向后移动
    def end(self):
        global time
        msg = Twist()
        msg.linear.x = -0.4 # -0.3
        msg.linear.y = -0.25 # -0.3
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        while (time <= 11): #15
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time = time + 1 

    def scan_cb(self, data):
        global left_distance
        global front_distance  
        global right_distance
        global back_distance
        
        # changdu = len(data.ranges)    
        # print("length=", changdu)

                # AR靶的射击稳定帧阈值
        self.shoot_frame_threshold = 1  # 阈值
        self.consecutive_aligned_frames = 0 # 射击帧计数器
    
        # 处理左边的距离：取倒数第六小到倒数第二小的平均值
        left_ranges = sorted(data.ranges[520:561])
        left_distance = sum(left_ranges[1:6]) / 5

        # 处理前方的距离：取倒数第六小到倒数第二小的平均值
        front_ranges = sorted(data.ranges[350:371])
        front_distance = sum(front_ranges[1:6]) / 5
    
        # 处理右边的距离：取倒数第六小到倒数第二小的平均值
        right_ranges = sorted(data.ranges[150:220])
        right_distance = sum(right_ranges[1:6]) / 5

        # 处理后方的距离：取倒数第六小到倒数第二小的平均值
        back_ranges = sorted(data.ranges[709:719]+data.ranges[0:19])
        back_distance = sum(back_ranges[6:11]) / 5

        # 打印各方向的距离（根据需要可以删除此行）
        #print("front=", front_distance)
        #print("left=", left_distance)
        #print("right=", right_distance)
        #print("back=", back_distance)

    # 控制机器人移动的回调函数
    def move_cb(self, position):
        global time2
        global openspeed
        global adjustdistence
        time2 = 0
        msg = Twist()
        if position ==0:
            delta_y = left_distance - 0.584                 #0.668pian you
            #delta_x = -(back_distance - 1.323)
	    #delta_x = -(back_distance - 1.217)
            delta_x = 1.257 - back_distance 
            msg.linear.x = openspeed if delta_x > 0 else -openspeed
            msg.linear.y = openspeed if delta_y > 0 else -openspeed
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0

            while time2 < 100 and (msg.linear.x != 0 or msg.linear.y != 0):
                delta_y = left_distance - 0.584      #0.588
                #delta_x = -(back_distance - 1.323)
                delta_x = 1.257 - back_distance 
                if abs(delta_x) < adjustdistence: # 偏差小于2cm，停止X方向运动
                    msg.linear.x = 0
                else :
                    msg.linear.x = openspeed if delta_x > 0 else -openspeed
                if abs(delta_y) < adjustdistence: # 偏差小于2cm，停止Y方向运动
                    msg.linear.y = 0
                else:
                    msg.linear.y = openspeed if delta_y > 0 else -openspeed
                self.pub.publish(msg)
                rospy.sleep(0.1)
                time2 += 1
        if position ==1:
            delta_y = -(right_distance - 0.6)        ##0.49      0.39pian you
            #delta_x = -(back_distance - 1.286)
            delta_x = 1.283 - back_distance 
            msg.linear.x = openspeed if delta_x > 0 else -openspeed
            msg.linear.y = openspeed if delta_y > 0 else -openspeed
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0

            while time2 < 100 and (msg.linear.x != 0 or msg.linear.y != 0):
                print("22222222")
                delta_y = -(right_distance - 0.6)
                #delta_x = -(back_distance - 1.306)
                delta_x = 1.283 - back_distance
                if abs(delta_x) < adjustdistence: # 偏差小于2cm，停止X方向运动
                    msg.linear.x = 0
                else :
                    msg.linear.x = openspeed if delta_x > 0 else -openspeed
                if abs(delta_y) < adjustdistence: # 偏差小于2cm，停止Y方向运动
                    msg.linear.y = 0
                else:
                    msg.linear.y = openspeed if delta_y > 0 else -openspeed
                self.pub.publish(msg)
                rospy.sleep(0.1)
                time2 += 1
        if position ==2:
            delta_y = -(right_distance - 0.59)
            #0.571
            #delta_x = -(back_distance - 1.30)
            delta_x = 1.330 - back_distance
            msg.linear.x = openspeed if delta_x > 0 else -openspeed
            msg.linear.y = openspeed if delta_y > 0 else -openspeed
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0

            while time2 < 100 and (msg.linear.x != 0 or msg.linear.y != 0):
                delta_y = -(right_distance - 0.59)
                #delta_x = -(back_distance - 1.30)
                delta_x = 1.330 - back_distance
                if abs(delta_x) < adjustdistence: # 偏差小于2cm，停止X方向运动
                    msg.linear.x = 0
                else :
                    msg.linear.x = openspeed if delta_x > 0 else -openspeed
                if abs(delta_y) < adjustdistence: # 偏差小于2cm，停止Y方向运动
                    msg.linear.y = 0
                else:
                    msg.linear.y = openspeed if delta_y > 0 else -openspeed
                self.pub.publish(msg)
                rospy.sleep(0.10)
                time2 += 1
        if position ==3:
            delta_y = -(right_distance - 0.277)
            delta_x = 0.32 - back_distance
            msg.linear.x = 0.13 if delta_x > 0 else -0.13
            msg.linear.y = 0.13 if delta_y > 0 else -0.13
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0

            while time2 < 100 and (msg.linear.x != 0 or msg.linear.y != 0):
                delta_y = -(right_distance - 0.297)
                delta_x = 0.320 - back_distance
                if abs(delta_x) < adjustdistence: # 偏差小于2cm，停止X方向运动
                    msg.linear.x = 0
                if abs(delta_y) < adjustdistence: # 偏差小于2cm，停止Y方向运动
                    msg.linear.y = 0
                self.pub.publish(msg)
                rospy.sleep(0.1)
                time2 += 1
        if position ==4:
            delta_x = 0.40 - back_distance
            msg.linear.x = 0.18 if delta_x > 0 else -0.18
            msg.linear.y = 0.0 
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0

            while time2 < 100 and (msg.linear.x != 0 or msg.linear.y != 0):
                delta_x = -(back_distance - 0.40)
                if abs(delta_x) < adjustdistence: # 偏差小于2cm，停止X方向运动
                    msg.linear.x = 0
                self.pub.publish(msg)
                rospy.sleep(0.1)
                time2 += 1

    # AR二维码标记消息的回调函数
    def ar_cb(self, data):
        global flog,qr_x,msg,ar_flog,ar_x,point_msg,ar_x_abs,flog4,ar_x_0_abs,ar_y_0,Min_y,Max_y,case,ar_id_area2 ,ar_id_area3,last_target_time
        # 获取所有AR标记
        ar_markers = data
        # 遍历接收到的所有AR标记
        for marker in data.markers:
            # 如果AR标记的ID为0
            if marker.id == ar_id_area2 and case == 2:#目前红点是2
                # 获取AR标记的x,y坐标
                ar_x_0 = marker.pose.pose.position.x
                ar_y_0 = marker.pose.pose.position.y
                #print("x=",ar_x_0)
                #print("y=",ar_y_0)
                # 计算AR标记x坐标的绝对值
                ar_x_0_abs = abs(ar_x_0)
                #current_time = time.time()##########################################################
                # 如果AR标记的x坐标绝对值大于等于Yaw阈值
                if (ar_x_0_abs >= Yaw_th) :#or (current_time - last_target_time < 2):
                    # 创建一个Twist消息
                    msg = Twist()
                    # 设置消息的角速度为AR标记x坐标的相反值（*-1）
                    msg.angular.z = -1.42* ar_x_0 #                 1.9                                1.2 1.3
                    #print(angular_z)
                    # 发布Twist消息
                    self.pub.publish(msg)
                else:
                    if Max_y >= ar_y_0 >= Min_y:
                        ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                        print('打印射击')
                        # 等待0.1秒
                        time.sleep(0.08)
                        # 发送停止射击指令
                        ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                        rospy.sleep(2)
                        self.goto(goals[3])
                        navi.move_cb(4)
                        self.goto(goals[7])
                        case = 3
                        #rospy.sleep(1)
                        self.goto(goals[4])
                        navi.move_cb(2)
                        #os.system('mplayer %s' % music33_path)
                        last_target_time = time.time()
                        rospy.sleep(1)
            if marker.id == ar_id_area3 and case == 3:#从左到右768
                # 获取AR标记的x,y坐标
                ar_x_0 = marker.pose.pose.position.x
                ar_y_0 = marker.pose.pose.position.y
                #print("x=",ar_x_0)
                #print("y=",ar_y_0)
                # 计算AR标记x坐标的绝对值
                ar_x_0_abs = abs(ar_x_0- self.ar_offset_x)
                current_time,dt = self.update_time()
                # 如果AR标记的x坐标绝对值大于等于Yaw阈值
                if (ar_x_0_abs >= Yaw_th3):

                    self.consecutive_aligned_frames = 0

                    print("ar_x_0_abs=",ar_x_0_abs  )


#                    angular_z = self.angle_controller2.update(ar_x_0, dt)
                    angular_z=1.8*(ar_x_0- self.ar_offset_x)

#                    if angular_z>=0.5:
#                        anglar_z=0.5
#                    elif angular_z<=-0.5:
#                        angular_z=-0.5
                    print(" angular_z=",angular_z  )
                    # 创建一个Twist消息
                    msg = Twist()
                    # 设置消息的角速度为PI控制输出
                    #msg.angular.z = -2.1 * ar_x_0
                    msg.angular.z = -angular_z
                    # 发布Twist消息
                    self.pub.publish(msg)
                else:
                # 2. 已对准：开始累积帧
                    self.consecutive_aligned_frames += 1  
                    rospy.loginfo("已对准，等待稳定... (%d/%d 帧)", 
                    self.consecutive_aligned_frames, 
                    self.shoot_frame_threshold)  
                
                # 3. 检查是否达到阈值
                    if self.consecutive_aligned_frames >= self.shoot_frame_threshold:  
                        # 4. 达到阈值：开火并跳转
                        #rospy.loginfo("连续 %d 帧满足条件，开火！", self.shoot_frame_threshold)  
                        
                        
                        ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                        rospy.loginfo("射击启动 (偏移补偿: %.2f)", self.ar_offset_x)
                        rospy.sleep(0.1)
                        ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                        rospy.loginfo("射击停止")
                        
                        self.last_target_time = rospy.get_time()
                        # 在射击完毕后重置积分项
                        self.reset_integrals()
                        rospy.sleep(2)
                        self.goto(goals[5])
                        navi.move_cb(3)
                        #os.system('mplayer %s' % music99_path)
                        rospy.sleep(1)
                        case = 4
                    #rospy.sleep(2) 
#                else:
#                    ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
#                    print('打印射击')
#                    # 等待0.1秒
#                    time.sleep(0.08)
#                    # 发送停止射击指令
#                    ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
#                    rospy.loginfo("okay_3")
#                    self.reset_integrals()
#                    rospy.sleep(2)
#                    self.goto(goals[5])
#                    navi.move_cb(3)
#                    #os.system('mplayer %s' % music99_path)
#                    rospy.sleep(1)
#                    case = 4
#                   #rospy.sleep(2)
    # 圆盘识别话题的回调函数
    def find_cb(self, data):
        if self.find_cb_executed:
           return
        global flog0, flog1, flog2, count, move_flog, point_msg, case, id, last_target_time, flag
        id = 255
        # 获取接收到的Point消息
        point_msg = data
        # 计算目标点与图像中心的偏差
        flog0 = point_msg.x - 337
        # 计算偏差的绝对值
        flog1 = abs(flog0)
        current_time = time.time()
        # 如果偏差的绝对值大于0.5
        if (flog1 > 0.5 and point_msg.z == 30 and flog2 >= 255 and case == 0) and (current_time - last_target_time < 9):
            # 创建一个Twist消息
            msg = Twist()
            # 设置消息的角速度为偏差乘以0.01
            msg.angular.z = -0.01 * flog0
            # 发布Twist消息
            self.pub.publish(msg)

        # 如果偏差的绝对值小于等于0.5
        elif ((flog1 <= 0.5 and point_msg.z == 30 and flog2 >= 255 and (current_time - last_target_time) > 2) or (current_time - last_target_time > 9)) and flag == 1:
            # 发送射击指令
            ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
            print('打印射击')
            # 等待0.1秒
            time.sleep(0.07)
            # 发送停止射击指令
            ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
            self.goto(goals[1])
            navi.move_cb(4)
            self.goto(goals[6])
            #rospy.sleep(1)
            # 到达第2个点位
            case = 1
            flog2=flog2-1
            self.goto(goals[2])
            navi.move_cb(1)
            #os.system('mplayer %s' % music22_path)
            rospy.sleep(1)
            last_target_time = time.time()
            case = 2
            self.find_cb_executed = True

    #def target_id_rotating_callback(self, msg):
    #   global target_id_rotating
    #    target_id_rotating = msg.data
    #    rospy.loginfo("收到 target_id_rotating： %d", target_id_rotating)

    #def target_id_moving_callback(self, msg):
    #    global target_id_moving
    #    target_id_moving = msg.data
    #    rospy.loginfo("收到 target_id_moving： %d", target_id_moving)

    # 设置初始位置
    def set_pose(self, p):
        if self.move_base is None:
            return False
        x, y, th = p
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        q = transformations.quaternion_from_euler(0.0, 0.0, th/180.0*pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]
        self.set_pose_pub.publish(pose)
        return True
    # 导航完成回调函数
    def _done_cb(self, status, result):
        rospy.loginfo("navigatlon done! status:%d result:%s" % (status, result))

    # 导航激活回调函数
    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")
    # 导航反馈回调函数
    def _feedback_cb(self, feedback):
        msg = feedback
        #rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)
    # 导航方法
    def goto(self, p):
        rospy.loginfo("[Navi] goto %s"%p)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!"%p)
        return True
    # 取消所有导航目标
    def cancel(self):
        self.move_base.cancel_all_goals()
        return True


if __name__ == "__main__":
    
    rospy.init_node('navigation_demo',anonymous=True)
    #goals = [[1.07,   -0.46,   0.0],
    #                [0.15,   -0.5,   0.0],
    #                [1.05,   -1.60,   0.0],
    #                [0.17,   -1.75,   0.0],
    #                [1.08,   -2.82,   0.0],
    #                [0.40,   -2.9,   0.0],
    #                [0.25,   -1.60,   0.0],
    #                [0.25,   -2.95,   0.0]
    #]
    goals = [[1.00,   -0.50,   0.0],
                    [0.17,   -0.55,   0.0],
                    [0.91,   -1.45,   0.0],
                    [0.17,   -1.65,   0.0],
                    [0.98,   -2.85,   0.0],
                    [0.40,   -2.85,   0.0],
                    [0.25,   -1.60,   0.0],
                    [0.25,   -2.65,   0.0]
    ]

    # 手动控制程序开始
    print ('Please 1 to continue: ')
    input = raw_input()
    # 控制循环的频率
    r = rospy.Rate(1)
    r.sleep()

    # 创建类的实例对象，内有回调函数自动执行
    navi = navigation_demo()
    rospy.loginfo("开始比赛")
    
    rospy.sleep(1)
    #os.system('mplayer %s' % music00_path)
    # 导航前往第一个点位
    navi.goto(goals[0])
    navi.move_cb(0)
    #os.system('mplayer %s' % music11_path)
    rospy.sleep(1)
    last_target_time = time.time()
    flag = 1
    # 表示已到达的点位，从0开始
    case = 0
    # 将ros程序挂起保持运行
    while not rospy.is_shutdown():
          r.sleep()
