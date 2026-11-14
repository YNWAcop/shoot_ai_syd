#!/home/abot/anaconda3/envs/py39/bin/python
# -*- coding: utf-8 -*-
import rospy
import time
import os#yuyin_mp3
from std_msgs.msg import String, Int32

# 定义全局变量存储目标ID
target_id_rotating = None   # 旋转靶的ID
target_id_moving = None   # 移动靶的ID
# 中文数字到旋转靶ID的映射字典
target_id_rotating_mapping = {
    "一": 1,
    "二": 2,
    "三": 3,
    "四": 4,
    "五": 5
}
# 中文数字到移动靶ID的映射字典
target_id_moving_mapping = {
    "六": 6,
    "七": 7,
    "八": 8,
}

MP3_DIR = "/home/abot/748573/src/robot_slam/mp3"

def play_mp3(path):
    if os.path.exists(path):
        os.system('mplayer "%s"' % path)
    else:
        rospy.logwarn("MP3 not found: %s", path)

# def chinese_callback(msg):
#     """
#     中文消息回调函数，解析包含中文数字的消息
#     :param msg: 订阅的String类型消息，包含中文数字
#     """
#     global arrive_pub, target_id_rotating, target_id_moving
#     # ------------------- 旋转靶处理逻辑 -------------------
#     # 遍历旋转靶映射字典
#     for keyword, value in target_id_rotating_mapping.items():
#         if keyword in msg.data:  # 检测消息中的中文关键词
#             target_id_rotating = value  # 更新全局变量
#             rospy.loginfo(f"检测到旋转靶关键词: {keyword}, 设置 target_id_rotating = {target_id_rotating}")
            
#             # 构造语音反馈字符串并发布
#             arrive_str = "旋转靶为{}号".format(keyword)
#             arrive_pub.publish(arrive_str)
            
#             try:
#                 # 发布旋转靶ID到指定话题
#                 target_id_rotating_pub.publish(target_id_rotating)
#             except Exception as e:
#                 rospy.logerr(f"发布旋转靶 target_id 时出错: {e}")
#             break  # 找到匹配后退出循环
    
#     time.sleep(3)
#     # ------------------- 移动靶处理逻辑 -------------------
#     # 遍历移动靶映射字典
#     for keyword, value in target_id_moving_mapping.items():
#         if keyword in msg.data:
#             target_id_moving = value  # 更新全局变量
#             rospy.loginfo(f"检测到移动靶关键词: {keyword}, 设置 target_id_moving = {target_id_moving}")
            
#             # 构造语音反馈字符串并发布
#             arrive_str = "移动靶为{}号".format(keyword)
#             arrive_pub.publish(arrive_str)
            
#             try:
#                 # 发布移动靶ID到指定话题
#                 target_id_moving_pub.publish(target_id_moving)
#             except Exception as e:
#                 rospy.logerr(f"发布移动靶 target_id 时出错: {e}")
#             break  # 找到匹配后退出循环
def chinese_callback(msg):
    global target_id_rotating, target_id_moving

    text = msg.data
    rospy.loginfo("received chinese_topic: %s", text)

    for keyword, value in target_id_rotating_mapping.items():
        if keyword in text:
            target_id_rotating = value
            rospy.loginfo("rotating target id = %d", target_id_rotating)

            mp3_path = os.path.join(
                MP3_DIR,
                "x%d.mp3" % target_id_rotating
            )
            play_mp3(mp3_path)

            target_id_rotating_pub.publish(target_id_rotating)
            break

    time.sleep(0.5)

    for keyword, value in target_id_moving_mapping.items():
        if keyword in text:
            target_id_moving = value
            rospy.loginfo("moving target id = %d", target_id_moving)

            mp3_path = os.path.join(
                MP3_DIR,
                "y%d.mp3" % target_id_moving
            )
            play_mp3(mp3_path)

            target_id_moving_pub.publish(target_id_moving)
            break

def chinese_subscriber():
    """
    初始化ROS节点并设置发布者/订阅者
    """
    global arrive_pub, target_id_rotating_pub, target_id_moving_pub
    
    # 初始化ROS节点（匿名节点防止重复）
    rospy.init_node('chinese_subscriber', anonymous=True)
    # ------------------- 发布者初始化 -------------------
    # 语音反馈发布者（String类型）
    arrive_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
    # 旋转靶ID发布者（Int32类型）
    target_id_rotating_pub = rospy.Publisher('target_id_rotating', Int32, queue_size=10)
    # 移动靶ID发布者（Int32类型） 
    target_id_moving_pub = rospy.Publisher('target_id_moving', Int32, queue_size=10)
    # ------------------- 订阅者初始化 -------------------
    # 订阅中文消息话题（String类型）
    rospy.Subscriber("chinese_topic", String, chinese_callback)
    rospy.loginfo("Chinese subscriber node started")
    rospy.spin()  # 保持节点运行

if __name__ == '__main__':
    try:
        chinese_subscriber()
    except rospy.ROSInterruptException:
        pass
