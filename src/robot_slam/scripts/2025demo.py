#!/home/abot/anaconda3/envs/py39/bin/python
# -*- coding: utf-8 -*-
import rospy
import pyaudio
import wave
import os
from funasr import AutoModel
import soundfile
from std_msgs.msg import String
music_path="/home/abot/LWJQCX/src/robot_slam/scripts/ding.mp3"
music1_path="/home/abot/LWJQCX/src/robot_slam/scripts/end_record.mp3"
os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

def audio_callback(msg):
    rospy.loginfo("Received audio message, starting recording and recognition")
    start_audio()
    rospy.loginfo("Recording and recognition completed")

def start_audio(time=3, save_file="/home/abot/LWJQCX/test.wav"):
    global model
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 2
    RATE = 16000
    RECORD_SECONDS = time  # 需要录制的时间
    WAVE_OUTPUT_FILENAME = save_file  # 保存的文件名

    p = pyaudio.PyAudio()  # 初始化
    rospy.loginfo("ON")
    os.system('mplayer %s' % music_path)
    if os.path.exists(save_file):
        os.remove(save_file)

    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)  # 创建录音文件
    frames = []

    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)  # 开始录音

    rospy.loginfo("OFF")
    #os.system('mplayer %s' % music1_path)

    stream.stop_stream()
    stream.close()
    p.terminate()

    wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')  # 保存
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

# 语音识别处理
    rospy.loginfo("Starting recognition")
    res = model.generate(input=save_file)
    result = res[0].get('text', '默认值')  # 获取识别结果
    print("识别结果:", result)
    # 等待发布者注册
    rospy.loginfo("Waiting for publisher to register...")
    while pub1.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.1)
    # 发布识别结果
    message = str(result)
    rospy.loginfo(f"Publishing message: {message}")
    pub1.publish(message)
    rospy.sleep(0.5)  # 确保消息发送

def audio_subscriber():
    """ROS节点初始化与订阅设置"""
    rospy.init_node('audio_subscriber', anonymous=True)
    global pub1
    # 创建发布者 (识别结果发布到chinese_topic)
    pub1 = rospy.Publisher('chinese_topic', String, queue_size=10)
    # 创建订阅者 (监听audio_topic)
    rospy.Subscriber("audio_topic", String, audio_callback)
    rospy.loginfo("Audio subscriber node started")
    rospy.spin()
if __name__ == '__main__':
    # 初始化语音识别模型
    model = AutoModel(model="/home/abot/LWJQCX/src/robot_slam/scripts/paraformer-zh", 
                     disable_update=True)
    # 启动ROS节点
    audio_subscriber()
