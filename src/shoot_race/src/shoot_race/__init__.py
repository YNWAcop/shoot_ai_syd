# -*- coding: utf-8 -*-
"""shoot_race：射击小车比赛程序（第一阶段）。

模块一览（详细说明见包根目录 README.md）：
    config      读取并检查 config/ 下的全部 yaml 配置
    ros_io      唯一和 ROS 打交道的类：传感器数据缓存、底盘速度出口、导航、定位
    geometry    纯计算：角度、坐标变换、激光墙线拟合
    motion      纯计算：路径跟随、贴墙精定位、原地转向、速度平滑、障碍检查
    aiming      纯计算：环形靶对准、旋转靶交点规划、移动靶到达预测（都用横向平移，不转向）
    shooter     串口发射器
    voice       语音播报（播放音频文件）
    recognition 识别区卡片识别：离线模板匹配 + 豆包视觉模型
    steps       比赛中的每一种步骤（播报、行驶、精定位、识别、打靶）
    mission     任务执行器：按 mission.yaml 依次执行步骤，并管理比赛时间
    preflight   开局自检
    runlog      每局运行记录
"""
