#!/bin/bash
# ============================================================
# check_car.sh：上车检查。只读取信息，不会让车动，也不会开火。
#
# 用法（在车上的终端里运行）：
#   bash ~/N8NNWC/src/shoot_race/scripts/check_car.sh
#       第 1 部分：系统、Python 库、硬件设备、音频、配置、单元测试。不需要启动 ROS。
#   bash ~/N8NNWC/src/shoot_race/scripts/check_car.sh topics
#       第 2 部分：传感器话题频率、定位 TF、CPU 占用。
#       先在另一个终端运行 roslaunch shoot_race bringup.launch，等它启动完（约 15 秒）再运行。
#
# 结果同时保存在 ~/race_logs/check_<时间>_<部分>.txt，可以把这个文件整个发回来分析。
# ============================================================

PKG_DIR="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd -P)"   # shoot_race 包目录（真实路径，见 env.sh 的说明）
WS_DIR="$(cd "$PKG_DIR/../.." && pwd -P)"          # 工作空间目录（旧车 ~/N8NNWC，新车 ~/204809）
PART="${1:-env}"
LOG_DIR="$HOME/race_logs"
mkdir -p "$LOG_DIR"
OUT="$LOG_DIR/check_$(date +%Y%m%d_%H%M%S)_${PART}.txt"

# 让 ROS 命令可用（新开的终端里环境变量可能没设置）。
# 车上 ~/.bashrc 往 PYTHONPATH 里加了 Python3 的库目录，ROS Melodic（Python2）会导入错版本的 yaml 而崩溃，
# 所以先清空 PYTHONPATH 再加载 ROS 环境（旧的 N8NNWC.sh 也是这样做的）
unset PYTHONPATH
source /opt/ros/melodic/setup.bash
[ -f "$WS_DIR/devel/setup.bash" ] && source "$WS_DIR/devel/setup.bash"
export ROS_PACKAGE_PATH="$WS_DIR/src:$ROS_PACKAGE_PATH"
# Python2 输出中文到管道时默认按 ASCII 编码会报错，统一指定 UTF-8
export PYTHONIOENCODING=utf-8

section() { echo; echo "==================== $1 ===================="; }

# ------------------------------------------------------------
# 第 1 部分：不需要 ROS 运行
# ------------------------------------------------------------
check_env() {
  section "系统"
  date
  echo "CPU 架构: $(uname -m)    系统: $(lsb_release -ds 2>/dev/null)    CPU 核数: $(nproc)"
  free -h | head -2
  echo "python  -> $(python --version 2>&1)  $(which python)"
  echo "python2 -> $(python2 --version 2>&1)  $(which python2)"
  echo "python3 -> $(python3 --version 2>&1)"
  echo "ROS_DISTRO=$ROS_DISTRO    LANG=$LANG"
  echo "工作空间: $WS_DIR    包: $PKG_DIR"

  section "ROS 包能否找到"
  for p in shoot_race abot_bringup robot_slam track_tag ar_track_alvar find_object_2d usb_cam map_server amcl; do
    printf '%-16s %s\n' "$p" "$(rospack find $p 2>&1 | tail -1)"
  done

  # 车上终端里的 python 可能是 anaconda 的 Python3（.bashrc 里激活了 conda），
  # ROS Melodic 的 tf 等库只支持系统 Python2，所以主程序和这里都明确用 python2
  section "Python2 库（主程序用 /usr/bin/env python2 运行）"
  python2 - <<'EOF'
# -*- coding: utf-8 -*-
from __future__ import print_function
for name in ['numpy', 'cv2', 'yaml', 'serial', 'rospy', 'tf']:
    try:
        module = __import__(name)
        version = getattr(module, '__version__', getattr(module, 'VERSION', ''))
        print('%-8s OK    %s' % (name, version))
    except Exception as error:
        print('%-8s 缺失  %s' % (name, error))
try:
    import cv2
    cv2.ORB_create(100)
    print('cv2.ORB_create 可用（离线卡片识别需要）')
except Exception as error:
    print('cv2.ORB_create 不可用: %s' % error)
EOF

  section "硬件设备"
  if [ -e /dev/shoot ]; then
    echo "/dev/shoot -> $(readlink -f /dev/shoot)（发射器串口）"
  else
    echo "/dev/shoot 不存在：发射器没插，或者 udev 规则没配置"
  fi
  echo "/dev 下的串口别名："
  find /dev -maxdepth 1 -type l -printf '  %f -> %l\n' 2>/dev/null | grep -i tty
  echo "摄像头： $(ls /dev/video* 2>/dev/null | tr '\n' ' ')"
  if id -nG | grep -qw dialout; then echo "当前用户在 dialout 组（可以直接读写串口）"; else echo "当前用户不在 dialout 组（串口可能没有权限）"; fi

  section "播报"
  echo "mplayer: $(which mplayer || echo 没有安装)"
  AUDIO_DIR="$WS_DIR/src/robot_slam/mp3"
  missing=""
  for f in 00.mp3 final.mp3 x1.mp3 x2.mp3 x3.mp3 x4.mp3 x5.mp3 y6.mp3 y7.mp3 y8.mp3; do
    [ -f "$AUDIO_DIR/$f" ] || missing="$missing $f"
  done
  echo "音频目录 $AUDIO_DIR 里共 $(ls "$AUDIO_DIR" 2>/dev/null | wc -l) 个文件"
  echo "voice.yaml 用到但缺少的文件：${missing:- 无}"

  section "豆包（只检查是否配置，不显示 key）"
  if [ -n "$ARK_API_KEY" ]; then echo "环境变量 ARK_API_KEY 已设置"; else echo "环境变量 ARK_API_KEY 未设置"; fi
  if [ -s "$HOME/.ark_api_key" ]; then echo "~/.ark_api_key 存在"; else echo "~/.ark_api_key 不存在"; fi
  grep -E "^\s*model:" "$PKG_DIR/config/recognition.yaml"
  python2 - <<'EOF'
# -*- coding: utf-8 -*-
from __future__ import print_function
import urllib2
try:
    urllib2.urlopen('https://ark.cn-beijing.volces.com/api/v3/models', timeout=5)
    print('能访问火山方舟接口')
except urllib2.HTTPError as error:
    print('能访问火山方舟接口（返回 %d，没带 key 时属正常）' % error.code)
except Exception as error:
    print('访问不了火山方舟接口：%s' % error)
EOF

  section "当前在运行的 ROS / 旧程序进程"
  ps -eo pid,etime,cmd | grep -E "roslaunch|rosmaster|shoot|N8NNWC" | grep -v grep | cut -c1-150

  section "配置检查"
  (cd "$PKG_DIR" && python2 - <<'EOF'
# -*- coding: utf-8 -*-
from __future__ import print_function
import sys
sys.path.insert(0, 'src')
from shoot_race.config import check_config, load_config
errors = check_config(load_config('config'))
print('配置引用关系没问题' if not errors else '\n'.join(errors))
EOF
)

  section "单元测试（python2）"
  (cd "$PKG_DIR" && python2 tests/test_units.py 2>&1 | grep -E "FAIL|通过|Error|Traceback|File \"" )
}

# ------------------------------------------------------------
# 第 2 部分：需要 bringup.launch 已经在运行
# ------------------------------------------------------------
topic_rate() {
  # 用 6 秒时间测一个话题的频率，只打印平均值那一行
  local rate
  rate=$(timeout 6 rostopic hz "$1" 2>/dev/null | grep "average rate" | tail -1)
  printf '%-26s %s\n' "$1" "${rate:-没有数据}"
}

check_topics() {
  if ! rostopic list >/dev/null 2>&1; then
    echo "连不上 roscore：先在另一个终端运行 roslaunch shoot_race bringup.launch"
    return
  fi

  section "节点列表"
  rosnode list

  section "话题频率（每个测 6 秒，约 1 分钟）"
  echo "参考：/scan_filtered 5~10 Hz；/wheel_odom >= 10 Hz；相机约 30 Hz；/ar_pose_marker 要 >= 10 Hz（动靶预测需要）"
  echo "      /object_position 只有画面里有环形靶时才有数据；/odom 是轮速计+IMU 融合结果，没有它 AMCL 定位不了"
  for t in /scan /scan_filtered /wheel_odom /raw_imu /imu/data_raw /odom /usb_cam/image_raw /ar_pose_marker /objects /object_position /amcl_pose; do
    topic_rate "$t"
  done

  section "参数"
  echo "ar_track_alvar max_frequency = $(rosparam get /ar_track_alvar/max_frequency 2>&1)"

  section "里程计 TF（odom -> base_link）"
  timeout 4 rosrun tf tf_echo odom base_link 2>&1 | head -6

  section "定位 TF（map -> base_link）"
  timeout 4 rosrun tf tf_echo map base_link 2>&1 | head -8

  section "一帧激光的范围"
  timeout 5 rostopic echo -n 1 /scan_filtered 2>/dev/null | grep -E "angle_min|angle_max|angle_increment|range_min|range_max|frame_id"

  section "CPU 占用最高的进程"
  top -b -n 1 | head -20
}

case "$PART" in
  env)    check_env 2>&1 | tee "$OUT" ;;
  topics) check_topics 2>&1 | tee "$OUT" ;;
  *)      echo "用法：bash check_car.sh [env|topics]"; exit 1 ;;
esac
echo
echo "结果已保存到 $OUT"
