#!/bin/bash
# ============================================================
# field_test.sh：场地测试菜单。输入编号回车，做完回到菜单。
#     bash ~/N8NNWC/src/shoot_race/scripts/field_test.sh      （车上有快捷方式 ~/ft.sh）
#
# [动] = 车会动，人守在旁边，随时 Ctrl+C 或断电；[火] = 会开火，要先输入 yes。
# 省电（2026-10-19）：整局跑完、菜单 IDLE_S 秒没人操作、退出菜单（q）、终端断开时，自动关闭底层节点
# （雷达、相机、底盘驱动、识别）；下次测试先选 0 重新启动（约 25 秒）。要保留底层节点退出菜单用 k。
# 运行主程序的项目启动后输入 1 才开始。记录在 ~/race_logs/<时间>/，录包在 ~/race_logs/*.bag。
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"   # 通过 ~/ft.sh 快捷方式运行时也能找到真实目录
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/env.sh" >/dev/null       # 清理 PYTHONPATH、加载 ROS 环境
mkdir -p ~/race_logs

BAG_TOPICS="/scan_filtered /object_position /wheel_odom /odom /tf /ar_pose_marker /amcl_pose /cmd_vel"
IDLE_S=${IDLE_S:-600}     # 菜单这么多秒没人操作就关闭底层节点（默认 10 分钟）

bringup_running() {
  pgrep -f "roslaunch shoot_race bringup.launch" >/dev/null
}

require_bringup() {
  bringup_running || { echo "底层节点没在运行，先选 0。"; return 1; }
}

# 关闭底层节点（雷达、相机、底盘驱动、识别）省电。$1 = 原因。
# 先发 SIGINT 让 roslaunch 按顺序关掉各节点（雷达电机停转、相机释放），最多等 20 秒，还有残留再强制结束。
stop_bringup() {
  bringup_running || return 0
  echo "$1：关闭底层节点（雷达、相机、底盘驱动、识别）省电。下次测试先选 0 重新启动。"
  pkill -INT -f "roslaunch shoot_race bringup.launch"
  for _ in $(seq 1 20); do
    bringup_running || break
    sleep 1
  done
  pkill -9 -f "roslaunch shoot_race bringup.launch" 2>/dev/null
  pkill -f "usb_cam_node|rplidarNode|ar_track_alvar|find_object_2d|abot_driver" 2>/dev/null
  rm -f ~/race_logs/.bringup_start
  sleep 1
  if pgrep -f "usb_cam_node|rplidarNode" >/dev/null; then
    echo "  相机或雷达进程还在，请手动检查：pgrep -af 'usb_cam|rplidar'"
  else
    echo "  已关闭。"
  fi
}

# 终端断开（MobaXterm 断线、关标签页）时也关掉底层节点，免得没人管一直耗电
trap 'stop_bringup "终端断开"; exit 0' HUP

confirm_fire() {
  read -r -p "会真的开火！确认前方安全、没人，输入 yes：" answer
  [ "$answer" = "yes" ]
}

# 移动靶打几号码（菜单 8 / 9）：结果放在 MOV 里，回车 = recognition.yaml 默认（6）
ask_moving() {
  read -r -p "打几号码（6 / 7 / 8，回车 = recognition.yaml 默认）：" MOV
  case "$MOV" in
    ""|6|7|8) return 0 ;;
    *) echo "只能输入 6、7、8 或直接回车"; return 1 ;;
  esac
}

# 后台录包：record_start <名字>；record_stop 结束并打印路径
record_start() {
  BAG=~/race_logs/$1_$(date +%H%M%S).bag
  rosbag record -q -O "$BAG" $BAG_TOPICS __name:=menu_bag >/dev/null 2>&1 &
  sleep 1
}

record_stop() {
  rosnode kill /menu_bag >/dev/null 2>&1
  sleep 1
  echo "录包：$BAG"
}

# 只运行比赛流程里的一步（mission.yaml 的步骤 id），并录包。其余参数原样传给 race.launch
run_step() {
  local step="$1"
  shift
  record_start "$step"
  roslaunch shoot_race race.launch from_step:="$step" to_step:="$step" force:=true "$@"
  record_stop
}

# 整局（从起点开始），并录包。参数传给 race.launch
# 2026-10-21（v27）：速度、加速度、单轮上限、爆发段都按 system.yaml（全程 0.5 m/s，爆发段默认开），回车就是比赛设置。
# 只有想临时慢一点时才输入速度（会按比例放宽行驶时间上限）
run_full() {
  read -r -p "最高速度 m/s（回车 = 按 system.yaml，现在 0.5；想慢一点才输入，例如 0.35）：" SPEED
  read -r -p "爆发段：回车 = 用；s = 用、拐角先停住再横移（慢约 1.7 秒）；n = 不用：" BURST
  case "$BURST" in
    n) EXTRA="system.path_follower.profiles_enabled=false" ;;
    s) EXTRA="system.path_follower.profiles_enabled=true; system.path_follower.profiles.burst.settle=true" ;;
    *) EXTRA="system.path_follower.profiles_enabled=true" ;;
  esac
  if [ -n "$SPEED" ]; then
    # 比 0.5 慢时，按比例放宽行驶步骤的时间上限和全场时间
    SCALE=$(awk -v s="$SPEED" 'BEGIN {k = 0.5 / s; print (k > 1) ? k : 1}')
    MATCH=$(awk -v k="$SCALE" 'BEGIN {print 120 * k}')
    EXTRA="$EXTRA; system.path_follower.max_speed=$SPEED; mission.drive_timeout_scale=$SCALE; mission.match_time_s=$MATCH"
  fi
  echo "车摆在起点框、车头朝靶。"
  record_start full
  roslaunch shoot_race race.launch "$@" set:="$EXTRA"
  record_stop
  stop_bringup "整局结束"
}

# 整局演练（2026-10-20 重做）：不问速度，按 system.yaml（v27：全程 0.5 m/s，离开任务框后退、通道横移是爆发段），
# 三个打靶步骤换成原地停 3 秒（mission.rehearsal_pause_s），不射击；照常播报、识别、精定位，并录包（发给 Claude 分析）
run_rehearsal() {
  echo "整局演练：全程最高 0.5 m/s，离开任务框后退、通道横移用爆发段（末端停住再换方向）；"
  echo "          p1 p2 p3 精定位后各停 3 秒，不射击。"
  echo "车摆在起点框、车头朝靶。"
  record_start full
  roslaunch shoot_race race.launch dry_fire:=true \
      set:="system.path_follower.profiles_enabled=true; mission.rehearsal_pause_s=3"
  record_stop
  stop_bringup "整局结束"
}

ask_box() {
  read -r -p "任务框（recog / p1 / p2 / p3 / end，回车 = p1）：" BOX
  BOX="${BOX:-p1}"
}

# 底层节点开了多久（满电实测能跑 60 分钟以上）
battery_note() {
  if bringup_running && [ -f ~/race_logs/.bringup_start ]; then
    local minutes=$(( ( $(date +%s) - $(cat ~/race_logs/.bringup_start) ) / 60 ))
    echo "  底层节点已运行 ${minutes} 分钟$( [ "$minutes" -ge 60 ] && echo '，注意电量' )（菜单 $((IDLE_S / 60)) 分钟没人操作会自动关闭）"
  fi
}

while true; do
  battery_note
  cat <<'MENU'

================ 场地测试 ================   [动] 车会动  [火] 会开火
 准备
  0  启动底层节点
  1  看四面墙距离（核对激光方向）
  2  示教：当前位置记为任务框 / 路径点
 单项
  3  精定位                          [动]
  4  环形靶 演练                     [动]
  5  环形靶 实弹                     [动][火]
  6  旋转靶 演练                     [动]
  7  旋转靶 实弹                     [动][火]
  8  移动靶 演练（可选码号）
  9  移动靶 实弹（可选码号）         [火]
 整局（从起点开始）
 10  整局演练：爆发段、p1 p2 p3 各停 3 秒、不射击  [动]
 11  整局实弹                        [动][火]
 12  采用最近 3 局的精定位位置作为路径点（中位数）
 标定
 13  精定位重复性                    [动]
 14  环形靶：按示教位置开火、记弹孔  [动][火]
 15  环形靶：按弹孔平均值修正 1 号框
 16  发射时长                        [火]
 17  开火记录（配合录像标弹着点）    [火]
 18  分析环形靶录包
 19  底盘诊断（转速 / 横移 / 车身尺寸 / 急停）[动]
 20  旋转 / 移动靶：用开火记录标定弹着点
 22  弹道测试：先对准环形靶（或停住的移动靶码）再连射  [动][火]
 23  环形靶找靶心检查（找同心圆：靶心位置、每帧用时；不动不打）
 其他
 21  打包记录，准备拷回电脑
  q  退出，并关闭底层节点（省电）
  k  退出，底层节点继续运行
MENU
  if bringup_running; then
    read -r -t "$IDLE_S" -p "编号：" choice
    status=$?
  else
    read -r -p "编号：" choice
    status=$?
  fi
  if [ "$status" -gt 128 ]; then                   # 超时：没人操作
    echo
    stop_bringup "菜单 $((IDLE_S / 60)) 分钟没人操作"
    continue
  elif [ "$status" -ne 0 ]; then                   # 输入结束（终端断开）
    stop_bringup "终端断开"
    exit 0
  fi
  case "$choice" in
    0)
      if bringup_running; then
        pkill -f "roslaunch shoot_race bringup.launch"
        sleep 6
      fi
      setsid nohup roslaunch shoot_race bringup.launch > ~/race_logs/bringup.log 2>&1 &
      date +%s > ~/race_logs/.bringup_start
      echo "已在后台启动（日志 ~/race_logs/bringup.log），25 秒后检查话题..."
      sleep 25
      bash "$SCRIPT_DIR/check_car.sh" topics | grep -E "average rate|没有数据|max_frequency"
      grep -i -E "error|died|no such slot" ~/race_logs/bringup.log | grep -v "Timed out waiting" | head -10
      ;;
    1)
      require_bringup && rosrun shoot_race teach.py watch
      ;;
    2)
      require_bringup && {
        read -r -p "名称（任务框 recog p1 p2 p3 end，或路径点 start lane_p1 …）：" NAME
        case "$NAME" in
          recog|p1|p2|p3|end)
            read -r -p "用哪几面墙（如 right back yaw=right，回车 = 按配置）：" WALLS
            rosrun shoot_race teach.py box "$NAME" $WALLS ;;     # 同时记下同名路径点
          "") echo "没有输入名称" ;;
          *)  rosrun shoot_race teach.py pose "$NAME" ;;
        esac
      }
      ;;
    3)
      require_bringup && ask_box && run_step "align_$BOX" dry_fire:=true
      ;;
    4)
      require_bringup && run_step shoot_ring dry_fire:=true
      ;;
    5)
      require_bringup && confirm_fire && run_step shoot_ring
      ;;
    6)
      require_bringup && {
        read -r -p "打几号码（1~5，回车 = recognition.yaml 默认）：" ROT
        if [ -n "$ROT" ]; then run_step shoot_rot dry_fire:=true set:="recognition.default_targets.rotating=$ROT"
        else run_step shoot_rot dry_fire:=true; fi
      }
      ;;
    7)
      require_bringup && confirm_fire && {
        read -r -p "打几号码（1~5，回车 = recognition.yaml 默认）：" ROT
        if [ -n "$ROT" ]; then run_step shoot_rot set:="recognition.default_targets.rotating=$ROT"
        else run_step shoot_rot; fi
      }
      ;;
    8)
      require_bringup && ask_moving && {
        if [ -n "$MOV" ]; then run_step shoot_move dry_fire:=true set:="recognition.default_targets.moving=$MOV"
        else run_step shoot_move dry_fire:=true; fi
      }
      ;;
    9)
      require_bringup && confirm_fire && ask_moving && {
        if [ -n "$MOV" ]; then run_step shoot_move set:="recognition.default_targets.moving=$MOV"
        else run_step shoot_move; fi
      }
      ;;
    10)
      require_bringup && run_rehearsal
      ;;
    11)
      require_bringup && confirm_fire && run_full
      ;;
    12)
      python2 "$SCRIPT_DIR/teach.py" adopt
      ;;
    13)
      require_bringup && ask_box && {
        read -r -p "重复几次（回车 = 5）：" COUNT
        echo "每次：车自己挪开（转 3~5°、横移 3~5 cm、前后 2~3 cm）→ 精定位 → 静止 4 秒。不发射。"
        read -r -p "车在 $BOX 框里、周围没人，回车开始：" _
        BAGS=""
        for i in $(seq 1 "${COUNT:-5}"); do
          echo "---------- 第 $i 次 ----------"
          rosrun shoot_race nudge.py
          record_start "align_$BOX"
          roslaunch shoot_race race.launch from_step:="align_$BOX" to_step:="align_$BOX" force:=true \
              dry_fire:=true auto_start:=true
          sleep 4                                   # 静止 4 秒：录靶心像素和墙距
          record_stop
          BAGS="$BAGS $BAG"
        done
        python2 "$SCRIPT_DIR/ring_bag_check.py" --last $BAGS
      }
      ;;
    14)
      require_bringup && confirm_fire && {
        read -r -p "打几枪（回车 = 3）：" COUNT
        for i in $(seq 1 "${COUNT:-3}"); do
          echo "---------- 第 $i 枪：车自己挪开 → 精定位 → 不看靶直接开火 ----------"
          rosrun shoot_race nudge.py
          record_start taught
          roslaunch shoot_race race.launch from_step:=align_p1 to_step:=shoot_ring force:=true auto_start:=true \
              set:="shooting.ring.mode=taught"
          sleep 3
          record_stop
          read -r -p "弹孔在靶心右边几 cm（左为负，没打出来输入 x，回车 = 不记）：" H
          read -r -p "弹孔在靶心下面几 cm（上为负）：" V
          [ -n "$H" ] && echo "$(date +%Y-%m-%d_%H:%M:%S),$H,$V,$BAG" >> ~/race_logs/ring_holes.csv
          if [ "$i" -lt "${COUNT:-3}" ]; then
            read -r -p "确认安全后回车打下一枪（q 结束）：" answer
            [ "$answer" = "q" ] && break
          fi
        done
        echo "弹孔记在 ~/race_logs/ring_holes.csv，用 15 修正。"
      }
      ;;
    15)
      read -r -p "用最近几次弹孔（回车 = 3）：" N
      MEAN=$(grep -v ",x," ~/race_logs/ring_holes.csv 2>/dev/null | tail -n "${N:-3}" | \
             awk -F, '{s += $2; n++} END {if (n > 0) printf "%.1f", s / n}')
      if [ -z "$MEAN" ]; then
        echo "没有弹孔记录，先做 14。"
      else
        echo "弹孔平均偏右 $MEAN cm → 1 号框示教位置向左挪 $MEAN cm"
        read -r -p "确认输入 y：" answer
        [ "$answer" = "y" ] && rosrun shoot_race teach.py shift p1 "$MEAN"
      fi
      ;;
    16)
      read -r -p "发射时长 秒（回车 = 0.07）：" BURST
      read -r -p "测几次（回车 = 5）：" COUNT
      python2 "$SCRIPT_DIR/fire_test.py" "${BURST:-0.07}" "${COUNT:-5}"
      ;;
    17)
      require_bringup && {
        read -r -p "哪个靶（rotating / moving / ring，回车 = rotating）：" TGT
        read -r -p "开几枪（回车 = 5）：" COUNT
        read -r -p "间隔几秒（回车 = 4）：" GAP
        record_start shotlog
        python2 "$SCRIPT_DIR/shot_log.py" "${TGT:-rotating}" "${COUNT:-5}" "${GAP:-4}"
        record_stop
      }
      ;;
    18)
      BAGS=$(ls -t ~/race_logs/shoot_ring_*.bag 2>/dev/null | head -10)   # 4、5 自动录的，取最近 10 个
      if [ -z "$BAGS" ]; then
        echo "没有 shoot_ring_*.bag，先做 4 或 5。"
      else
        python2 "$SCRIPT_DIR/ring_bag_check.py" $BAGS
      fi
      ;;
    19)
      require_bringup && {
        echo "  1  原地转：找能转起来的最小角速度（约 30 秒，每次几度）"
        echo "  2  原地转：0.5 / 0.7 / 1.0 rad/s 各左右转 1 秒（最多约 60°）"
        echo "  3  横移：0.02 ~ 0.05 m/s 各左右平移 0.8 秒（最多约 4 cm）"
        echo "  4  原地转快档：1.0 / 1.5 / 2.0 rad/s 各左右转 1 秒（最多约 70°，看指令加大后实际转速跟不跟着涨）"
        echo "  5  车身尺寸：手把车推到某一面贴住围栏，激光量出车头 / 车尾 / 侧面离中心多远（车不动）"
        echo "  6  急停滑行：0.5 m/s 直线前进后松开，量滑多远（车头前方要空 1.5 米，开出去约 0.9 米）"
        read -r -p "选哪个：" T
        case "$T" in
          1) rosrun shoot_race spin_test.py ;;
          2) rosrun shoot_race spin_test.py t=1.0 0.5 0.7 1.0 ;;
          3) rosrun shoot_race step_test.py ;;
          4) rosrun shoot_race spin_test.py t=1.0 1.0 1.5 2.0 ;;
          5) rosrun shoot_race body_test.py size ;;
          6) record_start brake; rosrun shoot_race body_test.py brake; record_stop ;;
          *) echo "没有这个选项" ;;
        esac
      }
      ;;
    20)
      python2 "$SCRIPT_DIR/aim_calib.py"
      ;;
    22)
      require_bringup && {
        echo "环形靶：车放在 1 号框（离靶约 1.9 米）。移动靶：滑块停住、目标码在画面里（倒靶打一枪就倒）。"
        echo "车会转车头 / 左右平移对准（平移最多 15 cm），然后连射。"
        read -r -p "哪个靶（回车 = 环形靶；移动靶输入码号 6 / 7 / 8）：" CODE
        read -r -p "开几枪（回车 = 5）：" COUNT
        read -r -p "间隔几秒（回车 = 2）：" GAP
        record_start aimtest
        python2 "$SCRIPT_DIR/aim_test.py" "${CODE:-ring}" "${COUNT:-5}" "${GAP:-2}"
        record_stop
      }
      ;;
    23)
      require_bringup && {
        read -r -p "看几秒（回车 = 5）：" SECS
        python2 "$SCRIPT_DIR/ring_check.py" "${SECS:-5}"
      }
      ;;
    21)
      for active in ~/race_logs/*.bag.active; do          # 断电时没写完的录包
        [ -e "$active" ] || continue
        rosbag reindex "$active" >/dev/null 2>&1 && mv "$active" "${active%.active}" && echo "已修复 $active"
      done
      PACK=~/logs_$(date +%m%d_%H%M).tgz
      tar czf "$PACK" -C ~ race_logs N8NNWC/src/shoot_race/config && ls -lh "$PACK"
      echo "用 MobaXterm 左边的文件栏把它下载到电脑。"
      ;;
    q|Q)
      stop_bringup "退出菜单"
      exit 0
      ;;
    k|K)
      echo "底层节点继续运行（记得用完关掉：再进菜单选 q，或 pkill -INT -f 'roslaunch shoot_race bringup.launch'）"
      exit 0
      ;;
    *)
      echo "没有这个选项：$choice"
      ;;
  esac
done
