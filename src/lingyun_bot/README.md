# lingyun_bot

射击小车比赛程序：以 0.2 m/s 跑完全程，三个靶各开一枪。全部用车上现成的 ROS 包：

| 干什么 | 用的包 |
|---|---|
| 走点 | move_base（robot_slam/navigation.launch，带 AMCL 定位） |
| 环形靶、识别卡片 | find_object_2d（模板放在本包 `templates/`） |
| 旋转靶、移动靶的码 | ar_track_alvar（track_tag/ar_track_camera.launch） |
| 开枪 | abot_bringup/shoot.launch（收到 `/shoot` 话题开一枪） |
| 播报 | mplayer 播 robot_slam/mp3 里的文件 |

## 文件

```
lingyun_bot/
├── config/task.yaml            所有参数（点位、速度、弹着点、等待时间）
├── launch/bringup_all.launch   底层节点
├── templates/                  find_object 模板：28~31 环形靶，101~115 十五张卡片
├── scripts/task_main.py        比赛主程序
├── scripts/mark_pose.py        记点位
├── scripts/check_view.py       看靶（标弹着点、查位置用，不动车不开枪）
├── nodes.sh                    启动底层节点
└── go.sh                       开赛
```

## 安装

1. 把整个文件夹拷到车上 `~/N8NNWC/src/lingyun_bot`。
2. 给脚本加执行权限：`chmod +x ~/N8NNWC/src/lingyun_bot/*.sh ~/N8NNWC/src/lingyun_bot/scripts/*.py`
3. 不用编译。用到的都是工作空间里已经有的包。

## 比赛

```bash
# 终端 1：底层节点（窗口会开在车自己的桌面上）
bash ~/N8NNWC/src/lingyun_bot/nodes.sh
# 等 20 秒左右，看到 find_object 窗口、rviz 窗口都出来了
# 终端 2：开赛（车摆在起点，回车开始）
bash ~/N8NNWC/src/lingyun_bot/go.sh
```

先试流程、不想开枪：`bash ~/N8NNWC/src/lingyun_bot/nodes.sh shoot:=false`，开枪节点不启动，其它照常。

一局的流程：

1. 播报开始，走到识别区，用 find_object 认卡片，最多看 6 秒。没认出来就用默认的 1 号、6 号。播报两个靶号。
2. 到 1 号框：按 AMCL 把车头转准，看环形靶靶心，横移对准（最多 3 次、累计 3 cm），开枪。
3. 经过通道到 2 号框：用转轮上的码拟合出圆，原地转一点车头，让圆经过弹着点；等目标码转到弹着点附近开枪。最多待 18 秒，到时间直接开。
4. 到 3 号框：车不动，目标码走到弹着点附近就开枪。最多等 14 秒。
5. 走到终点，播报结束。

每个靶最多等多久，会按"剩余时间 − 后面路上要花的时间"再压一压：后面路上的时间按这一局前面实际开的速度估。剩下不到 3 秒就跳过这个靶，先保证能到终点。

## 换车、换场地要做的

1. **建图**，`launch/bringup_all.launch` 里改 `map_name`。
2. **记点位**：启动底层节点，在 rviz 里给好初始位置，把车摆到每个点上，运行：
   ```bash
   cd ~/N8NNWC/src/lingyun_bot/scripts
   unset PYTHONPATH; python2 mark_pose.py p1     # 依次记 start recog p1 lane1 lane2 p2 lane3 p3 end
   ```
   车头朝向也会记下来：1、2、3 号框要车头正对靶，识别区要正对卡片。
3. **查 find_object 的模板编号**：find_object 窗口左边列出的编号，应该是 28~31 和 101~115。不是的话，按窗口里的编号改 `task.yaml` 的 `ring.ids`、`cards.answers`。
4. **标弹着点**：车摆在框里，开枪打几发，看弹孔偏多少。
   - 环形靶：`python2 check_view.py ring` 看靶心 x。弹孔偏右就把 `ring.aim_x` 加大，1 cm 约 2.5 像素。
   - 旋转、移动靶：`python2 check_view.py 3`（码号）看码的位置。弹孔偏右就把 `aim` / `aim_u` 加大，1 cm 约 3 像素。
5. **检查旋转靶**：车停在 2 号框，`python2 check_view.py wheel_fit` 会打印转轮圆心，以及车头要再转几度，圆才经过弹着点。比赛时程序会自己转，这里只是确认看得到 3 个以上的码。

## 常改的参数（config/task.yaml）

| 参数 | 现在 | 说明 |
|---|---|---|
| `speed` | 0.2 | 行驶最高速度 m/s。时间不够可以加到 0.25~0.3 |
| `goal_xy_tolerance` | 0.06 | move_base 到点容差（米）。车老停不进框就改小；到点前来回蹭就改大 |
| `end_reserve_s` | 8 | 算剩余时间时给终点播报留的余量。老是超时就调大 |
| `ring.aim_x` | 363 | 环形靶弹着点 |
| `ring.tolerance_px` | 6 | 环形靶对准容差 |
| `rotating.aim` | [350.5, 199.5] | 旋转靶弹着点 |
| `rotating.tolerance_px` | 7 | 目标码离弹着点这么近就开枪 |
| `rotating.timeout_s` | 18 | 到 2 号框后最多待多久（车不动不能超过 20 秒） |
| `moving.aim_u` | 350.5 | 移动靶弹着点 |
| `moving.timeout_s` | 14 | 到 3 号框后最多待多久 |
| `cards.default` | [1, 6] | 卡片没认出来时用的靶号 |

## 注意

- 车上的 `python` 是 anaconda 的 Python3。脚本都要用 `python2` 运行，运行前先 `unset PYTHONPATH`。两个 .sh 已经写好了这两步。
- `abot_bringup/shoot.launch` 和别的程序不能同时占着 `/dev/shoot`，其它比赛程序要先关掉。
- 0.2 m/s 比较慢。实际跑一局看总用时：程序会自己少等、跳靶来保证到终点，但跳得多就拿不到分，这时把 `speed` 调到 0.25~0.3。
