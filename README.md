# shoot_ai_syd

软弹竞赛小车的 ROS Melodic 工作空间。源码与配套资源来自车端 `/home/abot/748573`，整理日期为 2026-11-14。

主程序 `shoot_race` 负责起点播报、卡片识别、任务点行驶与定位、环形靶/旋转靶/移动靶任务以及终点播报。

## 目录

| 路径 | 内容 |
| --- | --- |
| `src/shoot_race/` | 当前竞赛主程序、配置、示教工具和离线测试 |
| `src/robot_slam/` | 地图、导航、旧版任务程序、语音资源 |
| `src/abot_base/` | 底盘、IMU、机器人模型与雷达相关代码 |
| `src/abot_find/` | `find_object_2d` 识别程序与模板 |
| `src/track_tag/` | 相机标定与 AR 标记识别 |
| `src/startGame/`、`src/robot_voice/` | 语音启动与语音 SDK 集成 |
| `src/` 中其他包 | 车端现有演示、视觉与辅助功能 |
| `vlm_shoot/` | 独立视觉大模型识别工具 |
| `tests/` | 旧版任务程序的离线回归测试 |
| `748573.sh` | 车端原有桌面启动脚本 |

各组件原有版权声明和许可证随源码保留。

## 部署环境

车端使用 ROS Melodic、Python 2 和 catkin。独立 VLM 工具使用车端 Python 3.9 环境。地图、卡片模板、相机标定、音频及项目自带的运行库一并保留。

现有配置含 `/home/abot/748573`、`/home/abot/N8NNWC`、`/home/abot/abot_ws` 等绝对路径。以下步骤以 `/home/abot/748573` 为工作空间，迁移到其他目录时需要核对资源路径与脚本中的 Python 环境。

在新设备的空目标目录中获取项目。现有车端同名目录已含项目时，应先备份并比较差异。

```bash
git clone https://github.com/YNWAcop/shoot_ai_syd.git /home/abot/748573
cd /home/abot/748573
unset PYTHONPATH
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
```

构建依赖以各包的 `package.xml`、`CMakeLists.txt` 和组件说明为准。新设备还需安装对应硬件驱动及语音 SDK 运行依赖。本次整理未在全新系统上验证全工作空间编译。

### 单独部署的大模型

`src/robot_slam/scripts/paraformer-zh/model.pt` 为 880,502,012 字节的语音模型权重，未纳入普通 Git。使用依赖该模型的语音功能前，将原车该文件复制到新工作空间的同一相对路径。配套 `config.yaml`、`tokens.json`、`seg_dict` 和 `am.mvn` 已保留。

### API 配置

`shoot_race` 支持从 `ARK_API_KEY` 环境变量或 `~/.ark_api_key` 读取密钥。独立 `vlm_shoot` 从 `ARK_API_KEY` 环境变量读取密钥。启动对应程序前可在当前终端设置，避免将密钥写入命令历史。

```bash
read -rsp 'ARK API key: ' ARK_API_KEY
echo
export ARK_API_KEY
```

推理接入点分别位于 `src/shoot_race/config/recognition.yaml` 与 `vlm_shoot/api_config.py`。使用自己的服务时按实际接入点修改。

## 运行完整流程

将小车放回已示教的起点，确认场地与默认地图 `sheji_lab.yaml` 一致。保持车端桌面登录，`find_object` 会在 `DISPLAY=:0` 上打开窗口。

使用三个交互终端。远程登录可使用 `ssh -t abot@<小车当前IP>`。

终端一启动 ROS master。

```bash
source /home/abot/748573/src/shoot_race/scripts/env.sh
roscore
```

终端二启动硬件、定位和视觉节点。

```bash
source /home/abot/748573/src/shoot_race/scripts/env.sh
export DISPLAY=:0
roslaunch shoot_race bringup.launch
```

终端三加载环境并检查包路径，应指向当前工作空间。

```bash
source /home/abot/748573/src/shoot_race/scripts/env.sh
rospack find shoot_race
rosnode list
```

演练命令如下。自检通过后输入 `1` 并回车，车辆开始实际行驶，发射器处于演练模式。

```bash
roslaunch shoot_race race.launch dry_fire:=true auto_start:=false force:=false
```

正式软弹竞赛使用以下命令，同样等待自检通过后输入 `1`。场地应具备现场停车条件。

```bash
roslaunch shoot_race race.launch dry_fire:=false auto_start:=false force:=false set:="mission.rehearsal_pause_s=0"
```

任务顺序为起点播报、识别区、1 号任务点、2 号任务点、3 号任务点、终点。具体步骤、时间限制及剩余时间不足时的处理以 `src/shoot_race/config/mission.yaml` 为准。

先在终端三按 `Ctrl+C` 停止主程序，再依次关闭终端二和终端一。断线或软件停止失效时使用现场急停。每局日志保存在 `~/race_logs/`。

原有 `748573.sh` 同时启动 `abot_bringup/shoot.launch` 与主程序，存在发射串口争用风险。`src/shoot_race/scripts/start_race.sh` 使用旧目录 `N8NNWC`。本 README 的三终端命令使用当前工作空间与 `bringup.launch`。

## 离线检查

在具备 NumPy、PyYAML、OpenCV 等依赖的兼容 Python 环境中，于仓库根目录运行。

```bash
python -m unittest discover -s tests -v
python src/shoot_race/tests/test_units.py
python src/shoot_race/tests/test_mission_sim.py
```

仿真使用假设场地及硬件模型，实车表现需在现场验证。详细功能、参数及示教操作见 [shoot_race 说明](src/shoot_race/README.md)。独立大模型工具见 [VLM 说明](vlm_shoot/README.md)。

### 2026-11-14 验证结果

- 根目录离线回归测试在本地通过，共 40 项。
- `shoot_race/tests/test_units.py` 在原车 Python 2 环境通过，共 163 项。
- `shoot_race/tests/test_mission_sim.py` 在原车退出码为 1。已有失败涉及识别区精定位超时、仿真碰撞帧、仿真环形靶参数与 YAML 不一致、环形靶精度/时序，以及终点朝向误差 6.5 度超过测试阈值 4 度。这些结果来自原车现有源码和配置。
- 本地 Windows Python 环境运行主程序测试时遇到默认 GBK 编码和缺少 OpenCV 的问题，因此主程序两项测试使用原车兼容环境执行。
- 本次完成源码整理、密钥外置和离线检查。全工作空间重新编译与实车完整比赛尚未验证。

## 本次整理范围

保留车端源码布局与配套运行资源，排除 catkin 构建目录、运行日志、缓存、备份、临时拍摄图和大模型权重。`src/CMakeLists.txt` 原为指向车端 catkin 的绝对符号链接，仓库保存其对应文件内容。`run_vlm.sh` 使用相对仓库目录的包装脚本。独立 VLM 的 API 密钥配置改为环境变量读取。
