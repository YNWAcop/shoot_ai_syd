# shoot_ai_syd

软弹竞赛小车的 ROS Melodic 工作空间。当前代码同步自车端 `/home/abot/syd`，同步日期为 2026-11-14。包含完整比赛入口、独立 API 配置和 IMU 校准就绪检查。

## 主要目录

| 路径 | 内容 |
| --- | --- |
| `src/shoot_race/` | 比赛主程序、配置、示教和离线测试 |
| `src/abot_base/` | 底盘、IMU 和雷达程序 |
| `src/robot_slam/` | 地图、导航、语音及旧版任务程序 |
| `src/abot_find/`、`src/track_tag/` | 图像、AR 标记识别与相机标定 |
| `vlm_shoot/` | 独立视觉大模型识别工具 |
| `start_syd.sh` | 单终端完整比赛入口 |
| `debug_logs/` | 从 2026-11-14 起的调试日志 |

其他包与原有版权、许可证保留。

## 运行

现有小车使用 `/home/abot/syd`、`devel_syd`、ROS Melodic 和 Python 2，独立 VLM 使用车端 Python 3.9 环境。在 SSH 交互终端执行：

```bash
cd /home/abot/syd
bash ./start_syd.sh
```

入口默认在小车 Ubuntu 桌面打开原版布局的八个终端标签页，以及 AR 相机 RViz 和图像识别窗口；桌面需已登录。在 Competition 标签页输入 `1` 开赛。使用 `bash ./start_syd.sh --terminal` 可保留 SSH 单终端方式。需要关闭 RViz 时使用 `SYD_RVIZ=false bash ./start_syd.sh`。

将车摆回已示教的起点，启动期间保持车身静止。程序等待 IMU 校准后发布的数据连续正常更新，再进行自检、设置初始定位。出现准备就绪提示后输入 `1`，会实际行驶并执行三个软弹打靶环节。输入后再次检查 IMU，未就绪则取消启动。无需手动增加固定 30 秒等待。

`748573.sh`、`start_syd_rehearsal.sh`、`src/shoot_race/scripts/start_race.sh` 均转入完整比赛入口；旧 rehearsal 文件名也会执行实际打靶。完整说明及三个终端的运行方式见 [SYD_RUN.md](SYD_RUN.md)。

停止运行按 `Ctrl+C`，等待清理完成。启动日志在车端 `syd/run_logs`，比赛详细日志在 `~/race_logs`。

## API 配置

比赛程序与独立 VLM 共用 `src/shoot_race/config/recognition.yaml`。当前模型 ID 为 `doubao-seed-2-1-pro-260915`。密钥从 `SYD_ARK_API_KEY` 环境变量或 `~/.ark_api_key_syd` 读取，实际密钥不入库。

```bash
python2 /home/abot/syd/configure_syd_api.py
```

按提示隐藏输入密钥，配置工具会限制密钥文件权限。

## 新设备部署

以下命令仅用于新的空目录。现有车端目录不要直接覆盖。

```bash
git clone https://github.com/YNWAcop/shoot_ai_syd.git /home/abot/syd
cd /home/abot/syd
unset PYTHONPATH
source /opt/ros/melodic/setup.bash
catkin_make --build build_syd -DCATKIN_DEVEL_PREFIX=/home/abot/syd/devel_syd
source env_syd.sh
```

依赖以各包的 `package.xml`、`CMakeLists.txt` 为准。部分旧程序仍含车端绝对路径，迁移时需核对；本次未在全新设备验证完整编译。语音模型 `src/robot_slam/scripts/paraformer-zh/model.pt`（约 880 MB）需另行提供，配套配置与词表已保留。

## 验证记录

- 原有根目录回归测试通过 40 项，主程序单元检查在车端 Python 2 通过 163 项。
- IMU 就绪判断的 7 项测试及车端 Python 2 语法检查通过。
- `20261114_220925` 验证等待校准后，识别区精定位成功。
- `20261114_221547` 记录修改后的完整流程运行 87.8 秒，识别区精定位成功。日志不能单独证明实际弹丸命中或击倒情况。
- 原有仿真检查存在已知失败：精定位超时、仿真碰撞、环靶标定/精度/时序和终点朝向。此次同步未修复这些仿真问题，不将其标记为通过。

```bash
python2 src/shoot_race/tests/test_imu_ready.py
python2 src/shoot_race/tests/test_units.py
python -m unittest discover -s tests -v
```

## 同步范围

保留当前车端源码和运行资源，以及 [2026-11-14 的调试日志](debug_logs/2026-11-14)。不导入更早历史日志；排除构建产物、缓存、个人编辑器配置、凭据、备份及大模型权重。源码绝对符号链接按文件内容保存。

以后每次调试结束按当天目录同步代码与新日志，约定见 [AGENTS.md](AGENTS.md)。此约定由调试工作流执行，车上没有后台定时推送任务。
