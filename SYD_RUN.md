# syd 完整比赛运行说明

更新日期为 2026-11-14。工作空间为 `/home/abot/syd`，构建环境为 `devel_syd`。

## 启动

SSH 登录小车，在交互终端执行。

```bash
ssh -t abot@10.137.231.79
bash /home/abot/syd/start_syd.sh
```

启动期间保持车身静止。主程序等待 IMU 校准后发布数据，并确认 `/imu/data_raw` 和 `/imu/data` 连续更新至少 1 秒，最长等待 60 秒；未就绪则取消启动。随后自检、设置初始定位，出现准备就绪提示后输入 `1` 并回车。输入后会再次检查 IMU 数据。无需再额外设置固定 30 秒等待。

这会开始实际行驶和软弹打靶。三个打靶步骤执行原有环形靶、旋转靶和移动靶比赛代码。原有超时、剩余比赛时间不足时跳过步骤的逻辑仍由 `mission.yaml` 控制。

默认启动参数如下。

```text
dry_fire:=false
auto_start:=false
force:=false
set:=mission.rehearsal_pause_s=0
```

`start_syd_rehearsal.sh`、`748573.sh` 和 `src/shoot_race/scripts/start_race.sh` 已统一转入这个完整比赛入口。旧的 rehearsal 文件名现在也会启动完整比赛。

按 `Ctrl+C` 结束主程序，启动脚本会清理本次启动的 ROS 和底层进程。网络中断或软件停车失效时使用现场急停。

启动前将车辆摆回已经示教的起点，保持车端桌面登录供图像识别窗口使用。启动日志位于 `/home/abot/syd/run_logs/race_*`，比赛日志位于 `~/race_logs`。

## 手动启动

每个新终端先加载独立环境。

```bash
source /home/abot/syd/env_syd.sh
```

三个终端依次执行 `roscore`、`roslaunch shoot_race bringup.launch`，以及以下主程序命令。

```bash
roslaunch shoot_race race.launch dry_fire:=false auto_start:=false force:=false set:="mission.rehearsal_pause_s=0"
```

## 豆包配置

- 接口地址为 `https://ark.cn-beijing.volces.com/api/v3`。
- 已验证模型 ID 为 `doubao-seed-2-1-pro-260915`。最初填写的 `doubao-seed-2.1-pro` 返回 404，随后依据该接口 `/models` 返回的 ID 修正。
- 专用密钥文件为 `/home/abot/.ark_api_key_syd`，权限为 `600`。
- 可选环境变量 `SYD_ARK_API_KEY` 优先于专用文件。正常启动时可直接使用已保存的文件。
- 主程序与独立 `vlm_shoot` 共同读取 `src/shoot_race/config/recognition.yaml` 中的模型、接口地址及专用密钥配置。
- 识别策略为现有的 `vlm_first`，保留 API 失败或等待超时时的离线回退。

以后更换密钥或模型，可执行以下命令。输入密钥时终端隐藏字符。

```bash
python2 /home/abot/syd/configure_syd_api.py
```

使用已有卡片图片验证真实 API。

```bash
source /home/abot/syd/env_syd.sh
python2 /home/abot/syd/test_syd_api.py
```

预期输出 `PASS`，识别结果为 `(1, 'durian')`。这项检查发送一张已保存的卡片图片。

独立 VLM 入口为 `bash /home/abot/syd/run_vlm.sh`。

## 修改内容

1. 新增 `start_syd.sh`，将原来演练参数改为完整比赛参数，统一四个启动入口。
2. `env_syd.sh` 和主程序环境脚本统一加载 `devel_syd`，源码搜索路径限定为 `syd/src` 与 ROS 系统目录。
3. 主程序与独立 VLM 统一专用密钥及模型配置，移除独立 VLM 中原有的硬编码密钥。
4. `run_vlm.sh` 从指向 `748573` 的符号链接改为启动本工作空间的包装脚本。
5. 独立 VLM 的运行、节点和相机启动脚本加载 `syd` 环境，节点入口使用本目录路径。
6. `find_object_2d.launch` 的模板目录使用 ROS 包路径。
7. 新增 `configure_syd_api.py` 与 `test_syd_api.py`，补充本地备份和凭据文件的 Git 忽略规则。

修改前文件备份在 `/home/abot/syd/.change_backups/20261114_184620`。其中可能包含旧配置中的密钥，备份目录仅供本机使用。

## 验证结果

- 修改的 Shell/Python 文件语法检查通过，ROS launch 节点引用解析通过。
- `shoot_race`、导航和识别包均解析到 `/home/abot/syd/src`。
- 使用模拟 ROS 命令检查四个启动入口，均传递完整比赛参数。
- 原有回归测试通过 40 项，主程序单元检查通过 163 项。
- 主比赛接口实际识别 `card01.jpg`，得到 `(1, 'durian')`，单次耗时约 3.17 秒。
- 独立 VLM 实际识别同一图片，得到 `(1, 6)`。

本次修改没有进行实车行驶或发射验收。原项目整局仿真中已发现的精定位超时、碰撞判定、仿真参数不一致和终点朝向误差仍需单独处理。上述离线检查和单张图片 API 测试不能保证现场每局比赛结果或网络时延。
