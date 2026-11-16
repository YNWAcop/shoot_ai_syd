# 2026-11-16 调试记录

## Ubuntu 可视化启动

用户要求 `start_syd.sh` 像原版 `748573.sh` 一样在 Ubuntu 桌面显示 RViz 等窗口。

- 单终端入口默认向 `bringup.launch` 传入 `rviz:=true`，使用原版 `track_tag/rviz/ar_track_camera.rviz` 配置。
- 图像识别窗口仍启用。启动终端保留输入 `1` 的比赛控制方式；IMU 校准就绪检查保留。
- SSH 会话未指定 XAUTHORITY 时，为本地 `DISPLAY=:0` 自动选择当前用户的 GDM 授权文件。
- `SYD_RVIZ=false bash /home/abot/syd/start_syd.sh` 可临时关闭 RViz。手动启动底层节点时可使用 `roslaunch shoot_race bringup.launch rviz:=true`。

## 验证与范围

已通过车端 Bash 语法检查、图形显示连接检查，以及 RViz 开关两种配置的 `roslaunch --nodes` 解析。GUI 开启时节点清单包含 `/rviz`，关闭时不包含；已有图像识别节点保留。见 `gui_validation.txt`。

本次验证没有启动 ROS 节点、车辆运动或发射，尚未观察修改后完整比赛中的 RViz 窗口。日志目录中的当天比赛记录发生在本次修改前，不作为修改后实测成功证据。

原文件已在车端 `.change_backups/gui_20261116_142326` 备份。当天日志按既定范围归档，排除凭据、旧日期记录及构建产物；清单见 `manifest.json`。

## 后续补全原版所有窗口

用户进一步明确需要完整的原版窗口布局。`start_syd.sh` 现默认打开一扇 Ubuntu 终端窗口，含八个标签页（ROS master、底盘/IMU、射击日志、导航、相机、AR/RViz、图像识别、比赛），以及原有 RViz 和图像识别窗口。

`run_syd_session.sh` 集中管理模块启动与退出，模块日志分别显示在对应标签页。比赛标签页输入 `1`，IMU 等待逻辑保留。射击标签页显示比赛主程序日志，不额外启动占用同一串口的旧 shoot.py。`--terminal` 可使用原来的 SSH 单终端模式。

车端 Bash 语法和 Python 2 参数测试通过；独立模块展开后的 21 个底层节点与集中 bringup 的节点集合完全一致，没有重复。验证记录见 `desktop_tabs_validation.txt`。未实际启动这套窗口或比赛；测试不作为实车运行成功证据。备份路径记录在验证文件中。

`manifest.json` 为本日首次归档快照；本节后续修改以 Git 提交及 `desktop_tabs_validation.txt` 为准。
