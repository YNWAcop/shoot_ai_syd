# 2026-11-17 调试记录

用户反馈第三处移动靶弹着点略偏右。本次在车端 `/home/abot/syd/src/shoot_race/config/shooting.yaml` 将 `moving.aim_u_px` 从 350.5 增至 351.5，按配置中“偏右增大”的标定方向先补偿 1 像素。发射时长保持 0.07 秒，其余参数未改。

已使用实车 Python 2 的生产配置加载函数确认生效值；修改前后 YAML 结构对比确认只有上述参数变化。未执行行驶或发射测试，实际补偿效果待重新启动比赛程序后观察，归档日志不代表本次调整后的效果验证。

修改前配置备份在车端 `/home/abot/syd/.change_backups/moving_right_20261117_221614/shooting.yaml`。具体值与验证结果见 `moving_adjustment.json`。

`race_logs/`、`startup_logs/`、`ros_logs/` 仅归档本日记录；`manifest.json` 记录车端导出快照的文件哈希及脱敏情况，其中 `code/` 条目是导出时的源代码清单。密钥、历史日期日志、构建产物及备份不上传。
