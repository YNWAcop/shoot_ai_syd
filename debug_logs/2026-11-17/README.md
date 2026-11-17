# 2026-11-17 调试记录

用户反馈第三处移动靶弹着点略偏右。本次在车端 `/home/abot/syd/src/shoot_race/config/shooting.yaml` 将 `moving.aim_u_px` 从 350.5 增至 351.5，按配置中“偏右增大”的标定方向先补偿 1 像素。发射时长保持 0.07 秒，其余参数未改。

已使用实车 Python 2 的生产配置加载函数确认生效值；修改前后 YAML 结构对比确认只有上述参数变化。未执行行驶或发射测试，实际补偿效果待重新启动比赛程序后观察，归档日志不代表本次调整后的效果验证。

修改前配置备份在车端 `/home/abot/syd/.change_backups/moving_right_20261117_221614/shooting.yaml`。具体值与验证结果见 `moving_adjustment.json`。

## 22:27 再次水平补偿

用户反馈第三靶仍略偏右，要求再补偿 1 像素；第一靶也略偏右。本次将 `moving.aim_u_px` 从 351.5 调至 352.5，将第一靶当前同心圆识别实际使用的 `ring.rings.aim_x_px` 从 353.0 调至 354.0。两处各增加 1 像素，发射时长均保持 0.07 秒。用户随后明确仅调整左右偏置，高低相关参数不变。

确认车端没有 `shooting_taught.yaml` 覆盖文件，第一靶使用 `detector: rings`、`mode: vision`；实车 Python 2 配置加载验证通过，结构比对确认仅上述两项变化。未运行行驶或发射测试，需重启比赛程序后观察补偿效果。备份位于 `/home/abot/syd/.change_backups/aim_right_20261117_222717/shooting.yaml`，详情见 `aim_adjustment_20261117_222717.json`。

## 22:33 第三靶继续水平补偿

用户反馈第三靶仍有少量偏差，沿前次偏右方向再补偿 1 像素：`moving.aim_u_px` 从 352.5 调至 353.5。第一靶保持 354.0，高低参数及 0.07 秒发射时长均不变。车端生产 Python 2 配置加载验证通过，结构比对确认仅此一项变化。未执行行驶或发射测试，需重启程序后观察实际效果。

备份：`/home/abot/syd/.change_backups/moving_right_20261117_223339/shooting.yaml`；验证记录：`aim_adjustment_20261117_223339.json`。

## 22:35 第一靶继续水平补偿

用户反馈第一靶仍偏右，要求再补偿 1 像素。将 `ring.rings.aim_x_px` 从 354.0 增至 355.0；第三靶保持 353.5，高低参数和两处 0.07 秒发射时长不变。确认无示教配置覆盖，第一靶仍使用同心圆视觉模式。实车 Python 2 配置加载与单项变更比对通过，未执行行驶或发射测试，需重新启动后观察效果。

备份：`/home/abot/syd/.change_backups/ring_right_20261117_223545/shooting.yaml`；验证记录：`aim_adjustment_20261117_223545.json`。

`race_logs/`、`startup_logs/`、`ros_logs/` 仅归档本日记录；`manifest.json` 记录车端导出快照的文件哈希及脱敏情况，其中 `code/` 条目是导出时的源代码清单。密钥、历史日期日志、构建产物及备份不上传。
