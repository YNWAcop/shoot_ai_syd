# VLM Shoot 视觉大模型射击任务框架

## 1. 任务要求说明
卡片上方是一道小学基础数学应用题，下方是水果图案：
- **第一个数字**：计算题的结果（整数）
- **第二个数字**：识别水果代号
  - 榴莲 -> **6**
  - 杨桃 -> **7**
  - 山竹 -> **8**
- **最终输出**：每次返回两个数字，例如 `1 6`（代表计算结果为 1，识别到榴莲 6）。

## 2. 目录结构
```text
~/vlm_shoot/
├── api_config.py        # 火山引擎 API Key、推理点与映射配置 (待接入 API 时填入)
├── vlm_api.py           # 视觉大模型调用接口与 JSON 结果解析模块 (含模拟回退机制)
├── vlm_node.py          # ROS 节点：监听相机画面、参数触发、发布识别结果
├── test_camera.py       # 摄像头硬件快速测试工具 (/dev/video0)
├── test_pipeline.py     # 单次端到端测试工具 (拍摄并运行大模型流程)
├── start_camera.sh      # 启动 ROS USB 相机节点的脚本
├── start_vlm_node.sh    # 启动 vlm_node 节点的脚本 (使用 Python 3.9)
├── trigger_detect.sh    # 命令行快速触发单次识别脚本
└── temp/                # 存放拍摄与识别过程中的图片 (vl_now.jpg 等)
```

## 3. 运行与测试方法

### 方法一：单次端到端独立测试 (无需预先启动 roscore)
```bash
/home/abot/anaconda3/envs/py39/bin/python ~/vlm_shoot/test_pipeline.py
```
该命令会自动调用摄像头拍摄一张当前画面，并进入大模型识别流程打印出两个数字。

### 方法二：ROS 节点协同模式 (如教程 PDF 流程)
1. **终端 1**：启动 ROS Core
   ```bash
   roscore
   ```
2. **终端 2**：启动 USB 摄像头
   ```bash
   bash ~/vlm_shoot/start_camera.sh
   ```
3. **终端 3**：启动 VLM 识别节点
   ```bash
   bash ~/vlm_shoot/start_vlm_node.sh
   ```
4. **终端 4**：触发一次识别
   ```bash
   bash ~/vlm_shoot/trigger_detect.sh
   # 或者使用 rosservice
   rosservice call /trigger_vlm
   ```

## 4. 后续接入真实 API 说明
在 `api_config.py` 中更新两个变量即可：
- `ARK_API_KEY`: 火山引擎控制台生成的 API Key
- `MODEL_ENDPOINT`: 火山引擎创建的 Doubao-seed-2.0-pro 推理接入点 ID (形如 `ep-2025xxxxxx`)
