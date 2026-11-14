# -*- coding: utf-8 -*-
"""
火山引擎方舟 (Volcengine Ark) VLM 配置参数
API Key 从环境变量 ARK_API_KEY 读取，推理接入点在下方配置。
"""
import os

# 1. 火山引擎 API Key
ARK_API_KEY = os.environ.get("ARK_API_KEY", "")

# 2. 火山引擎方舟服务地址 (默认北京地域)
BASE_URL = "https://ark.cn-beijing.volces.com/api/v3"

# 3. 推理接入点 ID (Model Endpoint)
# 对应在火山引擎控制台创建的 Doubao-seed-2.0-pro 推理接入点
MODEL_ENDPOINT = "ep-20261025102352-j8gp4"

# 4. 水果识别代号映射规则
# 识别到榴莲返回 6，识别到杨桃返回 7，识别到山竹返回 8
FRUIT_MAP = {
    "榴莲": 6,
    "杨桃": 7,
    "山竹": 8
}

FRUIT_CODE_TO_NAME = {
    6: "榴莲",
    7: "杨桃",
    8: "山竹"
}

# 5. 提示词 (Prompt)
# 任务：识别卡片中的数学计算题计算出结果，并识别下方的水果类型(榴莲/杨桃/山竹)
PROMPT = """你是一个智能视觉助手。请仔细观察图片中的卡片：
1. 卡片上方有一道中文数学题目，请理解题意并计算出最终数字结果。
2. 卡片下方是一张水果图片，请判断它是以下哪一种水果：
   - 榴莲：对应代号 6
   - 杨桃：对应代号 7
   - 山竹：对应代号 8

请必须且只能返回 JSON 格式，不要包含任何多余文字或 markdown 说明，格式如下：
{"math_result": 计算结果数字, "fruit_code": 水果代号数字}
例如：
{"math_result": 1, "fruit_code": 6}
"""
