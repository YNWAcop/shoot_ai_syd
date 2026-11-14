# -*- coding: utf-8 -*-
"""
火山引擎方舟 (Volcengine Ark) VLM 配置参数
读取 syd 主程序的模型配置和专用密钥文件。运行 configure_syd_api.py 配置。
"""
import os
import yaml

_config_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            'src', 'shoot_race', 'config', 'recognition.yaml')
with open(_config_path, 'rb') as _handle:
    _vlm = yaml.safe_load(_handle)['vlm']


# 1. 火山引擎 API Key
ARK_API_KEY = os.environ.get(_vlm['api_key_env'], '').strip()
if not ARK_API_KEY:
    _key_path = os.path.expanduser(_vlm['api_key_file'])
    if os.path.isfile(_key_path):
        with open(_key_path) as _handle:
            ARK_API_KEY = _handle.read().strip()

# 2. 火山引擎方舟服务地址 (默认北京地域)
BASE_URL = _vlm['base_url']

# 3. 推理接入点 ID (Model Endpoint)
# 对应在火山引擎控制台创建的 Doubao-seed-2.0-pro 推理接入点
MODEL_ENDPOINT = _vlm['model']

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
