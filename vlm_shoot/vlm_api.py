# -*- coding: utf-8 -*-
"""
视觉大模型 API 调用与解析模块
封装 Base64 图片编码、火山引擎 Ark API 调用结构、JSON 解析及占位/模拟回退
"""
import os
import re
import json
import base64
from api_config import ARK_API_KEY, BASE_URL, MODEL_ENDPOINT, FRUIT_MAP, FRUIT_CODE_TO_NAME, PROMPT

try:
    from volcenginesdkarkruntime import Ark
    VOLCENGINE_AVAILABLE = True
except ImportError:
    VOLCENGINE_AVAILABLE = False


def encode_image_to_base64(image_path):
    """将本地图片文件编码为 Base64 字符串"""
    if not os.path.exists(image_path):
        raise FileNotFoundError("Image not found: " + str(image_path))
    with open(image_path, "rb") as f:
        return base64.b64encode(f.read()).decode("utf-8")


def parse_vlm_output(raw_text):
    """
    解析大模型返回的内容，提取两个数字：
    math_result (计算题结果), fruit_code (水果代号 6/7/8)
    """
    math_res = None
    fruit_code = None

    # 1. 尝试直接以 JSON 格式解析
    try:
        clean_text = re.sub(r'```json\s*', '', raw_text)
        clean_text = re.sub(r'```', '', clean_text).strip()
        data = json.loads(clean_text)
        if isinstance(data, dict):
            math_res = int(data.get("math_result"))
            fruit_code = int(data.get("fruit_code"))
            return math_res, fruit_code
    except Exception:
        pass

    # 2. 正则查找 JSON 片段
    try:
        json_match = re.search(r'\{[^{}]*\}', raw_text)
        if json_match:
            data = json.loads(json_match.group(0))
            math_res = int(data.get("math_result"))
            fruit_code = int(data.get("fruit_code"))
            return math_res, fruit_code
    except Exception:
        pass

    # 3. 若模型未按 JSON 输出，则通过数字提取或关键词提取做鲁棒兼容
    numbers = re.findall(r'\d+', raw_text)
    if len(numbers) >= 2:
        math_res = int(numbers[0])
        fruit_code = int(numbers[1])
    elif len(numbers) == 1:
        math_res = int(numbers[0])
        for name, code in FRUIT_MAP.items():
            if name in raw_text:
                fruit_code = code
                break

    return math_res, fruit_code


def call_vlm_api(image_path, prompt=PROMPT):
    """
    调用视觉大模型接口
    当前框架层：若尚未配置真实有效的 API_KEY 或 MODEL_ENDPOINT，则走占位/模拟流程；
    一旦填入有效 key，自动无缝接入真实火山引擎 Doubao-seed-2.0-pro 模型。
    
    返回：
        tuple: (math_result, fruit_code)
        例如: (1, 6)
    """
    is_placeholder = (
        not ARK_API_KEY or 
        ARK_API_KEY == "YOUR_API_KEY_HERE" or 
        not MODEL_ENDPOINT or 
        MODEL_ENDPOINT == "YOUR_MODEL_ENDPOINT_ID_HERE"
    )

    if is_placeholder:
        print("[VLM-FRAMEWORK] 提示：当前处于框架就绪模式（尚未配置火山引擎 API Key / 推理点）。")
        print("[VLM-FRAMEWORK] 模拟返回预置测试值: (计算题结果: 1, 水果代号: 6 [榴莲])")
        return 1, 6

    if not VOLCENGINE_AVAILABLE:
        raise RuntimeError("未安装 volcenginesdkarkruntime 依赖包，请检查 python 环境！")

    client = Ark(
        base_url=BASE_URL,
        api_key=ARK_API_KEY,
        timeout=60.0,
    )

    base64_img = encode_image_to_base64(image_path)
    data_url = "data:image/jpeg;base64," + base64_img

    response = client.chat.completions.create(
        model=MODEL_ENDPOINT,
        messages=[
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {
                        "type": "image_url",
                        "image_url": {"url": data_url}
                    }
                ]
            }
        ],
        extra_body={"thinking": {"type": "disabled"}},
        timeout=30.0
    )

    raw_content = response.choices[0].message.content.strip()
    print("[VLM-RAW-RESPONSE] " + raw_content)

    math_result, fruit_code = parse_vlm_output(raw_content)
    return math_result, fruit_code
