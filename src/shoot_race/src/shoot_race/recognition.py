# -*- coding: utf-8 -*-
"""识别区卡片识别：得到旋转靶号（1~5）和移动靶号（6~8）。

卡片内容（2026 规则附件，共 15 张）：上方一道算术应用题，答案 1~5 → 旋转靶号；
下方一张水果图（榴莲/杨桃/山竹）→ 移动靶号（对应关系在 recognition.yaml 里配置，赛前确认）。

两种识别方法，互为备份：
  1. 离线模板匹配 CardMatcher：15 张卡片已经公开，直接认出"是哪一张"，再查表。
     不需要网络，约 0.3 秒。缺点：真实打印卡片和拍照条件下的效果需要在车上验证。
  2. 豆包视觉模型 VlmClient：把照片发给大模型，让它算题、认水果。
     需要网络，几秒钟；组委会教程推荐的方法。

CardRecognizer 把两者组合起来，按 recognition.yaml 的 policy 决定用谁的结果。
两种方法都在后台线程里运行，主循环每个周期调用 poll() 查询，不会被卡住。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import base64
import json
import os
import re
import threading
import time

import numpy as np

from .compat import to_native, to_text


# ============================================================
# 1. 离线模板匹配
# ============================================================
class CardMatcher(object):
    """两步法模板匹配。

    第一步：ORB 特征点匹配 + 单应性矩阵，把照片里的卡片"拉正"到模板的坐标系。
    第二步：只比较文字区域（卡片上方的题目）的相关系数（NCC）。

    为什么要第二步：同一种水果的 5 张卡片，水果图完全一样，只有题目文字不同。
    只数特征点的话，水果图贡献了大部分匹配点，5 张卡片得分几乎一样，很容易认错；
    拉正以后只比文字，区分度就高了（离线合成测试：120 张照片认错 1 张）。
    """

    def __init__(self, cards, cards_dir, params):
        import cv2
        self.cv2 = cv2
        self.cards = cards
        self.scale = params['template_scale']          # 模板缩放比例，越小越快
        self.min_inliers = params['min_inliers']        # 单应性内点少于这个数，认为不是这张卡
        self.orb = cv2.ORB_create(nfeatures=int(params['orb_features']))
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.templates = []
        for card in cards:
            path = os.path.join(cards_dir, card['file'])
            gray = cv2.imread(to_native(path), cv2.IMREAD_GRAYSCALE)
            if gray is None:
                raise IOError('卡片模板读取失败：%s' % path)
            gray = cv2.resize(gray, (int(gray.shape[1] * self.scale), int(gray.shape[0] * self.scale)))
            keypoints, descriptors = self.orb.detectAndCompute(gray, None)
            self.templates.append({
                'gray': gray,
                'keypoints': keypoints,
                'descriptors': descriptors,
                'text_bottom': self._find_text_bottom(gray),
            })

    @staticmethod
    def _find_text_bottom(gray):
        """找题目文字的下边界：从第一行有字的地方往下，遇到连续 18 行以上的空白就认为文字结束。"""
        has_ink = (gray < 120).sum(axis=1) > 2
        rows = np.where(has_ink)[0]
        if len(rows) == 0:
            return gray.shape[0] // 4
        last_ink_row = rows[0]
        for row in range(rows[0], gray.shape[0]):
            if has_ink[row]:
                if row - last_ink_row - 1 >= 18:
                    return last_ink_row + 6
                last_ink_row = row
        return gray.shape[0] // 4

    def match(self, bgr_image):
        """返回 {'index': 最像的卡片下标, 'score': 文字相关系数, 'margin': 与第二名的差, 'scores': [...]}。"""
        cv2 = self.cv2
        gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        scores = []
        for template in self.templates:
            scores.append(self._text_score(template, gray, keypoints, descriptors))
        order = np.argsort(scores)[::-1]
        best, second = int(order[0]), int(order[1])
        return {'index': best, 'score': float(scores[best]),
                'margin': float(scores[best] - scores[second]),
                'scores': [round(float(s), 3) for s in scores]}

    def _text_score(self, template, gray, keypoints, descriptors):
        """照片和某一张模板的文字相关系数；匹配失败返回 -1。"""
        cv2 = self.cv2
        if descriptors is None or template['descriptors'] is None:
            return -1.0
        pairs = self.matcher.knnMatch(template['descriptors'], descriptors, k=2)
        # Lowe 比值检验：最近邻明显比次近邻好，才认为是可靠的匹配点
        good = [p[0] for p in pairs if len(p) == 2 and p[0].distance < 0.75 * p[1].distance]
        if len(good) < self.min_inliers:
            return -1.0
        src = np.float32([template['keypoints'][m.queryIdx].pt for m in good])
        dst = np.float32([keypoints[m.trainIdx].pt for m in good])
        homography, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
        if homography is None or mask is None or int(mask.sum()) < self.min_inliers:
            return -1.0
        height, width = template['gray'].shape
        # 用单应性矩阵的逆把照片"拉正"到模板坐标系
        rectified = cv2.warpPerspective(gray, np.linalg.inv(homography), (width, height))
        bottom = min(height, template['text_bottom'])
        template_text = template['gray'][0:bottom].astype(np.float32)
        photo_text = rectified[0:bottom].astype(np.float32)
        return float(cv2.matchTemplate(photo_text, template_text, cv2.TM_CCOEFF_NORMED)[0, 0])


# ============================================================
# 2. 豆包视觉模型（火山方舟，OpenAI 兼容接口）
# ============================================================
class VlmClient(object):
    """调用火山方舟的 chat/completions 接口，只用标准库，Python2/3 都能跑。

    API Key 不写进代码和配置：从环境变量（默认 ARK_API_KEY）或 key 文件（默认 ~/.ark_api_key）读取。
    """

    def __init__(self, params):
        self.url = params['base_url'].rstrip('/') + '/chat/completions'
        self.model = params['model']
        self.timeout_s = params['request_timeout_s']
        self.prompt = params['prompt']
        self.max_width = int(params['max_image_width'])
        self.jpeg_quality = int(params['jpeg_quality'])
        self.extra_body = params.get('extra_body') or {}
        self.api_key = os.environ.get(params['api_key_env'], '').strip()
        key_file = os.path.expanduser(params['api_key_file'])
        if not self.api_key and os.path.exists(key_file):
            with open(key_file) as handle:
                self.api_key = handle.read().strip()

    def is_configured(self):
        """有 API Key、并且填写了推理接入点 model（不是占位符 REPLACE_ME）才算配置好。"""
        return bool(self.api_key) and bool(self.model) and self.model != 'REPLACE_ME'

    def ask(self, bgr_image):
        """发送图片和提示词，返回模型回答的文字。出错时抛出异常。"""
        import cv2
        try:
            from urllib2 import Request, urlopen          # Python2
        except ImportError:
            from urllib.request import Request, urlopen  # Python3
        image = bgr_image
        if image.shape[1] > self.max_width:
            ratio = self.max_width / float(image.shape[1])
            image = cv2.resize(image, (self.max_width, int(image.shape[0] * ratio)))
        ok, jpeg = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            raise RuntimeError('图片编码失败')
        image_url = 'data:image/jpeg;base64,' + to_text(base64.b64encode(jpeg.tobytes()))
        body = {
            'model': self.model,
            'messages': [{
                'role': 'user',
                'content': [
                    {'type': 'text', 'text': self.prompt},
                    {'type': 'image_url', 'image_url': {'url': image_url}},
                ],
            }],
        }
        body.update(self.extra_body)
        request = Request(to_native(self.url), data=json.dumps(body).encode('utf-8'))
        request.add_header('Content-Type', 'application/json')
        request.add_header('Authorization', to_native('Bearer ' + self.api_key))
        response = urlopen(request, timeout=self.timeout_s)
        reply = json.loads(response.read().decode('utf-8'))
        return to_text(reply['choices'][0]['message']['content'])


def parse_vlm_reply(text, fruit_aliases):
    """从模型回答中取出 (答案, 水果英文名)。取不到返回 None。

    先找 JSON（提示词要求模型输出 {"answer": 3, "fruit": "杨桃"}），
    找不到 JSON 时退一步：在文字里找 1~5 的数字和水果名。
    fruit_aliases：{'durian': ['榴莲'], 'starfruit': ['杨桃', '阳桃'], ...}
    """
    text = to_text(text)
    answer = None
    fruit_word = ''
    match = re.search(r'\{.*?\}', text, re.S)
    if match:
        try:
            data = json.loads(match.group(0))
            answer = int(data.get('answer'))
            fruit_word = to_text(data.get('fruit', ''))
        except (ValueError, TypeError):
            answer = None
    if answer is None:
        digits = re.findall(r'(?<!\d)([1-5])(?!\d)', text)
        if digits:
            answer = int(digits[-1])
        fruit_word = text
    fruit_key = None
    for key, aliases in fruit_aliases.items():
        if any(to_text(alias) in fruit_word for alias in aliases):
            fruit_key = key
            break
    if answer is None or not 1 <= answer <= 5 or fruit_key is None:
        return None
    return answer, fruit_key


# ============================================================
# 3. 组合：CardRecognizer
# ============================================================
class CardRecognizer(object):
    """同时启动离线匹配和豆包识别，按策略给出最终靶号。

    policy（recognition.yaml）：
      vlm_first     等豆包最多 vlm_wait_s 秒；豆包没结果/失败时用离线结果。（默认，最稳）
      offline_first 离线结果可信就立即采用（最快）；不可信再等豆包。
      offline_only  只用离线。
      vlm_only      只用豆包。
    都失败时：离线有结果就用离线的最佳猜测，否则用 default_targets，保证流程继续。
    """

    def __init__(self, rec_cfg, cards_dir, matcher=None, vlm=None):
        self.cfg = rec_cfg
        self.cards = rec_cfg['cards']
        self.fruit_to_moving = rec_cfg['fruit_to_moving_id']
        self.fruit_aliases = rec_cfg['fruit_aliases']
        self.policy = rec_cfg['policy']
        self.vlm_wait_s = rec_cfg['vlm']['wait_s']
        self.offline_min_score = rec_cfg['offline']['min_score']
        self.offline_min_margin = rec_cfg['offline']['min_margin']
        # matcher / vlm 可以从外部传入（测试时传假的）
        self.matcher = matcher
        self.vlm = vlm
        self.offline_result = None    # {'index','score','margin',...} 或 {'error': ...}
        self.vlm_result = None        # {'answer','fruit','text'} 或 {'error': ...}
        self.start_time = None

    # ---- 启动 ----
    def start(self, bgr_image, now):
        self.start_time = now
        self.offline_result = None
        self.vlm_result = None
        if self.policy != 'vlm_only' and self.matcher is not None:
            self._run_in_thread(self._offline_job, bgr_image)
        else:
            self.offline_result = {'error': 'disabled'}
        if self.policy != 'offline_only' and self.vlm is not None and self.vlm.is_configured():
            self._run_in_thread(self._vlm_job, bgr_image)
        else:
            self.vlm_result = {'error': 'disabled_or_no_key'}

    @staticmethod
    def _run_in_thread(job, image):
        thread = threading.Thread(target=job, args=(image,))
        thread.daemon = True
        thread.start()

    def _offline_job(self, image):
        try:
            started = time.time()
            result = self.matcher.match(image)
            result['seconds'] = round(time.time() - started, 3)
            self.offline_result = result
        except Exception as error:
            self.offline_result = {'error': to_text(error)}

    def _vlm_job(self, image):
        try:
            started = time.time()
            text = self.vlm.ask(image)
            parsed = parse_vlm_reply(text, self.fruit_aliases)
            if parsed is None:
                self.vlm_result = {'error': 'unparsable', 'text': text}
            else:
                self.vlm_result = {'answer': parsed[0], 'fruit': parsed[1], 'text': text,
                                   'seconds': round(time.time() - started, 3)}
        except Exception as error:
            self.vlm_result = {'error': to_text(error)}

    # ---- 查询 ----
    def _offline_confident(self):
        r = self.offline_result
        return (r is not None and 'error' not in r and r['score'] >= self.offline_min_score
                and r['margin'] >= self.offline_min_margin)

    def _targets_from_card(self, index, source):
        card = self.cards[index]
        return self._targets(card['answer'], card['fruit'], source)

    def _targets(self, answer, fruit, source):
        return {'rotating': int(answer), 'moving': int(self.fruit_to_moving[fruit]),
                'answer': int(answer), 'fruit': fruit, 'source': source}

    def poll(self, now):
        """返回最终结果字典；还需要等待时返回 None。"""
        elapsed = now - self.start_time
        vlm_ok = self.vlm_result is not None and 'error' not in self.vlm_result
        vlm_failed = self.vlm_result is not None and 'error' in self.vlm_result
        offline_done = self.offline_result is not None

        if self.policy in ('offline_first', 'offline_only') and self._offline_confident():
            return self._targets_from_card(self.offline_result['index'], 'offline')
        if self.policy == 'offline_only':
            return self.fallback() if offline_done else None
        if vlm_ok:
            return self._targets(self.vlm_result['answer'], self.vlm_result['fruit'], 'vlm')
        if vlm_failed or elapsed > self.vlm_wait_s:
            if self.policy == 'vlm_only':
                return self.fallback()
            if self._offline_confident():
                return self._targets_from_card(self.offline_result['index'], 'offline')
            return self.fallback() if offline_done else None
        return None

    def fallback(self):
        """都不可靠：离线有最佳猜测就用它，否则用默认靶号。"""
        r = self.offline_result
        if r is not None and 'error' not in r and r['score'] > 0:
            return self._targets_from_card(r['index'], 'offline_guess')
        default = self.cfg['default_targets']
        return {'rotating': int(default['rotating']), 'moving': int(default['moving']),
                'answer': None, 'fruit': None, 'source': 'default'}

    def summary(self):
        """给 runlog 保存的完整信息。"""
        return {'offline': self.offline_result, 'vlm': self.vlm_result, 'policy': self.policy}
