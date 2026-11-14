#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from __future__ import print_function
import os
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ROOT, 'src/shoot_race/src'))
from shoot_race.compat import drop_python3_paths
drop_python3_paths()
import cv2
from shoot_race.config import load_config
from shoot_race.recognition import VlmClient, parse_vlm_reply

cfg = load_config(os.path.join(ROOT, 'src/shoot_race/config'))
client = VlmClient(cfg['recognition']['vlm'])
if not client.is_configured():
    raise SystemExit('API key missing. Run python2 /home/abot/syd/configure_syd_api.py first.')
card = cv2.imread(os.path.join(ROOT, 'src/shoot_race/config/cards/card01.jpg'))
if card is None:
    raise SystemExit('card01.jpg could not be loaded.')
started = time.time()
try:
    reply = client.ask(card)
    parsed = parse_vlm_reply(reply, cfg['recognition']['fruit_aliases'])
except Exception as error:
    detail = str(error).replace(client.api_key, '<redacted>')
    if hasattr(error, 'read'):
        detail += ' ' + error.read().decode('utf-8', 'replace').replace(client.api_key, '<redacted>')[:1000]
    print(detail)
    raise SystemExit(1)
elapsed = time.time() - started
print('Model: %s; elapsed: %.2fs; parsed: %s' % (client.model, elapsed, parsed))
if parsed != (1, 'durian'):
    raise SystemExit('FAILED: card01 expected answer=1 and fruit=durian.')
print('PASS: live API image recognition and competition response parser.')
if elapsed > cfg['recognition']['vlm']['wait_s']:
    print('Response exceeded vlm.wait_s; competition may use the offline fallback.')
