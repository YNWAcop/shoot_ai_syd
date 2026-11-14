#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from __future__ import print_function
import getpass
import json
import os
import re
import tempfile
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ROOT, 'src/shoot_race/src'))
from shoot_race.compat import drop_python3_paths
drop_python3_paths()
import yaml
CONFIG = os.path.join(ROOT, 'src/shoot_race/config/recognition.yaml')

def atomic_write(path, text, mode):
    fd, temporary = tempfile.mkstemp(dir=os.path.dirname(path))
    try:
        os.fchmod(fd, mode)
        with os.fdopen(fd, 'wb') as handle:
            handle.write(text.encode('utf-8'))
        os.rename(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)

with open(CONFIG, 'rb') as handle:
    source = handle.read().decode('utf-8')
params = yaml.safe_load(source)['vlm']
try:
    input_text = raw_input
except NameError:
    input_text = input
default_model = params['model']
model = input_text('Model ID [%s]: ' % default_model).strip() or default_model
if not re.match(r'^[A-Za-z0-9_.:-]+$', model):
    raise SystemExit('Invalid model ID.')
key = getpass.getpass('Your actual API Key (hidden): ').strip()
if not key or any(char.isspace() for char in key):
    raise SystemExit('Enter the actual secret value, not its display name.')
source, count = re.subn(r'(?m)^  model:.*$', lambda match: '  model: ' + json.dumps(model), source)
if count != 1:
    raise SystemExit('Expected exactly one model entry.')
atomic_write(os.path.expanduser(params['api_key_file']), key + '\n', 0o600)
atomic_write(CONFIG, source, 0o644)
print('Saved syd API configuration. The key was not printed.')
print('Verify with: python2 /home/abot/syd/test_syd_api.py')
