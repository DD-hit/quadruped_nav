#!/usr/bin/env python3
"""Launch the isolated Isaac D1 keyboard teleop on the server desktop."""
import os
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[1]
args = sys.argv[1:]
if '--gpu-index' not in args:
    args += ['--gpu-index', '1']
command = [sys.executable, str(root / 'scripts/run_gait.py'), '--teleop', '--policy', '--gui', *args]
os.execv(sys.executable, command)
