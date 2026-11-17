#!/usr/bin/env python3
"""Start this project's saved-map localization runtime."""
from pathlib import Path
import sys
root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'ros2_ws/src/quadruped_localization/scripts'))
from localization_runtime import main
raise SystemExit(main(['--project-root', str(root), *sys.argv[1:]]))
