#!/usr/bin/env python3
"""Start the project FAST-LIO runtime in system ROS Python (no simulation launch)."""
from pathlib import Path
import sys
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'ros2_ws/src/quadruped_localization/scripts'))
from lio_runtime import main
raise SystemExit(main(['--project-root',str(root),*sys.argv[1:]]))
