#!/usr/bin/env python3
"""Run the installed Web package against existing ROS 2 Humble interfaces."""
import os
from pathlib import Path
import sys
root=Path(__file__).resolve().parents[1]
os.environ.update(ROS_DOMAIN_ID='42',ROS_LOCALHOST_ONLY='0',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',FASTRTPS_DEFAULT_PROFILES_FILE=str(root/'ros2_ws/src/quadruped_bringup/config/dds/local_sim.xml'))
from quadruped_web.server import main
sys.argv=[sys.argv[0],'--root',str(root),*sys.argv[1:]]
main()
