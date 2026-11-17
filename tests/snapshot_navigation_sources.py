"""Record exact production inputs for an owned regression run."""
import hashlib,json
from pathlib import Path
from datetime import datetime,timezone
FILES=[
 'ros2_ws/src/quadruped_navigation/CMakeLists.txt',
 'ros2_ws/src/quadruped_navigation/include/plan_manage/scan_replan_fsm.h',
 'ros2_ws/src/quadruped_navigation/src/uniform_bspline.cpp',
 'ros2_ws/src/quadruped_navigation/src/planner_manager.cpp',
 'ros2_ws/src/quadruped_navigation/src/bspline_optimizer.cpp',
 'ros2_ws/src/quadruped_navigation/src/dyn_a_star.cpp',
 'ros2_ws/src/quadruped_navigation/src/scan_replan_fsm.cpp',
 'ros2_ws/src/quadruped_navigation/src/grid_map.cpp',
 'ros2_ws/src/quadruped_navigation/src/closed_loop_controller.cpp',
 'ros2_ws/src/quadruped_navigation/include/plan_manage/route_reference.h',
 'ros2_ws/src/quadruped_navigation/include/plan_manage/planner_manager.h',
 'ros2_ws/src/quadruped_navigation/include/path_searching/dyn_a_star.h',
 'ros2_ws/src/quadruped_navigation/include/plan_env/grid_map.h',
 'ros2_ws/src/quadruped_navigation/include/bspline_opt/bspline_optimizer.h',
 'ros2_ws/src/quadruped_navigation/scripts/navigation_api.py',
 'ros2_ws/src/quadruped_navigation/config/navigation.yaml',
 'ros2_ws/src/quadruped_sim/scripts/gait_ros_cmd_ready.py',
 'ros2_ws/src/quadruped_sim/scripts/runtime_gait_lidar.py',
 'ros2_ws/src/quadruped_sim/scripts/runtime_obstacles.py',
 'ros2_ws/src/quadruped_sim/config/runtime_obstacles.yaml',
 'scripts/run_gait_cmd_vel.py',
 'ros2_ws/install/quadruped_navigation/lib/quadruped_navigation/local_planner',
 'ros2_ws/install/quadruped_navigation/lib/quadruped_navigation/navigation_api.py']
def write_snapshot(root,out):
 root,out=Path(root),Path(out)
 if not out.is_dir():raise ValueError('Run directory must exist')
 hashes={}
 for name in FILES:
  h=hashlib.sha256()
  with (root/name).open('rb') as f:
   for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
  hashes[name]=h.hexdigest()
 (out/'source_hashes.json').write_text(json.dumps(dict(recorded_utc=datetime.now(timezone.utc).isoformat(),files_sha256=hashes),indent=2)+'\n')
if __name__=='__main__':
 import sys
 write_snapshot(sys.argv[1],sys.argv[2])
