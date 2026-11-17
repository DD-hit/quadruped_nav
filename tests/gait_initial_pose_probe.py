#!/usr/bin/env python3
"""Test-only one-time initial placement; execute unchanged production runtime thereafter."""
import argparse,sys,json,hashlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--test-root',type=Path,required=True);p.add_argument('--test-map-x',type=float,required=True);p.add_argument('--test-map-y',type=float,required=True);p.add_argument('--test-map-yaw-deg',type=float,required=True);a,remaining=p.parse_known_args()
original=a.test_root/'ros2_ws/src/quadruped_sim/scripts/gait_ros_cmd_ready.py';source=original.read_text()
old="spawn_position=np.asarray(settled['base_position_world_m'],dtype=float).copy();spawn_position[0]+=a.spawn_offset_x\n robot.set_world_pose(position=spawn_position,orientation=np.asarray(settled['base_quat_wxyz']))"
new="""spawn_position=np.asarray(settled['base_position_world_m'],dtype=float).copy();spawn_position[0]+=a.spawn_offset_x
 test_gauge=json.loads(Path('/data0/rl/quadruped_nav/runs/d1_full_room_survey_20261109_02/contract.json').read_text())['world_from_odom_initial_placement']
 spawn_position[:2]+=np.array(test_gauge['rotation'])@np.array(TEST_MAP_XY)
 test_orientation=(Rotation.from_euler('z',TEST_MAP_YAW_DEG,degrees=True)*Rotation.from_quat(np.asarray(settled['base_quat_wxyz'])[[1,2,3,0]])).as_quat()
 (out/'initial_placement_test.json').write_text(json.dumps(dict(map_xy=TEST_MAP_XY,map_yaw_deg=TEST_MAP_YAW_DEG,original_source_sha256=ORIGINAL_SHA256,pose_changed_only_before_physics_loop=True),indent=2))
 robot.set_world_pose(position=spawn_position,orientation=test_orientation[[3,0,1,2]])"""
assert source.count(old)==1,'Production spawn changed; test patch requires review'
sys.path.insert(0,str(original.parent));sys.argv=[str(original),*remaining]
exec(compile(source.replace(old,new),str(original),'exec'),dict(__name__='__main__',__file__=str(original),TEST_MAP_XY=[a.test_map_x,a.test_map_y],TEST_MAP_YAW_DEG=a.test_map_yaw_deg,ORIGINAL_SHA256=hashlib.sha256(original.read_bytes()).hexdigest()))
