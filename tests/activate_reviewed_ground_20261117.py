from pathlib import Path
import json,shutil
r=Path('/home/rl/workspace/quadruped_nav');out=Path('/data0/rl/quadruped_nav/runs/ground_manual_edit_20261117_01');old='d1_full_room_web_20261116_02';new='d1_full_room_web_20261117_01'
assert json.loads((out/'verification.json').read_text())['result']=='PASS'
p=r/'ros2_ws/src/quadruped_web/quadruped_web/server.py';shutil.copy(p,out/'server_before.py');s=p.read_text();assert "default='"+old+"'" in s;s=s.replace("default='"+old+"'","default='"+new+"'");p.write_text(s)
p=r/'ros2_ws/src/quadruped_web/quadruped_web/maps.py';shutil.copy(p,out/'maps_before.py');s=p.read_text().replace("'D1 完整房间 · 地面修复版' if d.name=='"+old+"'", "'D1 完整房间 · 精修版' if d.name=='"+new+"' else 'D1 完整房间 · 自动修复版' if d.name=='"+old+"'");p.write_text(s)
p=r/'docs/project_manifest.json';m=json.loads(p.read_text());m['web_console']['map_id']=new;m['ground_manual_edit_20261117']=dict(map_id=new,source_map_id=old,reviewed_points=7,remaining_raised_ground_cells=0,localization_pcd_changed=False,upper_layer_unchanged=True,global_routes_passed=8,report=str(out/'verification.json'),hardware_verified=False,closed_loop_motion_retested=False,runtime_status='stopped');p.write_text(json.dumps(m,indent=2,ensure_ascii=False)+'\n')
