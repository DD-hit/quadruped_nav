from pathlib import Path
import json,shutil
r=Path('/home/rl/workspace/quadruped_nav');backup=Path('/data0/rl/quadruped_nav/runs/ground_fix_backup_20261116')
p=r/'ros2_ws/src/quadruped_web/quadruped_web/server.py';shutil.copy(p,backup/'web_server.py');s=p.read_text().replace("default='d1_full_room_web_20261113_01'","default='d1_full_room_web_20261116_02'");p.write_text(s)
p=r/'ros2_ws/src/quadruped_web/quadruped_web/maps.py';shutil.copy(p,backup/'web_maps.py');s=p.read_text().replace("label='D1 完整房间 · 当前导航图层' if d.name=='d1_full_room_web_20261113_01' else d.name", "label='D1 完整房间 · 地面修复版' if d.name=='d1_full_room_web_20261116_02' else 'D1 完整房间 · 历史图层' if d.name=='d1_full_room_web_20261113_01' else d.name");p.write_text(s)
p=r/'scripts/repair_ground_map.py';s=p.read_text().replace("if nav.joinpath('manifest.yaml').exists():print(nav);raise SystemExit(0)", "if nav.joinpath('manifest.yaml').exists():\n from localization_map_source import load_localization_source\n load_localization_source(nav);print(nav);raise SystemExit(0)");p.write_text(s)
p=r/'scripts/repair_ground_map.sh';p.write_text('''#!/usr/bin/env bash
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$project_root/ros2_ws/install/setup.bash"
/usr/bin/python3 "$project_root/scripts/repair_ground_map.py"
/usr/bin/python3 "$project_root/tests/test_map_alignment.py"
''');p.chmod(0o755)
p=r/'docs/project_manifest.json';m=json.loads(p.read_text());m['ground_map_repair_20261116']=dict(status='map_export_and_layers_verified',map_id='d1_full_room_web_20261116_02',localization_map_id='d1_full_room_20261116_01',report='/data0/rl/quadruped_nav/runs/ground_map_repair_20261116_02/verification.json',native_global_routes_passed=8,hardware_verified=False,closed_loop_motion_retested=False,estimator_root_cause_resolved=False,remaining_raised_ground_cells=7,original_maps_preserved=True);m['web_console']['map_id']='d1_full_room_web_20261116_02';p.write_text(json.dumps(m,indent=2,ensure_ascii=False)+'\n')
