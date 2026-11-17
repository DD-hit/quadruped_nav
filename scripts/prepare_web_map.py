#!/usr/bin/env python3
"""Compose existing verified native navigation snapshot + unchanged localization PCD."""
import argparse,hashlib,json,shutil
from pathlib import Path
import yaml
p=argparse.ArgumentParser();p.add_argument('--data-root',type=Path,required=True);a=p.parse_args();data=a.data_root
parent=data/'maps/d1_full_room_20261109_02';run=data/'runs/navigation_core_isaac_demo_20261112_01';out=data/'maps/d1_full_room_web_20261113_01'
if out.exists():
 print(out);raise SystemExit(0)
assert json.loads((run/'verification.json').read_text())['result']=='PASS'
parent_meta=yaml.safe_load((parent/'manifest.yaml').read_text());assert hashlib.sha256((parent/'localization.pcd').read_bytes()).hexdigest()==parent_meta['files_sha256']['localization.pcd']
old=data/'maps/d1_full_room_20261109_02_nav';out.mkdir();(out/'configuration').mkdir()
for name in ['meta.yaml','octomap_msg.npz','layers.npz']:shutil.copyfile(run/'navigation_map'/name,out/name)
for src,dst in [(parent/'localization.pcd',out/'localization.pcd'),(parent/'manifest.yaml',out/'source_manifest.yaml'),(old/'navigation.pcd',out/'navigation.pcd')]:shutil.copyfile(src,dst)
meta=yaml.safe_load((out/'meta.yaml').read_text());meta['map_id']=out.name;(out/'meta.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
(out/'configuration/map_tools.yaml').write_text(yaml.safe_dump(dict(source='native navigation snapshot',resolution=meta['resolution'],planner=meta['planner']),sort_keys=False))
shutil.copyfile(run/'navigation.yaml',out/'configuration/navigation.yaml')
(out/'configuration/provenance.json').write_text(json.dumps(dict(source_session=str(run),navigation_source=str(old/'navigation.pcd'),localization_source=str(parent),same_map_frame=True,hardware_verified=False,navigation_layer_warning='Saved snapshot uses native robot_radius 0.43; full-body and hardware clearance remain unverified.'),indent=2)+'\n')
files={str(f.relative_to(out)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(out.rglob('*')) if f.is_file()}
manifest=dict(schema_version=1,kind='navigation_map_bundle',status='complete',map_id=out.name,frame_id='map',parent_map_id=parent.name,parent_manifest_sha256=files['source_manifest.yaml'],parent_localization_sha256=files['localization.pcd'],navigation_from_parent=[[1.,0,0,0],[0,1.,0,0],[0,0,1.,0],[0,0,0,1.]],map_from_camera_init=parent_meta['map_from_camera_init'],files_sha256=files,navigation_ready=False,simulation_workflow_verified=True,real_hardware_validation_complete=False)
(out/'manifest.yaml').write_text(yaml.safe_dump(manifest,sort_keys=False));print(out)
