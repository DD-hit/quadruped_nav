# SPDX-License-Identifier: MIT
import hashlib,json,subprocess,tempfile,sys
from pathlib import Path
import numpy as np,yaml

class MapCatalog:
 def __init__(self,root,data_root,exporter):
  self.root,self.data_root,self.exporter=Path(root),Path(data_root),Path(exporter);self.cache={}
  sys.path.insert(0,str(self.root/'ros2_ws/src/quadruped_localization/scripts'))
 def listing(self):
  result=[]
  for d in sorted((self.data_root/'maps').iterdir()):
   if not (d/'manifest.yaml').is_file():continue
   m=yaml.safe_load((d/'manifest.yaml').read_text())
   if m.get('kind')=='navigation_map_bundle' and m.get('status')=='complete':result.append(dict(id=d.name,label='D1 完整房间 · 精修版' if d.name=='d1_full_room_web_20261117_01' else 'D1 完整房间 · 自动修复版' if d.name=='d1_full_room_web_20261116_02' else 'D1 完整房间 · 历史图层' if d.name=='d1_full_room_web_20261113_01' else d.name,simulation_verified=m.get('simulation_workflow_verified',False)))
  return result
 def load(self,map_id):
  if map_id not in [x['id'] for x in self.listing()]:raise ValueError('Unknown committed navigation map')
  if map_id in self.cache:return self.cache[map_id]
  from localization_map_source import load_localization_source
  directory=self.data_root/'maps'/map_id;info,_=load_localization_source(directory)
  meta=yaml.safe_load((directory/'meta.yaml').read_text());manifest=yaml.safe_load((directory/'manifest.yaml').read_text());octree=np.load(directory/'octomap_msg.npz',allow_pickle=False)
  if str(octree['frame_id'][0])!='map' or str(octree['octomap_id'][0])!='OcTree':raise ValueError('Expected map-frame OcTree')
  with tempfile.NamedTemporaryFile() as f:
   f.write(octree['data'].astype(np.int8).tobytes());f.flush();occupied=json.loads(subprocess.check_output([str(self.exporter),f.name,str(float(octree['resolution'][0])),'1' if bool(octree['binary'][0]) else '0'],timeout=30))
  layers=np.load(directory/'layers.npz',allow_pickle=False);payload={}
  for name,prefix in [('walk','traversable'),('blocked','preblocked'),('risk','risk')]:
   if str(layers[prefix+'_frame_id'][0])!='map':raise ValueError('Layer frame differs from map')
   scale=layers[prefix+'_scale'].tolist() if prefix+'_scale' in layers else [meta['resolution']]*3
   points=layers[prefix+'_points'];payload[name]=dict(points=points.tolist(),scale=scale)
   if name=='risk':payload[name]['intensity']=layers['risk_intensity'].tolist()
  payload['occupied']=dict(voxels=occupied)
  # Support surfaces are the top faces of the existing traversable voxels.
  support=layers['traversable_points'].astype(float).copy();support[:,2]+=float(layers['traversable_scale'][2])/2
  bounds=meta['bounds'];identity=hashlib.sha256((directory/'manifest.yaml').read_bytes()).hexdigest()
  body=dict(id=map_id,label=next(x['label'] for x in self.listing() if x['id']==map_id),frame='map',revision=identity,resolution=meta['resolution'],bounds=bounds,layers=payload,support_points=support.tolist(),support_scale=layers['traversable_scale'].tolist(),localization_sha256=info['localization_sha256'],localization_pcd_paired=True,hardware_verified=False,layer_parameters=meta['planner'],known_limits=['初始化可能误接受错误初值；跟踪状态不代表定位准确','当前导航外廓为原生双圆柱近似，未经完整整机与真机验收'])
  # The stand height is robot-specific configuration, independent of map/world z.
  profile=yaml.safe_load((self.root/'ros2_ws/src/quadruped_bringup/config/robots/d1_edu.yaml').read_text())
  body['robot']=dict(id='d1_edu',label=profile['display_name'],base_height_above_support_m=json.loads((self.root/'ros2_ws/src/quadruped_sim/config/d1_standby_initial_state.json').read_text())['base_position_world_m'][2],footprint_radius_m=profile['footprint']['radius_m'],height_m=profile['footprint']['height_m'],sdk_base_alignment='unverified')
  self.cache[map_id]=body;return body
