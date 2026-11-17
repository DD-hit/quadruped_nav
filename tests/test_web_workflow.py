#!/usr/bin/env python3
"""HTTP -> real ROS 2 -> Isaac workflow test. No simulator truth fed into navigation."""
import argparse,json,time,threading,urllib.request,urllib.error,math
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=False,parents=True)
BASE='http://127.0.0.1:8766';CLIENT='a1'*16;checks=[];running=True;last={};events=(a.output/'events.jsonl').open('w',buffering=1)
def call(path,body=None,client=CLIENT,origin=True):
 headers={}
 if body is not None:
  headers={'Content-Type':'application/json','X-Client-ID':client}
  if origin:headers['Origin']=BASE
 req=urllib.request.Request(BASE+path,data=None if body is None else json.dumps(body).encode(),headers=headers)
 try:
  with urllib.request.urlopen(req,timeout=90) as f:return json.load(f)
 except urllib.error.HTTPError as e:
  e.response_body=e.read().decode();raise
def check(name,passed,**data):
 row=dict(name=name,passed=bool(passed),**data);checks.append(row);(a.output/'checks.json').write_text(json.dumps(checks,indent=2)+'\n');print(json.dumps(row),flush=True)
 if not passed:raise AssertionError(name)
def wait(fn,timeout=120):
 end=time.monotonic()+timeout
 while True:
  state=call('/api/state');events.write(json.dumps({k:v for k,v in state.items() if k not in ('cloud','paths')})+'\n');events.flush()
  if fn(state):return state
  if time.monotonic()>end:raise TimeoutError(json.dumps(state))
  time.sleep(.2)
def heartbeat():
 while running:
  try:call('/api/heartbeat',{})
  except Exception:pass
  time.sleep(.7)
try:
 data=call('/api/map');check('real_octomap_loaded',len(data['layers']['occupied']['voxels'])>1000,count=len(data['layers']['occupied']['voxels']),revision=data['revision']);check('localization_pcd_paired',data['localization_sha256']=='323c21632a8c76eb23f73374e5720916b41d339356182cc4554ae5cbe9fd6b72')
 try:call('/api/stop',{},origin=False);check('cross_origin_rejected',False)
 except urllib.error.HTTPError as e:check('cross_origin_rejected',e.code==403)
 try:call('/api/initialpose',{'pose':{'x':float('nan'),'y':0,'z':0,'yaw':0}});check('invalid_pose_rejected',False)
 except urllib.error.HTTPError as e:check('invalid_pose_rejected',e.code==409)
 s=wait(lambda s:s['clock_s']>5 and s['inputs_ready'] and s['localization'].get('phase')=='waiting_initial_pose',300);check('explicit_initialpose_required',not s['localization_ready'])
 # Known approximate starting pose only. Height follows the displayed support layer.
 near=min(data['support_points'],key=lambda p:p[0]**2+p[1]**2+(p[2]+.3)**2)
 pose=dict(x=.35,y=-.2,z=near[2]+data['robot']['base_height_above_support_m'],yaw=.12)
 call('/api/initialpose',{'pose':pose});s=wait(lambda s:s['localization_ready'],60);check('initialpose_to_tracking',True,pose=s['pose'],quality=s['localization'].get('quality'))
 thread=threading.Thread(target=heartbeat,daemon=True);thread.start()
 def target(x,y,yaw=0):
  support=min(data['support_points'],key=lambda p:(p[0]-x)**2+(p[1]-y)**2+10*(p[2]+.3)**2);return dict(x=support[0],y=support[1],z=support[2],yaw=yaw)
 for name,kind,poses in [('single','navigate_to_pose',[target(.8,-.5)]),('multi','follow_waypoints',[target(.8,-1.),target(0,0)])]:
  call('/api/navigation',{'kind':kind,'poses':poses,'localization_confirmed':True});s=wait(lambda s:s['task'].get('phase') in ('succeeded','failed','rejected','canceled'),360)
  check(name+'_navigation',s['task']['phase']=='succeeded',task=s['task'],pose=s['pose'])
 call('/api/navigation',{'kind':'navigate_to_pose','poses':[target(3.,0)],'localization_confirmed':True});wait(lambda s:s['task'].get('phase')=='executing',20)
 call('/api/cancel',{});s=wait(lambda s:s['task']['phase'] in ('canceled','failed') and s['control']['stop_confirmed'],30);check('navigation_cancel_and_stop_ack',s['task']['phase']=='canceled',task=s['task'])
 call('/api/control',{'mode':'manual'});start_pose=call('/api/state')['pose'];begin=call('/api/state')['clock_s'];end=time.monotonic()+30
 while call('/api/state')['clock_s']<begin+2 and time.monotonic()<end:
  call('/api/manual',{'velocity':[.1,.05,.1]});time.sleep(.1)
 call('/api/manual',{'velocity':[.1,.05,.1]});s=wait(lambda s:max(abs(v) for v in s['control']['command'])>.01,4);check('manual_velocity_routed',True,command=s['control']['command']);displacement=math.hypot(s['pose']['x']-start_pose['x'],s['pose']['y']-start_pose['y']);check('manual_body_motion_observed',displacement>.03,localized_displacement_m=displacement)
 try:call('/api/control',{'mode':'manual'},client='b2'*16);check('control_ownership_enforced',False)
 except urllib.error.HTTPError as e:check('control_ownership_enforced',e.code==409)
 s=wait(lambda s:s['control']['command']==[0.,0.,0.],3);check('manual_timeout_zero',True,reason=s['control']['reason'])
 try:call('/api/manual',{'velocity':[.4,.4,0]});check('manual_norm_limit_enforced',False)
 except urllib.error.HTTPError as e:check('manual_norm_limit_enforced',e.code==409)
 call('/api/stop',{});s=wait(lambda s:s['control']['mode']=='idle' and s['control']['command']==[0.,0.,0.],5);check('stop_zero_command',True)
 # Explicitly stop heartbeat, acquire manual control and ensure server-side lease expires.
 running=False;thread.join(timeout=2);call('/api/control',{'mode':'manual'});s=wait(lambda s:s['control']['mode']=='idle',9);check('browser_lease_expiry',True,reason=s['control']['reason'])
 check('bridge_healthy',s['bridge_error'] is None,error=s['bridge_error'])
 call('/api/map/select',{'id':data['id']});s=call('/api/state');check('map_switch_clears_localization',not s['localization_ready'] and s['control']['command']==[0.,0.,0.])
 check('camera_and_actions_not_faked',not s['capabilities']['camera'] and not s['capabilities']['preset_actions'])
 (a.output/'verification.json').write_text(json.dumps(dict(result='PASS',checks=checks,scope='HTTP interfaces + actual Isaac sensors + FAST-LIO + native navigation; no truth feedback and no hardware acceptance'),indent=2)+'\n')
except Exception:
 import traceback
 (a.output/'error.txt').write_text(traceback.format_exc());(a.output/'verification.json').write_text(json.dumps(dict(result='FAIL',checks=checks),indent=2)+'\n');raise
finally:
 running=False
 try:call('/api/stop',{})
 except Exception:pass
 events.close()
