#!/usr/bin/env python3
"""Focused live HTTP -> ROS -> FAST-LIO -> Isaac manual-localization test."""
import json
import math
import os
from pathlib import Path
import threading
import time
from urllib.request import Request,urlopen
from urllib.error import HTTPError

OUT=Path(os.environ.get('MANUAL_TEST_OUTPUT','/data0/rl/quadruped_nav/evaluations/manual_localization_20261117/live_test'))
OUT.mkdir(exist_ok=False)
BASE='http://127.0.0.1:8766';CLIENT='d4'*16
checks=[];running=True
log=(OUT/'states.jsonl').open('w',buffering=1)

def call(path,body=None):
    headers={} if body is None else {'Content-Type':'application/json','Origin':BASE,'X-Client-ID':CLIENT}
    req=Request(BASE+path,data=None if body is None else json.dumps(body).encode(),headers=headers)
    with urlopen(req,timeout=15) as response:return json.load(response)

def check(name,condition,**details):
    checks.append(dict(name=name,passed=bool(condition),**details))
    (OUT/'checks.json').write_text(json.dumps(checks,indent=2))
    print(json.dumps(checks[-1]),flush=True)
    if not condition:raise AssertionError(name)

def state():
    s=call('/api/state')
    log.write(json.dumps({k:v for k,v in s.items() if k not in ['cloud','paths']})+'\n')
    return s

def wait(predicate,timeout=120):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        try:s=state()
        except OSError:time.sleep(.3);continue
        if predicate(s):return s
        time.sleep(.2)
    raise TimeoutError(json.dumps(s))

def heartbeat():
    while running:
        try:call('/api/heartbeat',{})
        except Exception:pass
        time.sleep(.5)

try:
    s=wait(lambda s:s['clock_s']>5 and s['inputs_ready'] and s['localization'].get('phase')=='waiting_initial_pose',300)
    check('waiting_for_user_initialpose',not s['localization_ready'] and s['pose'] is None)
    data=call('/api/map')
    check('existing_pcd_map_used',data['localization_sha256']=='e01bfcb97e24ef77985dddae401673d8fbe89da94c2d6d5a0e0e63f3843daece')
    support=min(data['support_points'],key=lambda p:(p[0]-.35)**2+(p[1]+.2)**2+10*(p[2]+.3)**2)
    initial=dict(x=support[0],y=support[1],z=support[2]+data['robot']['base_height_above_support_m'],yaw=math.radians(10))
    target=dict(x=support[0],y=support[1],z=support[2],yaw=0.)
    try:
        call('/api/navigation',dict(kind='navigate_to_pose',poses=[target],localization_confirmed=True))
        check('navigation_blocked_before_localization',False)
    except HTTPError as e:check('navigation_blocked_before_localization',e.code==409)
    check('manual_pose_published',call('/api/initialpose',dict(pose=initial))['result']['sent'],initial=initial)
    s=wait(lambda s:s['localization_ready'],90)
    check('manual_initialization_tracks',s['localization']['registration_backend']=='fast_lio_localization2',
        pose=s['pose'],quality=s['localization']['quality'],initialization_correction=s['localization'].get('initialization_correction'))
    t=s['clock_s'];stationary=[]
    while s['clock_s']<t+6:
        s=state();stationary.append(s);time.sleep(.2)
    check('stationary_tracking_stays_ready',all(x['localization_ready'] for x in stationary))
    # Nominal map origin is a user-provided approximate pose, not simulator truth feedback.
    check('near_expected_initial_location',math.hypot(s['pose']['x'],s['pose']['y'])<.18 and abs(s['pose']['yaw'])<math.radians(5),pose=s['pose'])
    threading.Thread(target=heartbeat,daemon=True).start()
    call('/api/control',dict(mode='manual'))
    start=s['pose'].copy();begin=s['clock_s'];moving=[]
    while s['clock_s']<begin+5:
        call('/api/manual',dict(velocity=[.12,0.,.10]));time.sleep(.08)
        s=state();moving.append(s)
    call('/api/stop',{})
    check('moving_tracking_stays_ready',all(x['localization_ready'] for x in moving),samples=len(moving))
    distance=math.hypot(s['pose']['x']-start['x'],s['pose']['y']-start['y'])
    check('body_motion_observed',distance>.15,localized_displacement_m=distance)
    t=s['clock_s'];s=wait(lambda s:s['clock_s']>t+5 and s['control']['mode']=='idle',90)
    check('tracking_after_stop',s['localization_ready'],pose=s['pose'])
    # Test a second request through the same UI instead of requiring process restart.
    support=min(data['support_points'],key=lambda p:(p[0]-s['pose']['x']-.2)**2+(p[1]-s['pose']['y']+.1)**2+10*(p[2]+.3)**2)
    initial=dict(x=support[0],y=support[1],z=support[2]+data['robot']['base_height_above_support_m'],yaw=s['pose']['yaw']+math.radians(5))
    old_epoch=s['localization']['epoch'];call('/api/initialpose',dict(pose=initial))
    s=wait(lambda s:s['localization_ready'] and s['localization']['epoch']>old_epoch,90)
    check('second_initialpose_without_restart',True,epoch=s['localization']['epoch'],pose=s['pose'])
    check('web_bridge_healthy',s['bridge_error'] is None)
    result=dict(result='PASS',checks=checks,scope='Live Web manual init + motion tracking; no independent accuracy certification or navigation-goal completion test')
except Exception:
    import traceback
    result=dict(result='FAIL',checks=checks,error=traceback.format_exc())
    print(result['error'],flush=True)
finally:
    running=False
    try:call('/api/stop',{})
    except Exception:pass
    log.close();(OUT/'result.json').write_text(json.dumps(result,indent=2))
if result['result']!='PASS':raise SystemExit(1)
