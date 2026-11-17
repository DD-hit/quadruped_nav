#!/usr/bin/env python3
"""Offline geometric tracking review; execution phase is reconstructed, not logged truth."""
import argparse, hashlib, json, math
from pathlib import Path
import numpy as np
from scipy.interpolate import BSpline
from scipy.spatial.transform import Rotation
import yaml

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--root',type=Path,required=True)
p.add_argument('--session',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args();out=a.output;out.mkdir(parents=True,exist_ok=False)
source=a.root/'ros2_ws/src/quadruped_navigation/src/closed_loop_controller.cpp'
hashes=json.loads((a.session/'navigation_source_sha256.json').read_text())
assert hashes[str(source.relative_to(a.root))]==hashlib.sha256(source.read_bytes()).hexdigest(), 'Controller changed since recording'
events=[json.loads(v) for v in (a.session/'events.jsonl').open()]
physics=[json.loads(v) for v in (a.session/'simulation/states.jsonl').open()]
assert json.loads((a.session/'verification.json').read_text())['result']=='PASS'
global_paths=[v for v in events if v['kind']=='global_path' and v['data']]
targets=[v for e in events if e['kind']=='request' for v in e['data']['targets']]
assert len(global_paths)==len(targets)==4

def stats(x):
    x=np.asarray(x,dtype=float)
    return dict(count=len(x),mean=float(x.mean()),median=float(np.median(x)),p95=float(np.percentile(x,95)),maximum=float(x.max())) if len(x) else None

def nearest(point,poly):
    first=poly[:-1,:2];segment=poly[1:,:2]-first
    fraction=np.clip(np.sum((point-first)*segment,axis=1)/np.maximum(np.sum(segment*segment,axis=1),1e-20),0.,1.)
    projected=first+fraction[:,None]*segment
    d2=np.sum((projected-point)**2,axis=1);i=int(np.argmin(d2))
    return float(np.sqrt(d2[i])),i,float(fraction[i])

curves={};samples=[];task_map={};active=None;task='';state='IDLE';phase=0.;last_cmd_t=0.;frozen=False
terminal_clock_checks=[];transition_checks=[];last_command=np.zeros(3)
for event in events:
    t=event['t'];kind=event['kind'];d=event['data']
    if kind=='controller_state':
        if d['task_id']!=task:
            task=d['task_id'];active=None;phase=0.;frozen=False
            if task and task not in task_map:task_map[task]=len(task_map)
        if d['state'] in ('ALIGNING','SUCCEEDED') and state not in ('ALIGNING','SUCCEEDED') and active is not None:
            terminal_clock_checks.append(dict(t=t,id=active['id'],estimated_phase_before_terminal_s=phase,duration_s=active['duration'],difference_s=active['duration']-phase))
        state=d['state']
        if active is not None and state in ('ALIGNING','SUCCEEDED'):phase=active['duration']
    elif kind=='spline':
        spline=BSpline(d['knots'],np.array(d['points']),d['order'])
        begin=float(d['knots'][d['order']]);end=float(d['knots'][-d['order']-1]);assert abs(begin)<1e-8
        times=np.linspace(begin,end,max(2,math.ceil((end-begin)/.01)+1))
        active=dict(id=d['id'],curve=spline,duration=end,received_t=t,start_time=d['start_time_s'],leg=task_map.get(task),times=times,poly=spline(times),max_phase=0.)
        curves[d['id']]=active;phase=0.;last_cmd_t=t;frozen=False
        transition_checks.append(dict(id=d['id'],receipt_minus_start_s=t-d['start_time_s']))
    elif kind=='cmd_vel':
        dt=t-last_cmd_t;dt=dt if 0<=dt<=.2 else 0.;last_cmd_t=t;last_command=np.array(d)
        if active is None:continue
        frozen=(np.linalg.norm(last_command[:2])<1e-12 and abs(last_command[2])>1e-12 and state not in ('ALIGNING','SUCCEEDED'))
        if state in ('ALIGNING','SUCCEEDED'):phase=active['duration']
        elif state=='TRACKING' and not frozen:phase=min(active['duration'],phase+dt)
        active['max_phase']=max(active['max_phase'],phase)
    elif kind=='body_pose' and active is not None and task and state in ('TRACKING','ALIGNING'):
        leg=active['leg']
        if leg is None:continue
        body=np.array(d);tau=phase
        if state=='TRACKING' and not frozen:tau=min(active['duration'],tau+max(0.,min(.2,t-last_cmd_t)))
        ref=active['curve'](tau);vref=active['curve'].derivative()(tau)
        geometric,i,f=nearest(body[:2],active['poly'])
        closest_phase=active['times'][i]*(1-f)+active['times'][i+1]*f
        endpoint=(closest_phase<.05 or closest_phase>active['duration']-.05)
        mode='final_heading' if state=='ALIGNING' else 'turning_in_place' if frozen else 'endpoint_correction' if tau>active['duration']-.01 else 'moving'
        global_distance=nearest(body[:2],np.array(global_paths[leg]['data']))[0]
        reference_global=nearest(ref[:2],np.array(global_paths[leg]['data']))[0]
        # A 100 ms neighbourhood shows sensitivity to unlogged progress/header timing.
        neighbourhood=np.linspace(max(0.,tau-.1),min(active['duration'],tau+.1),21)
        uncertainty=np.linalg.norm(active['curve'](neighbourhood)[:,:2]-body[:2],axis=1)
        samples.append(dict(t=t,leg=leg,id=active['id'],mode=mode,endpoint_projection=bool(endpoint),phase_s=tau,closest_phase_s=float(closest_phase),position=body.tolist(),reference=ref.tolist(),reference_speed=float(np.linalg.norm(vref[:2])),global_distance_m=global_distance,reference_global_distance_m=reference_global,local_geometric_distance_m=geometric,estimated_timed_error_m=float(np.linalg.norm(ref[:2]-body[:2])),timing_neighbourhood_min_m=float(uncertainty.min()),timing_neighbourhood_max_m=float(uncertainty.max()),command=last_command.tolist()))

assert len(task_map)==4 and len(curves)==18
moving=[v for v in samples if v['mode']=='moving']
interior=[v for v in moving if not v['endpoint_projection']]
legs=[]
for leg,target in enumerate(targets):
    selected=[v for v in samples if v['leg']==leg];m=[v for v in selected if v['mode']=='moving'];inside=[v for v in m if not v['endpoint_projection']]
    legs.append(dict(leg=leg+1,target=target,samples=len(selected),global_distance_m=stats([v['global_distance_m'] for v in m]),reference_global_distance_m=stats([v['reference_global_distance_m'] for v in m]),local_geometric_distance_m=stats([v['local_geometric_distance_m'] for v in m]),interior_geometric_distance_m=stats([v['local_geometric_distance_m'] for v in inside]),estimated_timed_error_m=stats([v['estimated_timed_error_m'] for v in m]),maximum_geometric_sample=max(m,key=lambda v:v['local_geometric_distance_m'])))

pt=np.array([v['t']-2 for v in physics]);positions=np.array([v['pos'] for v in physics]);quaternions=np.array([v['quat'] for v in physics]);rotation=Rotation.from_quat(quaternions[:,[1,2,3,0]])
applied=np.array([v['command'] for v in physics]);actual=np.array([v['body_velocity'] for v in physics])
effective=applied*np.array([v['policy_weight'] for v in physics])[:,None]
turn_mask=(np.linalg.norm(applied[:,:2],axis=1)<1e-12)&(abs(applied[:,2])>.01)&(pt>=0)
turn_starts=np.flatnonzero(turn_mask&~np.r_[False,turn_mask[:-1]]);turn_ends=np.flatnonzero(turn_mask&~np.r_[turn_mask[1:],False]);turning=[]
for i,j in zip(turn_starts,turn_ends):
    if pt[j]-pt[i]<.2:continue
    turning.append(dict(start_s=float(pt[i]),end_s=float(pt[j]),duration_s=float(pt[j]-pt[i]),physical_translation_m=float(np.linalg.norm(positions[j,:2]-positions[i,:2])),maximum_applied_translation_mps=float(np.linalg.norm(applied[i:j+1,:2],axis=1).max())))
bins=np.floor(pt*10).astype(int);binned=[]
for b in np.unique(bins[bins>=0]):
    mask=bins==b;cmd=applied[mask].mean(axis=0);vel=actual[mask].mean(axis=0);policy_command=effective[mask].mean(axis=0)
    binned.append(dict(t=float(pt[mask].mean()),command=cmd.tolist(),actual_velocity=vel.tolist(),effective_policy_command=policy_command.tolist(),translation_error_mps=float(np.linalg.norm(cmd[:2]-vel[:2]))))
translating=[v for v in binned if np.linalg.norm(v['command'][:2])>.05]
scene=yaml.safe_load((a.session/'simulation/scene_config.yaml').read_text())
config=yaml.safe_load((a.session/'navigation.yaml').read_text())['/navigation/local_planner']['ros__parameters']
radius=config['grid_map.double_cylinder_radius'];offset=config['grid_map.double_cylinder_offset'];forward=rotation.as_matrix()[:,:2,0]
clearances={};critical=None
for name,ob in scene['room'].items():
    if name in ('floor','ceiling'):continue
    center=np.array(ob['center'][:2]);half=np.array(ob['size'][:2])/2;best=None
    for sign in [-1,1]:
        cylinder=positions[:,:2]+sign*offset*forward;distance=np.linalg.norm(np.maximum(abs(cylinder-center)-half,0.),axis=1)-radius;i=int(np.argmin(distance))
        value=dict(obstacle=name,t=float(pt[i]),physical_index=i,minimum_gap_m=float(distance[i]),cylinder_sign=sign,physical_position=positions[i].tolist(),cylinder_center=cylinder[i].tolist(),physical_yaw_rad=float(rotation[i].as_euler('xyz')[2]))
        if best is None or value['minimum_gap_m']<best['minimum_gap_m']:best=value
    clearances[name]=best
    if critical is None or best['minimum_gap_m']<critical['minimum_gap_m']:critical=best
critical['tracking_sample']=min(samples,key=lambda v:abs(v['t']-critical['t']))

report=dict(scope='offline analysis of one saved simulation; no algorithm changes or new navigation run',source_session=str(a.session),source_events_sha256=hashlib.sha256((a.session/'events.jsonl').read_bytes()).hexdigest(),controller_source_matches_recorded_hash=True,
    limitations=['Controller exec_time and execution_frozen were not directly recorded. Timed reference phase is inferred from command/state callbacks and is not an exact controller trace.','Body-pose message header timestamps were not retained; receipt clock is used. Geometric distance is primary, timed error is an estimate.','Global-route difference is not tracking error: compare only the route for the active leg.','Clearance uses physical poses offline and the native double-cylinder approximation, not the complete D1/arm collision model.','Simulated lidar ignores robot self occlusion; this is not a real-hardware robustness acceptance.'],
    all_active_geometric_distance_m=stats([v['local_geometric_distance_m'] for v in samples]),moving_geometric_distance_m=stats([v['local_geometric_distance_m'] for v in moving]),moving_interior_geometric_distance_m=stats([v['local_geometric_distance_m'] for v in interior]),moving_estimated_timed_error_m=stats([v['estimated_timed_error_m'] for v in moving]),moving_global_distance_m=stats([v['global_distance_m'] for v in moving]),mode_sample_counts={k:sum(v['mode']==k for v in samples) for k in sorted(set(v['mode'] for v in samples))},legs=legs,
    terminal_phase_consistency=terminal_clock_checks,spline_receipt_timing=transition_checks,
    gait_velocity_response_100ms_bins=dict(translation_error_mps=stats([v['translation_error_mps'] for v in translating]),mean_command_speed_mps=float(np.mean([np.linalg.norm(v['command'][:2]) for v in translating])),mean_actual_speed_mps=float(np.mean([np.linalg.norm(v['actual_velocity'][:2]) for v in translating])),mean_effective_policy_command_speed_mps=float(np.mean([np.linalg.norm(v['effective_policy_command'][:2]) for v in translating]))),turning_drift_intervals=turning,
    minimum_native_clearance=critical,clearance_by_obstacle=clearances)
(out/'tracking_analysis.json').write_text(json.dumps(report,indent=2)+'\n')
(out/'tracking_samples.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in samples))
(out/'velocity_bins.json').write_text(json.dumps(binned)+'\n')
(out/'turning_drift.json').write_text(json.dumps(turning,indent=2)+'\n')

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Circle,Rectangle
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
fig,axes=plt.subplots(2,2,figsize=(13,10),dpi=160)
for leg,ax in enumerate(axes.ravel()):
    gp=np.array(global_paths[leg]['data']);sel=[v for v in samples if v['leg']==leg];xy=np.array([v['position'][:2] for v in sel])
    ax.plot(gp[:,0],gp[:,1],color='#d9a520',lw=2.5,label='当前目标的全局路线')
    first=True
    for c in curves.values():
        if c['leg']!=leg:continue
        t=np.linspace(0,max(.001,c['max_phase']),max(2,int(c['max_phase']/.025)+1));points=c['curve'](t)
        ax.plot(points[:,0],points[:,1],color='#d14949',lw=1.7,ls='--',label='局部参考的已执行前缀（估算进度）' if first else None);first=False
    ax.plot(xy[:,0],xy[:,1],color='#237cbd',lw=1.5,label='实际定位轨迹');ax.scatter(*targets[leg][:2],marker='x',color='black',s=50)
    ax.set_title(f"第 {leg+1} 段：局部几何偏差 P95 {legs[leg]['local_geometric_distance_m']['p95']*100:.1f} cm",fontproperties=font)
    ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2);ax.set_xlabel('map x / m');ax.set_ylabel('map y / m');ax.legend(prop=font,fontsize=8)
fig.tight_layout();fig.savefig(out/'三种轨迹分段对照.png');plt.close(fig)

fig,axes=plt.subplots(3,1,figsize=(14,10),dpi=160)
t=[];geometric=[];timed=[]
for v in moving:
    if t and v['t']-t[-1]>.25:
        t.append(float('nan'));geometric.append(float('nan'));timed.append(float('nan'))
    t.append(v['t']);geometric.append(v['local_geometric_distance_m']*100);timed.append(v['estimated_timed_error_m']*100)
axes[0].plot(t,geometric,color='#237cbd',label='到当前局部曲线的几何距离')
axes[0].plot(t,timed,color='#c94c4c',alpha=.7,label='估算时刻对齐误差（包含沿轨迹滞后）');axes[0].set_ylabel('cm');axes[0].set_title('移动阶段跟踪偏差：不把原地转向等待计为前进进度',fontproperties=font);axes[0].legend(prop=font)
axes[1].plot([v['t'] for v in binned],[np.linalg.norm(v['command'][:2]) for v in binned],label='仿真速度接口收到的指令（经看门狗）',color='#e2a822')
axes[1].plot([v['t'] for v in binned],[np.linalg.norm(v['actual_velocity'][:2]) for v in binned],label='物理机身速度（同一 0.1 s 时间箱均值）',color='#237cbd');axes[1].set_ylabel('m/s');axes[1].legend(prop=font)
for turn in turning:
    if turn['duration_s']>=1:
        axes[0].axvspan(turn['start_s'],turn['end_s'],color='gray',alpha=.1)
for leg in range(1,4):
    for ax in axes[:2]:ax.axvline(global_paths[leg]['t'],color='gray',alpha=.4,ls=':')
ax=axes[2];ob=scene['room'][critical['obstacle']];c=np.array(ob['center'][:2]);h=np.array(ob['size'][:2])/2;ax.add_patch(Rectangle(c-h,2*h[0],2*h[1],color='#777777',label='柱体实际几何'))
i=critical['physical_index'];mask=(pt>=critical['t']-8)&(pt<=critical['t']+8);ax.plot(positions[mask,0],positions[mask,1],color='#237cbd',label='实际物理轨迹（仅离线核对）')
for sign in [-1,1]:
    center=positions[i,:2]+sign*offset*forward[i];ax.add_patch(Circle(center,radius,fill=False,color='#c94c4c',lw=2,label='原生双圆柱外廓' if sign==-1 else None))
ax.scatter(*positions[i,:2],color='black',s=25);ax.set_aspect('equal');ax.set_xlabel('world x / m');ax.set_ylabel('world y / m');ax.set_title(f"最小物理余量：{critical['minimum_gap_m']*100:.2f} cm，ROS t={critical['t']:.3f} s；不代表完整整机外廓",fontproperties=font);ax.legend(prop=font)
for ax in axes:ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(out/'跟踪偏差与柱体余量.png');plt.close(fig)
print(json.dumps({k:report[k] for k in ['moving_geometric_distance_m','moving_interior_geometric_distance_m','moving_estimated_timed_error_m','moving_global_distance_m','terminal_phase_consistency','gait_velocity_response_100ms_bins']},indent=2))
print('critical clearance',critical['t'],critical['minimum_gap_m'])
