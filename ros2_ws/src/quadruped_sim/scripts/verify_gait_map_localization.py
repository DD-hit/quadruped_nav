"""Check accuracy and TF continuity using fresh Isaac observations, not a replay."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import yaml


def stamp(header):
    return header.stamp.sec*1_000_000_000+header.stamp.nanosec


def pose_row(header, pose):
    p,q=pose.position,pose.orientation
    return dict(ns=stamp(header),position=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w])


def matrix(row):
    t=np.eye(4);t[:3,:3]=Rotation.from_quat(row['quaternion']).as_matrix();t[:3,3]=row['position']
    return t


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project-root',type=Path,required=True)
    p.add_argument('--map-id',required=True)
    p.add_argument('--mapping-run',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--spawn-offset-x',type=float,default=0.)
    p.add_argument('--capture-registration-inputs',action='store_true')
    a=p.parse_args(argv);root=a.project_root.resolve();out=a.output_dir.resolve()
    sys.path.insert(0,str(root/'ros2_ws/src/quadruped_localization/scripts'))
    from map_bundle import identifier
    identifier(a.map_id)
    if not -4.8<=a.spawn_offset_x<=0.:
        p.error('Verified test start range is [-4.8, 0] m')
    host=yaml.safe_load((root/'ros2_ws/src/quadruped_bringup/config/hosts/server.yaml').read_text())
    cfg=yaml.safe_load((root/'ros2_ws/src/quadruped_sim/config/gait_map_localization_acceptance.yaml').read_text())
    stationary_cfg=yaml.safe_load((root/'ros2_ws/src/quadruped_localization/config/localization_dynamic_gait.yaml').read_text())['runtime']
    map_dir=Path(host['data_root'])/'maps'/a.map_id
    map_hashes={str(q.relative_to(map_dir)):hashlib.sha256(q.read_bytes()).hexdigest() for q in map_dir.rglob('*') if q.is_file()}
    out.mkdir(parents=True,exist_ok=False)
    (out/'acceptance_config.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    contract=json.loads((a.mapping_run/'contract.json').read_text())
    (out/'map_gauge.json').write_text(json.dumps(contract['world_from_odom_initial_placement'],indent=2)+'\n')
    initial=json.loads((a.mapping_run/'initial_state.json').read_text())
    (out/'map_initial_state.json').write_text(json.dumps(initial,indent=2)+'\n')
    os.environ['ROS_DOMAIN_ID']=str(host['ros_domain_id']);os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist,PoseStamped,PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    interrupted=False;children={};handles=[];failure=None;codes={};started=time.monotonic()
    def on_signal(signum,frame):
        nonlocal interrupted
        interrupted=True
    def launch(name,command):
        h=(out/(name+'.log')).open('w');handles.append(h)
        process=subprocess.Popen(command,cwd=root,stdout=h,stderr=subprocess.STDOUT,start_new_session=True)
        children[name]=process
    def stop(name):
        process=children[name]
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=10)
        return process.returncode

    class Check(Node):
        def __init__(self):
            super().__init__('quadruped_gait_map_localization_acceptance')
            self.clock=None;self.clock_wall=0.;self.poses=[];self.odoms=[];self.transforms=[];self.edges=set()
            self.status={};self.status_events=[];self.runtime={};self.base={};self.counts=dict(cloud=0,imu=0,clock=0)
            self.stage='startup';self.windows=[];self.command=[0.,0.];self.permitted=False;self.last_report=0.
            self.seed_attempts=[]
            self.registration_index=[];self.status_snapshots=[]
            if a.capture_registration_inputs:(out/'registration_inputs').mkdir()
            self.pub=self.create_publisher(Twist,'/cmd_vel',1)
            self.seed=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
            sensor=QoSProfile(depth=200,reliability=ReliabilityPolicy.BEST_EFFORT)
            latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Clock,'/clock',self.on_clock,sensor)
            self.create_subscription(PoseStamped,'/localization/pose',lambda m:self.poses.append(pose_row(m.header,m.pose)),200)
            self.create_subscription(Odometry,'/odom',self.on_odom,200)
            self.create_subscription(TFMessage,'/tf',self.on_tf,300)
            self.create_subscription(TFMessage,'/tf_static',self.on_tf,latched)
            self.create_subscription(String,'/localization/status',self.on_status,latched)
            self.create_subscription(String,'/lio/runtime_status',lambda m:self.runtime.update(json.loads(m.data)),50)
            self.create_subscription(String,'/lio/base_odometry_status',lambda m:self.base.update(json.loads(m.data)),50)
            from sensor_msgs.msg import PointCloud2,Imu
            self.create_subscription(PointCloud2,'/lidar/points',lambda m:self.count('cloud'),sensor)
            if a.capture_registration_inputs:self.create_subscription(PointCloud2,'/lio/cloud_registered',self.on_registered,50)
            self.create_subscription(Imu,'/imu/data',lambda m:self.count('imu'),sensor)
            self.create_timer(.05,self.publish)
        def count(self,key):self.counts[key]+=1
        def on_registered(self,m):
            from map_bundle import pointcloud_xyzi
            key=stamp(m.header);name=str(key)+'.npy'
            np.save(out/'registration_inputs'/name,pointcloud_xyzi(m,200000)[:,:3])
            self.registration_index.append(dict(ns=key,file=name))
        def on_odom(self,m):
            row=pose_row(m.header,m.pose.pose)
            v,w=m.twist.twist.linear,m.twist.twist.angular
            row['linear_velocity']=[v.x,v.y,v.z];row['angular_velocity']=[w.x,w.y,w.z]
            self.odoms.append(row)
        def stationary_ready(self):
            if not self.odoms:return False
            newest=self.odoms[-1]['ns']
            recent=[x for x in self.odoms if newest-x['ns']<=stationary_cfg['stationary_window_s']*1.2e9+1]
            if len(recent)<2 or newest-recent[0]['ns']<stationary_cfg['stationary_window_s']*1e9:return False
            positions=np.asarray([x['position'] for x in recent])
            angles=(Rotation.from_quat(recent[-1]['quaternion']).inv()*Rotation.from_quat([x['quaternion'] for x in recent])).magnitude()
            return (max(np.linalg.norm(x['linear_velocity']) for x in recent)<=stationary_cfg['stationary_linear_m_s']
                and max(np.linalg.norm(x['angular_velocity']) for x in recent)<=stationary_cfg['stationary_angular_rad_s']
                and np.linalg.norm(positions-positions[-1],axis=1).max()<=stationary_cfg['stationary_span_m']
                and angles.max()<=stationary_cfg['stationary_angle_rad'])
        def on_clock(self,m):
            value=m.clock.sec+m.clock.nanosec/1e9
            if self.clock is not None and value<self.clock-1e-6:raise RuntimeError('Simulation clock rollback')
            self.clock=value;self.clock_wall=time.monotonic();self.counts['clock']+=1
        def on_tf(self,m):
            for t in m.transforms:
                self.edges.add((t.header.frame_id,t.child_frame_id))
                if (t.header.frame_id,t.child_frame_id)==('map','odom'):
                    v,q=t.transform.translation,t.transform.rotation
                    self.transforms.append(dict(ns=stamp(t.header),position=[v.x,v.y,v.z],quaternion=[q.x,q.y,q.z,q.w]))
        def on_status(self,m):
            self.status=json.loads(m.data)
            entry=dict(t=self.clock,phase=self.status.get('phase'),reason=self.status.get('reason'),
                       accepted=self.status.get('accepted_matches',0),failed=self.status.get('failed_matches',0))
            if not self.status_events or any(entry[k]!=self.status_events[-1][k] for k in ('phase','reason','accepted','failed')):
                self.status_events.append(entry)
                self.status_snapshots.append(dict(t=self.clock,state=self.status))
        def publish(self):
            if not self.permitted:return
            vx,wz=self.command
            if time.monotonic()-self.clock_wall>2. or self.status.get('phase')!='tracking':vx=wz=0.
            m=Twist();m.linear.x=float(vx);m.angular.z=float(wz);self.pub.publish(m)
        def spin(self):
            if interrupted:raise InterruptedError('User interrupted localization acceptance')
            rclpy.spin_once(self,timeout_sec=.02)
            for name,process in children.items():
                if process.poll() is not None:raise RuntimeError(name+' exited before test completed')
            if self.permitted:
                if self.status.get('phase')!='tracking':
                    self.command=[0.,0.];self.publish();raise RuntimeError('Localization lost tracking: '+str(self.status))
                if time.monotonic()-self.clock_wall>2.:raise RuntimeError('Simulation clock stale')
                if self.runtime.get('generation')!=0 or not self.base.get('aligned'):raise RuntimeError('Estimator reset or base alignment lost')
            if time.monotonic()-self.last_report>5.:
                (out/'progress.json').write_text(json.dumps(dict(phase=self.stage,sim_time_s=self.clock,
                    poses=len(self.poses),status=self.status,elapsed_wall_s=time.monotonic()-started),indent=2)+'\n')
                self.last_report=time.monotonic()
        def wait(self,predicate,timeout=240):
            until=time.monotonic()+timeout
            while not predicate():
                self.spin()
                if time.monotonic()>until:raise TimeoutError('Timeout during '+self.stage)
        def initialize(self):
            m=PoseWithCovarianceStamped();m.header.frame_id='map'
            m.header.stamp.sec,m.header.stamp.nanosec=divmod(round(self.clock*1e9),1_000_000_000)
            m.pose.pose.position.x=a.spawn_offset_x;m.pose.pose.orientation.w=1.
            for i in range(6):m.pose.covariance[6*i+i]=.01
            self.seed.publish(m)
            self.seed_attempts.append(dict(t=self.clock,position_x=a.spawn_offset_x))
        def phase(self,spec):
            self.stage=spec['name'];begin=self.clock
            self.command=[spec['vx_mps'],spec['wz_radps']]
            self.publish()
            self.wait(lambda:self.clock>=begin+spec['duration_s'],timeout=180)
            self.command=[0.,0.];self.publish()
            self.windows.append(dict(name=spec['name'],begin_s=begin,end_s=self.clock,command=[spec['vx_mps'],spec['wz_radps']]))
            print('PHASE',json.dumps(self.windows[-1]),flush=True)

    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,on_signal);signal.signal(signal.SIGTERM,on_signal)
    node=Check()
    try:
        until=time.monotonic()+1.
        while time.monotonic()<until:node.spin()
        if node.get_subscriptions_info_by_topic('/cmd_vel') or node.get_publishers_info_by_topic('/localization/pose') or node.get_publishers_info_by_topic('/odom'):
            raise RuntimeError('Existing robot/localization session owns this ROS domain')
        launch('localization',['/usr/bin/python3',str(root/'scripts/run_localization.py'),'--map-id',a.map_id,'--with-lio',
            '--base-config-file',str(root/'ros2_ws/src/quadruped_localization/config/base_odometry_dynamic_gait.yaml'),
            '--localization-config-file',str(root/'ros2_ws/src/quadruped_localization/config/localization_dynamic_gait.yaml'),
            '--output-dir',str(out/'localization')])
        node.wait(lambda:(out/'localization/lio/status.json').exists(),timeout=60)
        if a.spawn_offset_x==0:
            launch('isaac',['/usr/bin/python3',str(root/'scripts/run_gait_cmd_vel.py'),'--max-seconds',str(cfg['max_sim_seconds']),
                           '--output-dir',str(out/'isaac_gait')])
        if a.spawn_offset_x!=0:
            # The launcher intentionally has no spawn override; use the same
            # owned Isaac entry with an explicitly configured initial placement.
            isaac=Path(host['isaac_root']);env=dict(os.environ);env.pop('PYTHONPATH',None)
            env['ROS_DISTRO']=host['ros_distro'];env['LD_LIBRARY_PATH']=str(isaac/'exts/isaacsim.ros2.bridge'/host['ros_distro']/'lib')+os.pathsep+env.get('LD_LIBRARY_PATH','')
            h=(out/'isaac.log').open('w');handles.append(h)
            children['isaac']=subprocess.Popen([str(isaac/'python.sh'),str(root/'ros2_ws/src/quadruped_sim/scripts/gait_ros_cmd_ready.py'),
                '--project-root',str(root),'--output-dir',str(out/'isaac_gait'),'--model','loaded','--policy','--gpu-index','1',
                '--max-seconds',str(cfg['max_sim_seconds']),'--spawn-offset-x',str(a.spawn_offset_x)],
                cwd=root,env=env,stdout=h,stderr=subprocess.STDOUT,start_new_session=True)
        node.wait(lambda:node.clock is not None and node.clock>=cfg['initial_pose_sensor_time_s'])
        node.stage='initializing';deadline=time.monotonic()+90
        while not (node.status.get('phase')=='tracking' and node.status.get('accepted_matches',0)>=2):
            if (node.status.get('phase')=='waiting_initial_pose' and node.stationary_ready()
                    and (not node.seed_attempts or node.clock-node.seed_attempts[-1]['t']>=.8)):
                node.initialize()
            node.spin()
            if time.monotonic()>deadline or node.clock>cfg['maximum_initialization_sensor_time_s']:
                raise TimeoutError('No accepted stationary initial pose within the initialization window')
        consumers=node.get_subscriptions_info_by_topic('/cmd_vel');producers=node.get_publishers_info_by_topic('/cmd_vel')
        if len(consumers)!=1 or consumers[0].node_name!='d1_runtime_gait_sensors' or len(producers)!=1 or producers[0].node_name!=node.get_name():
            raise RuntimeError('Velocity ownership is not exclusive to this Isaac session')
        node.permitted=True
        for spec in cfg['phases']:node.phase(spec)
        node.stage='complete'
    except Exception:
        failure=traceback.format_exc();(out/'error.txt').write_text(failure);print(failure,flush=True)
    finally:
        node.command=[0.,0.];node.publish()
        until=time.monotonic()+.3
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        for name in reversed(children):codes[name]=stop(name)
        evidence=dict(poses=node.poses,odometry=node.odoms,map_odom=node.transforms,status_events=node.status_events,
            windows=node.windows,status=node.status,runtime=node.runtime,base=node.base,sensor_counts=node.counts,
            tf_edges=sorted([list(x) for x in node.edges]),map_id=a.map_id,spawn_offset_x=a.spawn_offset_x,
            fresh_simulation=True,replay_used=False,runtime_truth_subscribed=False,initial_pose_source='predeclared_approximate_spawn_seed',
            mapping_run=str(a.mapping_run),seed_attempts=node.seed_attempts,status_snapshots=node.status_snapshots,
            registration_inputs=node.registration_index,failure=failure,process_exits=codes)
        (out/'observations.json').write_text(json.dumps(evidence)+'\n')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for h in handles:h.close()
    return evaluate(out,root,map_hashes,cfg)


def evaluate(out,root,map_hashes,cfg):
    e=json.loads((out/'observations.json').read_text());limits=cfg['limits'];checks=[]
    def check(name,passed,**details):checks.append(dict(name=name,passed=bool(passed),**details))
    check('session_completed',e['failure'] is None,failure=e['failure'])
    check('owned_processes_exited_cleanly',all(v==0 for v in e['process_exits'].values()),codes=e['process_exits'])
    check('all_phases_executed',len(e['windows'])==len(cfg['phases']),completed=len(e['windows']))
    if not e['poses'] or not e['windows']:
        (out/'verification.json').write_text(json.dumps(dict(result='FAIL',checks=checks,evidence=e),indent=2)+'\n');return 1
    gauge=json.loads((out/'map_gauge.json').read_text());r=np.eye(3);r[:2,:2]=np.asarray(gauge['rotation']);t=np.r_[gauge['translation'],json.loads((out/'map_initial_state.json').read_text())['base_position_world_m'][2]]
    truth=[json.loads(s) for s in (out/'isaac_gait/states.jsonl').open()]
    times=np.asarray([x['t'] for x in truth]);xyz=np.asarray([x['pos'] for x in truth]);quats=np.asarray([x['quat'] for x in truth])[:,[1,2,3,0]]
    stamps=np.asarray([x['ns']/1e9 for x in e['poses']]);physical=stamps+2.
    truth_xyz=np.column_stack([np.interp(physical,times,xyz[:,k]) for k in range(3)])
    if physical.max()>times[-1]+.02:raise ValueError('Estimated pose lies outside the recorded physics interval')
    truth_rotation=Slerp(times,Rotation.from_quat(quats))(np.clip(physical,times[0],times[-1]))
    expected_xyz=(truth_xyz-t)@r;estimated_xyz=np.asarray([x['position'] for x in e['poses']])
    expected_rotation=Rotation.from_matrix(r.T)*truth_rotation
    estimate_rotation=Rotation.from_quat([x['quaternion'] for x in e['poses']])
    position=np.linalg.norm(estimated_xyz-expected_xyz,axis=1)
    orientation=np.rad2deg((estimate_rotation.inv()*expected_rotation).magnitude())
    metrics={}
    def accuracy(mask):
        return dict(samples=int(mask.sum()),position_rmse_m=float(np.sqrt(np.mean(position[mask]**2))),
            position_p95_m=float(np.quantile(position[mask],.95)),position_max_m=float(position[mask].max()),
            orientation_rmse_deg=float(np.sqrt(np.mean(orientation[mask]**2))),orientation_p95_deg=float(np.quantile(orientation[mask],.95)),
            orientation_max_deg=float(orientation[mask].max()))
    for w in e['windows']:
        mask=(stamps>=w['begin_s'])&(stamps<w['end_s'])
        m=accuracy(mask) if mask.any() else dict(samples=0)
        metrics[w['name']]=m
        passed=m['samples']>=limits['minimum_poses_per_phase'] and all(m[k]<=limits[k] for k in ('position_rmse_m','position_p95_m','position_max_m','orientation_rmse_deg','orientation_p95_deg','orientation_max_deg'))
        check('accuracy_'+w['name'],passed,**m)
        ids=np.where((times>=w['begin_s']+2)&(times<w['end_s']+2))[0]
        if len(ids)>1 and w['name']=='straight':
            delta=float(np.linalg.norm(xyz[ids[-1],:2]-xyz[ids[0],:2]));check('actual_straight_motion',delta>=limits['minimum_straight_displacement_m'],distance_m=delta)
        if len(ids)>1 and w['name'] in ('left_turn','right_turn'):
            yaw=np.unwrap(Rotation.from_quat(quats[ids]).as_euler('xyz')[:,2]);angle=float(yaw[-1]-yaw[0]);direction=1 if w['name']=='left_turn' else -1
            check('actual_'+w['name'],angle*direction>=limits['minimum_turn_angle_rad'],angle_rad=angle)
    active=(stamps>=e['windows'][0]['begin_s'])&(stamps<e['windows'][-1]['end_s'])
    overall=accuracy(active)
    tf={x['ns']:matrix(x) for x in e['map_odom']};odom={x['ns']:matrix(x) for x in e['odometry']}
    composition=[]
    for row in e['poses']:
        if row['ns'] in tf and row['ns'] in odom:
            delta=np.linalg.inv(matrix(row))@tf[row['ns']]@odom[row['ns']]
            composition.append([float(np.linalg.norm(delta[:3,3])),float(Rotation.from_matrix(delta[:3,:3]).magnitude())])
    composition=np.asarray(composition)
    check('tf_pose_odometry_composition',len(composition)==len(e['poses']) and composition[:,0].max()<=limits['tf_composition_translation_m'] and composition[:,1].max()<=limits['tf_composition_rotation_rad'],
          matched=len(composition),poses=len(e['poses']),max_translation_m=float(composition[:,0].max()) if len(composition) else None)
    ordered=sorted(tf);transforms=[tf[k] for k in ordered if e['windows'][0]['begin_s']<=k/1e9<=e['windows'][-1]['end_s']]
    steps=[np.linalg.inv(a)@b for a,b in zip(transforms,transforms[1:])]
    max_step=float(max((np.linalg.norm(v[:3,3]) for v in steps),default=999.))
    max_angle=float(max((Rotation.from_matrix(v[:3,:3]).magnitude() for v in steps),default=999.))
    check('map_odom_continuity',max_step<=limits['map_odom_step_translation_m'] and max_angle<=limits['map_odom_step_rotation_rad'],max_translation_step_m=max_step,max_rotation_step_rad=max_angle)
    gaps=np.diff(stamps[active]);check('localized_pose_continuity',len(gaps)>0 and (gaps>0).all() and gaps.max()<=limits['pose_gap_s'],maximum_gap_s=float(gaps.max()) if len(gaps) else None)
    events=[x for x in e['status_events'] if x['t'] is not None and e['windows'][0]['begin_s']<=x['t']<=e['windows'][-1]['end_s']]
    check('tracking_never_lost',all(x['phase']=='tracking' for x in events),phases=sorted({x['phase'] for x in events}))
    status=e['status'];accepted=status.get('accepted_matches',0);failed=status.get('failed_matches',0)
    check('icp_match_health',accepted>=20 and failed/max(1,accepted+failed)<=limits['maximum_failed_match_fraction'],accepted=accepted,failed=failed)
    required={('map','odom'),('odom','base_link'),('base_link','lidar_link'),('lidar_link','lidar_imu_link')}
    check('required_tf_edges',required.issubset({tuple(v) for v in e['tf_edges']}),edges=e['tf_edges'])
    from gait_scene import load_gait_lidar_config
    from room_survey import clearance_xy
    clearance=clearance_xy(xyz[:,:2],load_gait_lidar_config(root)['room'],.8)
    check('actual_collection_geometry',clearance.min()>0,minimum_margin_m=float(clearance.min()))
    check('final_velocity_command_zero',np.allclose(truth[-1]['command'],0),command=truth[-1]['command'])
    physics=json.loads((out/'isaac_gait/verification.json').read_text());check('no_body_teleport',not physics['base_teleported_after_initialization'])
    map_dir=Path('/data0/rl/quadruped_nav/maps')/e['map_id'];changed=[s for s,h in map_hashes.items() if hashlib.sha256((map_dir/s).read_bytes()).hexdigest()!=h]
    check('map_files_unchanged',not changed,changed=changed)
    report=dict(result='PASS' if all(v['passed'] for v in checks) else 'FAIL',map_id=e['map_id'],spawn_offset_x=e['spawn_offset_x'],
        checks=checks,overall=overall,phases=metrics,sensor_counts=e['sensor_counts'],independent_observations=True,
        runtime_truth_subscribed=False,trajectory_alignment_fitted=False,initial_pose_source=e['initial_pose_source'],
        hardware=False,navigation_started=False,process_exits=e['process_exits'])
    (out/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
    np.savez_compressed(out/'accuracy.npz',stamp_s=stamps,estimated_map=estimated_xyz,expected_map=expected_xyz,
                        position_error_m=position,orientation_error_deg=orientation)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True,layout='constrained')
    axes[0].plot(stamps,position);axes[0].axhline(limits['position_p95_m'],color='r',ls='--',label='Position P95 limit')
    axes[0].set(ylabel='3D position error (m)',title=f"Fresh D1 map localization | start x={e['spawn_offset_x']} m | {report['result']}");axes[0].legend()
    axes[1].plot(stamps,orientation);axes[1].set(ylabel='Orientation error (deg)',xlabel='Sensor time (s)')
    for ax in axes:
        ax.grid(alpha=.2)
        for w in e['windows']:ax.axvline(w['begin_s'],color='grey',alpha=.25)
    fig.savefig(out/'localization_accuracy.jpg',dpi=130);plt.close(fig)
    print(json.dumps(dict(result=report['result'],overall=overall,failed_checks=[v for v in checks if not v['passed']],process_exits=e['process_exits']),indent=2),flush=True)
    return 0 if report['result']=='PASS' else 1
