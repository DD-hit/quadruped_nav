#!/usr/bin/env python3
"""Replay only native Livox sensor topics, optionally through candidate FAST-LIO."""
import argparse
from datetime import datetime
import json,os
from pathlib import Path
import signal,subprocess,sys,time,traceback
from zoneinfo import ZoneInfo
import yaml
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'ros2_ws/src/quadruped_localization/scripts'))
from livox_bag_check import check
from lio_config import prepare
from copy import deepcopy


def merge(target,updates):
    for key,value in updates.items():
        if isinstance(value,dict):merge(target.setdefault(key,{}),value)
        else:target[key]=value

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--bag',required=True,type=Path)
parser.add_argument('--profile',type=Path,default=root/'ros2_ws/src/quadruped_localization/config/livox_recording.yaml')
parser.add_argument('--robot',default='d1_edu')
parser.add_argument('--output-dir',type=Path)
parser.add_argument('--rate',type=float,default=1.0)
parser.add_argument('--with-lio',action='store_true',help='Run candidate recording parameters, not simulation tuning')
args=parser.parse_args()
if not 0<args.rate<=4:parser.error('--rate must be greater than 0 and at most 4')
host=yaml.safe_load((root/'ros2_ws/src/quadruped_bringup/config/hosts/server.yaml').read_text())
os.environ['ROS_DOMAIN_ID']=str(host['ros_domain_id']);os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
output=args.output_dir or Path(host['data_root'])/'runs'/('livox_replay_'+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'))
output.mkdir(parents=True,exist_ok=False)
profile=yaml.safe_load(args.profile.read_text())
try:report=check(args.bag,profile)
except Exception as error:report=dict(result='FAIL',error=repr(error),real_hardware_validation_complete=False)
(output/'preflight.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
if report['result']!='PASS':
    print('Recording contract check failed; inspect',output/'preflight.json',flush=True);raise SystemExit(2)
robot=yaml.safe_load((root/'ros2_ws/src/quadruped_bringup/config/robots'/f'{args.robot}.yaml').read_text())
profile['output']['lidar_frame']=robot['frame_aliases']['lidar'];profile['output']['imu_frame']=robot['frame_aliases']['imu']
profile_path=output/'recording_profile.yaml';profile_path.write_text(yaml.safe_dump(profile,sort_keys=False))
shutdown=False

def on_signal(signum,frame):
    global shutdown;shutdown=True
signal.signal(signal.SIGINT,on_signal);signal.signal(signal.SIGTERM,on_signal)
processes=[];logs=[];code=0;guard=None
startup={};clock_state=dict(samples=0,last_ns=None,minimum_delta_ns=None)

def start(name,command):
    handle=(output/(name+'.log')).open('w');logs.append(handle)
    p=subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append((name,p,command));return p

try:
    # Refuse topic collisions with a currently running simulation/adapter/clock.
    import rclpy
    from rclpy.node import Node
    rclpy.init(args=[]);guard=Node('livox_replay_startup_check')
    deadline=time.monotonic()+.8
    while time.monotonic()<deadline:rclpy.spin_once(guard,timeout_sec=.05)
    clashes=[]
    for topic in ('/clock',profile['output']['lidar_topic'],profile['output']['imu_topic'],profile['output']['lio_imu_topic']):
        if guard.get_publishers_info_by_topic(topic):clashes.append(topic)
    if clashes:raise RuntimeError('Stop the other sensor/clock publishers first: '+', '.join(clashes))
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from rosbag2_interfaces.srv import IsPaused,Resume
    def on_clock(message):
        current=message.clock.sec*1_000_000_000+message.clock.nanosec
        if clock_state['last_ns'] is not None:
            delta=current-clock_state['last_ns']
            previous=clock_state['minimum_delta_ns']
            clock_state['minimum_delta_ns']=delta if previous is None else min(previous,delta)
        clock_state['last_ns']=current;clock_state['samples']+=1
    guard.create_subscription(Clock,'/clock',on_clock,QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT))
    adapter=start('native_input_adapter',['/usr/bin/python3',str(root/'ros2_ws/src/quadruped_localization/scripts/livox_recording_adapter.py'),
        '--profile',str(profile_path),'--ros-args','-p','use_sim_time:=true'])
    if args.with_lio:
        from ament_index_python.packages import get_package_prefix
        _,base=prepare(root,args.robot,output/'generated_input_template')
        params=yaml.safe_load(base.read_text());p=params['/**']['ros__parameters']
        candidate=yaml.safe_load((root/'ros2_ws/src/quadruped_localization/config/fastlio_recording.yaml').read_text())
        merge(p,candidate['parameters'])
        p['common'].update(lid_topic=profile['output']['lidar_topic'],imu_topic=profile['output']['lio_imu_topic'],time_sync_en=False,time_offset_lidar_to_imu=0.0)
        p['preprocess'].update(scan_line=profile['input']['scan_lines'],scan_rate=round(profile['input']['nominal_scan_hz']))
        p['output_directory']=str(output/'algorithm');p['map_file_path']=str(output/'algorithm/PCD/map.pcd')
        params_path=output/'fastlio_recording.yaml';params_path.write_text(yaml.safe_dump(params,sort_keys=False))
        remaps=yaml.safe_load((root/'ros2_ws/src/quadruped_localization/config/fastlio_runtime.yaml').read_text())['remappings']
        command=[str(Path(get_package_prefix('fast_lio'))/'lib/fast_lio/fastlio_mapping'),'--ros-args','--params-file',str(params_path)]
        for source,target in remaps.items():command+=['-r',source+':='+target]
        estimator=start('fastlio_candidate',command)
    qos={profile['input']['lidar_topic']:dict(reliability='reliable',history='keep_last',depth=20,durability='volatile'),
         profile['input']['imu_topic']:dict(reliability='reliable',history='keep_last',depth=1000,durability='volatile')}
    qos_path=output/'playback_qos.yaml';qos_path.write_text(yaml.safe_dump(qos,sort_keys=False))
    command=['ros2','bag','play',str(args.bag.resolve()),'--rate',str(args.rate),'--clock','200',
        '--start-paused','--disable-keyboard-controls','--wait-for-all-acked','1000',
        '--qos-profile-overrides-path',str(qos_path),'--topics',profile['input']['lidar_topic'],profile['input']['imu_topic'],
        '--remap',profile['input']['lidar_topic']+':='+profile['replay_topics']['lidar'],profile['input']['imu_topic']+':='+profile['replay_topics']['imu']]
    player=start('sensor_bag_player',command)
    # Use wall time for startup deadlines: ROS time is deliberately paused.
    paused_client=guard.create_client(IsPaused,'/rosbag2_player/is_paused')
    resume_client=guard.create_client(Resume,'/rosbag2_player/resume')
    expected={profile['replay_topics']['lidar']:'livox_recording_adapter',
              profile['replay_topics']['imu']:'livox_recording_adapter'}
    if args.with_lio:
        expected.update({profile['output']['lidar_topic']:'laser_mapping',
                         profile['output']['lio_imu_topic']:'laser_mapping'})
    deadline=time.monotonic()+25;stable_since=None
    while not shutdown:
        rclpy.spin_once(guard,timeout_sec=.05)
        for name,process,_ in processes:
            if process.poll() is not None:raise RuntimeError(name+' exited during paused startup')
        ready=(clock_state['samples']>0 and paused_client.service_is_ready() and resume_client.service_is_ready()
               and all(any(info.node_name==name for info in guard.get_subscriptions_info_by_topic(topic))
                       for topic,name in expected.items()))
        if ready:
            if stable_since is None:stable_since=time.monotonic()
            if time.monotonic()-stable_since>=.3:break
        else:stable_since=None
        if time.monotonic()>deadline:raise RuntimeError('Timed out waiting for paused player and sensor/LIO subscriptions')
    def request(client,request):
        future=client.call_async(request);deadline=time.monotonic()+5
        while not future.done() and not shutdown and time.monotonic()<deadline:
            rclpy.spin_once(guard,timeout_sec=.05)
        if shutdown:return None
        if not future.done():raise RuntimeError('Playback control service timed out')
        return future.result()
    if not shutdown:
        if not request(paused_client,IsPaused.Request()).paused:raise RuntimeError('Player was not paused at startup')
        startup=dict(mode='start_paused_then_resume_service',subscriptions_discovered=expected,
                     paused_clock_ns=clock_state['last_ns'])
        request(resume_client,Resume.Request())
    print('REPLAY_READY',output,flush=True)
    while not shutdown and player.poll() is None:
        if adapter.poll() is not None:raise RuntimeError('Native input adapter exited')
        if args.with_lio and estimator.poll() is not None:raise RuntimeError('Candidate FAST-LIO exited')
        rclpy.spin_once(guard,timeout_sec=.05)
        if clock_state['minimum_delta_ns'] is not None and clock_state['minimum_delta_ns']<0:
            raise RuntimeError('Playback clock moved backwards')
    if player.poll() is not None and player.returncode!=0:raise RuntimeError('ros2 bag play failed')
    if not shutdown:time.sleep(.3)
except Exception:
    code=1;traceback.print_exc();(output/'error.txt').write_text(traceback.format_exc())
finally:
    metadata=[]
    for name,p,command in reversed(processes):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
        if p.returncode!=0:code=1
        metadata.append(dict(name=name,pid=p.pid,return_code=p.returncode,command=command))
    for handle in logs:handle.close()
    if guard is not None:guard.destroy_node()
    if 'rclpy' in globals() and rclpy.ok():rclpy.shutdown()
    (output/'replay_result.json').write_text(json.dumps(dict(result='PASS' if code==0 else 'FAIL',processes=metadata,
        data_origin=report['data_origin'],real_hardware_validation_complete=False,
        full_lio_trajectory_quality_verified=False,recording_profile_status=profile['status'],
        playback_scope='two_sensor_topics_only',candidate_lio_started=args.with_lio,
        startup=startup,clock=clock_state),indent=2)+'\n')
raise SystemExit(code)
