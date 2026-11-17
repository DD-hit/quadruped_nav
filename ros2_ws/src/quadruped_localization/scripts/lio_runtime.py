#!/usr/bin/env python3
"""Own the input adapter and FAST-LIO; restart the estimator on clock rollback."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback
from zoneinfo import ZoneInfo
import yaml
from lio_config import prepare


def merge(target,updates):
    for key,value in updates.items():
        if isinstance(value,dict):merge(target.setdefault(key,{}),value)
        else:target[key]=value


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root',type=Path,required=True)
    parser.add_argument('--robot',default='d1_edu')
    parser.add_argument('--output-dir',type=Path)
    parser.add_argument('--raw-only',action='store_true',help='Keep only isolated IMU-state LIO outputs; do not publish /odom or main TF')
    parser.add_argument('--base-config-file',type=Path,help='Base odometry profile for this LIO session')
    args=parser.parse_args(argv)
    root=args.project_root.resolve()
    host=yaml.safe_load((root/'ros2_ws/src/quadruped_bringup/config/hosts/server.yaml').read_text())
    os.environ['ROS_DOMAIN_ID']=str(host['ros_domain_id']);os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from ament_index_python.packages import get_package_prefix
    output=args.output_dir or Path(host['data_root'])/'runs'/('fastlio_'+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir(parents=True,exist_ok=False)
    adapter_file,base_params=prepare(root,args.robot,output/'input_config')
    config=yaml.safe_load((root/'ros2_ws/src/quadruped_localization/config/fastlio_runtime.yaml').read_text())
    (output/'runtime_profile.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    executable=Path(get_package_prefix('fast_lio'))/'lib/fast_lio/fastlio_mapping'
    if not executable.is_file():raise FileNotFoundError('Build fast_lio before starting this launcher')
    shutdown=False
    def on_signal(signum,frame):
        nonlocal shutdown;shutdown=True
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,on_signal);signal.signal(signal.SIGTERM,on_signal)
    processes=[];handles=[];node=None;exit_code=0

    def start(name,command,log_path):
        handle=log_path.open('w');handles.append(handle)
        child=subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        item=dict(name=name,pid=child.pid,command=command,started_wall_s=time.time(),return_code=None)
        processes.append((child,item));return child

    def stop(child):
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=5)
        if child is not None:
            for process,item in processes:
                if process is child:item.update(return_code=child.returncode,ended_wall_s=time.time())

    class Runtime(Node):
        def __init__(self):
            super().__init__('quadruped_lio_runtime')
            self.generation=0;self.clock_s=None;self.restart_requested=False
            self.last_odometry_s=None;self.first_odometry_s=None;self.initialized=False;self.estimator=None;self.adapter=None;self.base_adapter=None
            self.status_pub=self.create_publisher(String,config['status_topic'],10)
            self.create_subscription(Clock,'/clock',self.on_clock,QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(Odometry,'/lio/odometry',self.on_odometry,20)

        def on_clock(self,message):
            now=message.clock.sec+message.clock.nanosec/1e9
            if self.clock_s is not None and now<self.clock_s-1e-6:self.restart_requested=True
            self.clock_s=now

        def on_odometry(self,message):
            stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
            if self.clock_s is not None and stamp<=self.clock_s+.1:
                if self.first_odometry_s is None:self.first_odometry_s=stamp
                self.last_odometry_s=stamp;self.initialized=True

        def start_estimator(self):
            generation=output/f'generation_{self.generation:03d}';generation.mkdir()
            self.base_status_path=generation/'base_status.json'
            if not args.raw_only:
                base_command=['/usr/bin/python3',
                    str(root/'ros2_ws/src/quadruped_localization/scripts/base_odometry.py'),
                    '--project-root',str(root),'--robot',args.robot,'--generation',str(self.generation),
                    '--status-file',str(self.base_status_path)]
                if args.base_config_file is not None:
                    base_command.extend(['--config-file',str(args.base_config_file.resolve())])
                base_command.extend(['--ros-args','-p','use_sim_time:=true'])
                self.base_adapter=start('base_odometry_'+str(self.generation),base_command,generation/'base_odometry.log')
            params=yaml.safe_load(base_params.read_text());p=params['/**']['ros__parameters']
            merge(p,config['parameters'])
            p['output_directory']=str(generation/'algorithm')
            p['map_file_path']=str(generation/'algorithm/PCD/map.pcd')
            path=generation/'fastlio.yaml';path.write_text(yaml.safe_dump(params,sort_keys=False))
            command=[str(executable),'--ros-args','--params-file',str(path)]
            for source,target in config['remappings'].items():command+=['-r',source+':='+target]
            self.estimator=start('fastlio_'+str(self.generation),command,generation/'fastlio.log')
            self.initialized=False;self.last_odometry_s=None;self.first_odometry_s=None

        def report(self):
            state=dict(robot=args.robot,generation=self.generation,reset_count=self.generation,
                clock_s=self.clock_s,initialized=self.initialized,last_odometry_s=self.last_odometry_s,
                warmup_complete=(self.first_odometry_s is not None and self.clock_s is not None and self.clock_s-self.first_odometry_s>=config['stationary_warmup_s']),
                estimator_pid=self.estimator.pid if self.estimator else None,
                adapter_pid=self.adapter.pid if self.adapter else None,
                state_reference='imu',base_odometry_enabled=not args.raw_only,
                base_adapter_pid=self.base_adapter.pid if self.base_adapter else None,
                main_tf_published=False,output_directory=str(output))
            if self.base_status_path.is_file():
                try:
                    base=json.loads(self.base_status_path.read_text());state['base_odometry']=base
                    state['main_tf_published']=base['aligned'] and base['published']>0
                except (OSError,json.JSONDecodeError):pass
            self.status_pub.publish(String(data=json.dumps(state)))
            (output/'status.json').write_text(json.dumps(state,indent=2)+'\n')

    try:
        node=Runtime()
        node.adapter=start('input_adapter',['/usr/bin/python3',str(root/'ros2_ws/src/quadruped_localization/scripts/lio_input_adapter.py'),
            '--config',str(adapter_file),'--ros-args','-p','use_sim_time:=true'],output/'adapter.log')
        node.start_estimator();node.report();last_report=0
        while rclpy.ok() and not shutdown:
            rclpy.spin_once(node,timeout_sec=.02)
            if node.adapter.poll() is not None:raise RuntimeError('Input adapter exited unexpectedly')
            if node.estimator.poll() is not None:raise RuntimeError('FAST-LIO exited unexpectedly')
            if node.base_adapter is not None and node.base_adapter.poll() is not None:raise RuntimeError('Base odometry adapter exited unexpectedly')
            if node.restart_requested:
                node.get_logger().info('Simulation clock reset: restarting FAST-LIO from a fresh state')
                stop(node.base_adapter);stop(node.estimator)
                if node.estimator.returncode!=0:raise RuntimeError('FAST-LIO did not exit cleanly on reset')
                node.generation+=1;node.restart_requested=False;node.start_estimator();node.report()
            if time.monotonic()-last_report>.25:node.report();last_report=time.monotonic()
    except Exception:
        exit_code=1;traceback.print_exc();(output/'error.txt').write_text(traceback.format_exc())
    finally:
        for child,item in reversed(processes):
            stop(child)
            if child.returncode!=0:exit_code=1
        (output/'processes.json').write_text(json.dumps([item for child,item in processes],indent=2)+'\n')
        for handle in handles:handle.close()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return exit_code

if __name__=='__main__':raise SystemExit(main())
