#!/usr/bin/env python3
"""Own one simulation LIO runtime and one bounded localization-map recorder."""
import argparse
from datetime import datetime
import json,os
from pathlib import Path
import signal,subprocess,sys,time,traceback
from zoneinfo import ZoneInfo
import yaml
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'ros2_ws/src/quadruped_localization/scripts'))
from map_bundle import identifier
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--map-id',required=True);parser.add_argument('--robot',default='d1_edu');parser.add_argument('--output-dir',type=Path);parser.add_argument('--base-config-file',type=Path)
args=parser.parse_args();identifier(args.map_id)
host=yaml.safe_load((root/'ros2_ws/src/quadruped_bringup/config/hosts/server.yaml').read_text())
os.environ['ROS_DOMAIN_ID']=str(host['ros_domain_id']);os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
map_directory=Path(host['data_root'])/'maps'/args.map_id
if map_directory.exists():parser.error('Map id already exists; choose a new id')
output=args.output_dir or Path(host['data_root'])/'runs'/('mapping_'+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'))
output.mkdir(parents=True,exist_ok=False)
shutdown=False;processes=[];logs=[];code=0
def on_signal(signum,frame):
    global shutdown;shutdown=True
signal.signal(signal.SIGINT,on_signal);signal.signal(signal.SIGTERM,on_signal)
def start(name,command):
    handle=(output/(name+'.log')).open('w');logs.append(handle)
    p=subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True);processes.append((name,p,command))
    return p
try:
    lio_command=['/usr/bin/python3',str(root/'scripts/run_lio.py'),'--robot',args.robot,'--output-dir',str(output/'lio')]
    if args.base_config_file is not None:lio_command.extend(['--base-config-file',str(args.base_config_file.resolve())])
    lio=start('lio_runtime',lio_command)
    recorder=start('map_recorder',['/usr/bin/python3',str(root/'ros2_ws/src/quadruped_localization/scripts/map_recorder.py'),
        '--project-root',str(root),'--map-id',args.map_id,'--robot',args.robot,'--lio-directory',str(output/'lio'),'--ros-args','-p','use_sim_time:=true'])
    print('MAPPING_SESSION',map_directory,flush=True)
    while not shutdown:
        for name,p,_ in processes:
            if p.poll() is not None:raise RuntimeError(name+' exited unexpectedly')
        status_path=map_directory/'recording_status.json'
        if status_path.is_file():
            try:status=json.loads(status_path.read_text())
            except json.JSONDecodeError:status={}
            if status.get('phase')=='invalidated':raise RuntimeError('Map session invalidated: '+status['reason'])
        time.sleep(.05)
except Exception:
    code=1;traceback.print_exc();(output/'error.txt').write_text(traceback.format_exc())
finally:
    items=[]
    for name,p,command in reversed(processes):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=35)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM)
                try:p.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
        if p.returncode!=0:code=1
        items.append(dict(name=name,pid=p.pid,return_code=p.returncode,command=command))
    for handle in logs:handle.close()
    complete=(map_directory/'manifest.yaml').is_file()
    if code==0 and not complete:code=2
    (output/'mapping_result.json').write_text(json.dumps(dict(result='PASS' if code==0 else ('INCOMPLETE' if code==2 else 'FAIL'),map_directory=str(map_directory),processes=items,localization_map_saved=complete,navigation_map_present=False,real_hardware_validation_complete=False),indent=2)+'\n')
raise SystemExit(code)
