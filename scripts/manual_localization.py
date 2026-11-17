#!/usr/bin/env python3
"""One command: manual initialization, existing FAST-LIO, Web and optional sim.

Simulation owns only the children started here. Real mode starts localization
only, using an already running sensor/LIO pipeline; it never starts SDK control.
"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
from urllib.request import urlopen
import yaml


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=['sim','real'],default='sim')
    p.add_argument('--map-id',default='d1_full_room_web_20261117_01')
    p.add_argument('--robot',default='d1_edu')
    p.add_argument('--reuse-sim',action='store_true')
    p.add_argument('--reuse-lio',action='store_true')
    p.add_argument('--gpu-index',type=int,default=1)
    p.add_argument('--max-seconds',type=float,default=3600.)
    p.add_argument('--port',type=int,default=8766)
    p.add_argument('--output',type=Path)
    p.add_argument('--check-only',action='store_true')
    a=p.parse_args()
    root=Path(__file__).resolve().parents[1]
    if not 1024<=a.port<=65535:p.error('Invalid port')
    if not 1<=a.max_seconds<=3600:p.error('max-seconds must be 1..3600')
    if a.mode=='sim' and a.robot!='d1_edu':p.error('The current simulator supports d1_edu; other robots require an adapter')
    env=dict(os.environ)
    env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
    if a.mode=='sim':
        host=yaml.safe_load((root/'ros2_ws/src/quadruped_bringup/config/hosts/server.yaml').read_text())
        env.update(ROS_DOMAIN_ID=str(host['ros_domain_id']),ROS_LOCALHOST_ONLY='0',
            RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
            FASTRTPS_DEFAULT_PROFILES_FILE=str(root/'ros2_ws/src/quadruped_bringup/config/dds/local_sim.xml'))
    check=['/usr/bin/python3',str(root/'scripts/run_localization.py'),'--map-id',a.map_id,'--robot',a.robot,'--mode',a.mode,'--check-only']
    subprocess.run(check,env=env,check=True)
    if a.check_only:
        print('CHECK PASSED; no simulator, navigation, driver or localization process started',flush=True)
        return 0
    out=a.output or root/'data/runs'/('manual_localization_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir(parents=True,exist_ok=False)
    children=[];handles=[];stopping=False;code=0
    def stop_request(signum,frame):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,stop_request);signal.signal(signal.SIGTERM,stop_request)
    def start(name,cmd):
        h=(out/f'{name}.log').open('w');handles.append(h)
        child=subprocess.Popen(cmd,stdout=h,stderr=subprocess.STDOUT,env=env,start_new_session=True)
        children.append((name,child,cmd))
    try:
        if a.mode=='sim':
            with socket.socket() as probe:
                probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                probe.bind(('127.0.0.1',a.port))
            web=['/usr/bin/python3',str(root/'scripts/run_web.py'),'--map-id',a.map_id,'--output',str(out/'web'),'--port',str(a.port)]
            if not a.reuse_lio:web+=['--with-lio']
            start('web',web)
            if not a.reuse_sim:
                start('simulation',['/usr/bin/python3',str(root/'scripts/run_gait_cmd_vel.py'),
                    '--standby-on-stop','--gpu-index',str(a.gpu_index),'--max-seconds',str(a.max_seconds),'--output-dir',str(out/'simulation')])
        else:
            start('localization',['/usr/bin/python3',str(root/'scripts/run_localization.py'),
                '--map-id',a.map_id,'--robot',a.robot,'--mode','real','--output-dir',str(out/'localization')])
        print('SESSION',out,flush=True)
        print('Workflow: set mechanical-base position AND heading on the map; wait for tracking; then start navigation.',flush=True)
        if a.mode=='real':print('Real mode: existing sensor/FAST-LIO inputs required. Publish map-frame mechanical-base pose on /initialpose. Web motor control is not enabled.',flush=True)
        announced=False
        while not stopping:
            for name,child,_ in children:
                if child.poll() is not None:
                    if name=='simulation' and child.returncode==0:stopping=True
                    else:raise RuntimeError(f'{name} exited ({child.returncode}); see {out/name}.log')
            if a.mode=='sim' and not announced:
                try:
                    with urlopen(f'http://127.0.0.1:{a.port}/api/state',timeout=.5) as response:
                        response.read()
                    print(f'WEB http://127.0.0.1:{a.port} (HTTP ready; sensors/localization may still be starting)',flush=True)
                    announced=True
                except OSError:pass
            time.sleep(.1)
    except Exception as e:
        code=1;print(str(e),flush=True)
        (out/'error.txt').write_text(repr(e))
    finally:
        records=[]
        # Web owns navigation/LIO; stop it first, then the simulator.
        for name,child,cmd in sorted(children,key=lambda item:item[0]!='web'):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=55)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM)
                    try:child.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=5)
            records.append(dict(name=name,command=cmd,returncode=child.returncode))
        for h in handles:h.close()
        (out/'session_result.json').write_text(json.dumps(dict(result='FAIL' if code else 'STOPPED',processes=records,hardware_started=False),indent=2))
    return code


if __name__=='__main__':raise SystemExit(main())
