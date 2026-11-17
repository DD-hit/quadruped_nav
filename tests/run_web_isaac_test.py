#!/usr/bin/env python3
import argparse,json,os,signal,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=False,parents=True)
env=dict(os.environ);env.update(QUADRUPED_SIM_CAPTURE_RGB='0',ROS_DOMAIN_ID='42',ROS_LOCALHOST_ONLY='0',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',FASTRTPS_DEFAULT_PROFILES_FILE=str(a.root/'ros2_ws/src/quadruped_bringup/config/dds/local_sim.xml'))
h=(a.output/'simulation.log').open('w');sim=subprocess.Popen(['/usr/bin/python3',str(a.root/'scripts/run_gait_cmd_vel.py'),'--standby-on-stop','--gpu-index','1','--max-seconds','300','--output-dir',str(a.output/'simulation')],env=env,stdout=h,stderr=subprocess.STDOUT,start_new_session=True)
(a.output/'processes.json').write_text(json.dumps(dict(simulation=sim.pid),indent=2)+'\n')
try:
 result=subprocess.run(['/usr/bin/python3',str(a.root/'tests/test_web_workflow.py'),'--output',str(a.output/'http')],env=env);raise SystemExit(result.returncode)
finally:
 if sim.poll() is None:
  os.killpg(sim.pid,signal.SIGINT)
  try:sim.wait(timeout=40)
  except subprocess.TimeoutExpired:
   os.killpg(sim.pid,signal.SIGTERM)
   try:sim.wait(timeout=12)
   except subprocess.TimeoutExpired:os.killpg(sim.pid,signal.SIGKILL);sim.wait(timeout=5)
 h.close()
