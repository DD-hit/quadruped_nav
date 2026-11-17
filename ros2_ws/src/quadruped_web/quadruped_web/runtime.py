# SPDX-License-Identifier: MIT
"""Own only this Web session's localization/navigation children; simulator stays separate."""
import json,os,signal,subprocess,threading,time
from pathlib import Path
import yaml
class Runtime:
 def __init__(self,root,out,mode,with_lio):
  self.root,self.out,self.mode,self.with_lio=Path(root),Path(out),mode,with_lio;self.children=[];self.handles=[];self.generation=0;self.lock=threading.RLock()
 def stop(self):
  with self.lock:
   for name,p in reversed(self.children):
    if p.poll() is None:
     os.killpg(p.pid,signal.SIGINT)
     try:p.wait(timeout=35)
     except subprocess.TimeoutExpired:
      os.killpg(p.pid,signal.SIGTERM)
      try:p.wait(timeout=12)
      except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
   self.children=[]
   for h in self.handles:h.close()
   self.handles=[]
 def start(self,directory):
  with self.lock:
   self.generation+=1;out=self.out/('generation_%03d'%self.generation);out.mkdir(parents=True)
   profile=yaml.safe_load((self.root/'ros2_ws/src/quadruped_localization/config/localization.yaml').read_text())
   if self.mode=='sim':profile['registration']['tracking_level_start']=1;profile['runtime']['stationary_linear_m_s']=.12
   (out/'localization.yaml').write_text(yaml.safe_dump(profile,sort_keys=False))
   nav=yaml.safe_load((self.root/'ros2_ws/src/quadruped_navigation/config/navigation.yaml').read_text())
   if self.mode=='sim':nav['/navigation/trajectory_controller']['ros__parameters']['max_vyaw']=.3;nav['/navigation/trajectory_controller']['ros__parameters']['kp_pos']=2.;nav['/navigation/navigation_api']['ros__parameters']['goal_timeout']=600.
   (out/'navigation.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))
   local=['/usr/bin/python3',str(self.root/'scripts/run_localization.py'),'--map-directory',str(directory),'--mode',self.mode,'--localization-config-file',str(out/'localization.yaml'),'--output-dir',str(out/'localization')]
   if self.with_lio:local+=['--with-lio','--base-config-file',str(self.root/'ros2_ws/src/quadruped_localization/config/base_odometry_dynamic_gait.yaml')]
   commands=[('localization',local),('navigation',['ros2','launch','quadruped_navigation','navigation.launch.py','use_sim_time:='+str(self.mode=='sim').lower(),'map_package:='+str(directory),'cmd_vel_topic:=/navigation/cmd_vel_raw','params_file:='+str(out/'navigation.yaml')])]
   for name,cmd in commands:
    h=(out/(name+'.log')).open('w');self.handles.append(h);p=subprocess.Popen(cmd,stdout=h,stderr=subprocess.STDOUT,start_new_session=True);self.children.append((name,p))
   (out/'processes.json').write_text(json.dumps({k:p.pid for k,p in self.children},indent=2)+'\n')
 def status(self):
  with self.lock:return {name:dict(pid=p.pid,returncode=p.poll()) for name,p in self.children}
