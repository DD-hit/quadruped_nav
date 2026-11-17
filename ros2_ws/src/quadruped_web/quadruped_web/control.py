# SPDX-License-Identifier: MIT
"""Single velocity authority. Timeouts are wall-clock based, including paused sim."""
import math,time
class VelocityRouter:
 def __init__(self,cfg):
  self.cfg=cfg;self.mode='idle';self.owner=None;self.heartbeat=0.;self.manual_stamp=self.nav_stamp=0.;self.manual=[0.,0.,0.];self.nav=[0.,0.,0.];self.output=[0.,0.,0.];self.reason='idle';self.expired=False
 def acquire(self,mode,owner,now=None):
  now=time.monotonic() if now is None else now
  if mode not in ('idle','navigation','manual'):raise ValueError('Invalid control mode')
  if self.owner and self.owner!=owner and now-self.heartbeat<self.cfg['control_lease_wall_s']:raise ValueError('Another browser owns control')
  self.mode=mode;self.owner=owner if mode!='idle' else None;self.heartbeat=now;self.manual=[0.,0.,0.];self.manual_stamp=0.;self.nav_stamp=0.;self.nav=[0.,0.,0.];self.expired=False
 def ping(self,owner,now=None):
  if self.owner!=owner:raise ValueError('Control ownership changed')
  self.heartbeat=time.monotonic() if now is None else now
 def command(self,owner,vector,now=None):
  if self.mode!='manual' or self.owner!=owner:raise ValueError('Acquire manual control first')
  v=list(map(float,vector))
  if len(v)!=3 or not all(math.isfinite(x) for x in v):raise ValueError('Expected finite vx, vy, wz')
  if math.hypot(*v[:2])>self.cfg['manual_max_linear_m_s']+1e-8 or abs(v[2])>self.cfg['manual_max_angular_rad_s']+1e-8:raise ValueError('Manual command exceeds configured limit')
  self.manual=v;self.manual_stamp=time.monotonic() if now is None else now;self.ping(owner,self.manual_stamp)
 def stop(self,reason='stop requested'):
  self.mode='idle';self.owner=None;self.output=[0.,0.,0.];self.manual=[0.,0.,0.];self.reason=reason
 def tick(self,now=None,healthy=True):
  now=time.monotonic() if now is None else now
  if self.mode!='idle' and now-self.heartbeat>self.cfg['control_lease_wall_s']:
   self.stop('browser lease expired');self.expired=True
  output=[0.,0.,0.]
  if not healthy:self.reason='motion inputs unavailable or publisher conflict'
  elif self.mode=='manual':
   if now-self.manual_stamp<=self.cfg['manual_timeout_wall_s']:output=self.manual.copy();self.reason='manual'
   else:self.reason='manual command timeout'
  elif self.mode=='navigation':
   if now-self.nav_stamp<=self.cfg['navigation_timeout_wall_s']:output=self.nav.copy();self.reason='navigation'
   else:self.reason='navigation command timeout'
  self.output=output;return output
