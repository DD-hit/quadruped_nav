#!/usr/bin/env python3
"""Meaningful velocity authority regressions, independent of ROS/GUI."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2_ws/src/quadruped_web'))
from quadruped_web.control import VelocityRouter
CFG=dict(control_lease_wall_s=5.,manual_timeout_wall_s=.3,navigation_timeout_wall_s=.35,manual_max_linear_m_s=.5,manual_max_angular_rad_s=.3)
class TestControl(unittest.TestCase):
 def test_manual_expiry_and_no_nav_leak(self):
  r=VelocityRouter(CFG);r.nav=[.2,.1,.1];r.nav_stamp=10.;r.acquire('manual','a',10);r.command('a',[.1,-.1,.2],10);self.assertEqual(r.tick(10.1),[.1,-.1,.2]);self.assertEqual(r.tick(10.31),[0.,0.,0.])
 def test_navigation_expiry(self):
  r=VelocityRouter(CFG);r.acquire('navigation','a',10);r.nav=[.1,0.,0.];r.nav_stamp=10.;self.assertEqual(r.tick(10.2),[.1,0.,0.]);self.assertEqual(r.tick(10.36),[0.,0.,0.])
 def test_lease_expiry(self):
  r=VelocityRouter(CFG);r.acquire('navigation','a',10);r.nav=[.1,0.,0.];r.nav_stamp=15.1;self.assertEqual(r.tick(15.1),[0.,0.,0.]);self.assertEqual(r.mode,'idle');self.assertTrue(r.expired)
 def test_control_ownership(self):
  r=VelocityRouter(CFG);r.acquire('manual','a',10)
  with self.assertRaises(ValueError):r.acquire('navigation','b',11)
  with self.assertRaises(ValueError):r.command('b',[.1,0.,0.],11)
 def test_limits_and_finite(self):
  r=VelocityRouter(CFG);r.acquire('manual','a',10)
  for v in [[.4,.4,0.],[0,0,.31],[float('nan'),0,0],[.1,.2]]:
   with self.assertRaises(ValueError):r.command('a',v,10)
 def test_mode_change_clears_old_navigation_velocity(self):
  r=VelocityRouter(CFG);r.nav=[.2,0.,0.];r.nav_stamp=10.;r.acquire('navigation','a',10.1);self.assertEqual(r.tick(10.11),[0.,0.,0.])
 def test_stop_and_health(self):
  r=VelocityRouter(CFG);r.acquire('manual','a',10);r.command('a',[.1,0.,0.],10);self.assertEqual(r.tick(10.1,healthy=False),[0.,0.,0.]);r.stop();self.assertEqual(r.tick(10.1),[0.,0.,0.])
if __name__=='__main__':unittest.main()
