"""Regression: gravity finalization preserves 3D geometry and rejects bad windows."""
import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2_ws/src/quadruped_localization/scripts'))
from map_alignment import final_alignment
class AlignmentTests(unittest.TestCase):
 def samples(self,g):return np.c_[np.linspace(9,10,11),np.tile(g,(11,1))]
 def test_multilevel_geometry_and_origin(self):
  angle=np.radians(.45);g=[9.81*np.sin(angle),0,-9.81*np.cos(angle)]
  initial=dict(rotation=np.eye(3).tolist(),translation_m=[1,2,3]);new,R,t,report=final_alignment(initial,self.samples(g),10_000_000_000)
  p=np.array([[0,0,0],[1,0,0],[0,0,3],[0,1,.3],[.1,.1,.5]],float);q=p@R.T+t
  self.assertTrue(np.allclose(np.linalg.norm(p[:,None]-p[None,:],axis=2),np.linalg.norm(q[:,None]-q[None,:],axis=2)))
  self.assertTrue(np.allclose(np.array(new['rotation'])@g,[0,0,-9.81]))
  self.assertTrue(np.allclose(R@np.zeros(3)+t,0));self.assertFalse(report['flattened'])
 def test_reject_stale(self):
  with self.assertRaises(ValueError):final_alignment(dict(rotation=np.eye(3),translation_m=[0,0,0]),self.samples([0,0,-9.81]),12_000_000_000)
 def test_reject_unstable(self):
  rows=self.samples([0,0,-9.81]);rows[5,1]=1
  with self.assertRaises(ValueError):final_alignment(dict(rotation=np.eye(3),translation_m=[0,0,0]),rows,10_000_000_000)
 def test_reject_large_correction(self):
  with self.assertRaises(ValueError):final_alignment(dict(rotation=np.eye(3),translation_m=[0,0,0]),self.samples([1,0,-9.7]),10_000_000_000)
if __name__=='__main__':unittest.main()
