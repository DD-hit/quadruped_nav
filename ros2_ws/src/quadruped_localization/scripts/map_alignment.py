"""Finalize a map's vertical frame using estimated gravity, without fitting terrain."""
import numpy as np
from map_bundle import validate_rotation

def final_alignment(initial, samples, last_stamp_ns, window_s=1., max_spread_deg=.1):
    rows=np.asarray(samples,dtype=float)
    if rows.ndim!=2 or rows.shape[1]!=4 or not np.isfinite(rows).all():
        raise ValueError('Missing or invalid gravity samples')
    target=float(last_stamp_ns)*1e-9
    rows=rows[(rows[:,0]<=target+1e-6)&(rows[:,0]>=target-window_s)]
    if len(rows)<5 or np.ptp(rows[:,0])<window_s*.5 or target-rows[-1,0]>.25:
        raise ValueError('Insufficient recent gravity window; wait before saving')
    norms=np.linalg.norm(rows[:,1:],axis=1)
    if np.any((norms<7)|(norms>12.5)):raise ValueError('Invalid estimated gravity magnitude')
    unit=rows[:,1:]/norms[:,None];direction=np.mean(unit,axis=0);direction/=np.linalg.norm(direction)
    spread=np.degrees(np.arccos(np.clip(unit@direction,-1,1))).max()
    if spread>max_spread_deg:raise ValueError('Gravity direction is not stable; wait before saving')
    old_R=validate_rotation(initial['rotation']);old_t=np.asarray(initial['translation_m'],dtype=float)
    z=-direction;x=old_R[0]-z*np.dot(z,old_R[0]);x/=np.linalg.norm(x);y=np.cross(z,x)
    new_R=validate_rotation(np.vstack([x,y,z]));anchor=-old_R.T@old_t;new_t=-new_R@anchor
    delta_R=new_R@old_R.T;delta_t=new_t-delta_R@old_t
    angle=float(np.degrees(np.arccos(np.clip((np.trace(delta_R)-1)/2,-1,1))))
    if angle>2.:raise ValueError('Gravity alignment changed over 2 degrees; inspect mapping before export')
    return dict(rotation=new_R.tolist(),translation_m=new_t.tolist()),delta_R,delta_t,dict(
        method='rigid_rotation_from_stable_estimated_gravity',angle_deg=angle,
        window_start_s=float(rows[0,0]),window_end_s=float(rows[-1,0]),samples=len(rows),
        direction_spread_deg=float(spread),gravity_C=(direction*np.mean(norms)).tolist(),
        terrain_fitted=False,flattened=False,origin_preserved=True)
