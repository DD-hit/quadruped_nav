"""Resolve input contract and nominal FAST-LIO sensor parameters from profiles."""
from copy import deepcopy
import math
from pathlib import Path
import numpy as np
import yaml


def rotation(rpy):
    r,p,y=rpy
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


def generate(common,robot,adapter,upstream):
    resolved=deepcopy(adapter)
    resolved['input'].update(lidar_topic=common['interfaces']['lidar_points']['topic'],
        imu_topic=common['interfaces']['imu']['topic'],lidar_frame=robot['frame_aliases']['lidar'],
        imu_frame=robot['frame_aliases']['imu'])
    resolved['output']['lidar_topic']=common['interfaces']['lidar_custom']['topic']
    params=deepcopy(upstream)
    p=params['/**']['ros__parameters'];p['use_sim_time']=True
    p['common'].update(lid_topic=resolved['output']['lidar_topic'],imu_topic=resolved['output']['imu_topic'],
        time_sync_en=False,time_offset_lidar_to_imu=0.0)
    # Current geometry has no physical channel IDs. Feature extraction stays off.
    p.update(feature_extract_enable=False,point_filter_num=1)
    p['preprocess'].update(lidar_type=1,scan_line=1,blind=.1,timestamp_unit=3,scan_rate=round(resolved['scan_hz']))
    internal=robot['sensor']['lidar_to_imu']
    rotation_i_l=rotation(internal['rpy_rad']).T
    translation_i_l=-rotation_i_l@np.asarray(internal['translation_m'])
    p['mapping'].update(extrinsic_est_en=False,extrinsic_T=translation_i_l.tolist(),extrinsic_R=rotation_i_l.flatten().tolist())
    p['publish']['map_en']=False;p['pcd_save']['pcd_save_en']=False
    return resolved,params


def prepare(root,robot_id,output):
    import re
    if not re.fullmatch(r'[a-z][a-z0-9_]*',robot_id):raise ValueError('invalid robot profile identifier')
    configs=root/'ros2_ws/src/quadruped_bringup/config'
    common=yaml.safe_load((configs/'common.yaml').read_text())
    robot=yaml.safe_load((configs/'robots'/f'{robot_id}.yaml').read_text())
    adapter=yaml.safe_load((root/'ros2_ws/src/quadruped_localization/config/lio_input_adapter.yaml').read_text())
    upstream=yaml.safe_load((root/'ros2_ws/src/fast_lio/config/mid360_nuc_mapping.yaml').read_text())
    lidar_profile=root/'ros2_ws/src/quadruped_sim/config'/robot['sensor']['simulation_lidar_profile']
    adapter['scan_hz']=float(yaml.safe_load(lidar_profile.read_text())['scan_hz'])
    resolved,params=generate(common,robot,adapter,upstream)
    output.mkdir(parents=True,exist_ok=False)
    (output/'adapter.yaml').write_text(yaml.safe_dump(resolved,sort_keys=False))
    (output/'fastlio_sim.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
    (output/'README.txt').write_text('Sensor input profile only. FAST-LIO has not been started or mapping-validated.\n'
        'Noise/filter/range tuning is inherited from the pinned upstream and is NOT validated for this ideal simulation.\n'
        'Restart the downstream LIO after simulation time reset; buffer clearing is not a full estimator reset.\n'
        'Extrinsic is T_IMU_LIDAR; the 45 degree base mounting is not part of this internal transform.\n')
    return output/'adapter.yaml',output/'fastlio_sim.yaml'
