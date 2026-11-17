#!/usr/bin/env python3
"""Static Isaac raycast coverage; not a physical gait/LIO acceptance."""
import argparse,os,sys,json,hashlib,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
from isaacsim import SimulationApp
app=SimulationApp(dict(headless=True,active_gpu=1,physics_gpu=1,multi_gpu=False,renderer='RaytracedLighting',disable_viewport_updates=True))
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.extensions import enable_extension
from pxr import UsdGeom,UsdPhysics,Usd,Gf
import omni.usd
sys.path.insert(0,str(a.root/'ros2_ws/src/quadruped_sim/scripts'))
from runtime_gait_lidar import GeometricLidar,create_validation_room
from gait_scene import load_gait_lidar_config
world=None;node=None;lidar=None;code=1
try:
    os.environ['ROS_DOMAIN_ID']='45'
    omni.usd.get_context().new_stage();world=World(physics_dt=.005,rendering_dt=1/30,stage_units_in_meters=1.)
    world.get_physics_context().set_enable_scene_query_support(True)
    stage=omni.usd.get_context().get_stage();UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    asset=Path('/data0/rl/quadruped_nav/assets/d1_edu/model_review/d1_edu.usd')
    stage.DefinePrim('/World/D1','Xform').GetReferences().AddReference(str(asset));stage.Load()
    mass=UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath('/World/D1/lidar_imu_link'));mass.CreateMassAttr(1e-6);mass.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-9,1e-9,1e-9))
    robot=world.scene.add(Robot(prim_path='/World/D1',name='pose_scan_d1',position=np.array([0.,0.,.42])))
    cfg=load_gait_lidar_config(a.root);create_validation_room(stage,cfg)
    enable_extension('isaacsim.ros2.bridge')
    for _ in range(3):app.update()
    import rclpy
    from rclpy.node import Node
    rclpy.init();node=Node('initial_pose_static_scan_coverage');node.epoch=0;node.obstacles=None
    common=yaml.safe_load((a.root/'ros2_ws/src/quadruped_bringup/config/common.yaml').read_text());profile=yaml.safe_load((a.root/'ros2_ws/src/quadruped_bringup/config/robots/d1_edu.yaml').read_text())
    world.reset();world.step(render=False)
    lidar=GeometricLidar(stage,node,cfg,profile,common,a.output)
    settled=json.loads((a.root/'ros2_ws/src/quadruped_sim/config/d1_standby_initial_state.json').read_text())
    policy=yaml.safe_load((a.root/'data/assets/d1_edu/gait_source/policy/d1/base.yaml').read_text())['d1']
    names=list(robot.dof_names);mapping=[names.index(n) for n in policy['joint_names']];q=np.zeros(len(names));q[mapping]=settled['joint_position_policy_order_rad'];robot.set_joint_positions(q)
    gauge=json.loads(Path('/data0/rl/quadruped_nav/runs/d1_full_room_survey_20261109_02/contract.json').read_text())['world_from_odom_initial_placement']
    world_from_map=np.eye(4);world_from_map[:2,:2]=gauge['rotation'];world_from_map[:3,3]=[*gauge['translation'],settled['base_position_world_m'][2]]
    gauge_yaw=np.arctan2(world_from_map[1,0],world_from_map[0,0]);quat=np.array(settled['base_quat_wxyz']);tilt=Rotation.from_euler('z',-gauge_yaw)*Rotation.from_quat(quat[[1,2,3,0]])
    xy=[(x,y) for x in [-4.5,-2.8,0.,2.8,4.5] for y in [-1.8,0.,1.8]]+[(-4.7,-2.),(-4.7,2.),(4.7,-2.),(4.7,2.)]
    cases=[];start=time.monotonic()
    for location,(x,y) in enumerate(xy):
        for yaw_deg in range(0,360,45):
            index=len(cases);map_from_base=np.eye(4);map_from_base[:3,:3]=(Rotation.from_euler('z',yaw_deg,degrees=True)*tilt).as_matrix();map_from_base[:3,3]=[x,y,0.];world_from_base=world_from_map@map_from_base
            clearance=min(float(np.linalg.norm(np.maximum(abs(world_from_base[:2,3]-np.array(ob['center'][:2]))-np.array(ob['size'][:2])/2,0.)))-.8 for name,ob in cfg['room'].items() if name not in ('floor','ceiling'))
            if clearance<=0:raise ValueError(f'Invalid stand placement {x,y}: clearance {clearance}')
            qworld=Rotation.from_matrix(world_from_base[:3,:3]).as_quat();robot.set_world_pose(position=world_from_base[:3,3],orientation=qworld[[3,0,1,2]]);robot.set_linear_velocity(np.zeros(3));robot.set_angular_velocity(np.zeros(3))
            lidar.sample(index*.1,(index+1)*.1,world_from_base,world_from_base)
            cases.append(dict(id=index,location=location,map_xy=[x,y],yaw_deg=yaw_deg,map_from_base_truth=map_from_base.tolist(),world_from_base=world_from_base.tolist(),spawn_circle_margin_m=clearance,scan=index))
            (a.output/'cases.json').write_text(json.dumps(cases,indent=2)+'\n')
            if index%8==7:print(json.dumps(dict(scans=len(cases),locations=location+1,wall_s=time.monotonic()-start)),flush=True)
    lidar.close();lidar=None
    report=dict(result='GENERATED',scope='static Isaac scene raycasts with actual sensor mount; no gait dynamics, FAST-LIO or ROS localizer acceptance',cases=len(cases),locations=len(xy),yaw_values_deg=list(range(0,360,45)),sensor_mount=GeometricLidar.__module__,base_from_lidar=None,world_from_map=world_from_map.tolist(),model_sha256=hashlib.sha256(asset.read_bytes()).hexdigest(),scan_pattern_is_manufacturer=False,self_occlusion_modeled=False)
    # Mount is read from the exact sensor prim and saved by the lidar metadata.
    report['base_from_lidar']=json.loads((a.output/'lidar_scans.jsonl').open().readline())['mount_matrix']
    (a.output/'capture_report.json').write_text(json.dumps(report,indent=2)+'\n');code=0
except Exception:
    import traceback
    (a.output/'error.txt').write_text(traceback.format_exc());traceback.print_exc()
finally:
    if lidar:
        try:lidar.close()
        except Exception:pass
    if node:
        try:node.destroy_node();rclpy.shutdown()
        except Exception:pass
    app.app.post_quit(code);app.close()
raise SystemExit(code)
