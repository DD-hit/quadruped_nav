#!/usr/bin/env python3
"""Render recorded Isaac poses and ROS events. Replay never drives navigation."""
import argparse, json, math, subprocess, time, hashlib, sys
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--project-root',type=Path,required=True)
p.add_argument('--session',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
p.add_argument('--preview',action='store_true')
p.add_argument('--fps',type=int,default=10)
a=p.parse_args();root=a.project_root;out=a.output;out.mkdir(parents=True,exist_ok=False)
assert a.preview or json.loads((a.session/'verification.json').read_text())['result']=='PASS'
rows=[json.loads(v) for v in (a.session/'simulation/states.jsonl').open()]
events=[json.loads(v) for v in (a.session/'events.jsonl').open()]
ready=json.loads((a.session/'simulation/ready.json').read_text())
requested=next(v for v in events if v['kind']=='initial_pose_request')
confirmed=next(v for v in events if v['kind']=='initial_pose_confirmed')
requests=[v for v in events if v['kind']=='request']
results=[v for v in events if v['kind']=='result']
from isaacsim import SimulationApp
app=SimulationApp(dict(headless=True,active_gpu=1,physics_gpu=1,multi_gpu=False,renderer='RaytracedLighting',width=1280,height=800))
import numpy as np,yaml
from PIL import Image,ImageDraw,ImageFont
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from pxr import UsdGeom,UsdPhysics,UsdLux,Gf,Usd
import omni.usd,omni.replicator.core as rep,carb
sys.path.insert(0,str(root/'ros2_ws/src/quadruped_sim/scripts'))
from runtime_gait_lidar import create_validation_room
encoder=None;code=1
try:
    carb.settings.get_settings().set('/rtx/post/aa/op',3)
    omni.usd.get_context().new_stage()
    world=World(physics_dt=.005,rendering_dt=1/a.fps,stage_units_in_meters=1.)
    stage=omni.usd.get_context().get_stage();UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    asset=Path('/data0/rl/quadruped_nav/assets/d1_edu/model_review/d1_edu.usd')
    stage.DefinePrim('/World/D1','Xform').GetReferences().AddReference(str(asset));stage.Load()
    mass=UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath('/World/D1/lidar_imu_link'))
    mass.CreateMassAttr(1e-6);mass.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-9,1e-9,1e-9))
    robot=world.scene.add(Robot(prim_path='/World/D1',name='recorded_d1',position=np.array([0.,0.,.42])))
    cfg=yaml.safe_load((a.session/'simulation/scene_config.yaml').read_text());create_validation_room(stage,cfg)
    UsdGeom.Gprim(stage.GetPrimAtPath('/World/LidarRoom/floor')).GetDisplayColorAttr().Set([(.15,.19,.24)])
    light=UsdLux.DomeLight.Define(stage,'/World/ReplayLight');light.CreateIntensityAttr(1400.)
    for i,x in enumerate([-4.,0.,4.]):
        lamp=UsdLux.RectLight.Define(stage,f'/World/InteriorLight{i}');lamp.CreateIntensityAttr(2400.);lamp.CreateWidthAttr(4.);lamp.CreateHeightAttr(5.)
        UsdGeom.Xformable(lamp).AddTranslateOp().Set(Gf.Vec3d(x,0.,2.8))
    camera=UsdGeom.Camera.Define(stage,'/World/ReplayCamera');camera.CreateFocalLengthAttr(32.);camera.CreateHorizontalApertureAttr(36.);camera.CreateVerticalApertureAttr(22.5)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.03,100.));camera_op=UsdGeom.Xformable(camera).AddTransformOp()
    product=rep.create.render_product(str(camera.GetPath()),(1280,800));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([product]);rep.orchestrator.set_capture_on_play(False)
    world.reset();assert list(robot.dof_names)==ready['dof_names'];mapping=np.array(ready['policy_mapping'])
    robot.get_articulation_controller().set_gains(np.zeros(12),np.zeros(12))
    times=np.array([v['t']-2. for v in rows]);end=max(v['t'] for v in events)
    body_events=[v for v in events if v['kind']=='body_pose'];body_times=np.array([v['t'] for v in body_events]);body_points=np.array([v['data'] for v in body_events])
    commands=[v for v in events if v['kind']=='cmd_vel'];cmd_times=np.array([v['t'] for v in commands])
    paths=[v for v in events if v['kind']=='global_path'];path_times=np.array([v['t'] for v in paths])
    state_events=[v for v in events if v['kind']=='controller_state'];state_times=np.array([v['t'] for v in state_events])
    completed=[];seen_tasks=set()
    for v in state_events:
        if v['data']['state']=='SUCCEEDED' and v['data']['task_id'] not in seen_tasks:
            completed.append(v['t']);seen_tasks.add(v['data']['task_id'])
    locs=[v for v in events if v['kind']=='localization_status'];loc_times=np.array([v['t'] for v in locs])
    gauge=json.loads((Path('/data0/rl/quadruped_nav/runs/d1_full_room_survey_20261109_02')/'contract.json').read_text())['world_from_odom_initial_placement']
    R=np.array(gauge['rotation']);translation=np.array(gauge['translation'])
    fontpath='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';boldpath='/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
    fonts={s:ImageFont.truetype(fontpath,s) for s in [18,20,22,24,28,34]};bold={s:ImageFont.truetype(boldpath,s) for s in [18,20,22,24,28,34]}
    W,H=1920,1080;BG='#0b1220';PANEL='#131f30';WHITE='#edf4ff';MUTED='#9aacc2';BLUE='#4ab7fa';GREEN='#6dd6a2';ORANGE='#ffbd69'
    def xy(x,y):return (1352+(x+6.4)/12.8*528,240+(3.4-y)/6.8*280)
    base=Image.new('RGB',(W,H),BG);d=ImageDraw.Draw(base)
    d.rounded_rectangle((1328,126,1896,940),radius=18,fill=PANEL)
    d.text((24,18),'D1 EDU  |  jie_3d_nav + SCAN-Planner',font=bold[34],fill=WHITE)
    d.text((24,72),'初始位姿 → 点云地图定位 → 单点导航 → 多点依次到达',font=fonts[24],fill=MUTED)
    d.text((1352,150),'全局路线与实际定位轨迹',font=bold[24],fill=WHITE)
    for x in range(-6,7):d.line([xy(x,-3.1),xy(x,3.1)],fill='#263447',width=1)
    for y in range(-3,4):d.line([xy(-6.1,y),xy(6.1,y)],fill='#263447',width=1)
    for name,ob in cfg['room'].items():
        if name in ('floor','ceiling'):continue
        c=np.array(ob['center'][:2]);h=np.array(ob['size'][:2])/2
        corners=(np.array([c+[-h[0],-h[1]],c+[h[0],-h[1]],c+[h[0],h[1]],c+[-h[0],h[1]]])-translation)@R
        d.polygon([xy(*v) for v in corners],fill='#64748b')
    all_targets=[v for e in requests for v in e['data']['targets']]
    for i,v in enumerate(all_targets):
        x,y=xy(*v[:2]);d.ellipse((x-7,y-7,x+7,y+7),outline=GREEN,width=2);d.text((x+9,y-16),str(i+1),font=bold[18],fill=GREEN)
    d.text((1352,540),'黄色：全局路线   蓝色：定位轨迹',font=fonts[18],fill=MUTED)
    def latest(values,times,t,default=None):
        j=int(np.searchsorted(times,t,side='right')-1)
        return values[j]['data'] if j>=0 else default
    camera_xmin=cfg['room']['west']['center'][0]+cfg['room']['west']['size'][0]/2+.25
    camera_xmax=cfg['room']['east']['center'][0]-cfg['room']['east']['size'][0]/2-.25
    camera_ymin=cfg['room']['south']['center'][1]+cfg['room']['south']['size'][1]/2+.25
    camera_ymax=cfg['room']['north']['center'][1]-cfg['room']['north']['size'][1]/2-.25
    max_pose_error=0.;max_joint_error=0.;cam_center=None
    def render(t,freeze=''):
        global max_pose_error,max_joint_error,cam_center
        i=min(len(rows)-1,max(0,int(np.searchsorted(times,t))))
        row=rows[i];pos=np.array(row['pos']);quat=np.array(row['quat']);joints=np.zeros(12);joints[mapping]=row['q']
        robot.set_world_pose(position=pos,orientation=quat);robot.set_joint_positions(joints);robot.set_joint_velocities(np.zeros(12));robot.set_linear_velocity(np.zeros(3));robot.set_angular_velocity(np.zeros(3))
        cam_center=pos[:2].copy() if cam_center is None else .88*cam_center+.12*pos[:2]
        # Keep the presentation camera inside the room and above obstacle sight lines.
        nearest_obstacle = float('inf')
        for name,ob in cfg['room'].items():
            if name in ('floor','ceiling','east','west','north','south'):continue
            distance=np.maximum(abs(pos[:2]-np.array(ob['center'][:2]))-np.array(ob['size'][:2])/2,0.)
            nearest_obstacle=min(nearest_obstacle,float(np.linalg.norm(distance)))
        offset=np.array([1.75,-2.3]);offset*=min(1.,1.4*nearest_obstacle/np.linalg.norm(offset))
        eye=Gf.Vec3d(float(np.clip(cam_center[0]+offset[0],camera_xmin,camera_xmax)),float(np.clip(cam_center[1]+offset[1],camera_ymin,camera_ymax)),2.6);target=Gf.Vec3d(float(cam_center[0]),float(cam_center[1]),.45)
        camera_op.Set(Gf.Matrix4d(1.).SetLookAt(eye,target,Gf.Vec3d(0,0,1)).GetInverse())
        world.physics_sim_view.update_articulations_kinematic()
        from omni.physx import get_physx_interface
        get_physx_interface().update_transformations(False,True)
        rep.orchestrator.step(delta_time=0.,pause_timeline=False,rt_subframes=1,wait_for_render=True)
        image=np.asarray(rgb.get_data())
        for _ in range(10):
            if image.shape[:2]==(800,1280):break
            rep.orchestrator.step(delta_time=0.,pause_timeline=False,wait_for_render=True);image=np.asarray(rgb.get_data())
        assert image.shape[:2]==(800,1280) and image[::16,::16,:3].std()>5, (t,image.shape,float(image[::16,::16,:3].std()))
        actual,_=robot.get_world_pose();max_pose_error=max(max_pose_error,float(abs(np.asarray(actual)-pos).max()))
        max_joint_error=max(max_joint_error,float(abs(np.asarray(robot.get_joint_positions())[mapping]-np.array(row['q'])).max()))
        frame=base.copy();frame.paste(Image.fromarray(image[:,:,:3].astype(np.uint8)),(24,126));d=ImageDraw.Draw(frame)
        def text(x,y,s,size=22,color=WHITE,strong=False):d.text((x,y),s,font=(bold if strong else fonts)[size],fill=color)
        path=latest(paths,path_times,t,[])
        if len(path)>1:d.line([xy(*v[:2]) for v in path],fill=ORANGE,width=3)
        j=int(np.searchsorted(body_times,t,side='right'));history=body_points[:j:3]
        if len(history)>1:d.line([xy(*v[:2]) for v in history],fill=BLUE,width=3)
        current=body_points[j-1] if j>0 else None
        if current is not None:
            x,y=xy(*current[:2]);d.ellipse((x-6,y-6,x+6,y+6),fill=WHITE,outline=BLUE,width=2);d.line([(x,y),(x+math.cos(current[3])*20,y-math.sin(current[3])*20)],fill=WHITE,width=3)
        done=sum(v<=t for v in completed)
        text(1352,565,f'已到达 {done} / {len(all_targets)} 个目标点',18,GREEN)
        rq=requested['data'];rqtime=requested['t'];cftime=confirmed['t'];quality=confirmed['data']['quality']
        if t>=rqtime:
            x,y=xy(*rq[:2]);d.line([(x-9,y),(x+9,y)],fill=ORANGE,width=3);d.line([(x,y-9),(x,y+9)],fill=ORANGE,width=3)
        text(1352,590,'① 给定粗略初始位姿',24,ORANGE if t>=rqtime else MUTED,True)
        text(1352,629,f'x {rq[0]:+.2f}  y {rq[1]:+.2f} m  yaw {rq[2]:+.2f} rad',20)
        text(1352,675,'② 点云配准确认地图定位',24,GREEN if t>=cftime else MUTED,True)
        if t>=cftime:
            cf=confirmed['data']['pose'];text(1352,714,f'x {cf[0]:+.3f}  y {cf[1]:+.3f} m  yaw {cf[3]:+.3f} rad',20,GREEN)
            text(1352,747,f"匹配率 {quality['fitness']*100:.1f}%  RMSE {quality['rmse_m']*100:.2f} cm",20,MUTED)
        else:text(1352,714,'尚未定位；不发送导航任务',20,MUTED)
        cmd=latest(commands,cmd_times,t,[0.,0.,0.]);state=latest(state_events,state_times,t,{})
        text(1352,795,'③ 原生 SCAN 速度指令',24,BLUE,True)
        text(1352,835,f'vx {cmd[0]:+.2f}  vy {cmd[1]:+.2f} m/s',20)
        text(1352,870,f'wz {cmd[2]:+.2f} rad/s  |  {state.get("state","IDLE")}',20)
        if t<rqtime:title='等待用户给定初始位姿';detail='FAST-LIO 已启动；此时机器人尚未在已有地图中定位。'
        elif t<cftime:title='已发布 /initialpose，正在进行点云地图配准';detail='橙色十字是用户粗估计；定位结果由实际配准计算。'
        elif t<requests[0]['t']:title='定位确认：粗略初始位姿已被地图配准修正';detail='收到定位结果后，才允许本次演示发送导航任务。'
        elif t<results[0]['t']:title='单点导航：前往房间右侧目标';detail='jie 全局路径 → SCAN 局部轨迹、重规划和闭环速度控制。'
        elif len(requests)>1 and t>=requests[1]['t'] and t<results[-1]['t']:title='多点导航：按顺序到达 2 → 3 → 4';detail='途中绕过柱体；转弯时同时输出平移和转向速度。'
        elif t>=results[-1]['t']:title='所有目标已到达，导航速度归零';detail='本次使用 Isaac 物理步态和模拟雷达 / IMU；不代表真机验收。'
        else:title='单点已到达，准备下发多点任务';detail='任务通过标准 Nav2 Action 接口下发。'
        text(24,949,title,28,GREEN if t>=cftime else ORANGE,True);text(24,994,detail,22)
        text(24,1040,freeze or '1× 仿真时间 | 实际运行记录的离线三维回放 | 导航与定位未使用仿真真值',20,ORANGE if freeze else MUTED)
        text(1655,30,f'{t:06.1f} / {end:.1f} s',20)
        d.rectangle((0,H-5,int(W*max(0,t)/end),H),fill=BLUE)
        return frame
    initial_world_time=world.current_time
    for _ in range(5):render(0.)
    if a.preview:
        for i,t in enumerate([0.,requested['t']+.01,confirmed['t']+.01,requests[0]['t']+8,results[0]['t']]+[e['t']+1. for e in events if e['kind']=='waypoint_index']+[end]):
            cam_center=None
            for _ in range(3):frame=render(t)
            frame.save(out/f'preview_{i:02d}.jpg',quality=94)
    else:
        log=(out/'ffmpeg.log').open('w')
        encoder=subprocess.Popen(['/usr/bin/ffmpeg','-y','-loglevel','warning','-f','rawvideo','-pix_fmt','rgb24','-s','1920x1080','-r',str(a.fps),'-i','-','-an','-c:v','libx264','-preset','fast','-threads','4','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(out/'全流程导航演示.mp4')],stdin=subprocess.PIPE,stderr=log)
        freezes=[(requested['t']+.01,4,'停帧说明：真实发布的初始位姿请求；记录时钟暂停显示'),(confirmed['t']+.01,5,'停帧说明：实际点云配准确认定位；记录时钟暂停显示')]
        fi=0;frames=0;start=time.monotonic()
        for k in range(math.ceil(end*a.fps)):
            t=k/a.fps;frame=render(t)
            if fi<len(freezes) and t>=freezes[fi][0]:
                _,seconds,label=freezes[fi];frozen=render(t,label);frozen.save(out/f'定位步骤_{fi+1}.jpg',quality=95)
                for _ in range(seconds*a.fps):encoder.stdin.write(frozen.tobytes());frames+=1
                fi+=1
            encoder.stdin.write(frame.tobytes());frames+=1
            if k==round((requests[0]['t']+5)*a.fps):frame.save(out/'poster.jpg',quality=95)
            if k%(a.fps*10)==0:
                report=dict(frame=k,total=math.ceil(end*a.fps),recorded_s=t,wall_s=time.monotonic()-start)
                (out/'progress.json').write_text(json.dumps(report));print(json.dumps(report),flush=True)
        encoder.stdin.close();assert encoder.wait(timeout=120)==0;encoder=None;log.close()
    assert abs(world.current_time-initial_world_time)<1e-8
    assert max_pose_error<1e-5 and max_joint_error<1e-5
    report=dict(result='PASS',kind='offline replay of actual physics and ROS records',preview=a.preview,source_session=str(a.session),physics_advanced_during_replay=False,max_applied_position_error_m=max_pose_error,max_applied_joint_error_rad=max_joint_error,source_states_sha256=hashlib.sha256((a.session/'simulation/states.jsonl').read_bytes()).hexdigest(),source_events_sha256=hashlib.sha256((a.session/'events.jsonl').read_bytes()).hexdigest(),fps=a.fps)
    (out/'verification.json').write_text(json.dumps(report,indent=2)+'\n');code=0
except Exception:
    import traceback
    (out/'error.txt').write_text(traceback.format_exc());traceback.print_exc()
finally:
    if encoder:
        try:encoder.stdin.close();encoder.terminate()
        except Exception:pass
    app.app.post_quit(code);app.close()
raise SystemExit(code)
