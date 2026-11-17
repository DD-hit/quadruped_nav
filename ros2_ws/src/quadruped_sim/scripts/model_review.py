#!/usr/bin/env python3
"""Import and inspect static robot poses without advancing simulation time."""
import argparse
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--project-root', type=Path, required=True)
parser.add_argument('--robot', default='d1_edu')
parser.add_argument('--host', default='server')
parser.add_argument('--gui', action='store_true')
parser.add_argument('--reuse-import', action='store_true')
args = parser.parse_args()
import re
if not re.fullmatch(r'[a-z][a-z0-9_]*', args.robot):
    parser.error('Invalid robot identifier')
if not re.fullmatch(r'[a-z][a-z0-9_]*', args.host):
    parser.error('Invalid host identifier')

from isaacsim import SimulationApp
app = SimulationApp({'headless': not args.gui, 'active_gpu': 0, 'physics_gpu': 0,
                     'multi_gpu': False, 'renderer': 'RaytracedLighting',
                     'width': 1280, 'height': 960})

import hashlib
import json
import math
import xml.etree.ElementTree as ET
import numpy as np
import yaml
import omni.kit.commands
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from PIL import Image, ImageDraw
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics


def load_yaml(path):
    return yaml.safe_load(path.read_text())


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew@skew)


def urdf_origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = [float(v) for v in element.get('xyz', '0 0 0').split()]
        r, p, y = [float(v) for v in element.get('rpy', '0 0 0').split()]
        transform[:3, :3] = rotation([0, 0, 1], y)@rotation([0, 1, 0], p)@rotation([1, 0, 0], r)
    return transform


def joint_frame(position, quaternion):
    matrix = Gf.Matrix4d(1)
    matrix.SetRotate(Gf.Quatd(float(quaternion.GetReal()), Gf.Vec3d(*quaternion.GetImaginary())))
    matrix.SetTranslateOnly(Gf.Vec3d(*position))
    return np.array(matrix).T


def urdf_fk(joints, values):
    poses = {base_name: np.eye(4)}
    pending = list(joints)
    while pending:
        next_pending = []
        for joint in pending:
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if parent not in poses:
                next_pending.append(joint)
                continue
            motion = np.eye(4)
            if joint.get('type') == 'revolute':
                axis = [float(v) for v in joint.find('axis').get('xyz').split()]
                motion[:3, :3] = rotation(axis, values.get(joint.get('name'), 0.0))
            poses[child] = poses[parent]@urdf_origin(joint.find('origin'))@motion
        if len(next_pending) == len(pending):
            raise RuntimeError('URDF joint graph is disconnected')
        pending = next_pending
    return poses


def usd_fk(joints, values):
    poses = {base_name: np.eye(4)}
    pending = list(joints)
    while pending:
        following = []
        for prim in pending:
            joint = UsdPhysics.Joint(prim)
            parent = joint.GetBody0Rel().GetTargets()[0].name
            child = joint.GetBody1Rel().GetTargets()[0].name
            if parent not in poses:
                following.append(prim)
                continue
            t0 = joint_frame(joint.GetLocalPos0Attr().Get(), joint.GetLocalRot0Attr().Get())
            t1 = joint_frame(joint.GetLocalPos1Attr().Get(), joint.GetLocalRot1Attr().Get())
            motion = np.eye(4)
            if prim.IsA(UsdPhysics.RevoluteJoint):
                axis_name = UsdPhysics.RevoluteJoint(prim).GetAxisAttr().Get()
                axis = {'X': [1, 0, 0], 'Y': [0, 1, 0], 'Z': [0, 0, 1]}[axis_name]
                motion[:3, :3] = rotation(axis, values.get(prim.GetName(), 0.0))
            poses[child] = poses[parent]@t0@motion@np.linalg.inv(t1)
        if len(following) == len(pending):
            raise RuntimeError('USD joint graph is disconnected')
        pending = following
    return poses


def set_matrix(prim, matrix):
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTransformOp().Set(Gf.Matrix4d(*matrix.T.flatten().tolist()))


def camera_pose(position, target):
    matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*position), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
    set_matrix(camera.GetPrim(), np.array(matrix).T)


def display_pose(values):
    poses = usd_fk(usd_joints, values)
    for name, transform in poses.items():
        prim = stage.GetPrimAtPath(f'/World/Robot/{name}')
        parent = np.array(UsdGeom.Xformable(prim.GetParent()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
        robot_world = np.array(UsdGeom.Xformable(robot_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
        set_matrix(prim, np.linalg.inv(parent)@robot_world@transform)
    return poses


def capture(path, position, target, subframes=4):
    camera_pose(position, target)
    rep.orchestrator.step(rt_subframes=subframes, delta_time=0.0, pause_timeline=True)
    rgba = np.asarray(rgb.get_data())
    if rgba.ndim != 3 or rgba.shape[2] < 3 or float(rgba[:, :, :3].std()) < 2:
        raise RuntimeError(f'Empty or blank camera frame: {path}, {rgba.shape}')
    Image.fromarray(rgba[:, :, :3].astype(np.uint8)).save(path)
    print(f'REVIEW_IMAGE {path}', flush=True)


try:
    root = args.project_root.resolve()
    profiles = root/'ros2_ws/src/quadruped_bringup/config'
    profile = load_yaml(profiles/'robots'/f'{args.robot}.yaml')
    host = load_yaml(profiles/'hosts'/f'{args.host}.yaml')
    package = root/'ros2_ws/src'/profile['description_package']
    urdf_path = package/profile['model']['urdf_relative_path']
    preview_config = load_yaml(package/profile['model']['preview_pose_relative_path'])
    output = Path(host['data_root'])/'assets'/args.robot/'model_review'
    output.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(urdf_path).getroot()
    base_name = profile['model']['target_root_frame']
    joints = tree.findall('joint')
    movable = [j for j in joints if j.get('type') == 'revolute']
    links = [link.get('name') for link in tree.findall('link')]
    baseline = {name: math.radians(value) for name, value in preview_config['joint_positions_deg'].items()}
    for joint in movable:
        limit = joint.find('limit')
        value = baseline[joint.get('name')]
        assert float(limit.get('lower')) <= value <= float(limit.get('upper'))

    imported_path = output/f'{args.robot}.usd'
    enable_extension('isaacsim.asset.importer.urdf')
    if not args.reuse_import or not imported_path.exists():
        ok, config = omni.kit.commands.execute('URDFCreateImportConfig')
        assert ok
        config.merge_fixed_joints = False
        config.import_inertia_tensor = True
        config.fix_base = False
        config.distance_scale = 1.0
        ok, imported_prim = omni.kit.commands.execute('URDFParseAndImportFile',
            urdf_path=str(urdf_path), import_config=config, dest_path=str(imported_path))
        assert ok, imported_prim
    source_stage = Usd.Stage.Open(str(imported_path))
    source_robot = source_stage.GetPrimAtPath('/'+tree.get('name'))
    assert source_robot.IsValid()
    source_stage.SetDefaultPrim(source_robot)
    source_stage.GetRootLayer().Save()

    # A separate scene holds display-only pose and disabled physics overrides.
    # The imported articulation remains available in its own unmodified asset.
    scene_path = output/'model_review.usda'
    initial = Usd.Stage.CreateNew(str(scene_path))
    UsdGeom.SetStageMetersPerUnit(initial, 1.0)
    UsdGeom.SetStageUpAxis(initial, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(initial, '/World')
    initial.SetDefaultPrim(world.GetPrim())
    initial.GetRootLayer().Save()
    del initial
    omni.usd.get_context().open_stage(str(scene_path))
    for _ in range(5):
        app.update()
    stage = omni.usd.get_context().get_stage()
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    simulation_time_start = timeline.get_current_time()
    robot_prim = UsdGeom.Xform.Define(stage, '/World/Robot').GetPrim()
    robot_prim.GetReferences().AddReference('./'+imported_path.name)
    stage.Load()
    for _ in range(5):
        app.update()
    prims = list(Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()))
    meshes = [p for p in prims if p.IsA(UsdGeom.Mesh) and str(p.GetPath()).startswith('/World/Robot/')]
    usd_joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
    assert len(usd_joints) == len(joints), (len(usd_joints), len(joints))
    for name in links:
        prim = stage.GetPrimAtPath(f'/World/Robot/{name}')
        assert prim.IsValid(), name
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    assert len(meshes) >= 24, f'Visual meshes missing: {len(meshes)}'
    for joint in usd_joints:
        UsdPhysics.Joint(joint).CreateJointEnabledAttr(False)
    # Joint definitions and limits are verified independently of display posing.
    source_joints = {j.get('name'): j for j in movable}
    for prim in usd_joints:
        if prim.IsA(UsdPhysics.RevoluteJoint):
            source = source_joints[prim.GetName()]
            usd_joint = UsdPhysics.RevoluteJoint(prim)
            for field, attr in [('lower', usd_joint.GetLowerLimitAttr()), ('upper', usd_joint.GetUpperLimitAttr())]:
                assert abs(float(attr.Get())-math.degrees(float(source.find('limit').get(field)))) < 1e-4
    poses = [('standing', baseline, None)]
    for joint in movable:
        for endpoint in ('lower', 'upper'):
            values = dict(baseline)
            values[joint.get('name')] = float(joint.find('limit').get(endpoint))
            poses.append((joint.get('name')+'_'+endpoint, values, joint.get('name')))
    max_position_error = 0.0
    max_rotation_error = 0.0
    for name, values, _ in poses:
        reference, imported = urdf_fk(joints, values), usd_fk(usd_joints, values)
        assert set(reference) == set(imported) == set(links)
        for link in links:
            max_position_error = max(max_position_error, float(np.max(np.abs(reference[link][:3, 3]-imported[link][:3, 3]))))
            max_rotation_error = max(max_rotation_error, float(np.max(np.abs(reference[link][:3, :3]-imported[link][:3, :3]))))
    assert max_position_error < 5e-6 and max_rotation_error < 5e-6, (max_position_error, max_rotation_error)
    display_pose(baseline)
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], useExtentsHint=False)
    bounds = bbox.ComputeWorldBound(robot_prim).ComputeAlignedRange()
    minimum, maximum = np.array(bounds.GetMin()), np.array(bounds.GetMax())
    dimensions = maximum-minimum
    assert 0.3 < max(dimensions[:2]) < 2.0 and 0.3 < dimensions[2] < 2.0, dimensions
    height = -float(minimum[2])+0.005
    placement = np.eye(4)
    placement[2, 3] = height
    set_matrix(robot_prim, placement)
    # Debug axes are visual markers only, not simulated sensors.
    reference_poses = urdf_fk(joints, baseline)
    for label, frame, length in [('base', base_name, 0.25),
                                 ('lidar', profile['frame_aliases']['lidar'], 0.12),
                                 ('imu', profile['frame_aliases']['imu'], 0.08)]:
        transform = placement@reference_poses[frame]
        start = transform[:3, 3]
        for index, color in enumerate([(0.9, 0.05, 0.05), (0.05, 0.7, 0.1), (0.05, 0.2, 1.0)]):
            end = start+transform[:3, index]*length
            curve = UsdGeom.BasisCurves.Define(stage, f'/World/Frames/{label}_{index}')
            curve.CreateTypeAttr('linear')
            curve.CreateCurveVertexCountsAttr([2])
            curve.CreatePointsAttr([Gf.Vec3f(*start), Gf.Vec3f(*end)])
            curve.CreateWidthsAttr([0.004])
            curve.SetWidthsInterpolation('constant')
            curve.CreateDisplayColorAttr([color])
    ground = UsdGeom.Cube.Define(stage, '/World/Ground')
    ground.CreateSizeAttr(1.0)
    ground.CreateDisplayColorAttr([(0.20, 0.24, 0.29)])
    ground.AddScaleOp().Set(Gf.Vec3f(4, 4, 0.01))
    ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
    sun = UsdLux.DistantLight.Define(stage, '/World/Sun')
    sun.CreateIntensityAttr(2500)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-35, -25, 25))
    dome = UsdLux.DomeLight.Define(stage, '/World/Fill')
    dome.CreateIntensityAttr(800)
    camera = UsdGeom.Camera.Define(stage, '/World/ReviewCamera')
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100))
    camera.CreateFocalLengthAttr(35)
    rgb = rep.AnnotatorRegistry.get_annotator('rgb')
    render_product = rep.create.render_product(str(camera.GetPrim().GetPath()), (1280, 960))
    rgb.attach([render_product])
    target = ((minimum[0]+maximum[0])/2, (minimum[1]+maximum[1])/2,
              height+(minimum[2]+maximum[2])/2)
    views = {'isometric': (1.5, -1.8, height+1.1),
             'side': (0.0, -2.3, target[2]+0.10),
             'front': (2.3, 0, target[2]+0.10)}
    for label, position in views.items():
        capture(output/f'{label}.png', position, target, subframes=12)
    sheet = Image.new('RGB', (1240, len(movable)*480), '#edf1f6')
    draw = ImageDraw.Draw(sheet)
    for index, (label, values, joint_name) in enumerate(poses[1:]):
        display_pose(values)
        side = 1 if joint_name.startswith(('FL', 'RL')) else -1
        fore = 1 if joint_name.startswith(('FL', 'FR')) else -1
        capture(output/'sweep_current.png', (fore*1.5, side*1.8, height+1.0), target, subframes=2)
        image = Image.open(output/'sweep_current.png').resize((600, 450))
        x, y = (index%2)*620, (index//2)*480
        sheet.paste(image, (x, y+25))
        draw.text((x+8, y+6), f'{label}: {math.degrees(values[joint_name]):.3f} deg', fill='black')
    sheet.save(output/'joint_sweep.png')
    display_pose(baseline)
    camera_pose(views['isometric'], target)
    stage.GetRootLayer().customLayerData = {'purpose': 'Static model review, physics disabled; not locomotion validation'}
    stage.GetRootLayer().Save()
    lidar_fk = urdf_fk(joints, baseline)[profile['frame_aliases']['lidar']]
    imu_fk = urdf_fk(joints, baseline)[profile['frame_aliases']['imu']]
    simulation_time_end = timeline.get_current_time()
    assert abs(simulation_time_end-simulation_time_start) < 1e-12
    summary = dict(result='PASS', stage_meters_per_unit=UsdGeom.GetStageMetersPerUnit(stage),
                   up_axis=str(UsdGeom.GetStageUpAxis(stage)), robot=args.robot,
                   links=len(links), joints=len(joints), movable_joints=len(movable),
                   resolved_mesh_prims_including_instances=len(meshes),
                   fk_poses_checked=len(poses), fk_link_transforms_checked=len(poses)*len(links),
                   max_position_error_m=max_position_error, max_rotation_matrix_error=max_rotation_error,
                   standing_dimensions_m=dimensions.tolist(), display_base_height_m=height,
                   base_to_lidar=lidar_fk.tolist(), base_to_imu=imu_fk.tolist(),
                   urdf_sha256=hashlib.sha256(urdf_path.read_bytes()).hexdigest(),
                   scene=str(scene_path), imported_asset=str(imported_path),
                   simulation_time_start_s=simulation_time_start,
                   simulation_time_end_s=simulation_time_end,
                   timeline_playing_at_end=timeline.is_playing(), dynamics_validated=False,
                   remaining=['Physical mounting calibration', 'SDK reference alignment', 'Contact and inertial validation', 'Sensor and motion backend integration'])
    (output/'verification.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print('MODEL_REVIEW_PASS', json.dumps(summary), flush=True)
    if args.gui:
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=np.array(views['isometric']), target=np.array(target), camera_prim_path='/OmniverseKit_Persp')
        while app.is_running():
            app.update()
finally:
    app.close()
