"""Runtime USD collision boxes, changed only through simulation-owned preset services."""
import json
import math
import re
from copy import deepcopy


def validate_profiles(config):
    if config.get('schema_version') != 1 or config.get('frame_id') != 'sim_world':
        raise ValueError('Unsupported simulation obstacle frame/schema')
    if config.get('root_prim') != '/World/RuntimeObstacles':
        raise ValueError('Obstacle authoring must stay inside its dedicated USD root')
    profiles = deepcopy(config['profiles'])
    if not profiles or len(profiles) > 16:
        raise ValueError('Invalid profile count')
    for name, boxes in profiles.items():
        if not re.fullmatch(r'[a-z][a-z0-9_]{0,31}', name) or not 1 <= len(boxes) <= 16:
            raise ValueError('Invalid obstacle profile')
        ids = set()
        for box in boxes:
            if not re.fullmatch(r'[a-z][a-z0-9_]{0,31}', box['id']) or box['id'] in ids:
                raise ValueError('Invalid/duplicate obstacle id')
            ids.add(box['id'])
            velocity=box.get('velocity_mps',[0.,0.,0.])
            duration=float(box.get('motion_duration_s',0.))
            if len(velocity)!=3 or not all(math.isfinite(float(v)) and abs(float(v))<=1 for v in velocity) or not math.isfinite(duration) or not 0<=duration<=30:
                raise ValueError('Invalid obstacle motion')
            box['velocity_mps']=list(map(float,velocity));box['motion_duration_s']=duration
            for key in ('center_m', 'size_m', 'color_rgb'):
                values = box[key]
                if len(values) != 3 or not all(math.isfinite(float(v)) for v in values):
                    raise ValueError('Invalid obstacle vector')
                box[key] = list(map(float, values))
            if any(v <= 0 or v > 20 for v in box['size_m']):
                raise ValueError('Invalid obstacle size')
            if any(abs(v) > 30 for v in box['center_m']) or any(not 0 <= v <= 1 for v in box['color_rgb']):
                raise ValueError('Invalid obstacle position/color')
    return profiles


class RuntimeObstacles:
    def __init__(self, stage, node, config, output):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        self.stage, self.node, self.config = stage, node, config
        self.profiles = validate_profiles(config)
        self.output = output
        self.String = String
        self.revision = 0
        self.profile = 'clear'
        self.boxes = []
        self.events = []
        self.motion_start_s=0.;self.last_motion_s=-1.;self.translation_ops=[]
        self.motion_file=(self.output/"obstacle_motion.jsonl").open("w",buffering=1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = node.create_publisher(String, config['state_topic'], qos)
        self.services = [node.create_service(Trigger, config['clear_service'], self.clear)]
        for name in self.profiles:
            self.services.append(node.create_service(Trigger, config['profile_service_prefix']+name,
                lambda request, response, key=name: self.apply(key, response)))
        self.timer = node.create_timer(.2, self.publish)
        self.publish()

    def apply(self, name, response):
        from pxr import Gf, UsdGeom, UsdPhysics
        boxes = deepcopy(self.profiles[name])
        # Authoring occurs between physics steps in the same simulation thread.
        # No point cloud is synthesized or sent to navigation here.
        self.stage.RemovePrim(self.config['root_prim'])
        self.translation_ops=[]
        for box in boxes:
            cube = UsdGeom.Cube.Define(self.stage, self.config['root_prim']+'/'+box['id'])
            cube.CreateSizeAttr(1.)
            translate=cube.AddTranslateOp();translate.Set(Gf.Vec3d(*box['center_m']));self.translation_ops.append(translate)
            cube.AddScaleOp().Set(Gf.Vec3f(*box['size_m']))
            cube.CreateDisplayColorAttr([Gf.Vec3f(*box['color_rgb'])])
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        self.profile, self.boxes = name, boxes
        self.motion_start_s=float(self.node.last_clock);self.last_motion_s=-1.
        self.changed()
        response.success = True
        response.message = json.dumps(dict(profile=name, revision=self.revision,
            frame_id='sim_world', physics_visibility='next_physics_step'))
        return response

    def clear(self, request, response):
        self.stage.RemovePrim(self.config['root_prim'])
        self.boxes = []
        self.translation_ops=[]
        self.profile = 'clear'
        self.changed()
        response.success, response.message = True, 'Runtime obstacles removed; saved scene unchanged'
        return response

    def advance(self):
        # Called between actual physics steps. Only USD geometry is moved;
        # navigation sees it through raycasts, never a synthetic obstacle topic.
        from pxr import Gf
        now=float(self.node.last_clock)
        if now-self.last_motion_s < .05: return
        self.last_motion_s=now
        changed=False
        for box,initial,op in zip(self.boxes,self.profiles.get(self.profile,[]),self.translation_ops):
            elapsed=max(0.,min(now-self.motion_start_s,initial.get('motion_duration_s',0.)))
            center=[p+elapsed*v for p,v in zip(initial['center_m'],initial.get('velocity_mps',[0.,0.,0.]))]
            if center!=box['center_m']:
                op.Set(Gf.Vec3d(*center));box['center_m']=center;changed=True

        if changed:self.motion_file.write(json.dumps(dict(sim_time_s=now,boxes=deepcopy(self.boxes)))+"\n")

    def changed(self):
        self.revision += 1
        self.events.append(self.state())
        (self.output/'obstacle_events.json').write_text(json.dumps(self.events, indent=2)+'\n')
        self.publish()

    def state(self):
        return dict(schema_version=1, frame_id='sim_world', revision=self.revision,
            profile=self.profile, boxes=deepcopy(self.boxes), sim_time_s=self.node.last_clock,
            epoch=self.node.epoch, source='authored_usd_collision_geometry')

    def publish(self):
        state = self.state()
        self.publisher.publish(self.String(data=json.dumps(state)))
        temp = self.output/'obstacles.pending'
        temp.write_text(json.dumps(state, indent=2)+'\n')
        temp.replace(self.output/'obstacles.json')
