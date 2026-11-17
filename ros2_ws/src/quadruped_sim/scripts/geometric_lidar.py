"""Idealized timed raycasts of an Isaac scene; not a Livox hardware emulator."""
import json
import math
import numpy as np
from omni.physx import get_physx_scene_query_interface
from pxr import Gf, Usd, UsdGeom, UsdPhysics


POINT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                        ('intensity', '<f4'), ('time', '<f4'),
                        ('offset_time', '<u4'), ('ray_id', '<u4')])


def create_validation_room(stage, config):
    # Imported convex hulls of a complete arm can fill its open spaces and
    # contain the laser origin. Static optical queries require triangle faces.
    paths=[str(p.GetPath()) for p in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies())]
    for path in paths:
        if '/World/Robot/' not in path or '/collisions/' not in path:
            continue
        prim=stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        ancestor=prim
        while ancestor.IsInstanceProxy():
            ancestor=ancestor.GetParent()
        if ancestor.IsInstance():
            ancestor.SetInstanceable(False)
    count=0
    for prim in list(stage.Traverse()):
        if str(prim.GetPath()).startswith('/World/Robot/') and '/collisions/' in str(prim.GetPath()) and prim.IsA(UsdGeom.Mesh):
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('none')
            count+=1
    print('LIDAR_TRIANGLE_QUERY_MESHES',count,flush=True)
    for name, geometry in config['room'].items():
        cube = UsdGeom.Cube.Define(stage, '/World/LidarRoom/'+name)
        cube.CreateSizeAttr(1.0)
        # Translation before scaling keeps center in world metres.
        cube.AddTranslateOp().Set(Gf.Vec3d(*geometry['center']))
        cube.AddScaleOp().Set(Gf.Vec3f(*geometry['size']))
        cube.CreateDisplayColorAttr([(0.5,0.6,0.7) if name in ('box','pillar') else (0.75,0.78,0.8)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


class GeometricLidar:
    def __init__(self, stage, node, config, profile, common, output):
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        self.PointCloud2, self.String = PointCloud2, String
        self.node, self.config, self.stage = node, config, stage
        self.output = output
        self.hz = int(config['scan_hz'])
        self.size = int(config['samples_per_scan'])
        if self.hz <= 0 or self.size <= 0 or 1_000_000_000 % self.hz:
            raise ValueError('scan_hz and samples_per_scan must be positive, with an integral nanosecond scan period')
        self.rate = self.hz*self.size
        self.period_ns = 1_000_000_000//self.hz
        self.minimum = float(config['near_range_m'])
        self.maximum = float(config['far_range_m'])
        self.elevation = tuple(math.radians(v) for v in config['elevation_deg'])
        self.frame = common['frames']['lidar']
        if profile['frame_aliases']['lidar'] != self.frame:
            raise ValueError('Sensor frame names differ between robot and common profiles')
        base = stage.GetPrimAtPath('/World/Robot/'+profile['model']['target_root_frame'])
        sensor = stage.GetPrimAtPath('/World/Robot/'+self.frame)
        tb = np.array(UsdGeom.Xformable(base).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
        ts = np.array(UsdGeom.Xformable(sensor).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
        self.mount = np.linalg.inv(tb)@ts
        self.ignored = ('/World/Robot/'+self.frame+'/',
                        '/World/Robot/'+profile['frame_aliases']['imu']+'/')
        self.query = get_physx_scene_query_interface()
        self.cloud_pub = node.create_publisher(PointCloud2, common['interfaces']['lidar_points']['topic'],
            QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.status_pub = node.create_publisher(String, config['status_topic'], 10)
        self.fields = [PointField(name=name,offset=index*4,
            datatype=PointField.UINT32 if name in ('offset_time','ray_id') else PointField.FLOAT32,count=1)
            for index,name in enumerate(POINT_DTYPE.names)]
        self.stats_file = (output/'lidar_scans.jsonl').open('w',buffering=1)
        self.reset()

    def reset(self):
        self.next_index = 0
        self.scan_number = 0
        self.points = []
        self.self_hits = self.misses = 0
        self.scene_obstacle_hits = {}

    def cast(self, origin, direction):
        best = [math.inf, None]

        def report(hit):
            collision = str(hit.collision)
            if collision.startswith(self.ignored):
                return True
            if hit.distance < best[0]:
                best[:] = [float(hit.distance), collision]
            return True

        self.query.raycast_all(Gf.Vec3f(*origin),Gf.Vec3f(*direction),self.maximum,report)
        distance,path = best
        if path is None or not self.minimum <= distance <= self.maximum:
            self.misses += 1
            return None
        if path.startswith('/World/Robot/'):
            self.self_hits += 1
            return None
        if path.startswith('/World/RuntimeObstacles/'):
            self.scene_obstacle_hits[path] = self.scene_obstacle_hits.get(path, 0)+1
        return distance

    def sample(self, begin_time, end_time, begin_pose, end_pose):
        if end_time <= begin_time:
            return
        q0 = Gf.Matrix4d(*begin_pose.T.flatten().tolist()).ExtractRotationQuat()
        q1 = Gf.Matrix4d(*end_pose.T.flatten().tolist()).ExtractRotationQuat()
        low,high = (math.sin(v) for v in self.elevation)
        while self.next_index/self.rate < end_time-1e-10:
            index = self.next_index
            timestamp = index/self.rate
            if timestamp < begin_time-1e-7:
                raise RuntimeError('Lidar sampling skipped an unrecorded simulation interval')
            fraction = max(0.0,min(1.0,(timestamp-begin_time)/(end_time-begin_time)))
            rotation = np.array(Gf.Matrix3d(Gf.Slerp(fraction,q0,q1))).T
            position = (1-fraction)*begin_pose[:3,3]+fraction*end_pose[:3,3]
            sensor_rotation = rotation@self.mount[:3,:3]
            sensor_origin = position+rotation@self.mount[:3,3]
            # Incommensurate sequences vary coverage between scans. They do not
            # reproduce the manufacturer's proprietary firing trajectory.
            azimuth = 2*math.pi*(((index+1)*(math.sqrt(5)-1)/2)%1)
            sine_elevation = low+(high-low)*(((index+1)*(math.sqrt(2)-1))%1)
            cosine_elevation = math.sqrt(max(0.0,1-sine_elevation*sine_elevation))
            local_direction = np.array([cosine_elevation*math.cos(azimuth),
                                        cosine_elevation*math.sin(azimuth),sine_elevation])
            distance = self.cast(sensor_origin,sensor_rotation@local_direction)
            if distance is not None:
                x,y,z = distance*local_direction
                offset = round((index%self.size)*1_000_000_000/self.rate)
                self.points.append((float(x),float(y),float(z),float(self.config['intensity']),
                                    offset/1e9,offset,index%(2**32)))
            self.next_index += 1
            if self.next_index % self.size == 0:
                self.publish()

    def publish(self):
        data = np.array(self.points,dtype=POINT_DTYPE)
        cloud = self.PointCloud2()
        start_ns = self.scan_number*self.period_ns
        cloud.header.stamp.sec,cloud.header.stamp.nanosec = divmod(start_ns,1_000_000_000)
        cloud.header.frame_id = self.frame
        cloud.height,cloud.width = 1,len(data)
        cloud.fields = self.fields
        cloud.is_bigendian = False
        cloud.point_step = POINT_DTYPE.itemsize
        cloud.row_step = cloud.point_step*len(data)
        cloud.data = data.tobytes()
        cloud.is_dense = True
        self.cloud_pub.publish(cloud)
        metadata = dict(epoch=self.node.epoch,scan=self.scan_number,stamp_ns=start_ns,
            points=len(data),rays=self.size,frame=self.frame,self_returns_discarded=self.self_hits,
            misses=self.misses,nominal_scan_hz=self.hz,ray_rate_hz=self.rate,
            scene_obstacle_hits=dict(self.scene_obstacle_hits),
            scene_revision=self.node.obstacles.revision if self.node.obstacles else 0,
            mount_matrix=self.mount.tolist(),model=self.config['model'],
            scan_pattern=self.config['pattern'],manufacturer_pattern_reproduced=False,
            timing='per_ray_translation_and_quaternion_interpolation_between_physics_steps')
        self.status_pub.publish(self.String(data=json.dumps(metadata)))
        self.stats_file.write(json.dumps(metadata)+'\n')
        if self.scan_number == 0:
            np.save(self.output/'first_scan.npy',data)
        self.scan_number += 1
        self.points = []
        self.self_hits = self.misses = 0
        self.scene_obstacle_hits = {}

    def close(self):
        self.stats_file.close()
