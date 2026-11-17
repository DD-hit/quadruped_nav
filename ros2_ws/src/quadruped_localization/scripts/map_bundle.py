"""Bounded voxel accumulation and checked, portable localization PCD bundles."""
from pathlib import Path
import hashlib,json,math,os,re
import numpy as np
import yaml


def identifier(value):
    if not re.fullmatch(r'[a-z0-9][a-z0-9_-]{0,63}',value):raise ValueError('Map id must be 1..64 lowercase letters, digits, underscores or hyphens')
    return value


def sha256(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_rotation(R):
    R=np.asarray(R,dtype=float)
    if R.shape!=(3,3) or not np.isfinite(R).all() or not np.allclose(R.T@R,np.eye(3),atol=1e-7) or abs(np.linalg.det(R)-1)>1e-7:
        raise ValueError('Invalid rotation matrix')
    return R


def pointcloud_xyzi(message,max_points):
    count=message.width*message.height
    if count<1 or count>max_points:raise ValueError('Invalid registered cloud point count')
    if message.row_step<message.width*message.point_step or len(message.data)!=message.height*message.row_step:
        raise ValueError('Invalid cloud buffer/row stride')
    fields={f.name:f for f in message.fields};names=['x','y','z','intensity']
    if any(name not in fields for name in names):raise ValueError('Registered cloud must have XYZI fields')
    formats=[];offsets=[];endian='>' if message.is_bigendian else '<'
    for name in names:
        f=fields[name]
        if f.count!=1 or f.datatype not in (7,8):raise ValueError('XYZI fields must be scalar floating point')
        size=4 if f.datatype==7 else 8
        if f.offset+size>message.point_step:raise ValueError('Field exceeds point stride')
        formats.append(endian+f'f{size}');offsets.append(f.offset)
    dtype=np.dtype(dict(names=names,formats=formats,offsets=offsets,itemsize=message.point_step))
    rows=np.ndarray(shape=(message.height,message.width),dtype=dtype,buffer=bytes(message.data),strides=(message.row_step,message.point_step))
    points=np.column_stack([rows[name].ravel() for name in names]).astype(np.float64)
    if not np.isfinite(points).all():raise ValueError('Non-finite registered cloud')
    return points


class VoxelMap:
    def __init__(self,resolution,max_voxels):
        if not math.isfinite(resolution) or resolution<=0 or max_voxels<1:raise ValueError('Invalid voxel limits')
        self.resolution=resolution;self.max_voxels=max_voxels;self.cells={};self.input_points=0

    def add(self,points):
        points=np.asarray(points,dtype=np.float64)
        if points.ndim!=2 or points.shape[1]!=4 or not np.isfinite(points).all():raise ValueError('Expected finite Nx4 points')
        scaled=np.floor(points[:,:3]/self.resolution)
        if np.max(abs(scaled),initial=0)>1e12:raise ValueError('Coordinates exceed supported voxel range')
        unique,inverse,counts=np.unique(scaled.astype(np.int64),axis=0,return_inverse=True,return_counts=True)
        keys=[tuple(k) for k in unique]
        if len(self.cells)+sum(k not in self.cells for k in keys)>self.max_voxels:raise ValueError('Voxel limit reached; session cannot grow further')
        sums=np.zeros((len(keys),4));np.add.at(sums,inverse,points)
        if not np.isfinite(sums).all():raise ValueError('Voxel sums overflow')
        for k,total,count in zip(keys,sums,counts):
            if k in self.cells:
                previous,n=self.cells[k];self.cells[k]=(previous+total,n+int(count))
            else:self.cells[k]=(total,int(count))
        self.input_points+=len(points)

    def points(self):
        if not self.cells:return np.empty((0,4),dtype='<f4')
        return np.asarray([self.cells[k][0]/self.cells[k][1] for k in sorted(self.cells)],dtype='<f4')


def write_pcd(path,points):
    points=np.asarray(points,dtype='<f4')
    if points.ndim!=2 or points.shape[1]!=4 or len(points)<1 or not np.isfinite(points).all():raise ValueError('Expected non-empty finite Nx4 map')
    count=len(points)
    header=(f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {count}\nDATA binary\n').encode('ascii')
    with Path(path).open('xb') as stream:
        stream.write(header);stream.write(points.tobytes());stream.flush();os.fsync(stream.fileno())


def read_pcd(path,max_points=500000):
    with Path(path).open('rb') as stream:
        fields={};header_bytes=0
        while True:
            line=stream.readline(1024);header_bytes+=len(line)
            if not line or header_bytes>8192:raise ValueError('Invalid PCD header')
            text=line.decode('ascii').strip()
            if not text or text.startswith('#'):continue
            name,*values=text.split();fields[name]=values
            if name=='DATA':break
        expected={'VERSION':['0.7'],'FIELDS':['x','y','z','intensity'],'SIZE':['4']*4,'TYPE':['F']*4,'COUNT':['1']*4,'HEIGHT':['1'],'DATA':['binary']}
        if any(fields.get(k)!=v for k,v in expected.items()):raise ValueError('Unsupported PCD schema; expected exported binary XYZI')
        count=int(fields['POINTS'][0])
        if count<1 or count>max_points or fields['WIDTH']!=[str(count)]:raise ValueError('Invalid PCD point count')
        payload=stream.read(count*16+1)
        if len(payload)!=count*16:raise ValueError('PCD payload length mismatch')
    points=np.frombuffer(payload,dtype='<f4').reshape(count,4).copy()
    if not np.isfinite(points).all():raise ValueError('Non-finite map point')
    return points


def commit_bundle(directory,points,manifest,snapshots):
    directory=Path(directory)
    if (directory/'manifest.yaml').exists():raise ValueError('Map bundle is already committed')
    temporary=directory/'localization.pcd.pending';write_pcd(temporary,points)
    os.replace(temporary,directory/'localization.pcd')
    files={'localization.pcd':sha256(directory/'localization.pcd')}
    snapshot_dir=directory/'configuration';snapshot_dir.mkdir(exist_ok=True)
    for name,value in snapshots.items():
        if not re.fullmatch(r'[a-z_]+\.(yaml|json)',name):raise ValueError('Invalid snapshot filename')
        path=snapshot_dir/name
        with path.open('x') as stream:
            stream.write(yaml.safe_dump(value,sort_keys=False) if path.suffix=='.yaml' else json.dumps(value,indent=2)+'\n')
        files[str(path.relative_to(directory))]=sha256(path)
    manifest=dict(manifest,schema_version=1,kind='localization_pcd',status='complete',
        pcd_file='localization.pcd',point_count=int(len(points)),files_sha256=files,
        bounds=dict(min=points[:,:3].min(axis=0).tolist(),max=points[:,:3].max(axis=0).tolist()))
    path=directory/'manifest.yaml.pending'
    with path.open('x') as stream:stream.write(yaml.safe_dump(manifest,sort_keys=False));stream.flush();os.fsync(stream.fileno())
    os.replace(path,directory/'manifest.yaml')  # Commit marker, written last.
    return manifest


def load_bundle(directory):
    directory=Path(directory).resolve();manifest=yaml.safe_load((directory/'manifest.yaml').read_text())
    if manifest.get('schema_version')!=1 or manifest.get('kind')!='localization_pcd' or manifest.get('status')!='complete':raise ValueError('Incomplete or unsupported map bundle')
    identifier(manifest['map_id'])
    if manifest['frame_id']!='map' or manifest['pcd_file']!='localization.pcd':raise ValueError('Unexpected map frame or PCD path')
    transform=manifest['map_from_camera_init'];validate_rotation(transform['rotation'])
    if np.asarray(transform['translation_m']).shape!=(3,) or not np.isfinite(transform['translation_m']).all():raise ValueError('Invalid map transform')
    if 'localization.pcd' not in manifest['files_sha256']:raise ValueError('Missing PCD checksum')
    for relative,expected in manifest['files_sha256'].items():
        path=(directory/relative).resolve()
        if not path.is_relative_to(directory) or not path.is_file() or sha256(path)!=expected:raise ValueError('Missing or changed map bundle file: '+relative)
    points=read_pcd(directory/'localization.pcd')
    if len(points)!=manifest['point_count']:raise ValueError('Map point count differs from manifest')
    if not np.allclose(points[:,:3].min(axis=0),manifest['bounds']['min'],atol=1e-6) or not np.allclose(points[:,:3].max(axis=0),manifest['bounds']['max'],atol=1e-6):raise ValueError('Map bounds differ from manifest')
    return manifest,points
