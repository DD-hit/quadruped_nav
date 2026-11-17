"""Rigid IMU-to-base transforms; no ROS, simulator truth, or floor assumption."""
import numpy as np
from lio_config import rotation


def skew(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def quaternion_matrix(q):
    q=np.asarray(q,dtype=float)
    if not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-3:
        raise ValueError('Invalid unit quaternion')
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def matrix_quaternion(R):
    # Symmetric eigenproblem remains well conditioned at 180 degree rotations.
    m=np.asarray(R)
    K=np.array([[m[0,0]-m[1,1]-m[2,2],m[1,0]+m[0,1],m[2,0]+m[0,2],m[2,1]-m[1,2]],
        [m[1,0]+m[0,1],m[1,1]-m[0,0]-m[2,2],m[2,1]+m[1,2],m[0,2]-m[2,0]],
        [m[2,0]+m[0,2],m[2,1]+m[1,2],m[2,2]-m[0,0]-m[1,1],m[1,0]-m[0,1]],
        [m[2,1]-m[1,2],m[0,2]-m[2,0],m[1,0]-m[0,1],np.trace(m)]])/3
    _,vectors=np.linalg.eigh(K);q=vectors[:,-1]
    return q if q[3]>=0 else -q


def mount(robot):
    lidar=robot['sensor']['base_to_lidar'];imu=robot['sensor']['lidar_to_imu']
    RBL=rotation(lidar['rpy_rad']);RBI=RBL@rotation(imu['rpy_rad'])
    rBI=np.asarray(lidar['translation_m'])+RBL@np.asarray(imu['translation_m'])
    return RBI,rBI


def base_in_reference(pCI,RCI,RBI,rBI):
    RCB=RCI@RBI.T
    return pCI-RCB@rBI,RCB


def level_anchor(pCB,RCB,gravity_C):
    gravity_C=np.asarray(gravity_C,dtype=float);norm=np.linalg.norm(gravity_C)
    if not np.isfinite(gravity_C).all() or not 7<norm<12.5:
        raise ValueError('Invalid estimated gravity')
    z=-gravity_C/norm
    x=RCB[:,0]-z*np.dot(z,RCB[:,0])
    if np.linalg.norm(x)<.2:raise ValueError('Base forward axis too close to vertical')
    x/=np.linalg.norm(x);y=np.cross(z,x)
    ROC=np.column_stack((x,y,z)).T
    return ROC,-ROC@pCB


def convert(pCI,RCI,vI,wI,pose_cov,twist_cov,RBI,rBI,ROC,tOC):
    pCB,RCB=base_in_reference(pCI,RCI,RBI,rBI)
    lever_C=pCB-pCI
    Jpose=np.zeros((6,6));Jpose[:3,:3]=ROC;Jpose[:3,3:]=-ROC@skew(lever_C);Jpose[3:,3:]=ROC
    Jtwist=np.zeros((6,6));Jtwist[:3,:3]=RBI;Jtwist[:3,3:]=skew(rBI)@RBI;Jtwist[3:,3:]=RBI
    velocity=Jtwist@np.r_[vI,wI]
    return dict(position=ROC@pCB+tOC,rotation=ROC@RCB,linear_velocity=velocity[:3],angular_velocity=velocity[3:],
        pose_covariance=Jpose@pose_cov@Jpose.T,twist_covariance=Jtwist@twist_cov@Jtwist.T)
