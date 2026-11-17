# SPDX-License-Identifier: GPL-2.0-only
"""ROS-independent port of FAST_LIO_LOCALIZATION2 registration_at_scale.

Source: Smart-Wheelchair-RRC/FAST_LIO_LOCALIZATION2, ros2,
f04974907c8da976dd0495b272d18ac4c534d41f, global_localization.py.
The caller retains frame/time handling and quality checks. Metric voxel and
iteration settings are configurable for the project's room-scale map. Return
the Open3D result and sampled source so correspondence checks use exact points.
"""
import open3d as o3d


def registration_at_scale(scan, target, initial, *, scan_voxel_size,
                          max_correspondence_m, max_iterations,
                          relative_fitness=1e-7, relative_rmse=1e-7, min_points=0):
    """One upstream point-to-point ICP stage; target is already voxel sampled.

    The map pyramid is cached by the caller. Coarse -> fine chaining belongs to
    the caller: the coarse result, not the original seed, must seed fine ICP.
    """
    source = scan.voxel_down_sample(scan_voxel_size)
    if len(source.points) < min_points:
        return None, source
    result = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_m, initial,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=relative_fitness, relative_rmse=relative_rmse,
            max_iteration=max_iterations))
    return result, source
