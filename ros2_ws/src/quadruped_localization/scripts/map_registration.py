# SPDX-License-Identifier: GPL-2.0-only
"""Bounded, stateless map registration; no ROS, robot model, or simulator inputs.

Migrates That-nav's coarse/fine point-to-point ICP approach into this project.
See NOTICE and migration_sources.json for the pinned source and changes.
T_A_B maps points expressed in B into A. A registered LIO scan is already
in camera_init: never apply lidar-to-IMU installation extrinsics to it again.
"""
from dataclasses import dataclass, field, fields
from typing import Optional
import math

import numpy as np
import open3d as o3d
from fast_lio_localization2_core import registration_at_scale


def rigid_transform(value):
    """Validate SE(3) and return an independent float64 matrix."""
    T = np.array(value, dtype=np.float64, copy=True)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError('Expected a finite 4x4 rigid transform')
    R = T[:3, :3]
    if (not np.allclose(T[3], [0., 0., 0., 1.], atol=1e-9, rtol=0)
            or not np.allclose(R.T @ R, np.eye(3), atol=1e-6, rtol=0)
            or abs(np.linalg.det(R) - 1.) > 1e-6):
        raise ValueError('Transform must have a proper rotation and homogeneous last row')
    return T


def inverse_transform(value):
    T = rigid_transform(value)
    result = np.eye(4)
    result[:3, :3] = T[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ T[:3, 3]
    return result


def xyz_points(value, maximum):
    points = np.asarray(value, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) > maximum:
        raise ValueError('Expected Nx3 points within the configured point limit')
    if not np.isfinite(points).all() or np.max(np.abs(points), initial=0.) > 1e6:
        raise ValueError('Points must be finite and within supported metric coordinates')
    return np.array(points, dtype=np.float64, order='C', copy=True)


def transform_points(points, transform):
    T = rigid_transform(transform)
    points = xyz_points(points, 500000)
    return points @ T[:3, :3].T + T[:3, 3]


def base_pose_to_registration_guess(map_from_base, odom_from_base, odom_from_scan):
    """Convert a same-time mechanical-base initial pose to a scan-frame guess."""
    return (rigid_transform(map_from_base) @ inverse_transform(odom_from_base)
            @ rigid_transform(odom_from_scan))


def registration_to_map_odom(map_from_scan, odom_from_scan):
    return rigid_transform(map_from_scan) @ inverse_transform(odom_from_scan)


def crop_submap(points_map, map_from_sensor, *, max_range_m, horizontal_fov_rad,
                min_range_m=0.):
    """Spherical metric range and optional sensor-frame horizontal FOV.

    Returns a copy. The caller supplies the sensor pose; it must not substitute
    the mechanical base pose when a sensor has a different orientation.
    """
    if (not all(math.isfinite(x) for x in (min_range_m, max_range_m, horizontal_fov_rad))
            or not 0 <= min_range_m < max_range_m
            or not 0 < horizontal_fov_rad <= 2 * math.pi):
        raise ValueError('Invalid submap range or FOV')
    points = xyz_points(points_map, 500000)
    sensor_from_map = inverse_transform(map_from_sensor)
    local = points @ sensor_from_map[:3, :3].T + sensor_from_map[:3, 3]
    distance = np.linalg.norm(local, axis=1)
    keep = (distance >= min_range_m) & (distance <= max_range_m)
    if horizontal_fov_rad < 2 * math.pi - 1e-9:
        keep &= np.abs(np.arctan2(local[:, 1], local[:, 0])) <= horizontal_fov_rad / 2
    return points[keep].copy()


@dataclass(frozen=True)
class ICPLevel:
    map_voxel_m: float = .20
    scan_voxel_m: float = .12
    max_correspondence_m: float = .80
    max_iterations: int = 60

    def __post_init__(self):
        for value in (self.map_voxel_m, self.scan_voxel_m, self.max_correspondence_m):
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError('ICP distances must be finite and positive')
        if isinstance(self.max_iterations, bool) or not isinstance(self.max_iterations, int) or not 1 <= self.max_iterations <= 500:
            raise ValueError('ICP iteration limit must be 1..500')


@dataclass(frozen=True)
class RegistrationConfig:
    levels: tuple = field(default_factory=lambda: (
        ICPLevel(), ICPLevel(.08, .05, .20, 80)))
    tracking_level_start: int = 0
    max_map_points: int = 500000
    max_scan_points: int = 200000
    min_points: int = 100
    min_correspondences: int = 80
    min_fitness: float = .75
    max_rmse_m: float = .08
    min_spatial_eigen_ratio: float = .002
    min_spatial_std_m: float = .03
    max_range_m: float = 30.
    min_range_m: float = 0.
    horizontal_fov_rad: float = 2 * math.pi
    max_initial_shift_m: float = 1.
    max_initial_rotation_rad: float = math.radians(30)
    max_tracking_shift_m: float = .25
    max_tracking_rotation_rad: float = math.radians(10)

    def __post_init__(self):
        if not isinstance(self.levels, tuple) or not 1 <= len(self.levels) <= 4 or not all(isinstance(x, ICPLevel) for x in self.levels):
            raise ValueError('Expected 1..4 immutable ICP levels')
        if isinstance(self.tracking_level_start,bool) or not isinstance(self.tracking_level_start,int) or not 0<=self.tracking_level_start<len(self.levels):
            raise ValueError('Tracking level must select an existing ICP level')
        for a, b in zip(self.levels, self.levels[1:]):
            if any(x < y for x, y in zip((a.map_voxel_m, a.scan_voxel_m, a.max_correspondence_m),
                                        (b.map_voxel_m, b.scan_voxel_m, b.max_correspondence_m))):
                raise ValueError('ICP levels must progress from coarse to fine')
        for name in ('max_map_points', 'max_scan_points', 'min_points', 'min_correspondences'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or not 3 <= value <= 500000:
                raise ValueError('Point limits must be integers in 3..500000')
        if self.min_points > min(self.max_map_points, self.max_scan_points) or self.min_correspondences > self.max_scan_points:
            raise ValueError('Minimum counts exceed input limits')
        for f in fields(self):
            if f.name in ('levels','tracking_level_start','max_map_points','max_scan_points','min_points','min_correspondences'):
                continue
            value = getattr(self, f.name)
            if isinstance(value, bool) or not math.isfinite(value) or value < 0:
                raise ValueError('Registration thresholds must be finite and nonnegative')
        if not 0 < self.min_fitness <= 1 or not 0 < self.max_rmse_m <= self.levels[-1].max_correspondence_m:
            raise ValueError('Invalid fitness or fine-level RMSE threshold')
        if not 0 < self.min_spatial_eigen_ratio < 1 or self.min_spatial_std_m <= 0:
            raise ValueError('Degeneracy checks cannot be disabled')
        if not self.min_range_m < self.max_range_m or not 0 < self.horizontal_fov_rad <= 2 * math.pi:
            raise ValueError('Invalid submap geometry')
        if any(not 0 < getattr(self, n) <= math.pi for n in ('max_initial_rotation_rad', 'max_tracking_rotation_rad')):
            raise ValueError('Rotation gates must lie in (0, pi]')
        if self.max_initial_shift_m <= 0 or self.max_tracking_shift_m <= 0:
            raise ValueError('Translation gates must be positive')

    @classmethod
    def from_dict(cls, values):
        values = dict(values)
        if 'levels' in values:
            values['levels'] = tuple(ICPLevel(**item) for item in values['levels'])
        return cls(**values)


@dataclass(frozen=True)
class StageMetrics:
    level: int
    source_points: int
    target_points: int
    fitness: float
    rmse_m: float
    correspondences: int
    initial_map_from_scan: np.ndarray
    candidate_map_from_scan: np.ndarray


@dataclass(frozen=True)
class RegistrationResult:
    accepted: bool
    reason: str
    map_from_scan: Optional[np.ndarray]
    candidate_map_from_scan: Optional[np.ndarray]
    stages: tuple
    fitness: float = 0.
    rmse_m: Optional[float] = None
    correspondences: int = 0
    shift_m: Optional[float] = None
    rotation_rad: Optional[float] = None
    spatial_eigen_ratio: Optional[float] = None


def _frozen_matrix(T):
    result = rigid_transform(T)
    result.setflags(write=False)
    return result


def _cloud(points):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    return cloud


class MapRegistration:
    """Immutable map pyramid and one-shot ICP; accepted state belongs to caller.

    Rejected results never expose a usable map_from_scan and never update a
    previous accepted transform. This class neither estimates covariance nor
    guarantees uniqueness in repeated geometry.
    """
    def __init__(self, points_map, config=None):
        self.config = config if config is not None else RegistrationConfig()
        if not isinstance(self.config, RegistrationConfig):
            raise ValueError('Expected RegistrationConfig')
        points = xyz_points(points_map, self.config.max_map_points)
        if len(points) < self.config.min_points:
            raise ValueError('Map has too few points')
        cloud = _cloud(points)
        self._maps = tuple(np.asarray(cloud.voxel_down_sample(level.map_voxel_m).points).copy()
                           for level in self.config.levels)
        if any(len(p) < self.config.min_points for p in self._maps):
            raise ValueError('Map has too few points after downsampling')
        for points in self._maps:
            points.setflags(write=False)

    def register(self, points_scan, map_from_scan_guess, scan_from_sensor, *,
                 mode='initialize', previous_map_from_scan=None):
        """Register scan-frame XYZ with an explicit same-scan sensor pose.

        Tracking requires the previous accepted correction. Jump gates compare
        correction-induced sensor pose displacement, avoiding dependence on an
        arbitrary scan-frame origin. They do not limit actual robot motion.
        """
        cfg = self.config
        guess = rigid_transform(map_from_scan_guess)
        sensor = rigid_transform(scan_from_sensor)
        if mode not in ('initialize', 'tracking'):
            raise ValueError('Expected initialize or tracking mode')
        if mode == 'tracking' and previous_map_from_scan is None:
            raise ValueError('Tracking requires the previous accepted transform')
        if mode == 'initialize' and previous_map_from_scan is not None:
            raise ValueError('Initialization must not silently reuse tracking state')
        reference = rigid_transform(previous_map_from_scan) if mode == 'tracking' else guess
        scan = xyz_points(points_scan, cfg.max_scan_points)
        stages = []
        candidate = None

        def rejected(reason, **details):
            if stages:
                last = stages[-1]
                details = dict(fitness=last.fitness, rmse_m=last.rmse_m,
                               correspondences=last.correspondences, **details)
            return RegistrationResult(False, reason, None,
                                      None if candidate is None else _frozen_matrix(candidate),
                                      tuple(stages), **details)

        if len(scan) < cfg.min_points:
            return rejected('insufficient_scan_points')
        scan_cloud = _cloud(scan)
        current = guess
        final_source = final_target = final_pairs = None
        for index, (level, points_map) in enumerate(zip(cfg.levels, self._maps)):
            # Initialization searches broadly; tracking may remain inside the
            # already accepted fine basin instead of restarting coarse search.
            if mode=='tracking' and index<cfg.tracking_level_start:
                continue
            cropped = crop_submap(points_map, current @ sensor,
                                  max_range_m=cfg.max_range_m, min_range_m=cfg.min_range_m,
                                  horizontal_fov_rad=cfg.horizontal_fov_rad)
            if len(cropped) < cfg.min_points:
                return rejected('insufficient_submap_points')
            target = _cloud(cropped)
            result, source = registration_at_scale(
                scan_cloud, target, current, scan_voxel_size=level.scan_voxel_m,
                max_correspondence_m=level.max_correspondence_m,
                max_iterations=level.max_iterations, min_points=cfg.min_points)
            if len(source.points) < cfg.min_points:
                return rejected('insufficient_downsampled_scan_points')
            try:
                candidate = rigid_transform(result.transformation)
            except ValueError:
                candidate = None
                return rejected('invalid_icp_transform')
            fitness, rmse = float(result.fitness), float(result.inlier_rmse)
            pairs = np.asarray(result.correspondence_set).copy()
            stages.append(StageMetrics(index, len(source.points), len(target.points), fitness,
                                       rmse, len(pairs), _frozen_matrix(current), _frozen_matrix(candidate)))
            if not math.isfinite(fitness) or not math.isfinite(rmse):
                return rejected('nonfinite_icp_quality')
            if len(pairs) < cfg.min_correspondences:
                return rejected('insufficient_correspondences')
            # Carry the coarse result forward; never restart fine ICP at guess.
            current = candidate
            final_source, final_target, final_pairs = source, target, pairs

        if stages[-1].fitness < cfg.min_fitness:
            return rejected('low_overlap')
        if stages[-1].rmse_m > cfg.max_rmse_m:
            return rejected('high_residual')
        ratios = []
        for points, indices in ((np.asarray(final_source.points), final_pairs[:, 0]),
                                (np.asarray(final_target.points), final_pairs[:, 1])):
            matched = points[indices]
            centered = matched - matched.mean(axis=0)
            eigenvalues = np.linalg.eigvalsh(centered.T @ centered / len(centered))
            ratio = float(max(0., eigenvalues[0]) / max(eigenvalues[-1], 1e-15))
            ratios.append(ratio)
            if eigenvalues[0] < cfg.min_spatial_std_m ** 2 or ratio < cfg.min_spatial_eigen_ratio:
                return rejected('degenerate_geometry', spatial_eigen_ratio=min(ratios))
        predicted_sensor, accepted_sensor = candidate @ sensor, reference @ sensor
        shift = float(np.linalg.norm(predicted_sensor[:3, 3] - accepted_sensor[:3, 3]))
        relative_rotation = predicted_sensor[:3, :3] @ accepted_sensor[:3, :3].T
        angle = float(np.arccos(np.clip((np.trace(relative_rotation) - 1) / 2, -1., 1.)))
        limits = (cfg.max_initial_shift_m, cfg.max_initial_rotation_rad) if mode == 'initialize' else (cfg.max_tracking_shift_m, cfg.max_tracking_rotation_rad)
        diagnostics = dict(shift_m=shift, rotation_rad=angle, spatial_eigen_ratio=min(ratios))
        if shift > limits[0] or angle > limits[1]:
            return rejected('correction_jump', **diagnostics)
        return RegistrationResult(True, 'accepted', _frozen_matrix(candidate),
                                  _frozen_matrix(candidate), tuple(stages), stages[-1].fitness,
                                  stages[-1].rmse_m, stages[-1].correspondences, **diagnostics)
