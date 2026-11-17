"""Validate the owned localization profile before starting any process."""
import math
from map_registration import RegistrationConfig


def validate_profile(profile):
    if profile.get('schema_version') != 1:
        raise ValueError('Unsupported localization profile schema')
    RegistrationConfig.from_dict(profile['registration'])
    runtime = profile['runtime']
    topics = ('scan_topic', 'base_status_topic', 'runtime_status_topic')
    timings = ('future_tolerance_s', 'input_max_age_s', 'status_max_age_s',
        'status_timeout_wall_s', 'registration_interval_s', 'result_max_age_s',
        'correction_max_age_s', 'initial_pose_max_age_s', 'initial_pair_tolerance_s',
        'stationary_window_s', 'stationary_linear_m_s', 'stationary_angular_rad_s',
        'stationary_span_m', 'stationary_angle_rad')
    required = set(topics + timings + ('scan_frame', 'max_pending_pairs'))
    if set(runtime) != required:
        raise ValueError('Missing or unknown localization runtime settings')
    for name in topics:
        if not isinstance(runtime[name], str) or not runtime[name].startswith('/'):
            raise ValueError('Runtime topics must be absolute ROS names')
    if not isinstance(runtime['scan_frame'], str) or not runtime['scan_frame']:
        raise ValueError('A scan reference frame is required')
    count = runtime['max_pending_pairs']
    if isinstance(count, bool) or not isinstance(count, int) or not 5 <= count <= 200:
        raise ValueError('Pending-pair count must be 5..200')
    for name in timings:
        value = runtime[name]
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError('Runtime timing and motion thresholds must be finite and positive')
    if not runtime['stationary_window_s'] < runtime['initial_pose_max_age_s']:
        raise ValueError('Initial pose must remain valid across its stationary window')
    if runtime['result_max_age_s'] > runtime['correction_max_age_s']:
        raise ValueError('A result cannot outlive the correction expiry limit')
