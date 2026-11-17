"""Own localization lifecycle, optionally one existing simulation LIO runtime."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback
from zoneinfo import ZoneInfo
import yaml
from localization_map_source import load_localization_source
from map_bundle import identifier
from map_registration import RegistrationConfig
from localization_config import validate_profile


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, required=True)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--map-id')
    group.add_argument('--map-directory', type=Path)
    parser.add_argument('--robot', default='d1_edu')
    parser.add_argument('--mode', choices=('sim', 'real'), default='sim')
    parser.add_argument('--with-lio', action='store_true', help='Also own one existing simulation LIO runtime')
    parser.add_argument('--base-config-file', type=Path, help='Base odometry profile when owning simulation LIO')
    parser.add_argument('--localization-config-file', type=Path, help='Localization profile for this session')
    parser.add_argument('--domain-id', type=int)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args(argv)
    root = args.project_root.resolve()
    configs = root / 'ros2_ws/src/quadruped_bringup/config'
    identifier(args.robot)
    if not (configs / 'robots' / (args.robot + '.yaml')).is_file():
        parser.error('Unknown robot profile')
    host = yaml.safe_load((configs / 'hosts/server.yaml').read_text())
    common = yaml.safe_load((configs / 'common.yaml').read_text())
    directory = args.map_directory if args.map_directory else Path(host['data_root']) / 'maps' / identifier(args.map_id)
    map_info, _ = load_localization_source(directory)
    profile = yaml.safe_load((args.localization_config_file or root / 'ros2_ws/src/quadruped_localization/config/localization.yaml').read_text())
    validate_profile(profile)
    domain = args.domain_id if args.domain_id is not None else host['ros_domain_id']
    if not 0 <= domain <= 232:
        parser.error('ROS domain must lie in 0..232')
    if args.with_lio and (args.mode != 'sim' or domain != host['ros_domain_id']):
        parser.error('--with-lio currently supports the existing sim runtime on the configured host ROS domain')
    if args.base_config_file is not None and not args.with_lio:
        parser.error('--base-config-file requires --with-lio')
    if args.check_only:
        print(json.dumps(dict(result='PASS', map=map_info, robot=args.robot, mode=args.mode,
                              domain_id=domain, tf='map -> odom', processes_started=False), indent=2))
        return 0
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    os.environ.setdefault('OMP_NUM_THREADS', '2')
    os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
    output = args.output_dir or Path(host['data_root']) / 'runs' / ('localization_' + datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f'))
    output.mkdir(parents=True, exist_ok=False)
    status_file = output / 'status.json'
    (output / 'map_source.json').write_text(json.dumps(map_info, indent=2) + '\n')
    for name, value in [('localization', profile), ('common', common),
                        ('robot', yaml.safe_load((configs / 'robots' / (args.robot + '.yaml')).read_text()))]:
        (output / (name + '.yaml')).write_text(yaml.safe_dump(value, sort_keys=False))
    shutdown = False
    def on_signal(signum, frame):
        nonlocal shutdown
        shutdown = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    processes, logs = [], []
    code = 0
    try:
        if args.with_lio:
            import rclpy
            from rclpy.node import Node
            rclpy.init(args=[])
            guard = Node('quadruped_localization_lio_guard')
            try:
                deadline = time.monotonic() + .8
                while time.monotonic() < deadline:
                    rclpy.spin_once(guard, timeout_sec=.05)
                for topic in (common['interfaces']['odometry']['topic'], '/lio/odometry'):
                    if guard.get_publishers_info_by_topic(topic):
                        raise RuntimeError('Existing LIO detected; omit --with-lio to reuse it')
            finally:
                guard.destroy_node()
                rclpy.shutdown()
        def start(name, command):
            handle = (output / (name + '.log')).open('w')
            logs.append(handle)
            process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append((name, process, command))
        package = root / 'ros2_ws/src/quadruped_localization/scripts'
        localizer_command = ['/usr/bin/python3', str(package / 'map_localizer.py'),
            '--project-root', str(root), '--map-directory', str(directory.resolve()),
            '--robot', args.robot, '--status-file', str(status_file)]
        if args.localization_config_file is not None:
            localizer_command.extend(['--config-file', str(args.localization_config_file.resolve())])
        localizer_command.extend(['--ros-args', '-p', 'use_sim_time:=' + str(args.mode == 'sim').lower()])
        start('map_localizer', localizer_command)
        if args.with_lio:
            lio_command = ['/usr/bin/python3', str(root / 'scripts/run_lio.py'),
                '--robot', args.robot, '--output-dir', str(output / 'lio')]
            if args.base_config_file is not None:
                lio_command.extend(['--base-config-file', str(args.base_config_file.resolve())])
            start('lio_runtime', lio_command)
        print('LOCALIZATION_SESSION', output, flush=True)
        while not shutdown:
            for name, process, _ in processes:
                if process.poll() is not None:
                    raise RuntimeError(name + ' exited unexpectedly')
            time.sleep(.05)
    except Exception:
        code = 1
        (output / 'error.txt').write_text(traceback.format_exc())
        traceback.print_exc()
    finally:
        records = []
        for name, process, command in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)
            if process.returncode != 0:
                code = 1
            records.append(dict(name=name, pid=process.pid, return_code=process.returncode, command=command))
        for handle in logs:
            handle.close()
        status = json.loads(status_file.read_text()) if status_file.is_file() else {}
        (output / 'runtime_result.json').write_text(json.dumps(dict(
            result='FAIL' if code else 'STOPPED', processes=records,
            localized_at_least_once=status.get('accepted_matches', 0) > 0,
            map=map_info, owns_lio=args.with_lio, mode=args.mode,
            hardware_started=False, online_accuracy_verified=False), indent=2) + '\n')
    return code
