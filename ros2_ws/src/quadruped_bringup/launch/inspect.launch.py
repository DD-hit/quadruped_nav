"""Inspect selected profiles without launching a robot or simulator."""
import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('quadruped_bringup')
    script = os.path.join(get_package_prefix('quadruped_bringup'), 'lib',
                          'quadruped_bringup', 'inspect_config.py')
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='d1_edu'),
        DeclareLaunchArgument('mode', default_value='sim'),
        DeclareLaunchArgument('host', default_value='server'),
        ExecuteProcess(cmd=[sys.executable, script,
                            '--config-dir', os.path.join(share, 'config'),
                            '--robot', LaunchConfiguration('robot'),
                            '--mode', LaunchConfiguration('mode'),
                            '--host', LaunchConfiguration('host')], output='screen'),
    ])
