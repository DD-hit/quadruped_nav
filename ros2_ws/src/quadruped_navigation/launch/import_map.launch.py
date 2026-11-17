"""Import a PCD with upstream jie tools; map save/load uses standard project services."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('quadruped_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('pcd_file'),
        DeclareLaunchArgument('resolution', default_value='0.1'),
        DeclareLaunchArgument('planning_frame', default_value='map'),
        Node(package='quadruped_navigation', executable='pcd_to_octomap_node', namespace='navigation',
             parameters=[{'pcd_file': LaunchConfiguration('pcd_file'), 'frame_id': LaunchConfiguration('planning_frame'),
                          'resolution': ParameterValue(LaunchConfiguration('resolution'), value_type=float),
                          'min_points_per_voxel': 1, 'min_cluster_voxels': 1}], output='screen'),
        Node(package='quadruped_navigation', executable='global_planner', namespace='navigation',
             parameters=[str(share/'config/navigation.yaml'), {'frame_id': LaunchConfiguration('planning_frame')}], output='screen'),
        Node(package='quadruped_navigation', executable='map_package_manager', namespace='navigation', output='screen'),
        Node(package='quadruped_navigation', executable='octomap_to_occupied_markers_node', namespace='navigation', output='screen'),
    ])
