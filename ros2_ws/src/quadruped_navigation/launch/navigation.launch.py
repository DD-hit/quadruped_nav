"""Navigation only: external localization/odometry/cloud in, cmd_vel out."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('quadruped_navigation'))
    args = [DeclareLaunchArgument(name, default_value=default) for name, default in [
        ('use_sim_time', 'false'), ('map_package', ''), ('planning_frame', 'map'),
        ('base_frame', 'base_link'), ('lidar_frame', 'lidar_link'),
        ('odom_topic', '/odom'), ('cloud_topic', '/lio/cloud_registered'),
        ('cmd_vel_topic', '/cmd_vel'), ('params_file', str(share/'config/navigation.yaml'))]]
    common = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    frame = LaunchConfiguration('planning_frame')
    def node(executable, extra=None, remappings=None):
        return Node(package='quadruped_navigation', executable=executable,
                    name=executable.replace('.py', ''), namespace='navigation', output='screen',
                    parameters=[LaunchConfiguration('params_file'), common, extra or {}],
                    remappings=remappings or [])
    return LaunchDescription(args+[
        node('frame_adapter', {k: LaunchConfiguration(k) for k in [
            'planning_frame','base_frame','lidar_frame','odom_topic','cloud_topic']}),
        node('global_planner', {'frame_id': frame, 'path_topic': '/navigation/global_path',
                              'goal_pose_topic': '/navigation/global_goal_pose'}),
        node('local_planner', {'grid_map.frame_id': frame}),
        node('trajectory_controller', remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))]),
        node('navigation_api.py', {'planning_frame': frame}),
        node('map_package_manager', {'autoload_package_path': LaunchConfiguration('map_package')}),
        node('octomap_to_occupied_markers_node'),
    ])
