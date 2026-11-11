"""Execute the configured waypoints after Nav2 is active and AMCL is initialized."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from turtlebot3_recovery_nodes.mission_config import load_mission


def _launch_setup(context):
    package_share = Path(get_package_share_directory('turtlebot3_recovery_lab'))
    mission_file = Path(LaunchConfiguration('mission_file').perform(context)).expanduser().resolve()
    if not mission_file.is_file():
        raise RuntimeError(f'mission_file does not exist: {mission_file}')
    load_mission(str(mission_file), require_waypoints=True, require_recovery=True)
    bt_xml = package_share / 'behavior_trees' / 'navigate_w_recovery_divert.xml'
    if not bt_xml.is_file():
        raise RuntimeError(f'Installed behavior tree is missing: {bt_xml}')
    return [Node(
        package='turtlebot3_recovery_lab',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        output='screen',
        parameters=[{
            'mission_file': str(mission_file),
            'behavior_tree': str(bt_xml),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'server_timeout_sec': ParameterValue(
                LaunchConfiguration('server_timeout_sec'), value_type=float),
            'goal_timeout_sec': ParameterValue(
                LaunchConfiguration('goal_timeout_sec'), value_type=float),
            'cancel_timeout_sec': ParameterValue(
                LaunchConfiguration('cancel_timeout_sec'), value_type=float),
        }],
    )]


def generate_launch_description():
    package_share = Path(get_package_share_directory('turtlebot3_recovery_lab'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'mission_file', default_value=str(package_share / 'config' / 'mission.yaml'),
            description='Use the same mission file as lab5_bringup.launch.py',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('server_timeout_sec', default_value='60.0'),
        DeclareLaunchArgument('goal_timeout_sec', default_value='300.0'),
        DeclareLaunchArgument('cancel_timeout_sec', default_value='5.0'),
        OpaqueFunction(function=_launch_setup),
    ])
