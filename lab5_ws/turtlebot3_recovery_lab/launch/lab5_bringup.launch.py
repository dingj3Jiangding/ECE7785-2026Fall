"""Start Nav2 and the recovery service; waypoint execution is a separate command."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

from turtlebot3_recovery_nodes.mission_config import load_mission


def _existing_file(context, argument):
    path = Path(LaunchConfiguration(argument).perform(context)).expanduser().resolve()
    if not path.is_file():
        raise RuntimeError(f'{argument} must name an existing file: {path}')
    return str(path)


def _boolean(context, argument):
    value = LaunchConfiguration(argument).perform(context).lower()
    if value not in ('true', 'false'):
        raise RuntimeError(f'{argument} must be true or false')
    return value == 'true'


def _launch_setup(context):
    package_share = Path(get_package_share_directory('turtlebot3_recovery_lab'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))
    map_file = _existing_file(context, 'map')
    params_file = _existing_file(context, 'params_file')
    mission_file = _existing_file(context, 'mission_file')
    mission = load_mission(mission_file, require_recovery=True)

    # Catch a moved map YAML whose relative PGM/PNG path no longer exists.
    with open(map_file, encoding='utf-8') as stream:
        map_config = yaml.safe_load(stream)
    if not isinstance(map_config, dict) or not isinstance(map_config.get('image'), str):
        raise RuntimeError('The map YAML must contain an image file path')
    map_image = Path(map_config['image']).expanduser()
    if not map_image.is_absolute():
        map_image = Path(map_file).parent / map_image
    if not map_image.is_file():
        raise RuntimeError(f'Map image does not exist: {map_image}')

    use_sim_time = _boolean(context, 'use_sim_time')
    enable_sound = _boolean(context, 'enable_sound')
    use_composition = _boolean(context, 'use_composition')
    robot_base_frame = LaunchConfiguration('robot_base_frame').perform(context)
    if not robot_base_frame or robot_base_frame.startswith('/'):
        raise RuntimeError('robot_base_frame must be a nonempty TF frame without a leading slash')
    bt_xml = str(package_share / 'behavior_trees' / 'navigate_w_recovery_divert.xml')
    if not Path(bt_xml).is_file():
        raise RuntimeError(f'Installed behavior tree is missing: {bt_xml}')

    # Leaf-key rewrites update every nested costmap clock/base-frame parameter.
    # Full-key rewrites preserve the local costmap and behavior server's odom frame.
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'use_sim_time': str(use_sim_time).lower(),
            'robot_base_frame': robot_base_frame,
            'base_frame_id': robot_base_frame,
            'default_nav_to_pose_bt_xml': bt_xml,
            'bt_navigator.ros__parameters.global_frame': mission.frame_id,
            'amcl.ros__parameters.global_frame_id': mission.frame_id,
            'global_costmap.global_costmap.ros__parameters.global_frame': mission.frame_id,
            'map_server.ros__parameters.frame_id': mission.frame_id,
        },
        convert_types=True,
    )

    recovery = Node(
        package='turtlebot3_recovery_lab',
        executable='recovery_server',
        name='student_recovery_server',
        output='screen',
        parameters=[{
            'mission_file': mission_file,
            'use_sim_time': use_sim_time,
            'robot_base_frame': robot_base_frame,
            'enable_sound': enable_sound,
            'recovery_clearance': ParameterValue(
                LaunchConfiguration('recovery_clearance'), value_type=float),
            'min_recovery_distance': ParameterValue(
                LaunchConfiguration('min_recovery_distance'), value_type=float),
            'pose_timeout_sec': ParameterValue(
                LaunchConfiguration('pose_timeout_sec'), value_type=float),
            'costmap_timeout_sec': ParameterValue(
                LaunchConfiguration('costmap_timeout_sec'), value_type=float),
        }],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'bringup_launch.py')),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'false',
            # Humble includes PythonExpression(['not ', slam]); use Python boolean spelling.
            'slam': 'False',
            'map': map_file,
            'use_sim_time': str(use_sim_time).lower(),
            'params_file': configured_params,
            'autostart': 'true',
            'use_composition': str(use_composition),
        }.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_share / 'launch' / 'rviz_launch.py')),
        condition=IfCondition(LaunchConfiguration('rviz')),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'false',
            'rviz_config': str(nav2_share / 'rviz' / 'nav2_default_view.rviz'),
        }.items(),
    )
    stop_if_recovery_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=recovery,
            on_exit=[EmitEvent(event=Shutdown(reason='The recovery service exited'))],
        )
    )
    return [stop_if_recovery_exits, recovery, navigation, rviz]


def generate_launch_description():
    package_share = Path(get_package_share_directory('turtlebot3_recovery_lab'))
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to your saved map YAML'),
        DeclareLaunchArgument(
            'mission_file', default_value=str(package_share / 'config' / 'mission.yaml'),
            description='Mission YAML with at least one measured recovery waypoint',
        ),
        DeclareLaunchArgument(
            'params_file', default_value=str(package_share / 'config' / 'nav2_params.yaml'),
            description='Nav2 Humble parameters; adjust for your physical robot and maze',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_sound', default_value='true'),
        DeclareLaunchArgument('robot_base_frame', default_value='base_footprint'),
        DeclareLaunchArgument(
            'recovery_clearance', default_value='0.2',
            description='Recovery-point clearance in metres, including the robot radius',
        ),
        DeclareLaunchArgument(
            'min_recovery_distance', default_value='0.4',
            description='Minimum distance from current position to a recovery point, in metres',
        ),
        DeclareLaunchArgument(
            'pose_timeout_sec', default_value='1.0',
            description='Maximum acceptable age of the robot pose transform, in seconds',
        ),
        DeclareLaunchArgument(
            'costmap_timeout_sec', default_value='3.0',
            description='Maximum acceptable age of the live global costmap, in seconds',
        ),
        DeclareLaunchArgument('use_composition', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
