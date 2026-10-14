"""Launch Lab 4 after the existing robot, odometry, LiDAR, and TF drivers.

Lab partners: Ding Jiang, Tongning Zhang.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    default_params = PathJoinSubstitution([
        FindPackageShare("team_navigate_to_goal"), "config", "lab4.yaml",
    ])
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="Absolute path to the ROS 2 parameter YAML file."),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Enable only when the simulator publishes /clock."),
        Node(
            package="team_navigate_to_goal", executable="navigate_to_goal",
            name="navigate_to_goal", output="screen", emulate_tty=True,
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
    ])
