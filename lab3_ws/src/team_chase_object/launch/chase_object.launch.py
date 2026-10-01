"""Launch the color detector, range estimator, and motion controller.

Lab partners: TODO_NAME_1, TODO_NAME_2 (replace before submission).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    default_params = PathJoinSubstitution(
        [FindPackageShare("team_chase_object"), "config", "lab3.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Absolute path to the ROS 2 parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock; enable only when simulation publishes it.",
            ),
            *[
                Node(
                    package="team_chase_object",
                    executable=executable,
                    name=executable,
                    output="screen",
                    parameters=[params_file, {"use_sim_time": use_sim_time}],
                    emulate_tty=True,
                )
                for executable in ("detect_object", "get_object_range", "chase_object")
            ],
        ]
    )
