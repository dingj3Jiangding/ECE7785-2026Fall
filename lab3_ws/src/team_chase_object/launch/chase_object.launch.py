"""Launch the camera TF, color detector, range estimator, and motion controller.

Lab partners: TODO_NAME_1, TODO_NAME_2 (replace before submission).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
            DeclareLaunchArgument(
                "publish_camera_tf",
                default_value="true",
                description="Publish the lab camera mounting TF; disable if another node provides it.",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_static_tf",
                condition=IfCondition(LaunchConfiguration("publish_camera_tf")),
                output="screen",
                # Measured camera position relative to the LiDAR: 7 cm forward,
                # 0 cm left, 5 cm up. Rotation assumes a level, forward-facing
                # camera with an upright, unmirrored image (optical axes).
                arguments=[
                    "--x", "0.07", "--y", "0.0", "--z", "0.05",
                    "--qx", "-0.5", "--qy", "0.5", "--qz", "-0.5", "--qw", "0.5",
                    "--frame-id", "base_scan", "--child-frame-id", "camera",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
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
