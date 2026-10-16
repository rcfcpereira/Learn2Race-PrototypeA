"""Start light sensor reader and dragster race-start action server."""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R2]  Dragster rules: dragster start
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("learn2race_dragster")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_light_sensor", default_value="true"),
            DeclareLaunchArgument("start_action_server", default_value="true"),
            DeclareLaunchArgument(
                "light_params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "grove_light_sensor_ads1115.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "start_params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "drag_race_start.yaml"]
                ),
            ),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            Node(
                package="learn2race_dragster",
                executable="grove_light_sensor_ads1115_node",
                name="grove_light_sensor_ads1115_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_light_sensor")),
                parameters=[LaunchConfiguration("light_params_file")],
            ),
            Node(
                package="learn2race_dragster",
                executable="drag_race_start_action_server",
                name="drag_race_start_action_server",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_action_server")),
                parameters=[
                    LaunchConfiguration("start_params_file"),
                    {"cmd_vel_topic": LaunchConfiguration("cmd_vel_topic")},
                ],
            ),
        ]
    )
