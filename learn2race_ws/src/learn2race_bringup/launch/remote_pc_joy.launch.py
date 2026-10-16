"""Launch only the Xbox joystick and teleop on a remote PC.

This launch assumes the remote PC can see the Pi over the ROS 2 network. It
does not start the micro-ROS bridge or any hardware-specific nodes.
"""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R19] joy / teleop_twist_joy: joy
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[
                    {
                        "dev": LaunchConfiguration("joy_dev"),
                        "deadzone": 0.08,
                        "autorepeat_rate": 30.0,
                        "coalesce_interval_ms": 1,
                    }
                ],
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                output="screen",
                parameters=[
                    {
                        "require_enable_button": True,
                        "enable_button": 4,
                        "enable_turbo_button": 5,
                        "axis_linear": {"x": 1},
                        "scale_linear": {"x": 0.25},
                        "scale_linear_turbo": {"x": 0.60},
                        "axis_angular": {"yaw": 3},
                        "scale_angular": {"yaw": 1.20},
                        "scale_angular_turbo": {"yaw": 2.50},
                        "publish_stamped_twist": False,
                    }
                ],
                remappings=[
                    ("cmd_vel", LaunchConfiguration("cmd_vel_topic")),
                ],
            ),
        ]
    )
