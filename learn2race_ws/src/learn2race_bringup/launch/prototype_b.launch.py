"""Full bringup for Prototype B (teaching and mapping platform).

Competition nodes are off by default; enable them for classroom demonstration
of the state machine with competition:=true. Prototype B is built for teaching
and mapping, not for competition.
"""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R14] micro-ROS / micro_ros_arduino: micro-ROS agent
#   [R46] Madgwick (2010) orientation filter: imu_filter_madgwick
#   [R79] LDROBOT STL-06P lidar, ldlidar_stl_ros2: lidar driver
# More details: README.md of this package

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('learn2race_bringup'),
        'config', 'prototype_b.yaml')

    competition = LaunchConfiguration('competition')
    teleop = LaunchConfiguration('teleop')
    serial_port = LaunchConfiguration('serial_port')
    lidar_port = LaunchConfiguration('lidar_port')

    return LaunchDescription([
        DeclareLaunchArgument('competition', default_value='false'),
        DeclareLaunchArgument('teleop', default_value='true'),
        # Stable names created by scripts/setup_udev.sh. ttyUSB0/ttyUSB1 swap
        # depending on enumeration order when both the board and the LiDAR are
        # connected, so the agent can end up opening the LiDAR. Override with
        # serial_port:=/dev/ttyUSB0 if the udev rule is not installed.
        DeclareLaunchArgument('serial_port', default_value='/dev/mcu'),
        # Must equal ROS_SERIAL_BAUD in the firmware platformio.ini. A mismatch
        # does not produce an error: the agent simply never sees the client.
        DeclareLaunchArgument('agent_baud', default_value='921600'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/lidar'),

        Node(package='micro_ros_agent', executable='micro_ros_agent',
             name='micro_ros_agent', output='screen',
             arguments=['serial', '--dev', serial_port,
                        '-b', LaunchConfiguration('agent_baud')]),

        Node(package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node',
             name='ldlidar', output='screen',
             parameters=[{'product_name': 'LDLiDAR_LD06',
                          'topic_name': 'scan',
                          'frame_id': 'base_laser',
                          'port_name': lidar_port,
                          'port_baudrate': 230400}]),

        Node(package='learn2race_hardware', executable='vl6180_tof_node',
             name='vl6180_tof', parameters=[params]),
        Node(package='learn2race_hardware', executable='tof050c_node',
             name='tof050c', parameters=[params]),
        Node(package='learn2race_hardware', executable='lsm6ds3_imu_node',
             name='lsm6ds3_imu', parameters=[params]),
        Node(package='learn2race_hardware', executable='i2c_display_node',
                name='i2c_display', parameters=[{
                     'i2c_bus': 1,
                     'i2c_address': 0x3c,
                     'width': 128,
                     'height': 32,
                }]),
        Node(package='learn2race_hardware', executable='battery_monitor',
             name='battery_monitor', parameters=[params]),
        Node(package='learn2race_hardware', executable='rgb_stick_node',
                name='rgb_stick', parameters=[{
                     'gpio_pin': 10,
                     'led_count': 10,
                }]),

        Node(package='learn2race_odometry', executable='odom_tf_broadcaster_node',
             name='odometry', parameters=[params]),
        Node(package='learn2race_safety', executable='safety_monitor',
             name='safety_monitor', parameters=[params]),
        Node(package='learn2race_safety', executable='cmd_vel_mux',
             name='cmd_vel_mux', parameters=[params]),

        Node(package='joy', executable='joy_node', name='joy',
             condition=IfCondition(teleop)),
        Node(package='teleop_twist_joy', executable='teleop_node',
             name='teleop', condition=IfCondition(teleop),
             remappings=[('cmd_vel', 'teleop/cmd_vel')]),

        Node(package='learn2race_competition', executable='competition_fsm',
             name='competition_fsm', condition=IfCondition(competition),
             parameters=[params]),
        Node(package='learn2race_competition', executable='led_signalling',
             name='led_signalling', condition=IfCondition(competition),
             parameters=[params]),
    ])
