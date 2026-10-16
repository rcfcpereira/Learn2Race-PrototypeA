"""Full bringup for Prototype A (competition + education).

    ros2 launch learn2race_bringup prototype_a.launch.py              # dragster
    ros2 launch learn2race_bringup prototype_a.launch.py mode:=trackbotgp
    ros2 launch learn2race_bringup prototype_a.launch.py competition:=false
"""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R14] micro-ROS / micro_ros_arduino: micro-ROS agent
#   [R46] Madgwick (2010) orientation filter: imu_filter_madgwick
#   [R79] LDROBOT STL-06P lidar, ldlidar_stl_ros2: lidar driver
#   [R1]  TrackBotGP rules 2026: competition modes
# More details: README.md of this package

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('learn2race_bringup'),
        'config', 'prototype_a.yaml')

    mode = LaunchConfiguration('mode')
    competition = LaunchConfiguration('competition')
    serial_port = LaunchConfiguration('serial_port')
    lidar_port = LaunchConfiguration('lidar_port')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='dragster', choices=['dragster', 'trackbotgp']),
        DeclareLaunchArgument('competition', default_value='true'),
        DeclareLaunchArgument('line_array', default_value='false'),
        # Manual driving with a gamepad. The competition rules forbid
        # teleoperation during a run, so the multiplexer blocks it by default.
        #
        #   teleop:=true        run the gamepad ON THE ROBOT and lift the block
        #   allow_teleop:=true  lift the block only, for a gamepad on another
        #                       machine running teleop_pc.launch.py
        #
        # Never use either in competition.
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('allow_teleop', default_value='false'),
        DeclareLaunchArgument('rgb_stick', default_value='false'),
        DeclareLaunchArgument('track_light_level', default_value='255'),
        # TrackBotGP layout: config/track_<track>.yaml sets the race settings
        # that depend on it (pit side, lap length).
        DeclareLaunchArgument('track', default_value='portimao'),
        DeclareLaunchArgument('display', default_value='auto', choices=['auto', 'true', 'false']),
        DeclareLaunchArgument('sensor_host', default_value='esp32', choices=['esp32', 'pi'],
                             description='pi requires esp32dev_pi_sensors firmware and physical I2C wiring'),
        DeclareLaunchArgument('require_scan', default_value='false'),
        DeclareLaunchArgument('lidar', default_value='auto', choices=['auto', 'true', 'false']),
        DeclareLaunchArgument('autonomy_enabled', default_value='false'),
        # Stable names created by scripts/setup_udev.sh. ttyUSB0/ttyUSB1 swap
        # depending on enumeration order when both the board and the LiDAR are
        # connected, so the agent can end up opening the LiDAR. Override with
        # serial_port:=/dev/ttyUSB0 if the udev rule is not installed.
        DeclareLaunchArgument('serial_port', default_value='/dev/mcu'),
        # Must equal ROS_SERIAL_BAUD in the firmware platformio.ini. A mismatch
        # does not produce an error: the agent simply never sees the client.
        DeclareLaunchArgument('agent_baud', default_value='921600'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/lidar'),

        # --- embedded layer -------------------------------------------------
        Node(package='micro_ros_agent', executable='micro_ros_agent',
             name='micro_ros_agent', output='screen',
             arguments=['serial', '--dev', serial_port,
                        '-b', LaunchConfiguration('agent_baud')]),

        # --- sensing --------------------------------------------------------
        # The races are camera-only, so the LiDAR is off by default in
        # competition (dragster and trackbotgp) to free CPU for the follower.
        # lidar:=true or lidar:=false overrides; auto = on outside competition.
        Node(package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node',
             name='ldlidar', output='screen',
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('lidar'), "' == 'true' or ('",
                 LaunchConfiguration('lidar'), "' == 'auto' and '", competition,
                 "' != 'true')"])),
             parameters=[{'product_name': 'LDLiDAR_LD06',
                          'topic_name': 'scan',
                          'frame_id': 'base_laser',
                          'port_name': lidar_port,
                          'port_baudrate': 230400}]),
        # The reflective line array is a planned extension and is not fitted
        # yet; without the ADS7830 on the bus the node fails with a remote I/O
        # error. Enable with line_array:=true once the hardware is installed.
        Node(package='learn2race_hardware', executable='line_array_node',
             name='line_array', parameters=[params],
             condition=IfCondition(LaunchConfiguration('line_array'))),
        # Exactly one host must own these four sensors. The default keeps the
        # board wiring; the pi profile requires the matching firmware first.
        Node(package='learn2race_hardware', executable='prototype_a_sensors',
             name='prototype_a_sensors', parameters=[params],
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('sensor_host'), "' == 'pi'"]))),
        # The OLED costs ~40% of a core on the Pi 4; the races need that CPU
        # for the follower, so it is off in competition unless display:=true.
        Node(package='learn2race_hardware', executable='i2c_display_node',
             name='i2c_display', parameters=[params],
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('display'), "' == 'true' or ('",
                 LaunchConfiguration('display'), "' == 'auto' and '", competition,
                 "' != 'true')"]))),

        # --- orientation ----------------------------------------------------
        # The firmware publishes raw rate and acceleration on imu/data_raw and
        # the field on imu/mag. The Madgwick filter fuses them into imu/data
        # with the orientation set, which is what robot_localization, RViz and
        # the safety monitor consume.
        #
        # use_mag is FALSE by default. The compass sits centimetres from two
        # brushed motors whose field changes with load, which cannot be
        # calibrated out; fusing it would inject a load-dependent heading error
        # during every run. Enable it only for stationary heading tests.
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='imu_filter', output='screen',
             parameters=[{'use_mag': False,
                          'publish_tf': False,
                          'world_frame': 'enu',
                          'gain': 0.1,
                          'fixed_frame': 'odom'}]),

        # --- estimation -----------------------------------------------------
        Node(package='learn2race_odometry', executable='odom_tf_broadcaster_node',
             name='odometry', parameters=[params]),

        # --- preview --------------------------------------------------------
        # No standalone preview here. The camera line follower serves the
        # preview itself at http://<robot>:8080/ when started with
        # camera_line_follower.launch.py, reusing the frames it already has.
        # This launch file starts no camera, so a preview here would show
        # nothing.

        # --- safety ---------------------------------------------------------
        Node(package='learn2race_safety', executable='safety_monitor',
             name='safety_monitor', parameters=[params, {'require_scan': ParameterValue(
                 LaunchConfiguration('require_scan'), value_type=bool)}]),
        Node(package='learn2race_safety', executable='cmd_vel_mux',
             name='cmd_vel_mux',
             parameters=[params, {'competition_mode': ParameterValue(
                 # Competition lockout only in a race: with competition:=false
                 # the mux must honour autonomy_enabled, which it ignores in
                 # competition mode (autonomy then waits for an FSM arm that
                 # never comes, and the robot does not move).
                 PythonExpression(["'", competition, "' == 'true' and '",
                                   LaunchConfiguration('teleop'), "' != 'true' and '",
                                   LaunchConfiguration('allow_teleop'), "' != 'true'"]),
                 value_type=bool), 'autonomy_enabled': ParameterValue(
                     LaunchConfiguration('autonomy_enabled'), value_type=bool)}]),

        # Gamepad. Requires ros-jazzy-joy and ros-jazzy-teleop-twist-joy; if
        # either is missing, launch aborts before anything starts, so install
        # them before using teleop:=true.
        #
        # Publishes to teleop/cmd_vel, never straight to cmd_vel: the
        # multiplexer is the single publisher of cmd_vel, and a second publisher
        # there would alternate with the multiplexer's zero stream, making the
        # robot lurch between the command and a stop.
        Node(package='joy', executable='joy_node', name='joy',
             condition=IfCondition(LaunchConfiguration('teleop'))),
        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop',
             condition=IfCondition(LaunchConfiguration('teleop')),
             parameters=[{'require_enable_button': True,
                          'enable_button': 4,
                          'axis_linear.x': 1,
                          'axis_angular.yaw': 3,
                          'scale_linear.x': 0.25,
                          'scale_angular.yaw': 1.5}],
             remappings=[('cmd_vel', 'teleop/cmd_vel')]),

        # --- competition ----------------------------------------------------
        Node(package='learn2race_competition', executable='competition_fsm',
             name='competition_fsm', condition=IfCondition(competition),
             parameters=[params, {'mode': mode}]),
        # Dragster start and finish: the LTR303 start light and the camera's
        # finish band. Needs sensor_host:=pi and the camera line follower.
        Node(package='learn2race_competition', executable='dragster_referee',
             name='dragster_referee', parameters=[params],
             condition=IfCondition(PythonExpression([
                 "'", competition, "' == 'true' and '", mode, "' == 'dragster'"]))),
        # TrackBotGP start, laps and pit: the LTR303 start light, the camera's
        # finish band and fork detection, and odometry. Needs sensor_host:=pi
        # and the camera line follower. (lap_counter reads the line array,
        # which A does not have, so it is not started.)
        Node(package='learn2race_competition', executable='gp_referee',
             name='gp_referee',
             parameters=[params, PathJoinSubstitution([
                 os.path.dirname(params), ['track_', LaunchConfiguration('track'), '.yaml']])],
             condition=IfCondition(PythonExpression([
                 "'", competition, "' == 'true' and '", mode, "' == 'trackbotgp'"]))),
        Node(package='learn2race_competition', executable='led_signalling',
             name='led_signalling', condition=IfCondition(competition),
             parameters=[params]),
        # A's chainable P9813 LED is driven by the ESP32 on GPIO5/4 and receives
        # leds/set_color directly. This optional node is only for an ADDITIONAL
        # WS2813 stick physically attached to the Pi; leave it off for P9813.
        # Grove WS2813 stick (moved from Prototype B, 2026-10-01) as a floodlight
        # for the track in front of the camera: DIN on BCM GPIO10 (SPI MOSI,
        # header pin 19), steady white. rgb_stick:=true to enable.
        Node(package='learn2race_hardware', executable='rgb_stick_node',
             name='rgb_stick', condition=IfCondition(LaunchConfiguration('rgb_stick')),
             parameters=[{'gpio_pin': 10, 'led_count': 20, 'backend': 'spi',
                          'mode': 'light', 'brightness': 255,
                          'light_level': ParameterValue(LaunchConfiguration('track_light_level'),
                                                        value_type=int)}]),
    ])
