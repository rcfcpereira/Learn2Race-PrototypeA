"""Launch stereo cameras and the ROS camera calibration GUI.

Algorithm:
1. Start both Raspberry Pi Camera v2 streams through the existing stereo launch.
2. Disable stereo processing during calibration so raw images reach the GUI.
3. Wait a few seconds for image topics to appear.
4. Start `camera_calibration/cameracalibrator` with left/right image remaps.
5. The user moves a checkerboard until the GUI can solve intrinsics/extrinsics.
"""

# References (full entries: docs/references.md at the repository root):
#   [R34] Zhang (2000) camera calibration: checkerboard calibration
#   [R17] camera_ros (libcamera node): camera_ros
#   [R15] ROS 2 launch: launch
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    stereo_launch = PathJoinSubstitution(
        [FindPackageShare("learn2race_bringup"), "launch", "stereo_cameras.launch.py"]
    )

    calibrator = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "camera_calibration",
            "cameracalibrator",
            "--size",
            LaunchConfiguration("checkerboard_size"),
            "--square",
            LaunchConfiguration("checkerboard_square_m"),
            "--approximate",
            LaunchConfiguration("approximate_sync"),
            "--ros-args",
            "--remap",
            "left:=/stereo/left/image_raw",
            "--remap",
            "right:=/stereo/right/image_raw",
            "--remap",
            "left_camera:=/stereo/left",
            "--remap",
            "right_camera:=/stereo/right",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_calibrator")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_cameras", default_value="true"),
            DeclareLaunchArgument("start_calibrator", default_value="true"),
            DeclareLaunchArgument("left_camera", default_value="0"),
            DeclareLaunchArgument("right_camera", default_value="1"),
            DeclareLaunchArgument("stereo_width", default_value="640"),
            DeclareLaunchArgument("stereo_height", default_value="480"),
            DeclareLaunchArgument("stereo_sensor_mode", default_value="3280:2464"),
            DeclareLaunchArgument("checkerboard_size", default_value="8x6"),
            DeclareLaunchArgument("checkerboard_square_m", default_value="0.025"),
            DeclareLaunchArgument("approximate_sync", default_value="0.05"),
            DeclareLaunchArgument("calibrator_delay_s", default_value="4.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(stereo_launch),
                condition=IfCondition(LaunchConfiguration("start_cameras")),
                launch_arguments={
                    "start_stereo_cameras": "true",
                    "start_stereo_processing": "false",
                    "left_camera": LaunchConfiguration("left_camera"),
                    "right_camera": LaunchConfiguration("right_camera"),
                    "stereo_width": LaunchConfiguration("stereo_width"),
                    "stereo_height": LaunchConfiguration("stereo_height"),
                    "stereo_sensor_mode": LaunchConfiguration("stereo_sensor_mode"),
                }.items(),
            ),
            TimerAction(
                period=LaunchConfiguration("calibrator_delay_s"),
                actions=[calibrator],
            ),
        ]
    )
