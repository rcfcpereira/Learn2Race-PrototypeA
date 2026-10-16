# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch arguments and substitutions
#   [R17] camera_ros (libcamera node): camera_ros node
#   [R18] Raspberry Pi camera software: IMX219 sensor modes and frame rate
# More details: README.md of this package

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# The ROS apt libcamera (/opt/ros/jazzy) runs the Raspberry Pi IPA in an
# isolated process (its module is unsigned), and on the 6.8.0-1065-raspi
# kernel that process aborts on start ("A list of V4L2 controls requires a
# ControlInfoMap"), so no frames arrive. The Raspberry Pi libcamera in
# /usr/local (the one rpicam-* use) runs the IPA in-process and works; load it
# first when it is installed, as Prototype B's stereo launch does.
PI_LIBCAMERA = "/usr/local/lib/aarch64-linux-gnu"
CAMERA_ENV = (
    {"LD_LIBRARY_PATH": PI_LIBCAMERA + ":" + os.environ.get("LD_LIBRARY_PATH", "")}
    if os.path.isfile(os.path.join(PI_LIBCAMERA, "libcamera.so.0.7")) else {}
)


def generate_launch_description() -> LaunchDescription:
    common_parameters = {
        "image_topic": LaunchConfiguration("image_topic"),
        "roi_top_ratio": ParameterValue(LaunchConfiguration("roi_top_ratio"), value_type=float),
        "roi_bottom_ratio": ParameterValue(LaunchConfiguration("roi_bottom_ratio"), value_type=float),
        "black_threshold": ParameterValue(LaunchConfiguration("black_threshold"), value_type=int),
        "min_contour_area_ratio": ParameterValue(LaunchConfiguration("min_contour_area_ratio"), value_type=float),
        "target_line_distance_mm": ParameterValue(LaunchConfiguration("target_line_distance_mm"), value_type=float),
        "image_start_distance_mm": ParameterValue(LaunchConfiguration("image_start_distance_mm"), value_type=float),
        "image_depth_mm": ParameterValue(LaunchConfiguration("image_depth_mm"), value_type=float),
        "target_row_band_px": ParameterValue(LaunchConfiguration("target_row_band_px"), value_type=int),
        "curve_lookahead_px": ParameterValue(LaunchConfiguration("curve_lookahead_px"), value_type=int),
        "curve_gain": ParameterValue(LaunchConfiguration("curve_gain"), value_type=float),
        "opencv_threads": ParameterValue(LaunchConfiguration("opencv_threads"), value_type=int),
        "process_every_n_frames": ParameterValue(LaunchConfiguration("process_every_n_frames"), value_type=int),
        "sensor_qos_depth": ParameterValue(LaunchConfiguration("sensor_qos_depth"), value_type=int),
    }

    follower_parameters = dict(common_parameters)
    follower_parameters.update(
        {
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "max_linear_speed": ParameterValue(LaunchConfiguration("max_linear_speed"), value_type=float),
            "min_linear_speed": ParameterValue(LaunchConfiguration("min_linear_speed"), value_type=float),
            "max_angular_speed": ParameterValue(LaunchConfiguration("max_angular_speed"), value_type=float),
            "pid_kp": ParameterValue(LaunchConfiguration("pid_kp"), value_type=float),
            "pid_ki": ParameterValue(LaunchConfiguration("pid_ki"), value_type=float),
            "pid_kd": ParameterValue(LaunchConfiguration("pid_kd"), value_type=float),
            "pid_integral_limit": ParameterValue(LaunchConfiguration("pid_integral_limit"), value_type=float),
            "lost_turn_speed": ParameterValue(LaunchConfiguration("lost_turn_speed"), value_type=float),
            "slowdown_gain": ParameterValue(LaunchConfiguration("slowdown_gain"), value_type=float),
            "search_on_loss": ParameterValue(LaunchConfiguration("search_on_loss"), value_type=bool),
            "image_timeout_s": ParameterValue(LaunchConfiguration("image_timeout_s"), value_type=float),
            "command_rate_hz": ParameterValue(LaunchConfiguration("command_rate_hz"), value_type=float),
            "bend_speed": ParameterValue(LaunchConfiguration("bend_speed"), value_type=float),
            "snapshot_dir": LaunchConfiguration("snapshot_dir"),
            "record_every_n": ParameterValue(LaunchConfiguration("record_every_n"), value_type=int),
            "split_prefer_main": ParameterValue(LaunchConfiguration("split_prefer_main"), value_type=bool),
            "crossing_hold": ParameterValue(LaunchConfiguration("crossing_hold"), value_type=bool),
            "crossing_hold_m": ParameterValue(LaunchConfiguration("crossing_hold_m"), value_type=float),
            "crossing_hold_max_m": ParameterValue(LaunchConfiguration("crossing_hold_max_m"), value_type=float),
            "crossing_hold_imu": ParameterValue(LaunchConfiguration("crossing_hold_imu"), value_type=bool),
            "crossing_map": ParameterValue(LaunchConfiguration("crossing_map"), value_type=str),
            "crossing_map_window_m": ParameterValue(LaunchConfiguration("crossing_map_window_m"), value_type=float),
            "crossing_map_tol_deg": ParameterValue(LaunchConfiguration("crossing_map_tol_deg"), value_type=float),
            "crossing_map_pit_exit_m": ParameterValue(LaunchConfiguration("crossing_map_pit_exit_m"), value_type=float),
            "race_mode": LaunchConfiguration("race_mode"),
            "dragster_t_end": ParameterValue(LaunchConfiguration("dragster_t_end"), value_type=bool),
            "hairpin_latch_frames": ParameterValue(LaunchConfiguration("hairpin_latch_frames"), value_type=int),
            "merged_width_ratio": ParameterValue(LaunchConfiguration("merged_width_ratio"), value_type=float),
            "bumpless_steering": ParameterValue(LaunchConfiguration("bumpless_steering"), value_type=bool),
            "hairpin_after_crossing_frames": ParameterValue(LaunchConfiguration("hairpin_after_crossing_frames"), value_type=int),
            "bend_after_crossing_frames": ParameterValue(LaunchConfiguration("bend_after_crossing_frames"), value_type=int),
        }
    )

    preview_parameters = dict(common_parameters)
    preview_parameters.update({"preview_topic": LaunchConfiguration("preview_topic")})

    return LaunchDescription(
        [
            # Empty keeps the shell's RMW so the follower talks to the robot's
            # bringup. Set rmw:=rmw_cyclonedds_cpp only if everything else uses it.
            DeclareLaunchArgument("rmw", default_value=""),
            SetEnvironmentVariable(
                "RMW_IMPLEMENTATION", LaunchConfiguration("rmw"),
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration("rmw"), "' != ''"]))),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("camera", default_value="0"),
            DeclareLaunchArgument("camera_role", default_value="video"),
            DeclareLaunchArgument("camera_width", default_value="320"),
            DeclareLaunchArgument("camera_height", default_value="240"),
            DeclareLaunchArgument("camera_frame_id", default_value="camera_link"),
            DeclareLaunchArgument("camera_format", default_value="RGB888"),
            DeclareLaunchArgument("camera_sensor_mode", default_value="1640:1232"),
            # Frame period in microseconds; 23900 us is 41.8 fps, the IMX219's
            # limit in the full-field 1640:1232 mode (rpicam-hello: 41.85 fps).
            # Faster modes crop the view: 640x480 reaches 103 fps but sees only
            # ~40% of the width. Keep command_rate_hz equal to the frame rate.
            DeclareLaunchArgument("camera_frame_period_us", default_value="23900"),
            DeclareLaunchArgument("image_topic", default_value="image_raw"),
            # Through the safety multiplexer, never straight to cmd_vel.
            DeclareLaunchArgument("cmd_vel_topic", default_value="autonomy/cmd_vel"),
            DeclareLaunchArgument("preview_topic", default_value="line_preview/image_raw"),
            DeclareLaunchArgument("start_line_preview", default_value="false"),
            DeclareLaunchArgument("start_micro_ros_agent", default_value="false"),
            DeclareLaunchArgument("mcu_device", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("mcu_baud", default_value="921600"),
            DeclareLaunchArgument("roi_top_ratio", default_value="0.30"),
            DeclareLaunchArgument("roi_bottom_ratio", default_value="0.95"),
            DeclareLaunchArgument("black_threshold", default_value="70"),
            DeclareLaunchArgument("min_contour_area_ratio", default_value="0.003"),
            # 125 mm restores the lookahead the original code used by accident. The
            # old 50 mm default only took effect once the row-mapping bug was
            # fixed, and then it moved the steering point to the row nearest
            # the robot, which made it run off curves. 85 (the bottom row)
            # won a simulator sweep but reacted too late on the real track.
            DeclareLaunchArgument("target_line_distance_mm", default_value="125.0"),
            DeclareLaunchArgument("image_start_distance_mm", default_value="70.0"),
            DeclareLaunchArgument("image_depth_mm", default_value="220.0"),
            DeclareLaunchArgument("target_row_band_px", default_value="6"),
            DeclareLaunchArgument("curve_lookahead_px", default_value="45"),
            DeclareLaunchArgument("curve_gain", default_value="0.55"),
            DeclareLaunchArgument("max_linear_speed", default_value="0.15"),
            DeclareLaunchArgument("min_linear_speed", default_value="0.05"),
            DeclareLaunchArgument("max_angular_speed", default_value="1.80"),
            DeclareLaunchArgument("pid_kp", default_value="1.80"),
            DeclareLaunchArgument("pid_ki", default_value="0.00"),
            DeclareLaunchArgument("pid_kd", default_value="0.25"),
            DeclareLaunchArgument("pid_integral_limit", default_value="0.60"),
            DeclareLaunchArgument("lost_turn_speed", default_value="1.20"),
            DeclareLaunchArgument("slowdown_gain", default_value="1.5"),
            DeclareLaunchArgument("search_on_loss", default_value="true"),
            DeclareLaunchArgument("image_timeout_s", default_value="0.40"),
            DeclareLaunchArgument("command_rate_hz", default_value="42.0"),
            # Speed cap while a sharp bend is in view ahead (0 = off).
            DeclareLaunchArgument("bend_speed", default_value="0.08"),
            # Non-empty: save a frame for each band/split/checker sighting there.
            DeclareLaunchArgument("snapshot_dir", default_value=""),
            # > 0: record every n-th frame and a per-frame log (needs snapshot_dir).
            DeclareLaunchArgument("record_every_n", default_value="0"),
            # true: at a split take the referee's main line, not straight on (Suzuka).
            DeclareLaunchArgument("split_prefer_main", default_value="false"),
            # Hold the heading across a crossing instead of steering by the image.
            DeclareLaunchArgument("crossing_hold", default_value="false"),
            DeclareLaunchArgument("crossing_hold_m", default_value="0.08"),
            DeclareLaunchArgument("crossing_hold_max_m", default_value="0.25"),
            DeclareLaunchArgument("crossing_hold_imu", default_value="true"),
            DeclareLaunchArgument("crossing_map", default_value=""),
            DeclareLaunchArgument("crossing_map_window_m", default_value="0.5"),
            DeclareLaunchArgument("crossing_map_tol_deg", default_value="35.0"),
            DeclareLaunchArgument("crossing_map_pit_exit_m", default_value="0.0"),
            # trackbotgp | dragster: what the web page shows.
            DeclareLaunchArgument("race_mode", default_value="trackbotgp"),
            DeclareLaunchArgument("dragster_t_end", default_value="true"),
            # Suzuka behaviours (off by default; config/track_suzuka.yaml turns them on).
            DeclareLaunchArgument("hairpin_latch_frames", default_value="0"),
            DeclareLaunchArgument("merged_width_ratio", default_value="0.0"),
            DeclareLaunchArgument("bumpless_steering", default_value="false"),
            DeclareLaunchArgument("hairpin_after_crossing_frames", default_value="0"),
            DeclareLaunchArgument("bend_after_crossing_frames", default_value="0"),
            DeclareLaunchArgument("opencv_threads", default_value="1"),
            DeclareLaunchArgument("process_every_n_frames", default_value="1"),
            DeclareLaunchArgument("sensor_qos_depth", default_value="1"),
            Node(
                package="camera_ros",
                executable="camera_node",
                name="camera",
                additional_env=CAMERA_ENV,
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_camera")),
                parameters=[
                    {
                        "camera": ParameterValue(LaunchConfiguration("camera"), value_type=int),
                        "role": LaunchConfiguration("camera_role"),
                        "width": ParameterValue(LaunchConfiguration("camera_width"), value_type=int),
                        "height": ParameterValue(LaunchConfiguration("camera_height"), value_type=int),
                        "frame_id": LaunchConfiguration("camera_frame_id"),
                        "format": LaunchConfiguration("camera_format"),
                        "sensor_mode": LaunchConfiguration("camera_sensor_mode"),
                        # Concatenated into "[50000, 50000]"; with no value_type,
                        # launch_ros parses the string as YAML into an int list.
                        "FrameDurationLimits": ParameterValue([
                            "[", LaunchConfiguration("camera_frame_period_us"), ", ",
                            LaunchConfiguration("camera_frame_period_us"), "]",
                        ]),
                    }
                ],
                remappings=[("~/image_raw", LaunchConfiguration("image_topic"))],
            ),
            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                name="micro_ros_agent",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_micro_ros_agent")),
                arguments=[
                    "serial",
                    "--dev",
                    LaunchConfiguration("mcu_device"),
                    "-b",
                    LaunchConfiguration("mcu_baud"),
                ],
            ),
            Node(
                package="learn2race_camera_follower",
                executable="line_follower_node",
                name="line_follower",
                output="screen",
                parameters=[follower_parameters],
            ),
            Node(
                package="learn2race_camera_follower",
                executable="line_detection_preview_node",
                name="line_detection_preview",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_line_preview")),
                parameters=[preview_parameters],
            ),
        ]
    )
