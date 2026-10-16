#!/usr/bin/env python3
"""ROS 2 node that follows a dark line from a camera image."""

# References (full entries: docs/references.md at the repository root):
#   [R10] ROS 2 Jazzy documentation: rclpy node, QoS, parameters
#   [R17] camera_ros (libcamera node): camera_ros image input
#   [R20] cv_bridge: cv_bridge image conversion
#   [R40] Astrom & Murray, Feedback Systems (PID): PD steering and speed scheduling
#   [R1]  TrackBotGP rules 2026: race behaviour: never stop searching, lap limits
# More details: README.md of this package

from __future__ import annotations

import collections
import json
import math
import os
import threading
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, ColorRGBA, Float32, String
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image, Imu

from .control import PIDConfig, PIDController
from .detector import DetectorConfig, LineDetector, clamp, draw_preview
from .preview_server import FrameBuffer, start_server


class LineFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("line_follower_node")

        self.declare_parameter("image_topic", "image_raw")
        # Publish to the autonomy input of the command multiplexer, not straight
        # to cmd_vel. Publishing directly would bypass the arming lock and the
        # teleoperation lockout, so the follower could drive the robot before
        # the start signal, which the competition rules forbid.
        self.declare_parameter("cmd_vel_topic", "autonomy/cmd_vel")
        self.declare_parameter("roi_top_ratio", 0.30)
        self.declare_parameter("roi_bottom_ratio", 0.95)
        self.declare_parameter("black_threshold", 70)
        self.declare_parameter("min_contour_area_ratio", 0.003)
        self.declare_parameter("target_line_distance_mm", 125.0)
        self.declare_parameter("image_start_distance_mm", 70.0)
        self.declare_parameter("image_depth_mm", 220.0)
        self.declare_parameter("target_row_band_px", 6)
        self.declare_parameter("curve_lookahead_px", 45)
        self.declare_parameter("curve_gain", 0.55)
        self.declare_parameter("max_linear_speed", 0.15)
        self.declare_parameter("min_linear_speed", 0.05)
        self.declare_parameter("max_angular_speed", 1.80)
        self.declare_parameter("pid_kp", 1.80)
        self.declare_parameter("pid_ki", 0.00)
        self.declare_parameter("pid_kd", 0.25)
        self.declare_parameter("pid_integral_limit", 0.60)
        # Turn rate when the line is lost, towards the side it was last seen.
        # In a hairpin the line leaves the image sideways; 0.4 rad/s was far too
        # slow to catch it again before the robot left the track.
        self.declare_parameter("lost_turn_speed", 1.20)
        # Slow down on curvature, not only on offset: the speed factor is
        # 1 - slowdown_gain * |error|, so the robot brakes as a curve appears
        # at the top of the view instead of once it is already off-centre.
        self.declare_parameter("slowdown_gain", 1.5)
        self.declare_parameter("search_on_loss", True)
        # A search with no bound spins the robot indefinitely when the line is
        # gone for good. After this long, stop instead.
        self.declare_parameter("search_timeout_s", 3.0)
        # true: search without end (a race), see publish_control; false: spin
        # for search_timeout_s and then stop (practice on a table).
        self.declare_parameter("search_recover", True)
        self.declare_parameter("search_turns", 1.0)        # full turns per spin
        self.declare_parameter("search_back_s", 0.75)      # back up this long...
        self.declare_parameter("search_back_speed", 0.08)  # ...at this speed (m/s)
        # Speed cap (m/s) while a sharp bend is in view ahead (0 = off): the
        # robot must reach a 5-6 cm hairpin slowly enough to turn in it.
        # 0.08: the wheels lag ~0.3 s behind the command (the firmware ramps
        # each wheel and is slow to reverse the inner one), measured on
        # 2026-10-02 21:50: commanded 0.05 m/s and 1.44 rad/s in a hairpin,
        # the robot did 0.13 m/s and 0.75 rad/s. Arrive slowly instead.
        self.declare_parameter("bend_speed", 0.08)
        # ...held this long after the bend was last seen: its top often lies
        # above the window for a few frames on the way in.
        self.declare_parameter("bend_hold_s", 0.6)
        # Slower the nearer the bend: bend_speed at bend_slow_px rows ahead,
        # min_linear_speed at the bend; within bend_turn_px rows, turn at
        # least bend_turn_error towards it.
        self.declare_parameter("bend_slow_px", 100.0)
        self.declare_parameter("bend_turn_px", 20.0)
        self.declare_parameter("bend_turn_error", 0.8)
        # Forward speed at the bend itself (within bend_turn_px): nearly a
        # pivot, so the inner wheel is driven backwards.
        self.declare_parameter("bend_pivot_speed", 0.02)
        self.declare_parameter("band_topic", "line/band_detected")
        # A start or finish band can hide the line: drive straight across it
        # for up to this long instead of stopping in front of it.
        self.declare_parameter("band_coast_s", 1.5)
        # At a fork, take the branch named on this topic ('left' or 'right');
        # anything else keeps the branch nearest the line followed so far.
        # The TrackBotGP referee sets it to steer into and past the pit lane.
        self.declare_parameter("branch_preference_topic", "line/branch_preference")
        self.declare_parameter("fork_topic", "line/fork_detected")
        self.declare_parameter("checker_topic", "line/checker_detected")
        # Non-empty: save the camera frame (JPEG) each time a band, split or
        # chequered marker comes into view, at most snapshot_max per run, to
        # see afterwards what the referee was shown.
        self.declare_parameter("snapshot_dir", "")
        self.declare_parameter("snapshot_max", 400)
        # > 0 (with snapshot_dir): record every n-th camera frame to
        # snapshot_dir/rec and log each frame's detection, the command sent
        # and the wheel odometry to snapshot_dir/rec.csv, for offline replay.
        self.declare_parameter("record_every_n", 0)
        self.declare_parameter("band_min_side_ratio", 0.15)
        self.declare_parameter("band_align_ratio", 0.15)
        self.declare_parameter("split_prefer_main", False)
        # trackbotgp | dragster: what the web page shows (laps or one run).
        self.declare_parameter("race_mode", "trackbotgp")
        # Dragster finish T read as a band (detector band_t_end). Needed: without
        # it the T was missed and the follower searched (spun) until another
        # line counted, ~2 s later (frames of 2026-10-09 23:47). Its false
        # finishes at 1.4-3.5 m are ignored by the referee's finish_ignore_m (8 m).
        self.declare_parameter("dragster_t_end", True)
        # Dragster soft start: seconds from standstill to full speed (0 = off).
        self.declare_parameter("start_ramp_s", 1.0)
        # Suzuka behaviours (2026-10-08), off unless the track file turns them on;
        # Monza and Portimao keep the behaviour of their good runs.
        self.declare_parameter("hairpin_latch_frames", 0)
        self.declare_parameter("merged_width_ratio", 0.0)
        self.declare_parameter("bumpless_steering", False)
        self.declare_parameter("hairpin_after_crossing_frames", 0)
        self.declare_parameter("bend_after_crossing_frames", 0)
        # Crossing heading hold: at a crossing, stop steering from the image
        # and hold the heading the robot had (odometry yaw) for a distance,
        # extended while the X is still in view, then hand back.
        self.declare_parameter("crossing_hold", False)
        self.declare_parameter("crossing_hold_m", 0.08)
        self.declare_parameter("crossing_hold_after_m", 0.04)
        self.declare_parameter("crossing_hold_max_m", 0.25)
        self.declare_parameter("crossing_hold_speed", 0.10)
        self.declare_parameter("crossing_hold_kp", 3.0)
        # Hold the heading on the IMU (imu/data, gyro-integrated) when it is fresh: wheel odometry
        # over-counts turning by 28-53 % when the wheels skid, which is what a crossing provokes.
        self.declare_parameter("crossing_hold_imu", True)
        # Crossing map from the track drawing: "s:h,s:h" = wheel-odometry metres from the finish line and
        # the track heading there (deg, + = left of the start heading). Near s, with the IMU heading within
        # crossing_map_tol_deg of h and two or more branches in view (read as hairpin, split or crossing),
        # the crossing hold starts on heading h: the X met straight out of a hairpin reads as a hairpin
        # (Suzuka, 2026-10-16). Empty: off.
        self.declare_parameter("crossing_map", "")
        self.declare_parameter("crossing_map_window_m", 0.5)
        self.declare_parameter("crossing_map_tol_deg", 35.0)
        # where lap 3 starts (leaving the pit lane), in wheel metres past the finish line
        self.declare_parameter("crossing_map_pit_exit_m", 0.0)
        self.declare_parameter("fork_min_run_px", 4)

        # Built-in preview. It reuses the frame and the detection result this
        # node already has, so it adds only drawing and JPEG encoding, and only
        # when someone is watching. A separate preview node would subscribe to
        # the raw stream a second time, and rclpy deserialises every message
        # before the callback can decide to drop it, so the second subscriber
        # pays the full copy cost on every frame even when idle.
        self.declare_parameter("preview_enabled", True)
        self.declare_parameter("preview_topic", "line_preview/compressed")
        self.declare_parameter("preview_hz", 10.0)
        self.declare_parameter("preview_jpeg_quality", 50)
        self.declare_parameter("preview_http_port", 8080)
        self.declare_parameter("stats_period_s", 10.0)
        self.declare_parameter("max_line_width_ratio", 0.50)
        self.declare_parameter("threshold_mode", "relative")   # relative | otsu | fixed
        self.declare_parameter("line_ratio", 0.5)
        self.declare_parameter("min_contrast", 40)
        self.declare_parameter("image_timeout_s", 0.40)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("opencv_threads", 1)
        self.declare_parameter("process_every_n_frames", 1)
        self.declare_parameter("sensor_qos_depth", 1)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.min_linear_speed = float(self.get_parameter("min_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.lost_turn_speed = float(self.get_parameter("lost_turn_speed").value)
        self.slowdown_gain = max(0.0, float(self.get_parameter("slowdown_gain").value))
        self.search_on_loss = bool(self.get_parameter("search_on_loss").value)
        self.search_timeout_s = max(0.0, float(self.get_parameter("search_timeout_s").value))
        self.search_recover = bool(self.get_parameter("search_recover").value)
        self.search_turns = max(0.25, float(self.get_parameter("search_turns").value))
        self.search_back_s = max(0.0, float(self.get_parameter("search_back_s").value))
        self.search_back_speed = float(self.get_parameter("search_back_speed").value)
        self.bend_speed = max(0.0, float(self.get_parameter("bend_speed").value))
        self.bend_hold_s = max(0.0, float(self.get_parameter("bend_hold_s").value))
        self.bend_seen_s = -1e9
        self.bend_cap = self.bend_speed
        self.bend_slow_px = float(self.get_parameter("bend_slow_px").value)
        self.bend_turn_px = float(self.get_parameter("bend_turn_px").value)
        self.bend_turn_error = float(self.get_parameter("bend_turn_error").value)
        self.bend_pivot_speed = max(0.0, float(self.get_parameter("bend_pivot_speed").value))
        self.band_coast_s = max(0.0, float(self.get_parameter("band_coast_s").value))
        self.image_timeout_s = max(0.05, float(self.get_parameter("image_timeout_s").value))
        self.command_rate_hz = max(1.0, float(self.get_parameter("command_rate_hz").value))
        self.opencv_threads = max(1, int(self.get_parameter("opencv_threads").value))
        self.process_every_n_frames = max(1, int(self.get_parameter("process_every_n_frames").value))
        self.sensor_qos_depth = max(1, int(self.get_parameter("sensor_qos_depth").value))

        detector_cfg = DetectorConfig(
            roi_top_ratio=float(self.get_parameter("roi_top_ratio").value),
            roi_bottom_ratio=float(self.get_parameter("roi_bottom_ratio").value),
            black_threshold=int(self.get_parameter("black_threshold").value),
            min_contour_area_ratio=float(self.get_parameter("min_contour_area_ratio").value),
            target_line_distance_mm=float(self.get_parameter("target_line_distance_mm").value),
            image_start_distance_mm=float(self.get_parameter("image_start_distance_mm").value),
            image_depth_mm=float(self.get_parameter("image_depth_mm").value),
            target_row_band_px=int(self.get_parameter("target_row_band_px").value),
            curve_lookahead_px=int(self.get_parameter("curve_lookahead_px").value),
            curve_gain=float(self.get_parameter("curve_gain").value),
            max_line_width_ratio=float(self.get_parameter("max_line_width_ratio").value),
            threshold_mode=str(self.get_parameter("threshold_mode").value),
            line_ratio=float(self.get_parameter("line_ratio").value),
            min_contrast=int(self.get_parameter("min_contrast").value),
            band_min_side_ratio=float(self.get_parameter("band_min_side_ratio").value),
            band_align_ratio=float(self.get_parameter("band_align_ratio").value),
            fork_min_run_px=int(self.get_parameter("fork_min_run_px").value),
            split_prefer_main=bool(self.get_parameter("split_prefer_main").value),
            band_t_end=(bool(self.get_parameter("dragster_t_end").value)
                        and str(self.get_parameter("race_mode").value) == "dragster"),
            hairpin_latch_frames=int(self.get_parameter("hairpin_latch_frames").value),
            merged_width_ratio=float(self.get_parameter("merged_width_ratio").value),
            hairpin_after_crossing_frames=int(self.get_parameter("hairpin_after_crossing_frames").value),
            bend_after_crossing_frames=int(self.get_parameter("bend_after_crossing_frames").value),
        )
        pid_cfg = PIDConfig(
            kp=float(self.get_parameter("pid_kp").value),
            ki=float(self.get_parameter("pid_ki").value),
            kd=float(self.get_parameter("pid_kd").value),
            integral_limit=float(self.get_parameter("pid_integral_limit").value),
        )

        self.detector = LineDetector(detector_cfg)
        self.pid = PIDController(pid_cfg)
        self.pid_forced = False             # last frame was a forced turn
        self.error_jump = 0.3               # larger steps restart the derivative
        self.bumpless = bool(self.get_parameter("bumpless_steering").value)
        self.bridge = CvBridge()

        cv2.setNumThreads(self.opencv_threads)
        if hasattr(cv2, "ocl"):
            cv2.ocl.setUseOpenCL(False)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self.sensor_qos_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, cmd_qos)
        self.band_publisher = self.create_publisher(
            Bool, str(self.get_parameter("band_topic").value), 10)
        # Whether the line is in view this frame (the GP referee only counts a
        # pit stop while the robot is on a line), and a speed cap it can set
        # (slow pit entry); <= 0 means no cap.
        self.line_found_publisher = self.create_publisher(Bool, "line/found", 10)
        self.speed_cap = 0.0
        self.create_subscription(Float32, "line/speed_limit", self._on_speed_limit, 10)
        self.fork_publisher = self.create_publisher(
            Bool, str(self.get_parameter("fork_topic").value), 10)
        self.checker_publisher = self.create_publisher(
            Bool, str(self.get_parameter("checker_topic").value), 10)
        # Side of the branch not taken at a split (the GP referee learns the
        # pit lane's side from the split before the lap 1 finish line).
        self.fork_side_publisher = self.create_publisher(String, "line/fork_side", 10)
        self.snapshot_dir = str(self.get_parameter("snapshot_dir").value)
        self.snapshot_left = int(self.get_parameter("snapshot_max").value)
        if self.snapshot_dir:
            os.makedirs(self.snapshot_dir, exist_ok=True)
        self.marks_seen = (False, False, False, False)
        self.record_every_n = int(self.get_parameter("record_every_n").value)
        self.rec_csv = None
        self.odom_vw = (0.0, 0.0)
        self.last_detect_ms = 0.0           # detector time of the last frame (recorder)
        self.imu_yaw_wz = None              # (yaw, yaw rate) from imu/data (recorder, crossing hold)
        self.imu_stamp = 0.0                # monotonic time of the last imu/data message
        self.odom_pose = None               # (x, y, yaw) from the wheel odometry
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.crossing_hold = bool(self.get_parameter("crossing_hold").value)
        self.crossing_hold_m = float(self.get_parameter("crossing_hold_m").value)
        self.crossing_hold_after_m = float(self.get_parameter("crossing_hold_after_m").value)
        self.crossing_hold_max_m = float(self.get_parameter("crossing_hold_max_m").value)
        self.crossing_hold_speed = float(self.get_parameter("crossing_hold_speed").value)
        self.crossing_hold_kp = float(self.get_parameter("crossing_hold_kp").value)
        self.crossing_hold_imu = bool(self.get_parameter("crossing_hold_imu").value)
        self.crossing_map = []
        for item in str(self.get_parameter("crossing_map").value).split(","):
            if ":" in item:
                s_m, h_deg = item.split(":")
                self.crossing_map.append((float(s_m), math.radians(float(h_deg))))
        self.crossing_map_window = float(self.get_parameter("crossing_map_window_m").value)
        self.crossing_map_tol = math.radians(float(self.get_parameter("crossing_map_tol_deg").value))
        self.crossing_map_pit_exit = float(self.get_parameter("crossing_map_pit_exit_m").value)
        self.lap_dist0 = 0.0                # race_dist at the start of the current lap
        self.start_yaw_imu = None           # IMU heading at the start of the race
        self.map_used = set()               # crossing-map entries used this lap
        self.map_ignore_log_s = 0.0
        self.hold = None                    # active crossing hold
        if self.snapshot_dir and self.record_every_n > 0:
            os.makedirs(os.path.join(self.snapshot_dir, "rec"), exist_ok=True)
            self.rec_csv = open(os.path.join(self.snapshot_dir, "rec.csv"), "w", buffering=1)
            self.rec_csv.write("t,frame,state,error,center,curve,line_x,bend_px,bend_side,"
                               "cmd_v,cmd_w,odom_v,odom_w,preference,speed_cap,detect_ms,"
                               "imu_yaw,imu_wz\n")
            # The IMU's heading and turn rate with each recorded frame: wheel odometry
            # over-counts turning when the wheels skid in tight bends (2026-10-13 race:
            # 460-504 deg per 360 deg lap), so a lap path needs the IMU's heading.
        if self.rec_csv is not None or (self.crossing_hold and self.crossing_hold_imu) or self.crossing_map:
            self.create_subscription(Imu, "imu/data", self._on_imu, 10)
        self.frame_count = 0
        self.last_cmd = (0.0, 0.0)
        self.recent_frames = collections.deque(maxlen=10)
        self.found_since = None
        self.last_snapshot_s = 0.0
        self.branch_preference = None
        self.create_subscription(
            String, str(self.get_parameter("branch_preference_topic").value),
            self._on_branch_preference, 10)

        self.preview_enabled = bool(self.get_parameter("preview_enabled").value)
        self.preview_period = 1.0 / max(0.5, float(self.get_parameter("preview_hz").value))
        self.preview_quality = int(self.get_parameter("preview_jpeg_quality").value)
        self.preview_last = 0.0
        self.preview_buffer = FrameBuffer()
        self.preview_server = None
        # Race status for the web page: the status LED colour (as the LED
        # signalling node commands it), the race state, laps counted by the
        # referee, and the finish-marker and split sightings of this camera.
        self.race_state = ""
        self.laps_counted = 0
        self.finish_marks = 0
        self.checker_marks = 0
        self.fork_marks = 0
        self.last_mark = ""
        self.led_rgb = (0.0, 0.0, 0.0)
        self.led_lit_s = 0.0           # last time the LED was on / off
        self.led_dark_s = 0.0
        self.led_last_lit = (0.0, 0.0, 0.0)
        # Race clock and speed monitor (odometry): race time from the start
        # light to the finish, lap times as the referee logs them, distance,
        # current, peak and average speed. Dragster: one run, no laps.
        self.race_mode = str(self.get_parameter("race_mode").value)
        self.start_ramp_s = float(self.get_parameter("start_ramp_s").value)
        self.race_t0 = self.race_t1 = self.lap_t0 = None   # monotonic seconds
        self.lap_times: list = []
        self.race_dist = 0.0
        self.race_peak = 0.0
        self.race_xy = None
        self.status_last_s = 0.0
        self.create_subscription(ColorRGBA, "leds/set_color", self._on_led, 10)
        self.create_subscription(Bool, "lap_event", self._on_lap_event, 10)
        self.create_subscription(String, "competition/state", self._on_race_state, 10)
        self.dragster: dict = {}
        self.drag_yaw = None                # heading held after the dragster finish line
        self.create_subscription(String, "dragster/status", self._on_dragster, 10)
        self._update_status()

        # Encoding runs on its own thread. The control callback only hands over
        # a reference to the latest frame and result, and returns; it never
        # waits for drawing or JPEG compression. OpenCV releases the GIL inside
        # cvtColor and imencode, so the worker genuinely runs alongside the
        # control path rather than stalling it. A single slot means a slow
        # encoder drops frames instead of building a queue that lags reality.
        self._preview_slot = None
        self._preview_cv = threading.Condition()
        self._preview_stop = False
        if self.preview_enabled:
            threading.Thread(target=self._preview_worker, daemon=True).start()
        if self.preview_enabled:
            self.preview_publisher = self.create_publisher(
                CompressedImage, str(self.get_parameter("preview_topic").value), 1)
            port = int(self.get_parameter("preview_http_port").value)
            try:
                self.preview_server = start_server(self.preview_buffer, port)
                self.get_logger().info(f"Preview at http://<robot-address>:{port}/")
            except OSError as exc:
                self.get_logger().warn(f"Preview HTTP server unavailable: {exc}")

        # Processing-time statistics, so the cost of the pipeline is measured
        # rather than guessed, and can be reported in the evaluation.
        self.stats_period = float(self.get_parameter("stats_period_s").value)
        self.stats_times: list[float] = []
        self.stats_frames_in = 0
        self.stats_started = time.monotonic()
        self.create_subscription(Image, self.image_topic, self.image_callback, sensor_qos)

        self.last_image_time = None
        self.last_seen_error = 0.0
        self.line_found = False
        self.band_since = None
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.frame_counter = 0
        self.state_text = "idle"
        self.search_started_s = None
        self.lost_logged = set()

        self.create_timer(1.0 / self.command_rate_hz, self.publish_control)
        self.get_logger().info(
            "Line follower ready. "
            f"image_topic=/{self.image_topic} cmd_vel_topic=/{self.cmd_vel_topic}"
        )

    LOST_STATES = ("line_lost", "line_too_small", "low_contrast", "searching",
                   "search_backing", "search_timeout", "band_only")

    def set_state(self, text: str) -> None:
        if text == self.state_text:
            return
        self.state_text = text
        # While lost, the image callback and the control timer set different
        # states every frame; logging each flip wrote 80 lines/s (55,000 lines
        # in one run). Log each lost state once per loss.
        if text in self.LOST_STATES:
            if text in self.lost_logged:
                return
            self.lost_logged.add(text)
        else:
            self.lost_logged = set()
        self.get_logger().info(f"state={text}")

    def _on_speed_limit(self, msg: Float32) -> None:
        cap = float(msg.data)
        if cap != self.speed_cap:
            self.get_logger().info(f"speed limit: {cap:.2f} m/s" if cap > 0 else "speed limit off")
        self.speed_cap = cap

    def _on_branch_preference(self, msg: String) -> None:
        preference = msg.data if msg.data in ("left", "right") else None
        if preference != self.branch_preference:
            self.get_logger().info(f"branch preference: {preference or 'nearest'}")
        self.branch_preference = preference

    # Rule colour code (TrackBotGP rules V.3), as led_signalling sends it.
    LED_NAMES = (
        ((0.0, 1.0, 0.0), "green: ready"),
        ((0.0, 0.0, 1.0), "blue: lap"),
        ((1.0, 0.78, 0.0), "yellow: pit lane"),
        ((1.0, 0.39, 0.0), "orange: pit stop"),
        ((1.0, 1.0, 1.0), "white: finished"),
        ((1.0, 0.0, 0.0), "red: fault"),
    )

    def _led_name(self) -> str:
        now = time.monotonic()
        lit = now - self.led_lit_s < 1.0
        if not lit:
            return "off"
        colour = self.led_last_lit
        name = min(self.LED_NAMES, key=lambda item: sum((a - b) ** 2 for a, b in zip(item[0], colour)))[1]
        # led_signalling flashes at 2 Hz: on and off both seen within a second.
        if now - self.led_dark_s < 1.0:
            base = name.split(":")[0]
            meaning = ("finished" if base.startswith("white") else
                       "fault (SAFE_STOP)" if base.startswith("red") else "finish line")
            name = base + " flashing: " + meaning
        return name

    def _on_led(self, msg: ColorRGBA) -> None:
        self.led_rgb = (float(msg.r), float(msg.g), float(msg.b))
        if max(self.led_rgb) > 0.05:
            self.led_lit_s = time.monotonic()
            self.led_last_lit = self.led_rgb
        else:
            self.led_dark_s = time.monotonic()
        self._update_status()

    def _on_lap_event(self, msg: Bool) -> None:
        if msg.data:
            self.laps_counted += 1
            self._update_status()

    def _on_dragster(self, msg: String) -> None:
        try:
            self.dragster = json.loads(msg.data)
        except ValueError:
            return
        self._update_status()

    def _on_race_state(self, msg: String) -> None:
        now = time.monotonic()
        previous = self.race_state
        if msg.data == "LAP1" and self.race_state in ("", "READY"):
            # A new race: count from zero.
            self.laps_counted = self.finish_marks = self.checker_marks = self.fork_marks = 0
            self.last_mark = ""
            self.race_t0 = self.lap_t0 = now
            self.lap_dist0 = 0.0
            self.map_used = set()
            self.start_yaw_imu = self.imu_yaw_wz[0] if self.imu_yaw_wz is not None else None
            self.race_t1 = None
            self.lap_times = []
            self.race_dist = self.race_peak = 0.0
            self.race_xy = None
        elif (self.lap_t0 is not None and msg.data != previous
              and previous in ("LAP1", "LAP2", "LAP3", "PIT")
              and msg.data in ("LAP2", "LAP3", "FINISH")):
            # A lap ends: at the finish line, or for lap 2 on leaving the pit,
            # where the referee logs it. (LAP2 -> PIT is the stop, not a lap.)
            self.lap_times.append(now - self.lap_t0)
            self.lap_t0 = now
            # lap distance for the crossing map: from the finish line, or from the pit exit for lap 3
            self.lap_dist0 = self.race_dist - (self.crossing_map_pit_exit if previous == "PIT" else 0.0)
            self.map_used = set()
        if msg.data in ("FINISH", "SAFE_STOP") and self.race_t0 is not None and self.race_t1 is None:
            self.race_t1 = now
        # Lap 2 ends on entering the pit lane (rules II.7): the referee sends
        # pit_entry_event there, not lap_event, so count it from the state.
        laps_done = {"LAP2": 1, "PIT": 2, "LAP3": 2, "FINISH": 3}.get(msg.data, 0)
        self.laps_counted = max(self.laps_counted, laps_done)
        self.race_state = msg.data
        self._update_status()

    def _update_status(self) -> None:
        now = time.monotonic()
        running = self.race_t0 is not None and self.race_t1 is None
        race_time = 0.0 if self.race_t0 is None else (self.race_t1 or now) - self.race_t0
        lap_time = (now - self.lap_t0) if running and self.lap_t0 is not None else None
        status = {
            "mode": self.race_mode,
            "dragster": self.dragster,
            "state": self.race_state,
            "race_time": round(race_time, 2),
            "lap_time": None if lap_time is None else round(lap_time, 2),
            "lap_times": [round(t, 2) for t in self.lap_times],
            "speed": round(self.odom_vw[0], 3),
            "speed_peak": round(self.race_peak, 3),
            "speed_avg": round(self.race_dist / race_time, 3) if race_time > 0.5 else 0.0,
            "distance": round(self.race_dist, 2),
            "laps": self.laps_counted,
            "finish_marks": self.finish_marks,
            "finish_checker": self.checker_marks,
            "fork_marks": self.fork_marks,
            "last_mark": self.last_mark,
            "led": {"rgb": list(self.led_rgb), "name": self._led_name()},
        }
        with self.preview_buffer.status_lock:
            self.preview_buffer.status = status

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.imu_yaw_wz = (yaw, float(msg.angular_velocity.z))
        self.imu_stamp = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        self.odom_vw = (float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z))
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_pose = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), yaw)
        if self.race_t0 is not None and self.race_t1 is None:
            xy = self.odom_pose[:2]
            if self.race_xy is not None:
                self.race_dist += math.hypot(xy[0] - self.race_xy[0], xy[1] - self.race_xy[1])
            self.race_xy = xy
            self.race_peak = max(self.race_peak, abs(self.odom_vw[0]))
        if time.monotonic() - self.status_last_s >= 0.25:   # the page polls at 4 Hz
            self.status_last_s = time.monotonic()
            self._update_status()

    def _dragster_straight_step(self) -> bool:
        """Dragster: the line ends in a T at the finish line, and nothing is
        drawn up to the stop line. Once the referee has the finish line, drive
        straight on the heading held there until the stop, instead of
        searching for a line that is not there (2026-10-09: the follower
        turned to search after the T). True while driving straight."""
        if (self.race_mode != "dragster" or self.race_state != "LAP1"
                or self.dragster.get("finish_m") is None or self.odom_pose is None):
            self.drag_yaw = None
            return False
        yaw = self.odom_pose[2]
        if self.drag_yaw is None:
            self.drag_yaw = yaw
            self.get_logger().info(f"dragster: finish line, straight on at {math.degrees(yaw):.0f} deg")
        # Line follower off and no heading correction: both wheels at the same
        # speed, straight on, until the referee's stop (finish_overrun_m by
        # odometry). A heading hold here was a suspect for a turn to the right
        # at the stop (2026-10-09).
        self.line_found = True
        self.search_started_s = None
        self.pid.reset()
        top = self.max_linear_speed if self.speed_cap <= 0 else min(self.max_linear_speed, self.speed_cap)
        self.target_linear = top
        self.target_angular = 0.0
        self.set_state("dragster_straight")
        return True

    def _crossing_hold_step(self, result) -> bool:
        """Drive straight across a crossing on the heading held from where it
        was first seen, ignoring the image: on top of the X its arms read as
        hairpins, splits and bends, and steering by them turned the robot
        onto the wrong arm (Suzuka, 2026-10-08). True while holding."""
        if not self.crossing_hold or self.odom_pose is None:
            return False
        x, y, odom_yaw = self.odom_pose
        now = time.monotonic()
        imu_ok = self.crossing_hold_imu and self.imu_yaw_wz is not None and now - self.imu_stamp < 0.3
        if (self.hold is None and self.crossing_map and imu_ok and self.start_yaw_imu is not None
                and self.race_t0 is not None and self.race_t1 is None
                and getattr(result, "branch_count", 0) >= 2):
            lap_d = self.race_dist - self.lap_dist0
            rel = self.imu_yaw_wz[0] - self.start_yaw_imu
            for i, (s_m, h) in enumerate(self.crossing_map):
                err = math.atan2(math.sin(rel - h), math.cos(rel - h))
                if (i not in self.map_used and abs(lap_d - s_m) <= self.crossing_map_window
                        and abs(err) <= self.crossing_map_tol):
                    self.map_used.add(i)
                    yaw0 = self.start_yaw_imu + h
                    self.hold = {"yaw": yaw0, "src": "imu", "x0": x, "y0": y, "t0": now,
                                 "end": max(self.crossing_hold_m, 0.20)}
                    self.get_logger().info(
                        f"crossing (map {i + 1}): {lap_d:.2f} m into the lap, heading {math.degrees(rel):+.0f} deg, "
                        f"holding the drawn {math.degrees(h):+.0f} deg (imu); camera read {result.state}")
                    break
        racing = self.race_t0 is not None and self.race_t1 is None
        if (result.state == "crossing" and self.hold is None and self.crossing_map and racing
                and not any(abs(self.race_dist - self.lap_dist0 - s_m) <= self.crossing_map_window + 0.1
                            for s_m, _ in self.crossing_map)):
            # A "crossing" away from every mapped crossing is something else (the pit lane rejoining
            # 0.5 m after Suzuka's finish line, 2026-10-16): keep following the line, no hold.
            if now - self.map_ignore_log_s > 1.0:
                self.map_ignore_log_s = now
                self.get_logger().info(f"crossing ignored: {self.race_dist - self.lap_dist0:.2f} m into the lap, "
                                       "no mapped crossing there")
            return False
        if result.state == "crossing":
            if self.hold is None:
                src = "imu" if imu_ok else "odom"
                yaw0 = self.imu_yaw_wz[0] if imu_ok else odom_yaw
                self.hold = {"yaw": yaw0, "src": src, "x0": x, "y0": y, "end": self.crossing_hold_m, "t0": now}
                self.get_logger().info(f"crossing: holding heading {math.degrees(yaw0):.0f} deg ({src})")
            done = math.hypot(x - self.hold["x0"], y - self.hold["y0"])
            self.hold["end"] = min(self.crossing_hold_max_m,
                                   max(self.hold["end"], done + self.crossing_hold_after_m))
        if self.hold is None:
            return False
        done = math.hypot(x - self.hold["x0"], y - self.hold["y0"])
        if done >= self.hold["end"] or now - self.hold["t0"] > 4.0:
            self.get_logger().info(f"crossing: hold released after {done:.2f} m")
            self.hold = None
            self.pid.reset()                # no derivative kick on the way back
            return False
        # the same heading source for the whole hold; the IMU's if it was used and is still fresh
        yaw = self.imu_yaw_wz[0] if self.hold["src"] == "imu" and imu_ok else odom_yaw
        if self.hold["src"] == "imu" and not imu_ok:
            yaw = self.hold["yaw"]          # IMU went stale mid-hold: drive straight on (no correction)
        error = math.atan2(math.sin(self.hold["yaw"] - yaw), math.cos(self.hold["yaw"] - yaw))
        self.line_found = True
        self.search_started_s = None
        top = self.max_linear_speed if self.speed_cap <= 0 else min(self.max_linear_speed, self.speed_cap)
        self.target_linear = min(self.crossing_hold_speed, top)
        self.target_angular = clamp(self.crossing_hold_kp * error, -1.0, 1.0)
        self.set_state("crossing_hold")
        return True

    def _record(self, gray, result) -> None:
        """Every n-th frame to rec/<frame>.jpg; every frame's detection, the
        command currently sent and the odometry to rec.csv."""
        if self.rec_csv is None:
            return
        n = self.frame_count
        if n % self.record_every_n == 0:
            cv2.imwrite(os.path.join(self.snapshot_dir, "rec", f"{n:06d}.jpg"), gray,
                        [cv2.IMWRITE_JPEG_QUALITY, 85])
        r = result
        self.rec_csv.write(
            f"{time.time():.3f},{n},{r.state},{r.error:.3f},{r.center_error:.3f},"
            f"{r.curve_error:.3f},{'' if r.line_x is None else f'{r.line_x:.1f}'},"
            f"{'' if r.bend_px is None else f'{r.bend_px:.0f}'},{r.bend_side},"
            f"{self.last_cmd[0]:.3f},{self.last_cmd[1]:.3f},{self.odom_vw[0]:.3f},"
            f"{self.odom_vw[1]:.3f},{self.branch_preference or ''},{self.speed_cap:.2f},"
            f"{self.last_detect_ms:.2f},"
            + (f"{self.imu_yaw_wz[0]:.4f},{self.imu_yaw_wz[1]:.4f}\n" if self.imu_yaw_wz else ",\n"))

    def _note_marks(self, gray, result) -> None:
        """Log (and snapshot) each band, split or checker coming into view.

        The state log hides them while the line is followed (a band shows
        only as crossing_band when it also hides the line)."""
        marks = (bool(result.band_detected), bool(result.fork_detected),
                 bool(result.checker_detected), result.state_hint == "crossing")
        rising = [name for name, now, before in zip(("band", "fork", "checker", "crossing"),
                                                     marks, self.marks_seen)
                  if now and not before]
        # The last second of frames (every 4th, ~10 Hz), saved with a loss.
        self.frame_count += 1
        if self.snapshot_dir and self.frame_count % 4 == 0:
            self.recent_frames.append((time.time(), gray))
        # The line lost after at least half a second of following it (not a
        # one-frame flicker): what did the camera see where the robot spins?
        t = time.monotonic()
        if result.found:
            if self.found_since is None:
                self.found_since = t
        else:
            if self.found_since is not None and t - self.found_since >= 0.5:
                rising.append("lost")
            self.found_since = None
        self.marks_seen = marks
        if not rising:
            return
        tag = "+".join(rising)
        self.get_logger().info(f"mark: {tag}")
        # The finish marker: its stripe (a band) or its chequered part.
        if "band" in rising or "checker" in rising:
            self.finish_marks += 1
        self.checker_marks += "checker" in rising
        self.fork_marks += "fork" in rising
        self.last_mark = f"{tag} at {time.strftime('%H:%M:%S')}"
        self._update_status()
        now = time.monotonic()
        if self.snapshot_dir and self.snapshot_left > 0 and now - self.last_snapshot_s >= 0.2:
            self.snapshot_left -= 1
            self.last_snapshot_s = now
            path = os.path.join(self.snapshot_dir, f"{time.time():.2f}_{tag}.jpg")
            try:
                cv2.imwrite(path, gray, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if "lost" in rising:
                    # What led to the loss: the frames before it.
                    for stamp, frame in self.recent_frames:
                        cv2.imwrite(os.path.join(self.snapshot_dir, f"{stamp:.2f}_pre.jpg"),
                                    frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    self.recent_frames.clear()
            except cv2.error:
                self.snapshot_left = 0

    def image_callback(self, msg: Image) -> None:
        self.last_image_time = self.get_clock().now()
        self.frame_counter += 1
        self.stats_frames_in += 1
        started = time.perf_counter()

        if self.frame_counter % self.process_every_n_frames != 0:
            return

        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except CvBridgeError as exc:
            self.line_found = False
            self.pid.reset()
            self.set_state(f"image_error:{exc}")
            return

        # Commit to the preferred branch at a split only on the pit approach
        # (the referee caps the speed from arming the pit until the stop).
        detect_started = time.perf_counter()
        result = self.detector.detect(gray, self.branch_preference,
                                      commit=self.speed_cap > 0)
        # Per-frame detector time for the recorder (rec.csv detect_ms): the
        # distribution, not only the 10 s summaries in the log.
        self.last_detect_ms = (time.perf_counter() - detect_started) * 1000.0
        self._maybe_preview(gray, result, msg)

        # A transverse band is the finish line. Report it for lap counting
        # rather than steering towards its centroid.
        self.band_publisher.publish(Bool(data=bool(result.band_detected)))
        self.fork_publisher.publish(Bool(data=bool(result.fork_detected)))
        self.checker_publisher.publish(Bool(data=bool(result.checker_detected)))
        if result.fork_detected and result.other_branch_side:
            self.fork_side_publisher.publish(String(data=result.other_branch_side))
        self._note_marks(gray, result)
        self._record(gray, result)
        self.line_found_publisher.publish(Bool(data=bool(result.found)))

        if self._dragster_straight_step():
            return
        if self._crossing_hold_step(result):
            return

        if not result.found and result.band_detected:
            now = time.monotonic()
            if self.band_since is None:
                self.band_since = now
            if now - self.band_since <= self.band_coast_s:
                # Straight across at the current speed; the dragster referee
                # decides the finish, not the follower.
                self.line_found = True
                self.target_linear = max(self.target_linear, self.min_linear_speed)
                self.target_angular = 0.0
                self.pid.reset()
                self.set_state("crossing_band")
                return

        if not result.found:
            self.line_found = False
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.pid.reset()
            self.set_state(result.state)
            return

        now_s = self.get_clock().now().nanoseconds / 1e9
        error = result.error
        # A sharp bend right ahead (Suzuka hairpins, 5-6 cm radius): the
        # steering error there was only 0.45-0.75 (2026-10-02 19:21), a
        # turning circle of 8-16 cm at the speed it gave. Close to the bend,
        # turn at least bend_turn_error towards it. (Holding that turn for
        # bend_hold_s after the bend was tried on 21:32 and nearly doubled
        # the line losses: reverted.)
        if (self.bend_speed > 0 and result.bend_px is not None
                and result.bend_px <= self.bend_turn_px and result.bend_side
                and not result.state.startswith(("hairpin", "hold_turn", "fork_commit", "crossing"))):
            sign = 1.0 if result.bend_side == "right" else -1.0
            if error * sign < self.bend_turn_error:
                error = sign * self.bend_turn_error
        # Bumpless switching: a forced turn (hairpin, hold, fork commit) and
        # normal tracking compute the error differently, and the step between
        # them was taken as a derivative. 1.0 -> 0.55 in one frame at kd 0.25
        # commanded a full LEFT turn in a right hairpin (Suzuka 2026-10-08).
        # The same for any jump of the error in one frame: the bend override
        # switching on and off (0.16 <-> 0.8) as the bend hovers at
        # bend_turn_px, or a new branch chosen at a split. At the Suzuka pit
        # split the command flipped between full left and right 14 times in
        # 1.2 s (2026-10-08 22:52); a real line moves far less per frame.
        forced = result.state.startswith(("hairpin", "hold_turn", "fork_commit"))
        if self.bumpless and (forced != self.pid_forced
                              or abs(error - self.pid.last_error) > self.error_jump):
            self.pid.last_error = error
        self.pid_forced = forced
        pid_output = self.pid.update(error, now_s)
        angular = clamp(-pid_output, -self.max_angular_speed, self.max_angular_speed)

        turn_ratio = 1.0 - min(1.0, self.slowdown_gain * abs(error))
        linear = self.min_linear_speed + ((self.max_linear_speed - self.min_linear_speed) * turn_ratio)
        top = self.max_linear_speed if self.speed_cap <= 0 else min(self.max_linear_speed, self.speed_cap)
        if (self.race_mode == "dragster" and self.start_ramp_s > 0
                and self.race_t0 is not None and self.race_t1 is None):
            # Soft start: from standstill to full speed over start_ramp_s, so
            # the follower settles on the line first (2026-10-09, 1.0 m/s: the
            # line was lost 0.94 s after the start).
            ramp = (time.monotonic() - self.race_t0) / self.start_ramp_s
            if ramp < 1.0:
                top = max(self.min_linear_speed, top * ramp)
        if self.bend_speed > 0:
            now = time.monotonic()
            if result.bend_px is not None:
                # The nearer the bend, the slower: bend_speed when it shows
                # bend_slow_px ahead, down to the minimum speed at the bend.
                near = clamp(result.bend_px / max(1.0, self.bend_slow_px), 0.0, 1.0)
                cap = self.min_linear_speed + (self.bend_speed - self.min_linear_speed) * near
                if now - self.bend_seen_s > self.bend_hold_s or cap < self.bend_cap:
                    self.bend_cap = cap
                self.bend_seen_s = now
            if now - self.bend_seen_s <= self.bend_hold_s:
                top = min(top, self.bend_cap)       # held through the bend
        linear = clamp(linear, self.min_linear_speed, max(self.min_linear_speed, top))
        if (self.bend_speed > 0 and result.bend_px is not None
                and result.bend_px <= self.bend_turn_px and result.bend_side
                and not result.state.startswith(("fork_commit", "crossing"))):
            linear = min(linear, self.bend_pivot_speed)

        # Remember which side the line is on for the search. The line's
        # position is the better guide: at a curve exit the total error can be
        # small while the line is already near the edge.
        # In hairpin / hold-turn states the error is the turn to make, while the
        # nearby line can be on the other side: search the way it is turning.
        if result.state.startswith(("hairpin", "hold_turn", "fork_commit")):
            side = result.error
        else:
            side = result.center_error if abs(result.center_error) > 0.15 else result.error
        self.last_seen_error = side if abs(side) > 1e-4 else self.last_seen_error
        self.line_found = True
        self.band_since = None
        self.search_started_s = None
        self.target_linear = linear
        self.target_angular = angular
        self.set_state(result.state)
        self._record_time(started)

    def _preview_watched(self) -> bool:
        if not self.preview_enabled:
            return False
        return (self.preview_publisher.get_subscription_count() > 0
                or self.preview_buffer.clients > 0)

    def _maybe_preview(self, gray, result, msg: Image) -> None:
        if not self._preview_watched():
            return
        now = time.monotonic()
        if now - self.preview_last < self.preview_period:
            return
        self.preview_last = now
        # Hand over and return immediately. gray and result are fresh objects
        # for every frame, so the worker can use them without copying.
        with self._preview_cv:
            self._preview_slot = (gray, result, msg.header)
            self._preview_cv.notify()

    def _preview_worker(self) -> None:
        while True:
            with self._preview_cv:
                while self._preview_slot is None and not self._preview_stop:
                    self._preview_cv.wait(timeout=1.0)
                if self._preview_stop:
                    return
                gray, result, header = self._preview_slot
                self._preview_slot = None

            canvas = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            preview = draw_preview(canvas, result)
            ok, encoded = cv2.imencode(
                ".jpg", preview, [int(cv2.IMWRITE_JPEG_QUALITY), self.preview_quality])
            if not ok:
                continue
            jpeg = encoded.tobytes()

            out = CompressedImage()
            out.header = header
            out.format = "jpeg"
            out.data = jpeg
            try:
                self.preview_publisher.publish(out)
            except Exception:                      # shutting down
                return
            self.preview_buffer.publish(jpeg)

    def _record_time(self, started: float) -> None:
        self.stats_times.append((time.perf_counter() - started) * 1e3)
        elapsed = time.monotonic() - self.stats_started
        if elapsed < self.stats_period or not self.stats_times:
            return
        times = sorted(self.stats_times)
        mean = sum(times) / len(times)
        p95 = times[min(len(times) - 1, int(0.95 * len(times)))]
        in_rate = self.stats_frames_in / elapsed
        out_rate = len(times) / elapsed
        self.get_logger().info(
            f"frames in {in_rate:5.1f} Hz, processed {out_rate:5.1f} Hz, "
            f"processing mean {mean:5.1f} ms p95 {p95:5.1f} ms"
            + (", preview active" if self._preview_watched() else ""))
        self.stats_times.clear()
        self.stats_frames_in = 0
        self.stats_started = time.monotonic()

    def image_is_fresh(self) -> bool:
        if self.last_image_time is None:
            return False
        age_s = (self.get_clock().now() - self.last_image_time).nanoseconds / 1e9
        return age_s <= self.image_timeout_s

    def publish_control(self) -> None:
        cmd = Twist()

        if self.race_state in ("FINISH", "SAFE_STOP"):
            # The race is over and the drive disarmed: do not search or steer
            # (the mux blocks it anyway, but the state and page said "searching").
            self.pid.reset()
            self.search_started_s = None
            self.set_state("finished" if self.race_state == "FINISH" else "safe_stop")
        elif not self.image_is_fresh():
            self.line_found = False
            self.pid.reset()
            self.set_state("waiting_for_camera")
        elif self.line_found:
            cmd.linear.x = self.target_linear
            cmd.angular.z = self.target_angular
            self.set_state("crossing_hold" if self.hold is not None else "tracking")
        elif self.search_on_loss and self.search_recover:
            # Never give up in a race: on 2026-10-02 20:47 the 3 s search
            # ended, the robot stood still for 85 s and the lap timed out
            # (SAFE_STOP). Spin a full turn towards where the line was, back
            # up a little (the line is usually behind after overshooting a
            # bend), spin a full turn the other way, back up, and so on.
            now_s = self.get_clock().now().nanoseconds / 1e9
            if self.search_started_s is None:
                self.search_started_s = now_s
            spin_s = self.search_turns * 2.0 * math.pi / max(0.2, abs(self.lost_turn_speed))
            cycle = spin_s + self.search_back_s
            k, t = divmod(now_s - self.search_started_s, cycle)
            side = self.last_seen_error if abs(self.last_seen_error) > 0.05 else 1.0
            if int(k) % 2 == 1:
                side = -side                      # every other turn the other way
            if t < spin_s:
                cmd.angular.z = -math.copysign(abs(self.lost_turn_speed), side)
                self.set_state("searching")
            else:
                cmd.linear.x = -abs(self.search_back_speed)
                self.set_state("search_backing")
        elif self.search_on_loss and abs(self.last_seen_error) > 0.05:
            now_s = self.get_clock().now().nanoseconds / 1e9
            if self.search_started_s is None:
                self.search_started_s = now_s

            if (now_s - self.search_started_s) <= self.search_timeout_s:
                cmd.angular.z = -math.copysign(
                    abs(self.lost_turn_speed), self.last_seen_error)
                self.set_state("searching")
            else:
                # The line is not coming back. Stop rather than spin.
                self.set_state("search_timeout")
        else:
            self.set_state("line_lost")

        self.last_cmd = (float(cmd.linear.x), float(cmd.angular.z))
        self.publisher.publish(cmd)



def _spin(node):
    """Spin with rclpy's EventsExecutor when available.

    The default SingleThreadedExecutor rebuilds its wait set on every wake-up,
    which on the Pi 4 made small nodes cost 25-30 % of a core each at a few
    hundred messages a second (measured 2026-10-07 with pidstat): enough to
    starve the IMU node at start-up and trip the safety monitor. Set
    L2R_EXECUTOR=single to fall back to the default executor.
    """
    if os.environ.get("L2R_EXECUTOR", "events") != "single":
        try:
            from rclpy.experimental import EventsExecutor
        except ImportError:
            EventsExecutor = None
        if EventsExecutor is not None:
            executor = EventsExecutor()
            executor.add_node(node)
            executor.spin()
            return
    rclpy.spin(node)


def main() -> None:
    rclpy.init()
    node = LineFollowerNode()
    try:
        _spin(node)
    finally:
        try:
            node.publisher.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
