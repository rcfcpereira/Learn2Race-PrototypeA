# learn2race_camera_follower

Camera line following for Prototype A: finds the black track line in the IMX219 image,
steers along it, and reports the track features the race referee needs (finish band,
chequered marker, pit split, crossing). Also serves a live web preview and records
runs for offline analysis.

## Files

| File | Contents |
|---|---|
| `learn2race_camera_follower/detector.py` | Image processing only (no ROS): segmentation, line position, bands, chequered marker, splits, hairpins, crossings, sharp bends, branch choice. Unit-tested |
| `learn2race_camera_follower/line_follower_node.py` | ROS 2 node: runs the detector on each frame, steering (PD) and speed rules, slow-down before sharp bends, line-loss search, topics for the referee, snapshots and the run recorder |
| `learn2race_camera_follower/preview_server.py` | MJPEG web preview with a race status panel (port 8080) |
| `learn2race_camera_follower/control.py` | PID controller |
| `learn2race_camera_follower/path_memory.py` | Remembered path (used only by the simulator) |
| `learn2race_camera_follower/line_detection_preview_node.py` | Stand-alone preview node |
| `tools/follower_sim.py` | Closed-loop simulator: virtual track, camera model, the real detector and control law, unicycle robot with actuation delay |
| `launch/camera_line_follower.launch.py` | Camera node (`camera_ros`, IMX219 full-field mode at 41.8 fps, 320×240) + follower |
| `test/test_detector.py`, `test/test_control.py` | Unit tests (`python3 -m pytest -q test`) |

## How the detection works (short)

1. **Segmentation:** a pixel is line when darker than `line_ratio` (0.5) × the local
   floor brightness (129-px max filter), so shadows and lighting gradients do not
   matter. A blob holding two darkness levels has its lighter part dropped as a shadow
   only if that part is a wide patch (> 1.6 × the tape width), so the lit end of the
   tape near the robot is kept.
2. **Line position** on a target row ~125 mm ahead, plus a look-ahead term 45 px
   further: `error = centre + 0.55 · curve`.
3. **Features:** finish band (a cross with the line, wide on both sides);
   chequered marker (rows of squares); split (two branches joined below, or a wide
   V); hairpin (two legs joined only above); crossing (four ends and a straight way
   through on the stripe); sharp bend ahead (the line turns steeper than 45° or runs
   out sideways).
4. **Branch choice:** straight on at splits and crossings (the branch matching the
   robot's own direction); the referee's preferred branch only when the pit is armed,
   with a short commitment to it.

## Control (short)

PD steering (`pid_kp` 1.8, `pid_kd` 0.25, max 1.8 rad/s); speed falls with the error
(`slowdown_gain` 1.5). Before a sharp bend the speed is capped from `bend_slow_px`
(100 px) ahead down to `bend_speed` (0.08 m/s), with a firm turn and a near-pivot
(`bend_pivot_speed` 0.02 m/s) at the bend itself, because the drive lags the commands
by ~0.3 s. When the line is lost, the robot searches without end (`search_recover`):
a full turn towards where the line was, back up 6 cm, a full turn the other way,
repeat. Set `search_recover:=false` for the old 3 s search and stop.

## Running

Normally through `~/robot.sh` (practice or race); directly:

```bash
ros2 launch learn2race_camera_follower camera_line_follower.launch.py max_linear_speed:=0.3
```

Useful launch arguments: `max_linear_speed`, `curve_gain`, `target_line_distance_mm`,
`slowdown_gain`, `bend_speed`, `snapshot_dir` (save frames of each feature sighting and
the second before each line loss), `record_every_n` (save every n-th frame plus a
per-frame CSV of detection, command and wheel odometry).

## Topics

| Direction | Topic | Type |
|---|---|---|
| in | `image_raw` | `sensor_msgs/Image` |
| in | `line/branch_preference`, `line/speed_limit` | `String`, `Float32` (from the referee) |
| in | `leds/set_color`, `lap_event`, `competition/state` | for the web status panel |
| in | `odom` | `nav_msgs/Odometry` (recorder only) |
| out | `autonomy/cmd_vel` | `geometry_msgs/Twist` (through the safety mux) |
| out | `line/found`, `line/band_detected`, `line/fork_detected`, `line/checker_detected` | `Bool` |
| out | `line/fork_side` | `String` |
| out | `line_preview/compressed` + `http://<robot>:8080/` | preview |

## Troubleshooting

- **Robot runs off tight curves:** the wheels lag
  the commands; lower `bend_speed`.
- **Line lost near the robot / false hairpins:** check the shadow split (above) and the
  track light.
- **Preview stutters over Wi-Fi:** Wi-Fi power saving on the Pi
  (`learn2race_bringup/scripts/wifi_tuning.sh`); watch in a browser, not with RViz.

## References and more details

- [R30] OpenCV; [R31] Otsu; [R32] Gonzalez & Woods; [R33] Suzuki & Abe (contours);
  [R35] Hartley & Zisserman (camera model); [R40] Åström & Murray (PID);
  [R41] Coulter (pure pursuit); [R42] Siegwart et al. (kinematics);
  [R17] camera_ros; [R18] Raspberry Pi camera software; [R1] TrackBotGP rules.
- Full entries: [`docs/references.md`](../../../docs/references.md).
- Design decisions, measured results and what failed:,
  sections 3 and 5. Each rule in `detector.py` is commented with the run that
  motivated it.
