# learn2race_bringup (Prototype A)

Launch files and parameter sets that start the robot. Normally started through
`~/robot.sh` (see `tools/robot.sh`), not directly.

## Main launch file: `launch/prototype_a.launch.py`

Starts the micro-ROS agent, sensors, IMU filter, odometry TF, safety monitor and
drive mux, and in competition mode the competition FSM, the referee for the chosen
mode, the LED signalling and the track light.

| Argument | Default | Meaning |
|---|---|---|
| `competition` | `true` | Competition nodes on; mux in competition mode |
| `mode` | `dragster` | `dragster` or `trackbotgp` |
| `track` | `portimao` | TrackBotGP layout: loads `config/track_<track>.yaml` |
| `rgb_stick` | `false` | Start the WS2813 track light |
| `track_light_level` | `200` | Track-light brightness (0 = off) |
| `sensor_host` | `esp32` | Who reads the board's I2C sensors: `esp32` or `pi` (Prototype A uses `pi`) |
| `lidar` | `auto` | Off in competition, on otherwise |
| `display` | `auto` | OLED off in competition (saves CPU) |
| `autonomy_enabled` | `false` | Let the follower drive outside competition mode (practice) |
| `serial_port` / `agent_baud` | `/dev/mcu` / `921600` | ESP32 serial link |

The camera line follower is started separately
(`learn2race_camera_follower/launch/camera_line_follower.launch.py`).

## Configuration

| File | Contents |
|---|---|
| `config/prototype_a.yaml` | Parameters of every node: start light, referee guards, pit timing, FSM timeouts, LED colours, odometry geometry |
| `config/track_portimao.yaml`, `track_monza.yaml`, `track_suzuka.yaml` | Per-track race settings: pit side, lap length, first-lap minimum, pit arming distance |

`launch/hardware_check.launch.py` runs the first-line hardware check (`~/robot.sh check`, see USAGE.md).

Other launch files: `drive_only`, `robot_only`, `xbox_drive`, `teleop_pc`,
`remote_pc_joy` (driving and teleoperation), `i2c_sensors`, `all_sensors`,
`calibrate_imu`, `calibrate_sensors`, `calibrate_stereo_cameras`, `stereo_cameras`,
`mapping_room`, `slam_room` (mapping).
Scripts: `scripts/setup_udev.sh` (stable device names such as `/dev/mcu`),
`scripts/wifi_tuning.sh`.

## References and more details

- [R15] ROS 2 launch; [R14] micro-ROS agent; [R46] Madgwick IMU filter;
  [R47] SLAM Toolbox; [R79] lidar driver; [R19] joystick teleoperation;
  [R1] TrackBotGP rules (track files); [R92] udev.
- Full entries: [`docs/references.md`](../../../docs/references.md),
  sections 2 and 4.4.
