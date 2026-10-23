# Using Prototype A

Everything on the robot is started through `~/robot.sh`. It runs the ROS 2 stack inside a `tmux` session, so a dropped SSH connection does not end a run. Each run is logged to `~/run_logs/<date-time>/`. Install the robot first ([INSTALL.md](INSTALL.md)).

## Run helper: `~/robot.sh`

| Command | What it does |
|---|---|
| `~/robot.sh practice [SPEED] [args…]` | Line following without the referee (default 0.3 m/s) |
| `~/robot.sh race [trackbotgp\|dragster] [SPEED] [args…]` | Full competition stack: start light, laps, pit stop, LED code |
| `~/robot.sh attach` | Show the logs (leave with `Ctrl+b`, then `d`) |
| `~/robot.sh status` | What is running, load, memory and power flags |
| `~/robot.sh stop` | Stop everything, including orphan processes, and switch off the LED ring |
| `~/robot.sh build [packages…]` | `colcon build` (refused while the robot is running) |
| `~/robot.sh graph LABEL` | Export the node and topic graph of the running system |
| `~/robot.sh topics [SECONDS]` | Rate, timing and size of the main topics |
| `~/robot.sh check [wheels_lifted:=true]` | Hardware check (below) |
| `~/robot.sh update` | Pull the pinned revision and rebuild ([INSTALL.md](INSTALL.md#9-automatic-updates)) |

Extra arguments:

| Argument | Meaning |
|---|---|
| `track:=portimao\|monza\|suzuka` | Circuit layout; loads `config/track_<name>.yaml` (default `portimao`) |
| `light:=N` | LED ring brightness, 0 = off (default 255) |
| `lidar:=true\|false`, `display:=true\|false` | Optional subsystems (off in a race by default) |
| `preview:=on` | Keep the web preview encoding during the whole run |
| any follower argument | e.g. `curve_gain:=0.7`, `bend_speed:=0.25`, `target_line_distance_mm:=100` |

Examples:

```bash
~/robot.sh race trackbotgp 0.5 track:=monza
~/robot.sh race dragster 1.0
~/robot.sh practice 0.3 curve_gain:=0.7 record_every_n:=1    # record every frame
```

### Hardware check

`~/robot.sh check` tests the robot from the bus up and prints a PASS/FAIL list, saved to `~/run_logs/hardware_check-<time>.txt`. Use it after assembly, after a repair, and whenever the robot misbehaves, before looking at the software:

```bash
~/robot.sh check                       # robot.yaml, I2C devices, ESP32 link, sensor topics, status LED
~/robot.sh check wheels_lifted:=true   # also motors and encoders: put the robot on a stand first
```

It starts the bringup outside competition mode, then checks in order: `robot.yaml`; each fixed I2C device answers at its address (absent means wiring or power); `/odom` arrives (agent, serial link and firmware); each sensor topic in `robot.yaml` publishes at its rate (silent means the driver or the device, slow means bus contention or settings); the status LED shows red, green, blue and white (watch it); and, with the wheels lifted, a slow forward command and a left turn compared with `/odom`. A wheel that fails is diagnosed further with `wheel_direction_test` (learn2race_evaluation).

### Run logs

Each run folder holds:
- `args.txt`: the arguments used;
- `base.log` and `follower.log`: the node output;
- `snaps/`: camera snapshots of every track feature and of the second before each line loss, plus `rec.csv` when recording;
- `pidstat.txt`: CPU and memory per process every 2 s;
- `thermal.csv`: temperature, load and throttling every second.

`~/run_logs/latest` always points to the newest run.

### Race procedure (TrackBotGP)

1. Place the robot behind the start line on the main straight. No part may cross the line.
2. Run `~/robot.sh race trackbotgp 0.5 track:=<circuit>`. The status LED turns **green** when the robot is ready.
3. The start light starts the race. The LED shows the race state:
   - blue: lap in progress;
   - yellow: entering the pit lane;
   - orange: stopped in the pit (at least 5 s);
   - three flashes: finish;
   - white, flashing: end of race.
4. The robot stops by itself after the third lap. `~/robot.sh stop` ends the stack.

### Web preview

During any run, `http://<robot address>:8080` shows the camera view with the detected line, the steering point, the race state and the lap times. The page is read-only.

## Packages (`learn2race_ws/src`)

| Package | Role | Main entry points |
|---|---|---|
| `learn2race_bringup` | Launch files and parameters for the whole robot | `prototype_a.launch.py` (base and competition), `xbox_drive.launch.py`, `teleop_pc.launch.py`, `calibrate_*.launch.py`; `config/prototype_a.yaml`, `config/track_*.yaml`; `scripts/setup_udev.sh` |
| `learn2race_camera_follower` | Line detection and following, track features for the referee, web preview, run recorder | `camera_line_follower.launch.py`; `tools/follower_sim.py` (offline simulator) |
| `learn2race_competition` | TrackBotGP referee (start, laps, pit), competition state machine, LED code, dragster referee | started by `prototype_a.launch.py competition:=true` |
| `learn2race_safety` | Drive-command mux (competition > autonomy > teleoperation) and telemetry supervision | `cmd_vel_mux`, `safety_monitor`; reset with `ros2 topic pub --once /safety/reset std_msgs/msg/Bool "{data: true}"` |
| `learn2race_hardware` | Pi-side drivers: IMU, light sensor, INA219, LED ring, OLED | `prototype_a_sensors`, `rgb_stick_node`, `i2c_display_node` |
| `learn2race_odometry` | `odom` → `base_link` transform from wheel odometry | `config/odometry.yaml` |
| `learn2race_localization` | EKF fusion of wheel odometry and IMU (`robot_localization`) | `config/ekf_odom_imu.yaml` |
| `learn2race_control` | Optional body-velocity PID for tuning | |
| `learn2race_dragster`, `learn2race_dragster_interfaces` | Dragster start action and light sensor | |
| `learn2race_evaluation` | Calibration and measurement tools | see below |
| `learn2race_imu_calibration` | Stationary IMU bias calibration | |
| `learn2race_camera_calibration` | Camera calibration launch files | |
| `learn2race_interfaces` | Custom action definitions | |

Each package's `README.md` lists its parameters and topics.

### Main topics

| Topic | Type | From → to |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `cmd_vel_mux` → ESP32 (only publisher) |
| `/autonomy/cmd_vel`, `/teleop/cmd_vel`, `/competition/cmd_vel` | `Twist` | sources into the mux |
| `/odom` | `nav_msgs/Odometry` | ESP32, 20 Hz |
| `/imu/data_raw`, `/imu/data` | `sensor_msgs/Imu` | Pi sensor node (50 Hz), Madgwick filter |
| `/light/visible_plus_ir_raw` | `std_msgs/UInt32` | start light sensor |
| `/competition/state`, `/lap_event` | | referee and state machine |
| `/safety/fault`, `/safety/fault_reason` | | safety monitor |

## Driving by hand

- **Gamepad on the robot:** `ros2 launch learn2race_bringup xbox_drive.launch.py`.
- **Gamepad on a PC:** see the Learn2Race-PC-Control repository. The pad publishes `teleop/cmd_vel`, which the mux passes on whenever no autonomous source is armed.

## Calibration

Calibrate after any change to the wheels, the tyres or the camera mount.

| What | How |
|---|---|
| Wheel direction | `ros2 run learn2race_evaluation wheel_direction_test` |
| Wheel radius | Drive 3 × 2 m straight runs (`odom_calibration`), compare with a tape, and set `LEFT/RIGHT_WHEEL_RADIUS_M` in `firmware/platformio.ini` |
| Track width | 10 turns on the spot (`calibrate_odometry`) and the 1 m square test (UMBmark, 3 clockwise and 3 anticlockwise runs); set `TRACK_WIDTH_M` |
| IMU axes and bias | `ros2 run learn2race_evaluation calibrate_imu_axes`, then `calibrate_imu` with the robot still; values go into `config/prototype_a.yaml` |
| Camera position | Lens height, tilt and distance ahead of the axle are in the follower parameters (`target_line_distance_mm`) |

After changing a firmware constant, reflash the ESP32 (`pio run -e esp32dev_pi_sensors -t upload`).

## Offline tools

- `learn2race_camera_follower/tools/follower_sim.py`: closed-loop simulator using the real detector and control law, for trying settings without the robot.
- `python3 -m pytest -q test` inside `learn2race_camera_follower`: the detector unit tests.
- `tools/export_ros_graph.py` and `tools/topic_stats.py`: called by `robot.sh graph` and `robot.sh topics`.

## Working rules

- Do not run `ros2 topic` or `ros2 param` commands during a race: on the Pi 4 they can starve the drive. Check everything before READY, then run `ros2 daemon stop`.
- Do not run a ROS stack on another robot on the same network at the same time. Both robots use ROS domain 0, so their `/odom` and `/cmd_vel` would mix.
- Build only when the robot is stopped.
