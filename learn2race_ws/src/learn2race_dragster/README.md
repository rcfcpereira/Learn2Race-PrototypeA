# Learn2Race Dragster Package

This package adds a light-triggered race start for dragster-style runs.

## Hardware

The Grove Light Sensor v1.2 is analog, so it cannot be connected directly to a
Raspberry Pi GPIO as a readable input. Use an ADC. The default package config
uses an ADS1115 on I2C.

Recommended wiring:

- Grove Light Sensor `VCC` -> `3.3V`
- Grove Light Sensor `GND` -> common `GND`
- Grove Light Sensor `SIG` -> ADS1115 `A0`
- ADS1115 `VDD` -> Raspberry Pi `3.3V`
- ADS1115 `GND` -> common `GND`
- ADS1115 `SDA` -> Raspberry Pi GPIO2 / physical pin 3
- ADS1115 `SCL` -> Raspberry Pi GPIO3 / physical pin 5

## Algorithm

1. Read the Grove analog voltage through ADS1115.
2. Publish normalized light ratio on `/dragster/light_ratio`.
3. The action server averages a short ambient baseline.
4. The server arms and waits for a rising light edge by default.
5. If the light exceeds both the absolute threshold and baseline delta, the
   action succeeds.
6. Stale sensor samples are ignored.
7. Optionally, the action publishes a short launch `Twist`.

The Grove sensor reports an approximate trend, not calibrated lux. Tune the
threshold at the track.

Set `--threshold 0.0` to ignore the absolute ratio and trigger only on the
baseline delta. Use `--falling-edge` if your start gate gets darker instead of
brighter.

## Run Server

```bash
ros2 launch learn2race_dragster dragster_start.launch.py
```

Watch the sensor:

```bash
ros2 topic echo /dragster/light_ratio
```

Detect start only:

```bash
ros2 run learn2race_dragster drag_race_start_client \
  --threshold 0.65 --min-delta 0.15 --timeout-s 10
```

Detect start and launch forward:

```bash
ros2 run learn2race_dragster drag_race_start_client \
  --threshold 0.65 --min-delta 0.15 --timeout-s 10 \
  --launch --launch-linear 0.6 --launch-duration 0.75
```

If using the PID package, start the server with:

```bash
ros2 launch learn2race_dragster dragster_start.launch.py cmd_vel_topic:=/cmd_vel_setpoint
```

## Required Packages

- `learn2race_dragster_interfaces`
- `geometry_msgs`
- `rclpy`
- `std_msgs`
- `launch`
- `launch_ros`
- Python `smbus2`

## Sources

- Seeed Grove Light Sensor wiki: https://wiki.seeedstudio.com/Grove-Light_Sensor/
- Grove Light Sensor v1.2 analog output note: https://seeeddoc.github.io/Grove-Light_Sensor_v1.2

## References and more details

- [R2] Dragster rules; [R70] ADS1115; [R74] Grove Light Sensor; [R10] ROS 2 actions.
- Full entries: [`docs/references.md`](../../../docs/references.md).
