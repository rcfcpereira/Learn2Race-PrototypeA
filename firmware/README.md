# Learn2Race Prototype A firmware

ESP32 on the Waveshare General Driver for Robots, driving two
CHP-20GP-180ABHL geared motors. micro-ROS client on ROS 2 Jazzy.

    pio run -e esp32dev_pi_sensors -t upload

## Interface

| Direction | Topic | Type | Rate |
|---|---|---|---|
| subscribe | `/cmd_vel` | `geometry_msgs/Twist` | any |
| publish | `/odom` | `nav_msgs/Odometry` | 20 Hz |
| publish | `/imu/data` | `sensor_msgs/Imu` | 50 Hz (`esp32dev` profile only) |
| publish | `/light/visible_plus_ir_raw` | `std_msgs/UInt32` | 10 Hz (`esp32dev` profile only) |
| publish | `/battery_state` | `sensor_msgs/BatteryState` | 1 Hz (`esp32dev` profile only) |

Control runs at 50 Hz; a command gap of 500 ms stops the motors.

## What changed from the original firmware

1. **Gear ratio and encoder resolution set from the datasheet:** 29.16 and 11 PPR (320.76 hall pulses per output turn), 1283.04 counts per wheel turn, confirmed by hand-turning a wheel (1283 counts). The original firmware had 50 and 7. The CHP-20GP-180ABHL is i29. The old
   value made every distance read 29/50 of the truth. Verify with a one-metre
   drive test before trusting odometry.
2. **Command timeout added.** There was none: losing the agent left the motors
   running at their last commanded speed. FR10 is now enforced in firmware.
3. **Duty cycle capped (`MAX_PWM_DUTY`, default 200/255).** The cap protects the
   H-bridge from stall current (up to 2.7 A at 12 V per the datasheet). The
   motor itself is rated for 12 V, so a 3S pack is within its specification; an
   earlier cap of 120, based on the mistaken belief that it was a 6 V motor,
   limited the robot to about 45 % of its speed.
4. **IMU published.** The ROS graph previously subscribed to `/imu/data` with
   nothing publishing it. (The board's I2C bus later turned out to be wired to
   the Pi as well; the robot now runs the `esp32dev_pi_sensors` profile and the
   Pi reads the sensors.)
5. **LTR-303 published**, same reason.
6. **Pack voltage and current published** from the INA219 fitted on the driver
   board at address `0x42`. This makes FR11 measurable directly rather than by
   hand-recorded voltages, and feeds `endurance_logger` with no extra hardware.
   Bus voltage plus the shunt drop is used, since ignoring the shunt term
   under-reports the pack at high current, which matters when comparing against
   a discharge floor. Low voltage is reported through `power_supply_health`;
   the decision to stop is left to the SBC, because cutting drive in firmware
   on a voltage sag would stop the robot mid-run on a transient.
7. **Session time sync added**, matching Prototype B. Without it, stamps fall
   back to board uptime while the SBC stamps wall clock, and fusion never
   converges.
8. **Command dispatch moved out of the subscription callback** into the fixed
   control step, so a fast publisher cannot saturate the loop.
9. **Restructured into libraries** mirroring Prototype B: `RobotConfig`,
   `MotorDriver`, `DifferentialDrive`, `RobotApp`, `MicroRosSerialTransport`.

## Improvements over the restructured baseline

10. **Full quadrature decoding.** The encoders now interrupt on both edges of
    both channels and direction comes from a Gray-code state machine rather than
    from sampling one pin. This is four times the resolution, and it removes the
    race in which the sampled pin has already changed by the time the handler
    reads it. `COUNTS_PER_OUTPUT_REV` accounts for the factor, so odometry needs
    no other change. The illegal double transition produced by a missed
    interrupt counts as no movement rather than as a jump.
11. **Setpoint slew limiting** (`MAX_ACCEL_RAD_S2`, default 40 rad/s^2). A
    full-scale step on a 6 V motor fed from an 11.1 V pack draws a stall current
    the motor was never sized for, and shock-loads a 1:29 gearbox. The limiter
    costs tens of milliseconds of response. A safe stop bypasses it: a stop must
    be immediate.

## Build profiles

| Profile | Use |
|---|---|
| `esp32dev_pi_sensors` | **The robot firmware.** Motors, encoders and the status LED on the ESP32; the Pi reads the I2C sensors (launch with `sensor_host:=pi`) |
| `esp32dev` | The ESP32 also reads the IMU, light sensor and INA219 (launch with `sensor_host:=esp32`, OLED off) |
| `diag` | Prints what the I2C sensors return over serial (115 200 baud), no micro-ROS |
| `wheels` | Raw PWM and encoder counts from single keys in the serial monitor |

```bash
pio run -e esp32dev_pi_sensors -t upload
```

## Calibration in `platformio.ini`

| Constant | Value | Source |
|---|---|---|
| `GEAR_RATIO`, `ENCODER_PPR` | 29.16, 11 | Motor datasheet; 1283 counts per wheel turn checked by hand |
| `LEFT_WHEEL_RADIUS_M`, `RIGHT_WHEEL_RADIUS_M` | 0.03210, 0.03150 m | Straight runs and the 1 m square test (wheel diameter 64.9 mm) |
| `TRACK_WIDTH_M` | 0.1674 m | Effective value from rotation tests (wheel centres 180.5 mm) |

Recalibrate after changing the tyres: the effective radius and track width depend on them.

## References and more details

- [R14] micro-ROS / micro_ros_arduino; [R60] Waveshare General Driver for Robots; [R61] TB6612FNG; [R62] CHP-20GP-180ABHL motor; [R63] ESP32, PlatformIO; [R40] Åström & Murray (wheel PID); [R42] Siegwart et al. (odometry); [R43] UMBmark; [R67]–[R69] LTR303, QMI8658C/AK09918, INA219; [R76] P9813 / ChainableLED (bundled third-party library, MIT licence, unchanged).
- Full entries: [`docs/references.md`](../docs/references.md)
