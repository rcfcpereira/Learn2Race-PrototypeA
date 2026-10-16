// References (full entries: docs/references.md at the repository root):
//   [R60] Waveshare General Driver for Robots: board pins
//   [R61] TB6612FNG motor driver datasheet: H-bridge limits
//   [R62] CHP-20GP-180ABHL motor datasheet: gear ratio, PPR, speed
//   [R40] Astrom & Murray, Feedback Systems (PID): PID gains
//   [R43] Borenstein & Feng (1996) UMBmark: per-wheel radius, UMBmark
//   [R3]  Learn2Race dissertation, requirements FR/NFR: interface rates
// More details: firmware/README.md

#ifndef L2R_A_ROBOT_CONFIG_H
#define L2R_A_ROBOT_CONFIG_H

#include <stdint.h>

// Every value below can be overridden from platformio.ini build flags. Keeping
// defaults here makes the firmware buildable without overrides.

#ifndef ROS_SERIAL_BAUD
#define ROS_SERIAL_BAUD 921600
#endif

// ---------------------------------------------------------------- motor pins
// Waveshare General Driver for Robots, on-board H-bridge.
#ifndef PIN_PWMA
#define PIN_PWMA 25
#endif
#ifndef PIN_AIN1
#define PIN_AIN1 21
#endif
#ifndef PIN_AIN2
#define PIN_AIN2 17
#endif
#ifndef PIN_PWMB
#define PIN_PWMB 26
#endif
#ifndef PIN_BIN1
#define PIN_BIN1 22
#endif
#ifndef PIN_BIN2
#define PIN_BIN2 23
#endif

// -------------------------------------------------------------- encoder pins
#ifndef PIN_ENC_A_1
#define PIN_ENC_A_1 35
#endif
#ifndef PIN_ENC_A_2
#define PIN_ENC_A_2 34
#endif
#ifndef PIN_ENC_B_1
#define PIN_ENC_B_1 27
#endif
#ifndef PIN_ENC_B_2
#define PIN_ENC_B_2 16
#endif

// ------------------------------------------------------ physical wheel map
// Measured with the basic wheel test (env:wheels): motor channel A and encoder
// pins 35/34 belong to the physical RIGHT wheel, and channel B with encoder pins
// 27/16 to the physical LEFT wheel. The board labels do not follow the robot's
// left and right, so the mapping is stated here explicitly rather than assumed
// from the channel letters. Each encoder belongs to the same wheel as the motor
// channel beside it, which the test confirmed: the count always moved on the
// channel being driven.
#define LEFT_PWM_PIN      PIN_PWMB
#define LEFT_IN1_PIN      PIN_BIN1
#define LEFT_IN2_PIN      PIN_BIN2
#define LEFT_PWM_CHANNEL  PWM_CHANNEL_B
#define LEFT_ENC_A_PIN    PIN_ENC_B_1      // 27
#define LEFT_ENC_B_PIN    PIN_ENC_B_2      // 16

#define RIGHT_PWM_PIN     PIN_PWMA
#define RIGHT_IN1_PIN     PIN_AIN1
#define RIGHT_IN2_PIN     PIN_AIN2
#define RIGHT_PWM_CHANNEL PWM_CHANNEL_A
#define RIGHT_ENC_A_PIN   PIN_ENC_A_1      // 35
#define RIGHT_ENC_B_PIN   PIN_ENC_A_2      // 34

// ------------------------------------------------------------------- LEDC
#ifndef PWM_FREQ_HZ
#define PWM_FREQ_HZ 20000
#endif
#ifndef PWM_RESOLUTION_BITS
#define PWM_RESOLUTION_BITS 8
#endif
#ifndef PWM_CHANNEL_A
#define PWM_CHANNEL_A 5
#endif
#ifndef PWM_CHANNEL_B
#define PWM_CHANNEL_B 6
#endif

// Below MIN_PWM_DUTY the gearbox stalls rather than turning slowly.
#ifndef MIN_PWM_DUTY
#define MIN_PWM_DUTY 30
#endif
// Ceiling that protects the H-bridge from stall current. See platformio.ini.
#ifndef MAX_PWM_DUTY
#define MAX_PWM_DUTY 200
#endif

// -------------------------------------------------------------- drivetrain
#ifndef GEAR_RATIO
#define GEAR_RATIO 29.16f   // datasheet: 320.76 hall pulses / 11 PPR
#endif
#ifndef ENCODER_PPR
#define ENCODER_PPR 11      // measured: 1276 counts per wheel turn
#endif
// Full quadrature decoding counts every edge of both channels, giving four
// counts per encoder pulse. The original firmware counted one edge of one
// channel, so it resolved a quarter of this.
#ifndef ENCODER_EDGES_PER_PULSE
#define ENCODER_EDGES_PER_PULSE 4
#endif
#define COUNTS_PER_OUTPUT_REV \
    ((float)(ENCODER_PPR) * (GEAR_RATIO) * (float)(ENCODER_EDGES_PER_PULSE))

// Maximum rate of change of the wheel setpoint, rad/s per second. Limits
// H-bridge current and gearbox shock loads during acceleration. Was 40: in a
// hairpin at 0.3 m/s the inner wheel goes from +9.4 to -2 rad/s, which took
// 0.29 s at 40 and takes 0.11 s at 100. With the 5 duty per rad/s feedforward
// and a motor lagging the ramp by ~50 ms, the extra current is ~0.3 A, well
// inside the TB6612FNG's 1.2 A continuous rating.
#ifndef MAX_ACCEL_RAD_S2
#define MAX_ACCEL_RAD_S2 100.0f
#endif

// Clear the speed integral when the setpoint changes direction (MotorDriver).
#ifndef RESET_INTEGRAL_ON_REVERSAL
#define RESET_INTEGRAL_ON_REVERSAL 1
#endif

// An explicit zero command short-brakes the TB6612FNG (IN1 = IN2 = HIGH)
// instead of ramping down and then coasting (IN1 = IN2 = LOW). Braking current
// is bounded by the motor's back-EMF, i.e. at most its stall current, which is
// within the driver's 3.2 A peak; reversing the motor to brake would not be.
// Set to 0 for the old smooth ramp-and-coast behaviour.
#ifndef BRAKE_ON_ZERO
#define BRAKE_ON_ZERO 1
#endif

#ifndef WHEEL_RADIUS_M
#define WHEEL_RADIUS_M 0.0325f
#endif
#ifndef TRACK_WIDTH_M
#define TRACK_WIDTH_M 0.1755f
#endif

// Per-wheel effective radius. UMBmark separates the wheels' diameter ratio
// from the track width; a robot that drifts sideways on a straight command
// has unequal effective wheels. Both default to the common radius.
#ifndef LEFT_WHEEL_RADIUS_M
#define LEFT_WHEEL_RADIUS_M WHEEL_RADIUS_M
#endif
#ifndef RIGHT_WHEEL_RADIUS_M
#define RIGHT_WHEEL_RADIUS_M WHEEL_RADIUS_M
#endif

#ifndef LEFT_MOTOR_SIGN
#define LEFT_MOTOR_SIGN 1
#endif
#ifndef RIGHT_MOTOR_SIGN
#define RIGHT_MOTOR_SIGN -1
#endif
// Encoder directions, independent of the motor directions. Defaults assume the
// right motor is mounted mirrored, so its encoder counts backwards when the
// robot drives forward. Set with the wheel direction test.
#ifndef LEFT_ENCODER_SIGN
#define LEFT_ENCODER_SIGN 1
#endif
#ifndef RIGHT_ENCODER_SIGN
#define RIGHT_ENCODER_SIGN -1
#endif

#ifndef MAX_WHEEL_RPM
#define MAX_WHEEL_RPM 515.0f   // datasheet no-load speed at 12 V
#endif

// -------------------------------------------------- speed control scaling
// Measured with the basic wheel test: at duty 200 on a pack at 11.31 V the
// wheels turn at about 377 rpm, which is 39.5 rad/s. The feedforward gain is
// therefore roughly 200 / 39.5 = 5 duty counts per rad/s, and it does most of
// the work: the target duty is known from the commanded speed, so the PID only
// has to correct the remainder.
//
// Without feedforward the gains below could not reach full speed in any useful
// time. With an error of 40 rad/s, an integral gain of 0.05 adds 0.04 duty per
// 20 ms step, so full duty would take about 100 seconds.
#ifndef MAX_WHEEL_RAD_S
#define MAX_WHEEL_RAD_S 39.5f
#endif
#ifndef PID_KFF
#define PID_KFF ((float)(MAX_PWM_DUTY) / (MAX_WHEEL_RAD_S))
#endif

// ---------------------------------------------------------------- PID gains
#ifndef PID_KP
#define PID_KP 1.5f
#endif
#ifndef PID_KI
#define PID_KI 8.0f
#endif
#ifndef PID_KD
#define PID_KD 0.0f
#endif

// ------------------------------------------------------------------ timing
// Matches the SBC-MCU interface contract of the dissertation: control at 50 Hz,
// odometry at 20 Hz, safe stop after 500 ms without a command.
#ifndef CONTROL_PERIOD_MS
#define CONTROL_PERIOD_MS 20
#endif
#ifndef ODOM_PUBLISH_PERIOD_MS
#define ODOM_PUBLISH_PERIOD_MS 50
#endif
#ifndef IMU_PUBLISH_PERIOD_MS
#define IMU_PUBLISH_PERIOD_MS 20
#endif
#ifndef CMD_TIMEOUT_MS
#define CMD_TIMEOUT_MS 500
#endif
#ifndef AGENT_PING_PERIOD_MS
#define AGENT_PING_PERIOD_MS 200
#endif
#ifndef TIME_SYNC_PERIOD_MS
#define TIME_SYNC_PERIOD_MS 10000
#endif

// --------------------------------------------------------------- on-board I2C
// Set to 0 only when the Raspberry Pi is physically wired to the sensor bus
// and is its sole I2C master. The ESP32 then never initialises that bus.
#ifndef ESP32_SENSOR_OWNER
#define ESP32_SENSOR_OWNER 1
#endif

// Old Grove Chainable RGB v2 (P9813), not a WS2812/NeoPixel device.
#ifndef STATUS_LED_CLK_PIN
#define STATUS_LED_CLK_PIN 5
#endif
#ifndef STATUS_LED_DATA_PIN
#define STATUS_LED_DATA_PIN 4
#endif
#ifndef STATUS_LED_MAX_LEVEL
// Full brightness: at 48 (19%) the TrackBotGP colours yellow (pit entry),
// orange (pit stop) and red (fault) were hard to tell apart in a lit room.
#define STATUS_LED_MAX_LEVEL 255
#endif
#ifndef LED_COMMAND_TIMEOUT_MS
#define LED_COMMAND_TIMEOUT_MS 2000
#endif

// Conservative initial diagonal variances; tune with measured odometry error.
// Position: m^2; yaw: rad^2; twist: (m/s)^2 and (rad/s)^2.
#ifndef ODOM_POSITION_VARIANCE
#define ODOM_POSITION_VARIANCE 0.01
#endif
#ifndef ODOM_YAW_VARIANCE
#define ODOM_YAW_VARIANCE 0.04
#endif
#ifndef ODOM_LINEAR_VARIANCE
#define ODOM_LINEAR_VARIANCE 0.01
#endif
#ifndef ODOM_ANGULAR_VARIANCE
#define ODOM_ANGULAR_VARIANCE 0.04
#endif
#ifndef ODOM_UNOBSERVED_VARIANCE
#define ODOM_UNOBSERVED_VARIANCE 1000000.0
#endif

// QMI8658C 6-axis IMU and AK09918C compass sit on the driver board I2C bus,
// together with the LTR-303 ambient light sensor.
#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 32
#endif
#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 33
#endif
#ifndef QMI8658_ADDRESS
#define QMI8658_ADDRESS 0x6B
#endif
#define QMI8658_WHO_AM_I_VALUE 0x05

// AK09918C 3-axis magnetometer, on the same board I2C bus.
#ifndef AK09918_ADDRESS
#define AK09918_ADDRESS 0x0C
#endif
#define AK09918_WIA2_VALUE 0x0C
#ifndef MAG_PUBLISH_PERIOD_MS
#define MAG_PUBLISH_PERIOD_MS 50
#endif

// Magnetometer axes. The AK09918C is a separate die from the QMI8658C and is
// not necessarily aligned with it, so it has its own mapping into the REP-103
// body frame. It cannot be found with the gravity-based axis calibration,
// since a magnetometer does not see gravity; see calibrate_magnetometer.
#ifndef MAG_X_SOURCE
#define MAG_X_SOURCE 0
#endif
#ifndef MAG_X_SIGN
#define MAG_X_SIGN 1
#endif
#ifndef MAG_Y_SOURCE
#define MAG_Y_SOURCE 1
#endif
#ifndef MAG_Y_SIGN
#define MAG_Y_SIGN (-1)
#endif
#ifndef MAG_Z_SOURCE
#define MAG_Z_SOURCE 2
#endif
#ifndef MAG_Z_SIGN
#define MAG_Z_SIGN (-1)
#endif

// Hard-iron offsets in microtesla, subtracted before publishing. Measured with
// calibrate_magnetometer by rotating the robot through a full turn.
#ifndef MAG_OFFSET_X_UT
#define MAG_OFFSET_X_UT 0.0f
#endif
#ifndef MAG_OFFSET_Y_UT
#define MAG_OFFSET_Y_UT 0.0f
#endif
#ifndef MAG_OFFSET_Z_UT
#define MAG_OFFSET_Z_UT 0.0f
#endif

// ---------------------------------------------------------------- IMU axes
// REP-103 requires a body frame with x forward, y left, z up. The QMI8658C
// reports its own axes, and how those relate to the robot depends both on the
// orientation of the chip on the board and on how the board is mounted, so the
// mapping cannot be taken from a datasheet. Determine it with
// `ros2 run learn2race_evaluation calibrate_imu_axes` and set these to match.
//
// Source is 0, 1 or 2 for the sensor X, Y or Z channel; sign is +1 or -1.
//
// The QMI8658C on the general driver board is mounted x forward, y right,
// z down, which is the aerospace NED convention. The defaults below convert it
// to the REP-103 body frame by negating y and z. Verify with
// `calibrate_imu_axes` after any change to how the board is mounted.
#ifndef IMU_X_SOURCE
#define IMU_X_SOURCE 0
#endif
#ifndef IMU_X_SIGN
#define IMU_X_SIGN 1
#endif
#ifndef IMU_Y_SOURCE
#define IMU_Y_SOURCE 1
#endif
#ifndef IMU_Y_SIGN
#define IMU_Y_SIGN (-1)
#endif
#ifndef IMU_Z_SOURCE
#define IMU_Z_SOURCE 2
#endif
#ifndef IMU_Z_SIGN
#define IMU_Z_SIGN (-1)
#endif
// Applied to all three gyroscope channels after remapping, so that a positive
// rate corresponds to an anticlockwise rotation under the right-hand rule.
#ifndef IMU_GYRO_SIGN
#define IMU_GYRO_SIGN 1
#endif

// Gyroscope bias in rad/s, subtracted after remapping. Measure with
// `calibrate_imu` and set these; an uncorrected bias integrates straight into
// heading error.
#ifndef IMU_GYRO_BIAS_X
#define IMU_GYRO_BIAS_X 0.0f
#endif
#ifndef IMU_GYRO_BIAS_Y
#define IMU_GYRO_BIAS_Y 0.0f
#endif
#ifndef IMU_GYRO_BIAS_Z
#define IMU_GYRO_BIAS_Z 0.0f
#endif
#ifndef LTR303_ADDRESS
#define LTR303_ADDRESS 0x29
#endif

// INA219 voltage and current monitor, fitted on the General Driver board and
// wired across the battery input. Waveshare place it at 0x42, not the 0x40
// default used by most breakout boards.
// Named L2R_INA219_ADDRESS, not INA219_ADDRESS: the INA219_WE library declares
// a class member called INA219_ADDRESS, and a macro of the same name is
// substituted into the library header and breaks it.
#ifndef L2R_INA219_ADDRESS
#define L2R_INA219_ADDRESS 0x42
#endif

#ifndef BATTERY_PUBLISH_PERIOD_MS
#define BATTERY_PUBLISH_PERIOD_MS 1000
#endif

// INA219 current shunt: R11 = 0.01R on the General Driver for Robots schematic.
#ifndef INA219_SHUNT_OHMS
#define INA219_SHUNT_OHMS 0.01f
#endif

#ifndef BATTERY_CELLS
#define BATTERY_CELLS 3
#endif
#ifndef CELL_FLOOR_V
#define CELL_FLOOR_V 3.50f
#endif
#ifndef CELL_FULL_V
#define CELL_FULL_V 4.20f
#endif

#endif  // L2R_A_ROBOT_CONFIG_H
