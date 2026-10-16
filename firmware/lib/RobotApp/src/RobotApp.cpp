// References (full entries: docs/references.md at the repository root):
//   [R14] micro-ROS / micro_ros_arduino: micro-ROS entities and executor
//   [R12] REP 103, units and axes: units
//   [R13] REP 105, coordinate frames: frames
//   [R67] LTR-303ALS-01 datasheet: LTR303
//   [R68] QMI8658C / AK09918 datasheets: QMI8658C, AK09918
//   [R69] INA219 / INA226 datasheets, INA219_WE: INA219
//   [R76] P9813 / ChainableLED: P9813 status LED
//   [R3]  Learn2Race dissertation, requirements FR/NFR: FR10 command timeout
// More details: firmware/README.md

#include "RobotApp.h"

#include <ChainableLED.h>
#include <MicroRosSerialTransport.h>
#if ESP32_SENSOR_OWNER
#include <INA219_WE.h>
#include <Wire.h>
#endif
#include <esp_timer.h>
#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>

namespace {

MotorDriver g_left(LEFT_PWM_PIN, LEFT_IN1_PIN, LEFT_IN2_PIN, LEFT_PWM_CHANNEL,
                   LEFT_MOTOR_SIGN, LEFT_ENCODER_SIGN);
MotorDriver g_right(RIGHT_PWM_PIN, RIGHT_IN1_PIN, RIGHT_IN2_PIN, RIGHT_PWM_CHANNEL,
                    RIGHT_MOTOR_SIGN, RIGHT_ENCODER_SIGN);
DifferentialDrive g_drive(g_left, g_right);

ChainableLED g_status_led(STATUS_LED_CLK_PIN, STATUS_LED_DATA_PIN, 1);
#if ESP32_SENSOR_OWNER
INA219_WE g_ina219(L2R_INA219_ADDRESS);
#endif

// Both channels of each encoder raise interrupts on both edges, and the
// handler reads the pair. This is what makes full quadrature decoding possible.
void IRAM_ATTR leftEncoderIsr() {
  g_left.onEncoderEdge(digitalRead(LEFT_ENC_A_PIN) == HIGH,
                       digitalRead(LEFT_ENC_B_PIN) == HIGH);
}

void IRAM_ATTR rightEncoderIsr() {
  g_right.onEncoderEdge(digitalRead(RIGHT_ENC_A_PIN) == HIGH,
                        digitalRead(RIGHT_ENC_B_PIN) == HIGH);
}

#if ESP32_SENSOR_OWNER
bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t address, uint8_t reg, uint8_t* out, uint8_t count) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(address, count) != count) {
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) {
    out[i] = Wire.read();
  }
  return true;
}
#endif

char kOdomFrame[] = "odom";
char kBaseFrame[] = "base_link";
#if ESP32_SENSOR_OWNER
char kImuFrame[] = "imu_link";
char kEmpty[] = "";
#endif

void setFrameId(rosidl_runtime_c__String& field, char* text) {
  field.data = text;
  field.size = strlen(text);
  field.capacity = field.size + 1;
}

// Teardown after an agent loss legitimately fails part-way, and there is no
// recovery from a failed publish that the next cycle does not already provide,
// so results are discarded explicitly rather than silently.
inline void ignoreRclResult(rcl_ret_t result) { (void)result; }

// Combine two little-endian bytes into a signed 16-bit value.
//
// Deliberately NOT named "word". The ESP32 Arduino core defines word(...) as a
// preprocessor macro for makeWord(), so a local lambda called word is silently
// replaced at every call: word(4) becomes makeWord(4), which returns 4. The
// firmware then published the byte offsets 0, 2, 4, 6, 8, 10 as sensor data,
// with no compiler error or warning. The values on the ROS side decoded to
// exactly those integers, which is how the fault was found.
#if ESP32_SENSOR_OWNER
inline int16_t le16(const uint8_t* bytes, uint8_t index) {
  return (int16_t)((bytes[index + 1] << 8) | bytes[index]);
}

uint16_t readLtr303Word(uint8_t reg) {
  Wire.beginTransmission(LTR303_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return 0;
  }
  Wire.requestFrom((uint8_t)LTR303_ADDRESS, (uint8_t)2);
  if (Wire.available() < 2) {
    return 0;
  }
  const uint8_t low = Wire.read();
  const uint8_t high = Wire.read();
  return (uint16_t)((high << 8) | low);
}
#endif

}  // namespace

RobotApp g_app;

// ---------------------------------------------------------------- lifecycle

bool RobotApp::begin() {
  // Set motor outputs safe before any transport/sensor startup delay.
  g_drive.begin();
  g_status_led.init();
  updateStatusLed();
  // Same transport as Prototype B, which is known to build and link on this
  // toolchain. micro_ros_arduino provides set_microros_transports(), not the
  // set_microros_serial_transports() of micro_ros_platformio; the project
  // transport also opens Serial at ROS_SERIAL_BAUD rather than the library
  // default of 115200. Nothing else may print to Serial from here on, since the
  // port now carries the XRCE-DDS byte stream.
  l2r::beginMicroRosSerialTransport();
  delay(2000);

#if ESP32_SENSOR_OWNER
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  // 400 kHz fast mode. Every device on this bus supports it (QMI8658C,
  // AK09918C, INA219, LTR-303). At the 100 kHz default, the IMU, compass, light
  // and battery reads take long enough to stretch the loop, which is why the
  // first sensor check measured the IMU at 36 Hz and odometry at 18 Hz instead
  // of 50 and 20.
  Wire.setClock(400000);
#endif

  // Plain INPUT, as in the basic wheel test that verified these encoders.
  // GPIO 34 and 35 are input-only and have no internal pull-ups, so the
  // encoder circuit provides them; the same setting is used on all four pins.
  pinMode(LEFT_ENC_A_PIN, INPUT);
  pinMode(LEFT_ENC_B_PIN, INPUT);
  pinMode(RIGHT_ENC_A_PIN, INPUT);
  pinMode(RIGHT_ENC_B_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN), leftEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B_PIN), leftEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), rightEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B_PIN), rightEncoderIsr, CHANGE);

#if ESP32_SENSOR_OWNER
  imu_available_ = initImu();
  mag_available_ = initMagnetometer();

  // LTR-303: active mode, 100 ms integration.
  Wire.beginTransmission(LTR303_ADDRESS);
  Wire.write(0x80);
  Wire.write(0x01);
  Wire.endTransmission();

  // INA219 on the General Driver board: its shunt R11 (0.01 ohm) sits between DC_IN and VIN, so it
  // measures the pack voltage but only the current of the VIN branch (the 5 V regulator: Pi, camera,
  // sensors, LEDs). The TB6612 motor supply (VM) is taken from DC_IN, before the shunt, so motor
  // current is not included (schematic checked 2026-10-15). A missing monitor is not fatal: the robot
  // still drives, it simply cannot report pack state, so record the fact and
  // continue rather than refusing to start.
  // The library defaults after init() are continuous measurement, 12-bit
  // conversion, 32 V bus range and +/-320 mV shunt range, all appropriate for a
  // 3S pack. They are left as they are rather than set explicitly, because
  // INA219_WE 1.4 moved its mode enumerations inside the class and the names
  // are not stable across library versions.
  battery_available_ = g_ina219.init();
  // The library assumes 0.1 ohm; the board fits 0.01 ohm (R11 on the
  // schematic), so without this every current reading is ten times too low.
  g_ina219.setShuntSizeInOhms(INA219_SHUNT_OHMS);
#endif

  allocator_ = rcl_get_default_allocator();
  return true;
}

bool RobotApp::createEntities() {
  if (rclc_support_init(&support_, 0, NULL, &allocator_) != RCL_RET_OK) {
    return false;
  }
  if (rclc_node_init_default(&node_, "learn2race_a", "", &support_) != RCL_RET_OK) {
    return false;
  }

  if (rclc_subscription_init_default(
      &cmd_vel_sub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) {
    return false;
  }
  if (rclc_subscription_init_default(
      &led_sub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA), "leds/set_color") != RCL_RET_OK) {
    return false;
  }
  if (rclc_subscription_init_default(
      &safety_fault_sub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "safety/fault") != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
      &odom_pub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom") != RCL_RET_OK) {
    return false;
  }

#if ESP32_SENSOR_OWNER
  if (rclc_publisher_init_default(
      &imu_pub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw") != RCL_RET_OK) {
    return false;
  }

  // The firmware publishes raw rate and acceleration with no orientation, so
  // the topic follows the imu_tools convention: imu/data_raw in, and an
  // orientation filter on the SBC produces imu/data with the quaternion set.
  if (rclc_publisher_init_default(
      &mag_pub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), "imu/mag") != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
      &light_pub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
      "light/visible_plus_ir_raw") != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(
      &battery_pub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      "battery_state") != RCL_RET_OK) {
    return false;
  }
#endif

  // The packaged micro-ROS library permits 5 subscriptions; these use 3.
  if (rclc_executor_init(&executor_, &support_.context, 3, &allocator_) != RCL_RET_OK ||
      rclc_executor_add_subscription(&executor_, &safety_fault_sub_, &safety_fault_msg_,
                                    &RobotApp::safetyFaultCallback, ON_NEW_DATA) != RCL_RET_OK ||
      rclc_executor_add_subscription(&executor_, &cmd_vel_sub_, &cmd_vel_msg_,
                                    &RobotApp::cmdVelCallback, ON_NEW_DATA) != RCL_RET_OK ||
      rclc_executor_add_subscription(&executor_, &led_sub_, &led_msg_,
                                    &RobotApp::ledCallback, ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  // micro-ROS string fields are plain C structs. Pointing them at static
  // storage avoids heap allocation, needs no utility library, and is safe
  // because the messages are never finalised or resized.
  setFrameId(odom_msg_.header.frame_id, kOdomFrame);
  setFrameId(odom_msg_.child_frame_id, kBaseFrame);
  for (size_t i = 0; i < 36; ++i) {
    odom_msg_.pose.covariance[i] = 0.0;
    odom_msg_.twist.covariance[i] = 0.0;
  }
  odom_msg_.pose.covariance[0] = ODOM_POSITION_VARIANCE;
  odom_msg_.pose.covariance[7] = ODOM_POSITION_VARIANCE;
  odom_msg_.pose.covariance[35] = ODOM_YAW_VARIANCE;
  odom_msg_.twist.covariance[0] = ODOM_LINEAR_VARIANCE;
  odom_msg_.twist.covariance[7] = ODOM_UNOBSERVED_VARIANCE;
  odom_msg_.twist.covariance[35] = ODOM_ANGULAR_VARIANCE;
  for (size_t i : {14u, 21u, 28u}) {
    odom_msg_.pose.covariance[i] = ODOM_UNOBSERVED_VARIANCE;
    odom_msg_.twist.covariance[i] = ODOM_UNOBSERVED_VARIANCE;
  }
#if ESP32_SENSOR_OWNER
  setFrameId(imu_msg_.header.frame_id, kImuFrame);
  setFrameId(mag_msg_.header.frame_id, kImuFrame);

  // BatteryState carries two strings, location and serial_number. Left
  // zero-initialised their data pointer is NULL, and serialising a NULL string
  // dereferences it on the first publish. Point them at an empty string.
  setFrameId(battery_msg_.location, kEmpty);
  setFrameId(battery_msg_.serial_number, kEmpty);
#endif

  g_drive.stop();
  stale_command_ = true;
  led_command_received_ = false;
  syncTime();
  last_control_ms_ = millis();
  return true;
}

void RobotApp::destroyEntities() {
  // With the agent gone, each fini would otherwise wait for a reply that never
  // comes. A zero destroy-session timeout makes teardown immediate, so the
  // robot returns to waiting for the agent without a multi-second stall.
  rmw_context_t* context = rcl_context_get_rmw_context(&support_.context);
  if (context != nullptr) {
    (void)rmw_uros_set_context_entity_destroy_session_timeout(context, 0);
  }

  ignoreRclResult(rclc_executor_fini(&executor_));
  ignoreRclResult(rcl_subscription_fini(&cmd_vel_sub_, &node_));
  ignoreRclResult(rcl_subscription_fini(&led_sub_, &node_));
  ignoreRclResult(rcl_subscription_fini(&safety_fault_sub_, &node_));
  ignoreRclResult(rcl_publisher_fini(&odom_pub_, &node_));
#if ESP32_SENSOR_OWNER
  ignoreRclResult(rcl_publisher_fini(&imu_pub_, &node_));
  ignoreRclResult(rcl_publisher_fini(&light_pub_, &node_));
  ignoreRclResult(rcl_publisher_fini(&battery_pub_, &node_));
  ignoreRclResult(rcl_publisher_fini(&mag_pub_, &node_));
#endif
  ignoreRclResult(rcl_node_fini(&node_));
  ignoreRclResult(rclc_support_fini(&support_));
}

// -------------------------------------------------------------------- time

void RobotApp::syncTime() {
  // Without this the message stamps fall back to board uptime while the SBC
  // stamps wall clock, and sensor fusion on the SBC never converges.
  if (rmw_uros_sync_session(1000) == RMW_RET_OK) {
    time_offset_ns_ = rmw_uros_epoch_nanos() - (int64_t)esp_timer_get_time() * 1000LL;
    last_sync_ms_ = millis();
  }
}

int64_t RobotApp::nowNanos() const {
  return ((int64_t)esp_timer_get_time() * 1000LL) + time_offset_ns_;
}

// --------------------------------------------------------------- callbacks

void RobotApp::cmdVelCallback(const void* message) {
  const auto* twist = (const geometry_msgs__msg__Twist*)message;
  if (g_app.safety_fault_ || !isfinite(twist->linear.x) || !isfinite(twist->angular.z)) {
    g_drive.stop();
    g_app.stale_command_ = true;
    return;
  }
  // Store only. Dispatch happens in controlStep at a fixed rate; commanding
  // the motors from the callback lets a fast publisher saturate the loop.
  g_drive.setTwist((float)twist->linear.x, (float)twist->angular.z);
  g_app.last_cmd_ms_ = millis();
  g_app.stale_command_ = false;
}

void RobotApp::ledCallback(const void* message) {
  const auto* colour = (const std_msgs__msg__ColorRGBA*)message;
  if (!isfinite(colour->r) || !isfinite(colour->g) || !isfinite(colour->b)) {
    return;
  }
  const float channels[] = {colour->r, colour->g, colour->b};
  for (size_t i = 0; i < 3; ++i) {
    g_app.requested_rgb_[i] = (uint8_t)(constrain(channels[i], 0.0f, 1.0f) * STATUS_LED_MAX_LEVEL);
  }
  g_app.last_led_cmd_ms_ = millis();
  g_app.led_command_received_ = true;
}

void RobotApp::safetyFaultCallback(const void* message) {
  const bool fault = ((const std_msgs__msg__Bool*)message)->data;
  if (fault || fault != g_app.safety_fault_) {
    // Clearing a fault does not replay the old command; a fresh cmd_vel is required.
    g_drive.stop();
    g_app.stale_command_ = true;
  }
  g_app.safety_fault_ = fault;
}

void RobotApp::updateStatusLed() {
  const uint32_t now = millis();
  const bool flash = (now / 250u) % 2u == 0;
  uint8_t red = 0, green = 0, blue = 0;
  if (safety_fault_) {
    red = flash ? STATUS_LED_MAX_LEVEL : 0;
  } else if (agent_state_ != AgentState::kConnected) {
    blue = flash ? STATUS_LED_MAX_LEVEL : 0;
  } else if (stale_command_) {
    // Purple: no drive command. Amber looked like the race's yellow/orange.
    red = STATUS_LED_MAX_LEVEL / 2;
    blue = STATUS_LED_MAX_LEVEL / 2;
  } else if (led_command_received_ && now - last_led_cmd_ms_ <= LED_COMMAND_TIMEOUT_MS) {
    red = requested_rgb_[0];
    green = requested_rgb_[1];
    blue = requested_rgb_[2];
  } else {
    green = STATUS_LED_MAX_LEVEL;
  }
  const uint32_t rgb = ((uint32_t)red << 16) | ((uint32_t)green << 8) | blue;
  if (rgb != displayed_rgb_) {
    // A single chainable pixel; no blocking blink delay or writes on every spin.
    g_status_led.setColorRGB(0, red, green, blue);
    displayed_rgb_ = rgb;
  }
}

// ------------------------------------------------------------------ stepping

void RobotApp::controlStep() {
  const uint32_t now = millis();
  const float dt_s = (float)(now - last_control_ms_) / 1000.0f;
  last_control_ms_ = now;

  if (safety_fault_ || (now - last_cmd_ms_) > CMD_TIMEOUT_MS) {
    if (!stale_command_) {
      stale_command_ = true;
      g_drive.stop();
    }
    g_drive.setTwist(0.0f, 0.0f);
  }

  g_drive.update(dt_s);
}

void RobotApp::publishOdometry() {
  const int64_t stamp = nowNanos();
  odom_msg_.header.stamp.sec = (int32_t)(stamp / 1000000000LL);
  odom_msg_.header.stamp.nanosec = (uint32_t)(stamp % 1000000000LL);

  odom_msg_.pose.pose.position.x = g_drive.x();
  odom_msg_.pose.pose.position.y = g_drive.y();
  odom_msg_.pose.pose.position.z = 0.0;

  const float half_yaw = g_drive.yaw() * 0.5f;
  odom_msg_.pose.pose.orientation.z = sinf(half_yaw);
  odom_msg_.pose.pose.orientation.w = cosf(half_yaw);

  odom_msg_.twist.twist.linear.x = g_drive.measuredLinear();
  odom_msg_.twist.twist.angular.z = g_drive.measuredAngular();

  ignoreRclResult(rcl_publish(&odom_pub_, &odom_msg_, NULL));
}

#if ESP32_SENSOR_OWNER
void RobotApp::publishImu() {
  if (!imu_available_) {
    const uint32_t now = millis();
    if (now - last_imu_retry_ms_ >= 2000) {
      last_imu_retry_ms_ = now;
      imu_available_ = initImu();
    }
    return;
  }
  // A short or failed read must not be published. Leaving the unread bytes at
  // zero produces a sample that looks like weightlessness, which the estimator
  // would fuse as real data. Skip the cycle instead; the next one is 20 ms away.
  uint8_t raw[12] = {0};
  if (!readRegisters(QMI8658_ADDRESS, 0x35, raw, 12)) {
    return;
  }


  constexpr float kAccelScale = 9.80665f / 4096.0f;   // +/- 8 g
  // +/-1024 dps full scale, as written to CTRL3 in initImu: 32768 / 1024 is
  // exactly 32 LSB per dps. The QMI8658C ranges are powers of two, so the
  // 32.8 used for +/-1000 dps parts elsewhere would be wrong here by 2.5 %.
  constexpr float kGyroScale = 0.017453293f / 32.0f;

  const int64_t stamp = nowNanos();
  imu_msg_.header.stamp.sec = (int32_t)(stamp / 1000000000LL);
  imu_msg_.header.stamp.nanosec = (uint32_t)(stamp % 1000000000LL);

  // Sensor channels in their own frame, before remapping.
  const float accel_raw[3] = {le16(raw, 0) * kAccelScale,
                              le16(raw, 2) * kAccelScale,
                              le16(raw, 4) * kAccelScale};
  const float gyro_raw[3] = {le16(raw, 6) * kGyroScale,
                             le16(raw, 8) * kGyroScale,
                             le16(raw, 10) * kGyroScale};

  // Remap into the REP-103 body frame: x forward, y left, z up. Doing this in
  // firmware rather than on the SBC means every consumer of the topic sees a
  // correct frame, including tools that are not part of this project.
  const float gyro_bias[3] = {IMU_GYRO_BIAS_X, IMU_GYRO_BIAS_Y, IMU_GYRO_BIAS_Z};

  imu_msg_.linear_acceleration.x = IMU_X_SIGN * accel_raw[IMU_X_SOURCE];
  imu_msg_.linear_acceleration.y = IMU_Y_SIGN * accel_raw[IMU_Y_SOURCE];
  imu_msg_.linear_acceleration.z = IMU_Z_SIGN * accel_raw[IMU_Z_SOURCE];

  imu_msg_.angular_velocity.x =
      (IMU_GYRO_SIGN * IMU_X_SIGN * gyro_raw[IMU_X_SOURCE]) - gyro_bias[0];
  imu_msg_.angular_velocity.y =
      (IMU_GYRO_SIGN * IMU_Y_SIGN * gyro_raw[IMU_Y_SOURCE]) - gyro_bias[1];
  imu_msg_.angular_velocity.z =
      (IMU_GYRO_SIGN * IMU_Z_SIGN * gyro_raw[IMU_Z_SOURCE]) - gyro_bias[2];

  // No orientation estimate is produced on the MCU; -1 marks it unavailable.
  imu_msg_.orientation_covariance[0] = -1.0;

  ignoreRclResult(rcl_publish(&imu_pub_, &imu_msg_, NULL));
}

void RobotApp::publishLight() {
  const uint16_t ir = readLtr303Word(0x88);
  const uint16_t visible = readLtr303Word(0x8A);
  light_msg_.data = (uint32_t)(visible > ir ? (visible - ir) : 0);
  ignoreRclResult(rcl_publish(&light_pub_, &light_msg_, NULL));
}

bool RobotApp::initImu() {
  // Identity check first: a wrong address or a missing device should be
  // reported, not silently read as zeros that look like a level robot.
  uint8_t who = 0;
  if (!readRegisters(QMI8658_ADDRESS, 0x00, &who, 1) ||
      who != QMI8658_WHO_AM_I_VALUE) {
    return false;
  }

  // Soft reset first. Resetting the ESP32 does not power-cycle the IMU, so the
  // chip otherwise keeps whatever state the previous firmware left it in. The
  // standalone I2C diagnostic resets it and reads correct data; this makes the
  // robot firmware start from the same known state.
  writeRegister(QMI8658_ADDRESS, 0x60, 0xB0);
  delay(50);

  // CTRL1: register address auto-increment, little-endian data. Without
  // auto-increment the 12-byte burst read is not guaranteed to walk the
  // registers, and may return the same byte repeatedly.
  writeRegister(QMI8658_ADDRESS, 0x02, 0x40);

  // CTRL2: accelerometer +/-8 g (full-scale code 010), output data rate code
  // 6. CTRL3: gyroscope +/-1024 dps (code 110), rate code 6. Both MUST be set:
  // the power-on defaults are +/-2 g and +/-16 dps, and the scale factors used
  // when publishing assume the ranges written here. Leaving them at default
  // makes acceleration read four times too high and rate about sixty times.
  //
  // Verify the rate code against the output data rate table of the QMI8658C
  // datasheet; it only needs to exceed the 50 Hz publishing rate.
  writeRegister(QMI8658_ADDRESS, 0x03, 0x26);
  writeRegister(QMI8658_ADDRESS, 0x04, 0x66);

  // CTRL7: enable accelerometer and gyroscope.
  writeRegister(QMI8658_ADDRESS, 0x08, 0x03);
  delay(50);

  // Read the configuration back. A write that did not take would otherwise
  // leave the sensor at its defaults while the scale factors assume these
  // ranges, and the published values would be wrong without any error.
  //
  // Only the fields this firmware depends on are compared. Reserved bits may
  // read back differently from what was written, and a byte-exact comparison
  // would then reject a correctly configured sensor and silence it entirely.
  //   CTRL1 bits 6..5: address auto-increment set, big-endian clear
  //   CTRL2 bits 6..0: +/-8 g range and rate code (bit 7 is self-test)
  //   CTRL3 bits 6..0: +/-1024 dps range and rate code (bit 7 is self-test)
  //   CTRL7 bits 1..0: accelerometer and gyroscope enabled
  uint8_t ctrl[3] = {0};
  if (!readRegisters(QMI8658_ADDRESS, 0x02, ctrl, 3)) {
    return false;
  }
  uint8_t ctrl7 = 0;
  if (!readRegisters(QMI8658_ADDRESS, 0x08, &ctrl7, 1)) {
    return false;
  }
  return (ctrl[0] & 0x60) == 0x40 &&
         (ctrl[1] & 0x7F) == 0x26 &&
         (ctrl[2] & 0x7F) == 0x66 &&
         (ctrl7 & 0x03) == 0x03;
}

bool RobotApp::initMagnetometer() {
  uint8_t wia2 = 0;
  if (!readRegisters(AK09918_ADDRESS, 0x01, &wia2, 1) ||
      wia2 != AK09918_WIA2_VALUE) {
    return false;
  }
  writeRegister(AK09918_ADDRESS, 0x32, 0x01);   // CNTL3: soft reset
  delay(20);                                    // as in the working diagnostic
  writeRegister(AK09918_ADDRESS, 0x31, 0x08);   // CNTL2: continuous, 100 Hz
  return true;
}

void RobotApp::publishMagnetometer() {
  // A failed start-up is retried rather than accepted. Initialising once at
  // boot and giving up left the compass silent until the next reset whenever
  // the first attempt lost a race with the rest of the bus coming up.
  if (!mag_available_) {
    const uint32_t now = millis();
    if (now - last_mag_retry_ms_ >= 2000) {
      last_mag_retry_ms_ = now;
      mag_available_ = initMagnetometer();
    }
    return;
  }

  uint8_t status = 0;
  if (!readRegisters(AK09918_ADDRESS, 0x10, &status, 1) || (status & 0x01) == 0) {
    return;                              // no new sample yet
  }

  // Read the six data bytes, the temperature placeholder and ST2 in one burst.
  // Reading ST2 is not optional: the device holds the data registers until it
  // is read, and further samples are discarded in the meantime.
  uint8_t raw[8] = {0};
  if (!readRegisters(AK09918_ADDRESS, 0x11, raw, 8)) {
    return;
  }
  if (raw[7] & 0x08) {
    return;                              // ST2.HOFL: magnetic sensor overflow
  }

  constexpr float kMicroteslaPerLsb = 0.15f;
  const float ut[3] = {le16(raw, 0) * kMicroteslaPerLsb,
                       le16(raw, 2) * kMicroteslaPerLsb,
                       le16(raw, 4) * kMicroteslaPerLsb};
  const float offset[3] = {MAG_OFFSET_X_UT, MAG_OFFSET_Y_UT, MAG_OFFSET_Z_UT};

  const int64_t stamp = nowNanos();
  mag_msg_.header.stamp.sec = (int32_t)(stamp / 1000000000LL);
  mag_msg_.header.stamp.nanosec = (uint32_t)(stamp % 1000000000LL);

  // Remap into REP-103, subtract the hard-iron offset, convert to tesla.
  mag_msg_.magnetic_field.x = (MAG_X_SIGN * ut[MAG_X_SOURCE] - offset[0]) * 1e-6;
  mag_msg_.magnetic_field.y = (MAG_Y_SIGN * ut[MAG_Y_SOURCE] - offset[1]) * 1e-6;
  mag_msg_.magnetic_field.z = (MAG_Z_SIGN * ut[MAG_Z_SOURCE] - offset[2]) * 1e-6;

  ignoreRclResult(rcl_publish(&mag_pub_, &mag_msg_, NULL));
}

void RobotApp::publishBattery() {
  if (!battery_available_) {
    return;
  }

  // Bus voltage is measured after the shunt, so the pack voltage is the bus
  // voltage plus the drop across the shunt. Ignoring the shunt term
  // under-reports the pack by a few tens of millivolts at high current, which
  // matters when the reading is being compared against a discharge floor.
  const float bus_v = g_ina219.getBusVoltage_V();
  const float shunt_mv = g_ina219.getShuntVoltage_mV();
  const float pack_v = bus_v + (shunt_mv / 1000.0f);
  const float current_a = g_ina219.getCurrent_mA() / 1000.0f;

  const float floor_v = BATTERY_CELLS * CELL_FLOOR_V;
  const float full_v = BATTERY_CELLS * CELL_FULL_V;
  float fraction = (pack_v - floor_v) / (full_v - floor_v);
  fraction = constrain(fraction, 0.0f, 1.0f);

  const int64_t stamp = nowNanos();
  battery_msg_.header.stamp.sec = (int32_t)(stamp / 1000000000LL);
  battery_msg_.header.stamp.nanosec = (uint32_t)(stamp % 1000000000LL);

  battery_msg_.voltage = pack_v;
  // REP-147 sign convention: negative current means discharging.
  battery_msg_.current = -current_a;
  battery_msg_.percentage = fraction;
  battery_msg_.power_supply_technology =
      sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LIPO;
  battery_msg_.present = true;

  battery_msg_.power_supply_status = (uint8_t)(
      (current_a > 0.05f)
          ? (uint8_t)sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING
          : (uint8_t)sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_NOT_CHARGING);

  if (pack_v <= floor_v) {
    battery_msg_.power_supply_health =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_DEAD;
    if (!low_voltage_warned_) {
      low_voltage_warned_ = true;
      // The supervisory stop is left to the SBC: cutting drive in firmware on
      // a voltage sag would stop the robot mid-run on a transient.
    }
  } else {
    battery_msg_.power_supply_health =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_GOOD;
  }

  ignoreRclResult(rcl_publish(&battery_pub_, &battery_msg_, NULL));
}
#endif

// ------------------------------------------------------------------- spin

void RobotApp::spin() {
  const uint32_t now = millis();

  switch (agent_state_) {
    case AgentState::kWaiting:
      if ((now - last_ping_ms_) > AGENT_PING_PERIOD_MS) {
        last_ping_ms_ = now;
        agent_state_ = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
                           ? AgentState::kAvailable
                           : AgentState::kWaiting;
      }
      break;

    case AgentState::kAvailable:
      agent_state_ = createEntities() ? AgentState::kConnected
                                      : AgentState::kWaiting;
      if (agent_state_ == AgentState::kWaiting) {
        destroyEntities();
      }
      break;

    case AgentState::kConnected:
      if ((now - last_ping_ms_) > AGENT_PING_PERIOD_MS) {
        last_ping_ms_ = now;
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
          g_drive.stop();
          stale_command_ = true;
          agent_state_ = AgentState::kDisconnected;
          break;
        }
      }

      rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));

      if ((now - last_control_ms_) >= CONTROL_PERIOD_MS) {
        controlStep();
      }
      if ((now - last_odom_ms_) >= ODOM_PUBLISH_PERIOD_MS) {
        // Advance by the period, not to `now`: with `= now` every late loop
        // pass (a slow spin or publish) pushed the whole schedule back, so the
        // rate sagged below 20 Hz while driving. After a long stall, resync
        // instead of publishing a burst to catch up.
        last_odom_ms_ += ODOM_PUBLISH_PERIOD_MS;
        if ((now - last_odom_ms_) >= ODOM_PUBLISH_PERIOD_MS) {
          last_odom_ms_ = now;
        }
        publishOdometry();
      }
#if ESP32_SENSOR_OWNER
      if ((now - last_imu_ms_) >= IMU_PUBLISH_PERIOD_MS) {
        last_imu_ms_ = now;
        publishImu();
      }
      if ((now - last_light_ms_) >= 100) {
        last_light_ms_ = now;
        publishLight();
      }
      if ((now - last_mag_ms_) >= MAG_PUBLISH_PERIOD_MS) {
        last_mag_ms_ = now;
        publishMagnetometer();
      }
      if ((now - last_battery_ms_) >= BATTERY_PUBLISH_PERIOD_MS) {
        last_battery_ms_ = now;
        publishBattery();
      }
#endif
      if ((now - last_sync_ms_) >= TIME_SYNC_PERIOD_MS) {
        syncTime();
      }
      break;

    case AgentState::kDisconnected:
      // Losing the agent must stop the robot, not leave it at the last speed.
      g_drive.stop();
      stale_command_ = true;
      led_command_received_ = false;
      destroyEntities();
      agent_state_ = AgentState::kWaiting;
      break;
  }
  updateStatusLed();
}
