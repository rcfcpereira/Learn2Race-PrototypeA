// References (full entries: docs/references.md at the repository root):
//   [R14] micro-ROS / micro_ros_arduino: micro-ROS
// More details: RobotApp.cpp; firmware/README.md

#ifndef L2R_A_ROBOT_APP_H
#define L2R_A_ROBOT_APP_H

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/color_rgba.h>

#include "DifferentialDrive.h"
#include "MotorDriver.h"
#include "RobotConfig.h"

/// micro-ROS application for Prototype A.
///
/// Mirrors the structure used on Prototype B so that both robots present the
/// same interface to the SBC:
///
///   subscribes  /cmd_vel                     geometry_msgs/Twist
///               /leds/set_color              std_msgs/ColorRGBA
///               /safety/fault                std_msgs/Bool
///   publishes   /odom                        nav_msgs/Odometry
///               /imu/data_raw                sensor_msgs/Imu           (QMI8658C)
///               /imu/mag                     sensor_msgs/MagneticField (AK09918C)
///               /light/visible_plus_ir_raw   std_msgs/UInt32           (LTR-303)
///               /battery_state               sensor_msgs/BatteryState  (INA219)
///
/// All sensor publishers are omitted when ESP32_SENSOR_OWNER=0. The SBC must
/// then be wired to the sensor bus and provide those messages itself.
///
/// Two defects found in the original firmware are fixed here:
///   * the gear ratio was 50.0 for an i29 motor, so odometry read short;
///   * there was no command timeout, so loss of the agent left the motors
///     running at their last commanded speed.
class RobotApp {
 public:
  bool begin();
  void spin();

 private:
  enum class AgentState { kWaiting, kAvailable, kConnected, kDisconnected };

  bool createEntities();
  void destroyEntities();
  void syncTime();

  void controlStep();
  void publishOdometry();
  void publishImu();
  void publishLight();
  void publishBattery();
  void publishMagnetometer();
  void updateStatusLed();
  bool initImu();
  bool initMagnetometer();

  static void cmdVelCallback(const void* message);
  static void ledCallback(const void* message);
  static void safetyFaultCallback(const void* message);

  int64_t nowNanos() const;

  AgentState agent_state_ = AgentState::kWaiting;

  rcl_allocator_t allocator_;
  rclc_support_t support_;
  rcl_node_t node_;
  rclc_executor_t executor_;

  rcl_subscription_t cmd_vel_sub_;
  rcl_subscription_t led_sub_;
  rcl_subscription_t safety_fault_sub_;
  rcl_publisher_t odom_pub_;
  rcl_publisher_t imu_pub_;
  rcl_publisher_t light_pub_;
  rcl_publisher_t battery_pub_;
  rcl_publisher_t mag_pub_;

  geometry_msgs__msg__Twist cmd_vel_msg_;
  std_msgs__msg__ColorRGBA led_msg_;
  std_msgs__msg__Bool safety_fault_msg_;
  nav_msgs__msg__Odometry odom_msg_;
  sensor_msgs__msg__Imu imu_msg_;
  std_msgs__msg__UInt32 light_msg_;
  sensor_msgs__msg__BatteryState battery_msg_;
  sensor_msgs__msg__MagneticField mag_msg_;

  uint32_t last_cmd_ms_ = 0;
  uint32_t last_control_ms_ = 0;
  uint32_t last_odom_ms_ = 0;
  uint32_t last_imu_ms_ = 0;
  uint32_t last_light_ms_ = 0;
  uint32_t last_battery_ms_ = 0;
  uint32_t last_mag_ms_ = 0;
  uint32_t last_mag_retry_ms_ = 0;
  uint32_t last_imu_retry_ms_ = 0;
  uint32_t last_ping_ms_ = 0;
  uint32_t last_sync_ms_ = 0;
  uint32_t last_led_cmd_ms_ = 0;
  uint32_t displayed_rgb_ = 0xFFFFFFFF;
  uint8_t requested_rgb_[3] = {0, 0, 0};

  bool stale_command_ = true;
  bool led_command_received_ = false;
  bool safety_fault_ = false;
  bool battery_available_ = false;
  bool imu_available_ = false;
  bool mag_available_ = false;
  bool low_voltage_warned_ = false;
  int64_t time_offset_ns_ = 0;
};

extern RobotApp g_app;

#endif  // L2R_A_ROBOT_APP_H
