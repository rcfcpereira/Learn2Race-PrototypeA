// References (full entries: docs/references.md at the repository root):
//   [R42] Siegwart et al., Autonomous Mobile Robots: kinematics
// More details: DifferentialDrive.cpp

#ifndef L2R_A_DIFFERENTIAL_DRIVE_H
#define L2R_A_DIFFERENTIAL_DRIVE_H

#include "MotorDriver.h"
#include "RobotConfig.h"

/// Differential-drive kinematics and dead-reckoned pose.
///
/// Converts a body twist into per-wheel angular velocities, and integrates the
/// measured wheel speeds back into a pose. Wheel radius and track width are the
/// two constants that dominate odometry accuracy; both are build flags.
class DifferentialDrive {
 public:
  DifferentialDrive(MotorDriver& left, MotorDriver& right);

  void begin();

  /// Command a body twist (m/s, rad/s).
  void setTwist(float linear_x, float angular_z);

  /// Run one control step and integrate odometry.
  void update(float dt_s);

  /// Zero both wheels and clear the controllers.
  void stop();

  float x() const { return x_; }
  float y() const { return y_; }
  float yaw() const { return yaw_; }
  float measuredLinear() const { return measured_linear_; }
  float measuredAngular() const { return measured_angular_; }

  void resetPose();

 private:
  MotorDriver& left_;
  MotorDriver& right_;

  float x_ = 0.0f, y_ = 0.0f, yaw_ = 0.0f;
  float measured_linear_ = 0.0f, measured_angular_ = 0.0f;
};

#endif  // L2R_A_DIFFERENTIAL_DRIVE_H
