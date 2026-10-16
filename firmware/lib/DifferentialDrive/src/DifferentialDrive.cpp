// References (full entries: docs/references.md at the repository root):
//   [R42] Siegwart et al., Autonomous Mobile Robots: differential-drive kinematics and odometry
// More details: firmware/README.md

#include "DifferentialDrive.h"

#include <math.h>

DifferentialDrive::DifferentialDrive(MotorDriver& left, MotorDriver& right)
    : left_(left), right_(right) {}

void DifferentialDrive::begin() {
  left_.begin();
  right_.begin();
}

void DifferentialDrive::setTwist(float linear_x, float angular_z) {
  const float half_track = TRACK_WIDTH_M * 0.5f;

  // Wheel linear speeds, then angular speeds at the wheel.
  const float left_mps  = linear_x - (angular_z * half_track);
  const float right_mps = linear_x + (angular_z * half_track);

  left_.setTargetSpeed(left_mps / LEFT_WHEEL_RADIUS_M);
  right_.setTargetSpeed(right_mps / RIGHT_WHEEL_RADIUS_M);
}

void DifferentialDrive::update(float dt_s) {
  left_.update(dt_s);
  right_.update(dt_s);

  const float left_mps  = left_.measuredSpeed() * LEFT_WHEEL_RADIUS_M;
  const float right_mps = right_.measuredSpeed() * RIGHT_WHEEL_RADIUS_M;

  measured_linear_  = (right_mps + left_mps) * 0.5f;
  measured_angular_ = (right_mps - left_mps) / TRACK_WIDTH_M;

  // Midpoint integration of the unicycle model.
  const float delta_yaw = measured_angular_ * dt_s;
  const float mid_yaw = yaw_ + (delta_yaw * 0.5f);
  x_ += measured_linear_ * cosf(mid_yaw) * dt_s;
  y_ += measured_linear_ * sinf(mid_yaw) * dt_s;
  yaw_ += delta_yaw;

  while (yaw_ > (float)M_PI)  { yaw_ -= 2.0f * (float)M_PI; }
  while (yaw_ < -(float)M_PI) { yaw_ += 2.0f * (float)M_PI; }
}

void DifferentialDrive::stop() {
  left_.stop();
  right_.stop();
  measured_linear_ = 0.0f;
  measured_angular_ = 0.0f;
}

void DifferentialDrive::resetPose() {
  x_ = y_ = yaw_ = 0.0f;
}
