// References (full entries: docs/references.md at the repository root):
//   [R40] Astrom & Murray, Feedback Systems (PID): PID
//   [R61] TB6612FNG motor driver datasheet: TB6612FNG
// More details: MotorDriver.cpp

#ifndef L2R_A_MOTOR_DRIVER_H
#define L2R_A_MOTOR_DRIVER_H

#include <Arduino.h>

#include "RobotConfig.h"

/// One brushed DC motor: H-bridge output, quadrature encoder input, and a
/// velocity PID closed on measured wheel speed.
///
/// The duty cycle is clamped to MAX_PWM_DUTY, which protects the H-bridge from
/// stall current. The motor itself is rated for 12 V, so a 3S pack is within
/// its specification.
class MotorDriver {
 public:
  /// motor_sign maps a positive (forward) command onto the H-bridge polarity.
  /// encoder_sign maps the encoder count onto forward-positive wheel speed.
  /// They are independent because the motor leads and the encoder channels can
  /// each be wired either way round. Both must be right: a wrong motor sign
  /// drives the wheel backwards, and a wrong encoder sign makes the speed
  /// controller push harder as the wheel goes the wrong way, so it runs away.
  MotorDriver(uint8_t pwm_pin, uint8_t in1_pin, uint8_t in2_pin,
              uint8_t pwm_channel, int8_t motor_sign, int8_t encoder_sign);

  void begin();

  /// Set the target wheel speed in rad/s at the output shaft.
  void setTargetSpeed(float rad_per_s);

  /// Run one PID step. dt_s must be the actual elapsed time.
  void update(float dt_s);

  /// Cut drive immediately and reset the integrator (safe stop path).
  void stop();

  /// Short-brake the motor (TB6612FNG IN1 = IN2 = HIGH) and reset the PID.
  void brake();

  float measuredSpeed() const { return measured_rad_s_; }
  float appliedTarget() const { return applied_rad_s_; }
  long  encoderCount()  const { return count_; }

  /// Called from both encoder channel ISRs.
  ///
  /// Full quadrature decoding: every edge of both channels is counted, and the
  /// direction comes from comparing the two channel states rather than from
  /// sampling one pin. Counting a single edge of a single channel, as the
  /// original firmware did, discards three quarters of the available
  /// resolution and infers direction from a pin that may already have changed.
  void IRAM_ATTR onEncoderEdge(bool channel_a_level, bool channel_b_level);

 private:
  void applyDuty(float duty);
  float slewLimit(float target, float dt_s);

  const uint8_t pwm_pin_, in1_pin_, in2_pin_, pwm_channel_;
  const int8_t  direction_sign_;   // motor polarity
  const int8_t  encoder_sign_;     // encoder count direction

  volatile long count_ = 0;
  long          last_count_ = 0;
  volatile uint8_t last_state_ = 0;   // previous (A,B) pair, for quadrature

  float target_rad_s_   = 0.0f;   // requested
  float applied_rad_s_  = 0.0f;   // after slew limiting
  float measured_rad_s_ = 0.0f;
  float integral_       = 0.0f;
  float previous_error_ = 0.0f;
  float previous_commanded_ = 0.0f;
};

#endif  // L2R_A_MOTOR_DRIVER_H
