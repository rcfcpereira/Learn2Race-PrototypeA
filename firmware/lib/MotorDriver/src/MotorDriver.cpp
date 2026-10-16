// References (full entries: docs/references.md at the repository root):
//   [R40] Astrom & Murray, Feedback Systems (PID): feed-forward + PID, anti-windup, setpoint slew limit
//   [R61] TB6612FNG motor driver datasheet: TB6612FNG direction pins and short brake
//   [R62] CHP-20GP-180ABHL motor datasheet: encoder resolution
//   [R63] ESP32 TRM / Arduino-ESP32 / PlatformIO: LEDC PWM
// More details: firmware/README.md

#include "MotorDriver.h"

namespace {
constexpr float kTwoPi = 6.28318530718f;
}  // namespace

MotorDriver::MotorDriver(uint8_t pwm_pin, uint8_t in1_pin, uint8_t in2_pin,
                         uint8_t pwm_channel, int8_t motor_sign, int8_t encoder_sign)
    : pwm_pin_(pwm_pin),
      in1_pin_(in1_pin),
      in2_pin_(in2_pin),
      pwm_channel_(pwm_channel),
      direction_sign_(motor_sign),
      encoder_sign_(encoder_sign) {}

void MotorDriver::begin() {
  pinMode(in1_pin_, OUTPUT);
  pinMode(in2_pin_, OUTPUT);
  ledcSetup(pwm_channel_, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(pwm_pin_, pwm_channel_);
  stop();
}

void IRAM_ATTR MotorDriver::onEncoderEdge(bool channel_a_level,
                                          bool channel_b_level) {
  // Gray-code state machine over the two channels. The lookup gives +1, -1 or
  // 0 for every transition; 0 covers the illegal double transition produced by
  // a missed interrupt, which is counted as no movement rather than as a jump.
  // DRAM_ATTR keeps the table in RAM. A const table otherwise lands in flash,
  // and an ISR that reads flash while the cache is disabled, for instance
  // during a flash write, crashes the ESP32.
  static const int8_t DRAM_ATTR kTransition[16] = {
      0, -1, +1,  0,
     +1,  0,  0, -1,
     -1,  0,  0, +1,
      0, +1, -1,  0};

  const uint8_t state = (uint8_t)((channel_a_level ? 2 : 0) |
                                  (channel_b_level ? 1 : 0));
  count_ += kTransition[(last_state_ << 2) | state];
  last_state_ = state;
}

void MotorDriver::setTargetSpeed(float rad_per_s) {
  target_rad_s_ = rad_per_s;
}

float MotorDriver::slewLimit(float target, float dt_s) {
  // Limit how fast the setpoint may change. Two reasons: an instantaneous
  // full-scale step draws near-stall current (up to 2.7 A at 12 V per the
  // datasheet) through the H-bridge, and a step input through a planetary
  // gearbox shock-loads the teeth. The limit costs a few tens of milliseconds
  // of response and removes both.
  const float max_delta = MAX_ACCEL_RAD_S2 * dt_s;
  const float delta = target - applied_rad_s_;
  if (delta > max_delta) {
    applied_rad_s_ += max_delta;
  } else if (delta < -max_delta) {
    applied_rad_s_ -= max_delta;
  } else {
    applied_rad_s_ = target;
  }
  return applied_rad_s_;
}

void MotorDriver::update(float dt_s) {
  if (dt_s <= 0.0f) {
    return;
  }

#if BRAKE_ON_ZERO
  if (target_rad_s_ == 0.0f) {
    brake();
  }
#endif
  const float commanded = slewLimit(target_rad_s_, dt_s);

  const long count = count_;
  const long delta = count - last_count_;
  last_count_ = count;

  // Encoder counts -> output shaft rad/s.
  const float revolutions = (float)delta / COUNTS_PER_OUTPUT_REV;
  // Convert the count into the same forward-positive frame as the target.
  // Without this, a mirrored wheel reports forward motion as negative, the
  // controller sees the error grow as the wheel speeds up, and it runs away.
  measured_rad_s_ = encoder_sign_ * (revolutions * kTwoPi) / dt_s;

  if (commanded == 0.0f && target_rad_s_ == 0.0f) {
    // Explicit zero means stop, not "hold zero with integral wind-up".
    integral_ = 0.0f;
    previous_error_ = 0.0f;
#if !BRAKE_ON_ZERO
    applyDuty(0.0f);
#endif
    return;
  }

  const float error = commanded - measured_rad_s_;

#if RESET_INTEGRAL_ON_REVERSAL
  // The integral holds the friction offset for the current direction. When
  // the setpoint changes sign it points the wrong way and, at KI = 8, takes
  // ~0.4 s to unwind: the inner wheel kept rolling forward in the Suzuka
  // hairpin when it should have reversed.
  if ((commanded > 0.0f && previous_commanded_ < 0.0f) ||
      (commanded < 0.0f && previous_commanded_ > 0.0f)) {
    integral_ = 0.0f;
  }
  previous_commanded_ = commanded;
#endif

  // Anti-windup: cap the integral by what it may contribute in duty, rather
  // than by a fixed figure in error-seconds. A constant limit would mean
  // something different for every gain value, and would either saturate
  // instantly or never take effect.
  const float integral_limit = (float)MAX_PWM_DUTY / max(PID_KI, 0.001f);
  integral_ += error * dt_s;
  integral_ = constrain(integral_, -integral_limit, integral_limit);

  const float derivative = (error - previous_error_) / dt_s;
  previous_error_ = error;

  // Feedforward carries the commanded speed; the PID corrects load, friction
  // and the difference between the two motors.
  const float feedforward = PID_KFF * commanded;
  const float output = feedforward + (PID_KP * error) + (PID_KI * integral_) +
                       (PID_KD * derivative);
  applyDuty(output);
}

void MotorDriver::applyDuty(float duty) {
  const bool forward = (duty * direction_sign_) >= 0.0f;
  float magnitude = fabsf(duty);

  if (magnitude < 1.0f) {
    digitalWrite(in1_pin_, LOW);
    digitalWrite(in2_pin_, LOW);
    ledcWrite(pwm_channel_, 0);
    return;
  }

  magnitude = constrain(magnitude, (float)MIN_PWM_DUTY, (float)MAX_PWM_DUTY);

  digitalWrite(in1_pin_, forward ? HIGH : LOW);
  digitalWrite(in2_pin_, forward ? LOW : HIGH);
  ledcWrite(pwm_channel_, (uint32_t)magnitude);
}

void MotorDriver::stop() {
  target_rad_s_ = 0.0f;
  // Safe stop bypasses the slew limiter: a stop must be immediate.
#if BRAKE_ON_ZERO
  brake();
#else
  applied_rad_s_ = 0.0f;
  integral_ = 0.0f;
  previous_error_ = 0.0f;
  digitalWrite(in1_pin_, LOW);
  digitalWrite(in2_pin_, LOW);
  ledcWrite(pwm_channel_, 0);
#endif
}

void MotorDriver::brake() {
  // TB6612FNG short brake: both inputs HIGH; PWM does not matter.
  applied_rad_s_ = 0.0f;
  integral_ = 0.0f;
  previous_error_ = 0.0f;
  ledcWrite(pwm_channel_, 0);
  digitalWrite(in1_pin_, HIGH);
  digitalWrite(in2_pin_, HIGH);
}
