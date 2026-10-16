// References (full entries: docs/references.md at the repository root):
//   [R62] CHP-20GP-180ABHL motor datasheet: motor and encoder
//   [R63] ESP32 TRM / Arduino-ESP32 / PlatformIO: PCNT/LEDC
// More details: firmware/README.md

// Basic wheel test for Prototype A (Waveshare General Driver for Robots).
//
// Nothing but the wheels: no micro-ROS, no IMU, no speed controller. Keys typed
// in the serial monitor apply raw PWM to the motors, and the raw encoder counts
// are printed back. With no controller in the loop, what you see is exactly what
// the wiring does, which is what makes the direction signs unambiguous.
//
//   pio run -e wheels -t upload
//   pio device monitor -e wheels
//
// LIFT THE ROBOT so the wheels spin freely.
//
// Keys refer to the PHYSICAL wheels and to real forward motion. The board's
// channel and sign mapping, measured earlier with this same test, is applied
// here, so this program now checks exactly the constants the robot firmware
// uses: if a wheel turns the wrong way here, it will there too.
//
//   1  LEFT wheel forward      w  both forward
//   2  RIGHT wheel forward     s  both backward
//   3  LEFT wheel backward     a  turn left   (left back, right forward)
//   4  RIGHT wheel backward    d  turn right  (left forward, right back)
//   +  more PWM   -  less PWM  x  or space: stop
//   z  zero the counts
//   r  raw mode: ignore the mapping and drive board channels A and B directly,
//      for re-measuring the signs if the robot is ever rewired.
//
// Counts are shown in the robot's frame: driving a wheel forward makes its
// count go UP. In raw mode they are shown exactly as the encoder produces them.
//
// The motors stop automatically 3 s after the last key, so a lost connection or
// a forgotten key cannot leave them running.

#include <Arduino.h>

// ---- pins, from the original Learn2Race_1 firmware --------------------------
// Board channels. Channel A drives the physical RIGHT wheel and channel B the
// physical LEFT wheel: the board labels do not follow the robot's sides, which
// is what the first run of this test established.
static const int PWMA = 25, AIN1 = 21, AIN2 = 17;   // channel A
static const int PWMB = 26, BIN1 = 22, BIN2 = 23;   // channel B
static const int ENC_A_1 = 35, ENC_A_2 = 34;        // encoder on channel A
static const int ENC_B_1 = 27, ENC_B_2 = 16;        // encoder on channel B

// Measured mapping and signs, the same values the robot firmware is built with.
// LEFT is channel B, RIGHT is channel A, and on both wheels a "forward" bridge
// command turns the wheel backwards, while the count falls as the wheel turns
// physically forward.
static const bool LEFT_IS_CHANNEL_A = false;
static const int  LEFT_MOTOR_SIGN = -1, LEFT_ENCODER_SIGN = -1;
static const int  RIGHT_MOTOR_SIGN = -1, RIGHT_ENCODER_SIGN = -1;

static const int CH_A = 5, CH_B = 6;
static const int PWM_FREQ = 20000, PWM_BITS = 8;

// The motor is rated for 12 V, so the cap protects the H-bridge from stall
// current rather than the motor from voltage. Matches MAX_PWM_DUTY.
static const int DUTY_MAX = 200;
static const int DUTY_STEP = 10;

// CHP-20GP-180ABHL: 11 pulses per motor revolution, 1:29 gearbox, and four
// counts per pulse with full quadrature decoding. Measured by hand-turning each
// wheel one revolution: 1283 and 1306 counts.
static const float COUNTS_PER_WHEEL_REV = 320.76f * 4.0f;  // 1283.04, datasheet

static const uint32_t AUTO_STOP_MS = 3000;
static const uint32_t PRINT_MS = 250;

// ---- encoders ---------------------------------------------------------------
static volatile long g_count_a = 0, g_count_b = 0;   // raw, per board channel
static volatile uint8_t g_state_a = 0, g_state_b = 0;

// Gray-code transition table: +1, -1, or 0 for an illegal double step.
static const int8_t DRAM_ATTR kStep[16] = {
    0, -1, +1,  0,
   +1,  0,  0, -1,
   -1,  0,  0, +1,
    0, +1, -1,  0};

void IRAM_ATTR isrChannelA() {
  uint8_t s = (digitalRead(ENC_A_1) << 1) | digitalRead(ENC_A_2);
  g_count_a += kStep[(g_state_a << 2) | s];
  g_state_a = s;
}

void IRAM_ATTR isrChannelB() {
  uint8_t s = (digitalRead(ENC_B_1) << 1) | digitalRead(ENC_B_2);
  g_count_b += kStep[(g_state_b << 2) | s];
  g_state_b = s;
}

// ---- motors -----------------------------------------------------------------
// dir: +1 = "forward command" (IN1 high, IN2 low), -1 = reverse, 0 = stop.
void motor(int in1, int in2, int ch, int dir, int duty) {
  if (dir == 0 || duty == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(ch, 0);
    return;
  }
  digitalWrite(in1, dir > 0 ? HIGH : LOW);
  digitalWrite(in2, dir > 0 ? LOW : HIGH);
  ledcWrite(ch, duty);
}

static int g_duty = 60;
static int g_left_dir = 0, g_right_dir = 0;   // +1 = physically forward
static bool g_raw = false;                    // raw mode drives channels directly
static uint32_t g_last_key = 0;

// Drive one board channel. dir is the bridge polarity: +1 sets IN1 high.
void channel(int in1, int in2, int ch, int dir, int duty) {
  if (dir == 0 || duty == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(ch, 0);
    return;
  }
  digitalWrite(in1, dir > 0 ? HIGH : LOW);
  digitalWrite(in2, dir > 0 ? LOW : HIGH);
  ledcWrite(ch, duty);
}

void apply() {
  if (g_raw) {
    // g_left_dir and g_right_dir mean channel A and channel B here.
    channel(AIN1, AIN2, CH_A, g_left_dir, g_duty);
    channel(BIN1, BIN2, CH_B, g_right_dir, g_duty);
    return;
  }
  // Physical mode: translate a forward request into bridge polarity using the
  // measured motor signs, and route each side to its own board channel.
  const int a_dir = LEFT_IS_CHANNEL_A ? g_left_dir * LEFT_MOTOR_SIGN
                                      : g_right_dir * RIGHT_MOTOR_SIGN;
  const int b_dir = LEFT_IS_CHANNEL_A ? g_right_dir * RIGHT_MOTOR_SIGN
                                      : g_left_dir * LEFT_MOTOR_SIGN;
  channel(AIN1, AIN2, CH_A, a_dir, g_duty);
  channel(BIN1, BIN2, CH_B, b_dir, g_duty);
}

void setDirs(int left, int right, const char* what) {
  g_left_dir = left;
  g_right_dir = right;
  apply();
  Serial.printf("\n>> %s   (duty %d/255%s)\n", what, g_duty, g_raw ? ", RAW" : "");
}

// Counts in the robot's frame: up means that wheel turned forward.
long leftCount() {
  noInterrupts();
  long raw = LEFT_IS_CHANNEL_A ? g_count_a : g_count_b;
  interrupts();
  return g_raw ? raw : raw * LEFT_ENCODER_SIGN;
}

long rightCount() {
  noInterrupts();
  long raw = LEFT_IS_CHANNEL_A ? g_count_b : g_count_a;
  interrupts();
  return g_raw ? raw : raw * RIGHT_ENCODER_SIGN;
}

// ---- setup / loop -----------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  ledcSetup(CH_A, PWM_FREQ, PWM_BITS); ledcAttachPin(PWMA, CH_A);
  ledcSetup(CH_B, PWM_FREQ, PWM_BITS); ledcAttachPin(PWMB, CH_B);
  apply();

  // GPIO 34-39 are input-only with no internal pull-ups; the encoder circuit
  // must provide them. INPUT is correct for all four encoder pins here.
  pinMode(ENC_A_1, INPUT); pinMode(ENC_A_2, INPUT);
  pinMode(ENC_B_1, INPUT); pinMode(ENC_B_2, INPUT);
  g_state_a = (digitalRead(ENC_A_1) << 1) | digitalRead(ENC_A_2);
  g_state_b = (digitalRead(ENC_B_1) << 1) | digitalRead(ENC_B_2);
  attachInterrupt(digitalPinToInterrupt(ENC_A_1), isrChannelA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_2), isrChannelA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_1), isrChannelB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_2), isrChannelB, CHANGE);

  Serial.println("\n\nLearn2Race basic wheel test. LIFT THE ROBOT.");
  Serial.println("Keys are PHYSICAL wheels: 1/2 = left/right forward,");
  Serial.println("3/4 = left/right backward, w/s/a/d = both, +/- = PWM,");
  Serial.println("x = stop, z = zero counts, r = raw channel mode.");
  Serial.println("Forward motion makes that wheel's count go UP.");
  Serial.println("Auto-stop after 3 s without a key.");
}

void loop() {
  static long last_l = 0, last_r = 0;
  static uint32_t last_print = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') continue;
    g_last_key = millis();
    switch (c) {
      case '1': setDirs(+1, 0, "LEFT wheel forward"); break;
      case '2': setDirs(0, +1, "RIGHT wheel forward"); break;
      case '3': setDirs(-1, 0, "LEFT wheel backward"); break;
      case '4': setDirs(0, -1, "RIGHT wheel backward"); break;
      case 'w': setDirs(+1, +1, "both forward"); break;
      case 's': setDirs(-1, -1, "both backward"); break;
      case 'a': setDirs(-1, +1, "turn left"); break;
      case 'd': setDirs(+1, -1, "turn right"); break;
      case 'r': setDirs(0, 0, "stop");
                g_raw = !g_raw;
                Serial.printf("\n>> %s mode: keys 1/3 drive %s, 2/4 drive %s;"
                              " counts shown %s\n",
                              g_raw ? "RAW" : "PHYSICAL",
                              g_raw ? "channel A" : "the LEFT wheel",
                              g_raw ? "channel B" : "the RIGHT wheel",
                              g_raw ? "as the encoder produces them"
                                    : "in the robot frame (forward counts up)");
                break;
      case '+': g_duty = min(DUTY_MAX, g_duty + DUTY_STEP); apply();
                Serial.printf("\n>> duty %d/255\n", g_duty); break;
      case '-': g_duty = max(0, g_duty - DUTY_STEP); apply();
                Serial.printf("\n>> duty %d/255\n", g_duty); break;
      case 'z': noInterrupts(); g_count_a = g_count_b = 0; interrupts();
                last_l = last_r = 0; Serial.println("\n>> counts zeroed"); break;
      case 'x': case ' ': setDirs(0, 0, "STOP"); break;
      default: break;
    }
  }

  if ((g_left_dir || g_right_dir) && millis() - g_last_key > AUTO_STOP_MS) {
    setDirs(0, 0, "auto-stop (no key for 3 s)");
  }

  uint32_t now = millis();
  if (now - last_print >= PRINT_MS) {
    float dt = (now - last_print) / 1000.0f;
    last_print = now;
    long l = leftCount(), r = rightCount();
    long dl = l - last_l, dr = r - last_r;
    last_l = l; last_r = r;
    float rpm_l = dl / COUNTS_PER_WHEEL_REV / dt * 60.0f;
    float rpm_r = dr / COUNTS_PER_WHEEL_REV / dt * 60.0f;
    if (g_left_dir || g_right_dir || dl || dr) {
      Serial.printf("%s %7ld (%+5ld) %+6.1f rpm | %s %7ld (%+5ld) %+6.1f rpm\n",
                    g_raw ? "A count" : "LEFT ",
                    l, dl, rpm_l, g_raw ? "B count" : "RIGHT", r, dr, rpm_r);
    }
  }
}
