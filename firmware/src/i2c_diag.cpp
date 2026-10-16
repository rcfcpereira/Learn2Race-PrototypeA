// References (full entries: docs/references.md at the repository root):
//   [R60] Waveshare General Driver for Robots: board I2C bus
//   [R68] QMI8658C / AK09918 datasheets: IMU/magnetometer addresses
// More details: firmware/README.md

// I2C diagnostic for the Waveshare General Driver for Robots (Prototype A).
//
// Built as a separate PlatformIO environment with no micro-ROS, so the serial
// port is free for plain text. It answers one question: what do the IMU and the
// compass actually return, byte for byte, before any scaling or remapping.
//
//   pio run -e diag -t upload
//   pio device monitor -e diag
//
// Copy everything it prints and send it back.

#include <Arduino.h>
#include <Wire.h>

static const int SDA_PIN = 32;
static const int SCL_PIN = 33;

static const uint8_t QMI_CANDIDATES[] = {0x6B, 0x6A};
static const uint8_t AK09918 = 0x0C;
static const uint8_t INA219 = 0x42;

static uint8_t g_qmi = 0;

bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

int readRegs(uint8_t addr, uint8_t reg, uint8_t* out, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  int got = Wire.requestFrom(addr, n);
  for (int i = 0; i < got; ++i) out[i] = Wire.read();
  return got;
}

uint8_t readReg(uint8_t addr, uint8_t reg) {
  uint8_t v = 0xEE;
  readRegs(addr, reg, &v, 1);
  return v;
}

void hexdump(const uint8_t* b, int n) {
  for (int i = 0; i < n; ++i) Serial.printf("%02X ", b[i]);
}

void scanBus() {
  Serial.println("\n== I2C scan ==");
  for (uint8_t a = 1; a < 127; ++a) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf("  device at 0x%02X\n", a);
  }
}

void setupQmi() {
  Serial.println("\n== QMI8658 ==");
  for (uint8_t a : QMI_CANDIDATES) {
    uint8_t who = readReg(a, 0x00);
    Serial.printf("  WHO_AM_I at 0x%02X = 0x%02X %s\n", a, who,
                  who == 0x05 ? "(QMI8658, expected 0x05)" : "");
    if (who == 0x05 && g_qmi == 0) g_qmi = a;
  }
  if (!g_qmi) { Serial.println("  QMI8658 not found"); return; }

  // Soft reset, then the same configuration the robot firmware writes.
  writeReg(g_qmi, 0x60, 0xB0);
  delay(50);
  writeReg(g_qmi, 0x02, 0x40);   // CTRL1: auto-increment, little-endian
  writeReg(g_qmi, 0x03, 0x26);   // CTRL2: +/-8 g
  writeReg(g_qmi, 0x04, 0x66);   // CTRL3: +/-1024 dps
  writeReg(g_qmi, 0x08, 0x03);   // CTRL7: accel + gyro on
  delay(100);

  Serial.print("  CTRL1..CTRL9 readback (0x02..0x0A): ");
  uint8_t ctrl[9]; readRegs(g_qmi, 0x02, ctrl, 9); hexdump(ctrl, 9);
  Serial.println("\n  expected                          : 40 26 66 .. .. .. 03 ..");
}

void setupAk() {
  Serial.println("\n== AK09918 ==");
  Serial.printf("  WIA1 = 0x%02X (expect 0x48)  WIA2 = 0x%02X (expect 0x0C)\n",
                readReg(AK09918, 0x00), readReg(AK09918, 0x01));
  writeReg(AK09918, 0x32, 0x01);   // soft reset
  delay(20);
  writeReg(AK09918, 0x31, 0x08);   // continuous 100 Hz
  delay(20);
  Serial.printf("  CNTL2 readback = 0x%02X (expect 0x08)\n", readReg(AK09918, 0x31));
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n\nLearn2Race I2C diagnostic");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  scanBus();
  setupQmi();
  setupAk();
  Serial.println("\n== live (keep the robot level and still) ==");
}

void loop() {
  if (g_qmi) {
    uint8_t st = readReg(g_qmi, 0x2E);          // STATUS0: data ready bits
    uint8_t b[12];
    int got = readRegs(g_qmi, 0x35, b, 12);
    int16_t le[6], be[6];
    for (int i = 0; i < 6; ++i) {
      le[i] = (int16_t)((b[2 * i + 1] << 8) | b[2 * i]);
      be[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
    }
    auto mag = [](const int16_t* v) {
      return sqrtf((float)v[0] * v[0] + (float)v[1] * v[1] + (float)v[2] * v[2]) / 4096.0f;
    };
    Serial.printf("QMI st=%02X got=%d raw=", st, got); hexdump(b, 12);
    Serial.printf("| LE a=%6d %6d %6d |a|=%.2fg | BE |a|=%.2fg\n",
                  le[0], le[1], le[2], mag(le), mag(be));
  }

  uint8_t st1 = readReg(AK09918, 0x10);
  uint8_t m[8];
  readRegs(AK09918, 0x11, m, 8);                 // HXL..HZH, TMPS, ST2
  int16_t hx = (int16_t)((m[1] << 8) | m[0]);
  int16_t hy = (int16_t)((m[3] << 8) | m[2]);
  int16_t hz = (int16_t)((m[5] << 8) | m[4]);
  float ut = 0.15f * sqrtf((float)hx * hx + (float)hy * hy + (float)hz * hz);
  Serial.printf("AK  st1=%02X st2=%02X raw=", st1, m[7]); hexdump(m, 6);
  Serial.printf("| %6d %6d %6d |B|=%.1f uT\n", hx, hy, hz, ut);

  uint8_t v[2]; readRegs(INA219, 0x02, v, 2);
  float bus = (((v[0] << 8) | v[1]) >> 3) * 0.004f;
  Serial.printf("INA bus=%.2f V\n\n", bus);

  delay(500);
}
