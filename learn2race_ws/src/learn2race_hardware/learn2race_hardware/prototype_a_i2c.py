"""Prototype A sensor registers, matching the existing ESP32 drivers.

The caller owns the SMBus. Only use this driver after the ESP32 has released
the board's I2C bus (esp32dev_pi_sensors firmware). Never infer a valid zero
sample from a short read. Registers/scales mirror RobotApp.cpp and INA219_WE.
"""

# References (full entries: docs/references.md at the repository root):
#   [R60] Waveshare General Driver for Robots: General Driver board I2C
#   [R68] QMI8658C / AK09918 datasheets: QMI8658C, AK09918
#   [R69] INA219 / INA226 datasheets, INA219_WE: INA219 / INA219_WE scaling
#   [R67] LTR-303ALS-01 datasheet: LTR303
# More details: README.md of this package; firmware/lib/RobotApp/src/RobotApp.cpp

import math
import struct
import time


class PrototypeAI2c:
    def __init__(self, bus, imu_address=0x6B, mag_address=0x0C,
                 battery_address=0x42, light_address=0x29, shunt_ohms=0.01):
        if not math.isfinite(shunt_ohms) or shunt_ohms <= 0:
            raise ValueError("shunt_ohms must be positive and finite")
        self.bus = bus
        self.imu = imu_address
        self.mag = mag_address
        self.battery = battery_address
        self.light = light_address
        self.shunt_ohms = shunt_ohms

    def read(self, address, register, count):
        data = bytes(self.bus.read_i2c_block_data(address, register, count))
        if len(data) != count:
            raise OSError("Short I2C sensor read")
        return data

    def init_imu(self):
        if self.read(self.imu, 0x00, 1)[0] != 0x05:
            raise OSError("QMI8658 identity mismatch")
        self.bus.write_byte_data(self.imu, 0x60, 0xB0)
        time.sleep(0.05)
        for reg, value in [(0x02, 0x40), (0x03, 0x26), (0x04, 0x66), (0x08, 0x03)]:
            self.bus.write_byte_data(self.imu, reg, value)
        time.sleep(0.05)
        ctrl = self.read(self.imu, 0x02, 3)
        if (ctrl[0] & 0x60, ctrl[1] & 0x7F, ctrl[2] & 0x7F) != (0x40, 0x26, 0x66) or self.read(self.imu, 0x08, 1)[0] & 3 != 3:
            raise OSError("QMI8658 configuration did not latch")

    def read_imu(self):
        values = struct.unpack('<6h', self.read(self.imu, 0x35, 12))
        return ([v * 9.80665 / 4096.0 for v in values[:3]],
                [v * math.pi / (180.0 * 32.0) for v in values[3:]])

    def init_mag(self):
        if self.read(self.mag, 0x01, 1)[0] != 0x0C:
            raise OSError("AK09918 identity mismatch")
        self.bus.write_byte_data(self.mag, 0x32, 1)
        time.sleep(0.02)
        self.bus.write_byte_data(self.mag, 0x31, 0x08)

    def read_mag(self):
        if not self.read(self.mag, 0x10, 1)[0] & 1:
            return None
        # Include ST2 to release the sample latch, then reject magnetic overflow.
        data = self.read(self.mag, 0x11, 8)
        if data[7] & 8:
            return None
        return [v * 0.15e-6 for v in struct.unpack('<3h', data[:6])]

    def init_battery(self):
        # 32 V bus, +/-320 mV shunt, 12-bit continuous shunt+bus conversion.
        self.bus.write_i2c_block_data(self.battery, 0x00, [0x39, 0x9F])

    def read_battery(self):
        raw_bus = int.from_bytes(self.read(self.battery, 0x02, 2), 'big')
        if raw_bus & 1:
            raise OSError("INA219 overflow")
        if not raw_bus & 2:
            return None
        raw_shunt = int.from_bytes(self.read(self.battery, 0x01, 2), 'big', signed=True)
        shunt_v = raw_shunt * 1e-5
        # Use the actual fitted shunt; current is negative when discharging.
        return (raw_bus >> 3) * 0.004 + shunt_v, -shunt_v / self.shunt_ohms

    def init_light(self):
        self.bus.write_byte_data(self.light, 0x80, 1)
        self.bus.write_byte_data(self.light, 0x85, 0x01)  # 100 ms at gain 1.

    def read_light(self):
        ir, visible_ir = struct.unpack('<2H', self.read(self.light, 0x88, 4))
        return visible_ir, ir


def remap(values, sources, signs, bias=(0.0, 0.0, 0.0)):
    """Map sensor axes into the robot body frame before subtracting bias."""
    return [signs[i] * values[sources[i]] - bias[i] for i in range(3)]
