"""Minimal ADS1115 I2C ADC helper for analog Grove sensors."""

# References (full entries: docs/references.md at the repository root):
#   [R70] ADS1115 / ADS7830 / PCF8591 datasheets: ADS1115 registers
# More details: README.md of this package

import time

try:
    from smbus2 import SMBus
except ImportError:  # Hardware library is only expected on the Raspberry Pi.
    SMBus = None


class Ads1115:
    """Read one ADS1115 single-ended channel as volts."""

    POINTER_CONVERSION = 0x00
    POINTER_CONFIG = 0x01

    MUX_SINGLE = {
        0: 0x4000,
        1: 0x5000,
        2: 0x6000,
        3: 0x7000,
    }
    PGA_RANGE = {
        6.144: 0x0000,
        4.096: 0x0200,
        2.048: 0x0400,
        1.024: 0x0600,
        0.512: 0x0800,
        0.256: 0x0A00,
    }
    DATA_RATE = {
        8: 0x0000,
        16: 0x0020,
        32: 0x0040,
        64: 0x0060,
        128: 0x0080,
        250: 0x00A0,
        475: 0x00C0,
        860: 0x00E0,
    }

    def __init__(
        self,
        bus_id: int,
        address: int = 0x48,
        gain_v: float = 4.096,
        data_rate_sps: int = 128,
    ) -> None:
        if SMBus is None:
            raise RuntimeError("Python package smbus2 is not installed")
        if gain_v not in self.PGA_RANGE:
            raise ValueError(f"Unsupported ADS1115 gain range: {gain_v}")
        if data_rate_sps not in self.DATA_RATE:
            raise ValueError(f"Unsupported ADS1115 sample rate: {data_rate_sps}")

        self.bus = SMBus(bus_id)
        self.address = address
        self.gain_v = gain_v
        self.data_rate_sps = data_rate_sps

    def close(self) -> None:
        self.bus.close()

    def read_single_ended(self, channel: int) -> float:
        if channel not in self.MUX_SINGLE:
            raise ValueError(f"ADS1115 channel must be 0..3, got {channel}")

        # Single-shot conversion avoids stale channel data after switching inputs.
        config = (
            0x8000
            | self.MUX_SINGLE[channel]
            | self.PGA_RANGE[self.gain_v]
            | 0x0100
            | self.DATA_RATE[self.data_rate_sps]
            | 0x0003
        )
        self._write_register(self.POINTER_CONFIG, config)
        time.sleep(max(0.002, 1.0 / float(self.data_rate_sps) + 0.001))
        raw = self._read_register(self.POINTER_CONVERSION)
        if raw & 0x8000:
            raw -= 0x10000
        return max(0.0, raw * self.gain_v / 32768.0)

    def _write_register(self, pointer: int, value: int) -> None:
        self.bus.write_i2c_block_data(
            self.address,
            pointer,
            [(value >> 8) & 0xFF, value & 0xFF],
        )

    def _read_register(self, pointer: int) -> int:
        data = self.bus.read_i2c_block_data(self.address, pointer, 2)
        return (data[0] << 8) | data[1]
