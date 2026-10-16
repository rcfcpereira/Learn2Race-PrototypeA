"""Publish Grove Light Sensor v1.2 analog readings through ADS1115."""

# References (full entries: docs/references.md at the repository root):
#   [R74] Seeed Grove module wiki: Grove Light Sensor v1.2
#   [R70] ADS1115 / ADS7830 / PCF8591 datasheets: ADS1115
# More details: README.md of this package

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from learn2race_dragster.ads1115 import Ads1115


class GroveLightSensorAds1115Node(Node):
    """Read the analog light sensor and publish voltage plus normalized ratio."""

    def __init__(self) -> None:
        super().__init__("grove_light_sensor_ads1115_node")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("adc_address", 0x48)
        self.declare_parameter("adc_channel", 0)
        self.declare_parameter("adc_gain_v", 4.096)
        self.declare_parameter("adc_data_rate_sps", 128)
        self.declare_parameter("reference_voltage", 3.3)
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("voltage_topic", "/dragster/light_voltage")
        self.declare_parameter("ratio_topic", "/dragster/light_ratio")
        self.declare_parameter("dry_run", False)

        self.channel = int(self.get_parameter("adc_channel").value)
        self.reference_voltage = float(self.get_parameter("reference_voltage").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.last_error_log_time = self.get_clock().now()
        self.adc: Optional[Ads1115] = None

        self.voltage_pub = self.create_publisher(
            Float32, str(self.get_parameter("voltage_topic").value), 10
        )
        self.ratio_pub = self.create_publisher(
            Float32, str(self.get_parameter("ratio_topic").value), 10
        )

        self._init_adc()
        rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate_hz, self._publish_sample)

    def _init_adc(self) -> None:
        if self.dry_run:
            self.get_logger().warn("Light sensor dry_run=true; not touching I2C.")
            return

        bus = int(self.get_parameter("i2c_bus").value)
        address = self._parse_address(self.get_parameter("adc_address").value)
        try:
            self.adc = Ads1115(
                bus_id=bus,
                address=address,
                gain_v=float(self.get_parameter("adc_gain_v").value),
                data_rate_sps=int(self.get_parameter("adc_data_rate_sps").value),
            )
            self.get_logger().info(
                f"Started Grove Light Sensor v1.2 via ADS1115 on bus {bus}, "
                f"address 0x{address:02X}, channel A{self.channel}."
            )
        except (OSError, RuntimeError, ValueError) as exc:
            self.adc = None
            self.dry_run = True
            self.get_logger().error(
                "Could not start ADS1115 light sensor. Check ADS1115 wiring, I2C "
                f"enablement, and address with `i2cdetect -y {bus}`. Error: {exc}"
            )

    def _publish_sample(self) -> None:
        if self.dry_run or self.adc is None:
            return

        try:
            voltage = self.adc.read_single_ended(self.channel)
        except OSError as exc:
            self._log_read_error(exc)
            return

        ratio = 0.0
        if self.reference_voltage > 0.0:
            ratio = max(0.0, min(1.0, voltage / self.reference_voltage))

        voltage_msg = Float32()
        voltage_msg.data = float(voltage)
        self.voltage_pub.publish(voltage_msg)

        ratio_msg = Float32()
        ratio_msg.data = float(ratio)
        self.ratio_pub.publish(ratio_msg)

    def _log_read_error(self, exc: OSError) -> None:
        now = self.get_clock().now()
        elapsed_s = (now - self.last_error_log_time).nanoseconds / 1e9
        if elapsed_s >= 2.0:
            self.last_error_log_time = now
            self.get_logger().warn(f"Could not read ADS1115 light sample: {exc}")

    @staticmethod
    def _parse_address(value) -> int:
        if isinstance(value, str):
            return int(value, 0)
        return int(value)

    def destroy_node(self) -> bool:
        if self.adc is not None:
            self.adc.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = GroveLightSensorAds1115Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
