"""Optional Pi owner of Prototype A's QMI8658, AK09918, INA219 and LTR303.

Each sensor retries independently. Missing devices suppress their topic, so
the safety monitor sees missing telemetry instead of fabricated measurements.
"""

# References (full entries: docs/references.md at the repository root):
#   [R68] QMI8658C / AK09918 datasheets: QMI8658C, AK09918
#   [R69] INA219 / INA226 datasheets, INA219_WE: INA219
#   [R67] LTR-303ALS-01 datasheet: LTR303
#   [R12] REP 103, units and axes: units
# More details: README.md of this package; prototype_a_i2c.py

import os
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, MagneticField
from std_msgs.msg import UInt32

from .prototype_a_i2c import PrototypeAI2c, remap


class PrototypeASensorsNode(Node):
    def __init__(self):
        super().__init__('prototype_a_sensors')
        defaults = {
            'i2c_bus': 1, 'imu_address': 0x6B, 'mag_address': 0x0C,
            'battery_address': 0x42, 'light_address': 0x29, 'shunt_ohms': 0.01,
            'frame_id': 'imu_link', 'imu_sources': [0, 1, 2], 'imu_signs': [1, -1, -1],
            'gyro_bias': [0.0, 0.0, 0.0], 'mag_sources': [0, 1, 2],
            'mag_signs': [1, -1, -1], 'mag_offset_ut': [0.0, 0.0, 0.0],
            'battery_cells': 3, 'cell_floor_v': 3.5, 'cell_full_v': 4.2,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = lambda key: self.get_parameter(key).value
        for prefix in ('imu', 'mag'):
            if sorted(self.p(prefix + '_sources')) != [0, 1, 2] or len(self.p(prefix + '_signs')) != 3 or any(v not in (-1, 1) for v in self.p(prefix + '_signs')):
                raise ValueError('Sensor mapping needs three unique axes and +/-1 signs')
        for key in ('gyro_bias', 'mag_offset_ut'):
            if len(self.p(key)) != 3 or not all(math.isfinite(v) for v in self.p(key)):
                raise ValueError(key + ' needs three finite values')
        if self.p('battery_cells') <= 0 or not 0 < self.p('cell_floor_v') < self.p('cell_full_v'):
            raise ValueError('Invalid battery voltage limits')
        from smbus2 import SMBus
        self.bus = SMBus(int(self.p('i2c_bus')))
        self.driver = PrototypeAI2c(self.bus, **{k: self.p(k) for k in (
            'imu_address', 'mag_address', 'battery_address', 'light_address', 'shunt_ohms')})
        self.ready = {name: False for name in ('imu', 'mag', 'battery', 'light')}
        self.retry_at = dict.fromkeys(self.ready, 0.0)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.light_pub = self.create_publisher(UInt32, 'light/visible_plus_ir_raw', 10)
        self.ir_pub = self.create_publisher(UInt32, 'light/ir_raw', 10)
        for name, period in [('imu', 0.02), ('mag', 0.05), ('battery', 1.0), ('light', 0.1)]:
            self.create_timer(period, lambda name=name: self._tick(name))

    def _tick(self, name):
        now = time.monotonic()
        if now < self.retry_at[name]:
            return
        try:
            if not self.ready[name]:
                getattr(self.driver, 'init_' + name)()
                self.ready[name] = True
            sample = getattr(self.driver, 'read_' + name)()
            if sample is not None:
                getattr(self, '_publish_' + name)(sample)
        except OSError as exc:
            self.ready[name] = False
            self.retry_at[name] = now + 2.0
            self.get_logger().error(f'{name}: {exc}; retrying in 2 seconds')

    def _header(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.p('frame_id'))

    def _publish_imu(self, sample):
        accel, gyro = sample
        msg = Imu()
        self._header(msg)
        a = remap(accel, self.p('imu_sources'), self.p('imu_signs'))
        g = remap(gyro, self.p('imu_sources'), self.p('imu_signs'), self.p('gyro_bias'))
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = g
        msg.orientation_covariance[0] = -1.0
        self.imu_pub.publish(msg)

    def _publish_mag(self, sample):
        msg = MagneticField()
        self._header(msg)
        values = remap(sample, self.p('mag_sources'), self.p('mag_signs'), [v * 1e-6 for v in self.p('mag_offset_ut')])
        msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z = values
        self.mag_pub.publish(msg)

    def _publish_battery(self, sample):
        voltage, current = sample
        msg = BatteryState()
        self._header(msg)
        msg.header.frame_id = 'base_link'
        msg.voltage, msg.current, msg.present = voltage, current, True
        floor = self.p('battery_cells') * self.p('cell_floor_v')
        full = self.p('battery_cells') * self.p('cell_full_v')
        msg.percentage = max(0.0, min(1.0, (voltage - floor) / (full - floor)))
        msg.temperature = msg.charge = msg.capacity = msg.design_capacity = float('nan')
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING if current < 0 else BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD if voltage < floor else BatteryState.POWER_SUPPLY_HEALTH_GOOD
        self.battery_pub.publish(msg)

    def _publish_light(self, sample):
        visible, ir = sample
        self.light_pub.publish(UInt32(data=visible))
        self.ir_pub.publish(UInt32(data=ir))

    def destroy_node(self):
        self.bus.close()
        return super().destroy_node()



def _spin(node):
    """Spin with rclpy's EventsExecutor when available.

    The default SingleThreadedExecutor rebuilds its wait set on every wake-up,
    which on the Pi 4 made small nodes cost 25-30 % of a core each at a few
    hundred messages a second (measured 2026-10-07 with pidstat): enough to
    starve the IMU node at start-up and trip the safety monitor. Set
    L2R_EXECUTOR=single to fall back to the default executor.
    """
    if os.environ.get("L2R_EXECUTOR", "events") != "single":
        try:
            from rclpy.experimental import EventsExecutor
        except ImportError:
            EventsExecutor = None
        if EventsExecutor is not None:
            executor = EventsExecutor()
            executor.add_node(node)
            executor.spin()
            return
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    node = PrototypeASensorsNode()
    try:
        _spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
