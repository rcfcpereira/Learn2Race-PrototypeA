#!/usr/bin/env python3
"""Supervise telemetry and publish the mux's fault and readiness heartbeats."""

# References (full entries: docs/references.md at the repository root):
#   [R3]  Learn2Race dissertation, requirements FR/NFR: telemetry supervision requirements
#   [R10] ROS 2 Jazzy documentation: rclpy node
# More details: README.md of this package; safety_policy.py

import os
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from learn2race_safety.safety_policy import TelemetryWatchdog, positive_seconds


class SafetyMonitor(Node):

    def __init__(self):
        super().__init__('safety_monitor')
        self.declare_parameter('odom_timeout_seconds', 0.5)
        self.declare_parameter('imu_timeout_seconds', 1.0)
        self.declare_parameter('scan_timeout_seconds', 1.0)
        self.declare_parameter('require_scan', False)
        self.declare_parameter('check_hz', 10.0)
        self.declare_parameter('startup_grace_seconds', 15.0)

        required = ['odom', 'imu']
        if bool(self.get_parameter('require_scan').value):
            required.append('scan')
        self.watchdog = TelemetryWatchdog(
            {key: self.get_parameter(f'{key}_timeout_seconds').value for key in required},
            started=time.monotonic(),
            startup_grace=self.get_parameter('startup_grace_seconds').value)

        self.pub_fault = self.create_publisher(Bool, 'safety/fault', 10)
        self.pub_ready = self.create_publisher(Bool, 'safety/ready', 10)
        self.pub_reason = self.create_publisher(String, 'safety/fault_reason', 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        for kind, topic, key in ((Odometry, 'odom', 'odom'),
                                 (Imu, 'imu/data', 'imu'),
                                 (LaserScan, 'scan', 'scan')):
            self.create_subscription(
                kind, topic,
                lambda msg, sensor=key: self.watchdog.mark(sensor, time.monotonic()),
                qos_profile_sensor_data, raw=True)   # arrival only: skip deserialising
        self.create_subscription(Bool, 'safety/reset', self._on_reset, 10)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        hz = positive_seconds(self.get_parameter('check_hz').value, 'check_hz')
        self.create_timer(1.0 / hz, self._check, clock=self.wall_clock)

    def _on_reset(self, msg):
        if msg.data:
            self.get_logger().warn('Operator requested safety reset; rechecking telemetry.')
            self.watchdog.reset()
            self._check()

    def _check(self):
        was_faulted = self.watchdog.fault_latched
        ready, faulted, reasons = self.watchdog.check(time.monotonic())
        if faulted and not was_faulted:
            self.get_logger().error('Safety fault: ' + '; '.join(reasons))
        self.pub_fault.publish(Bool(data=faulted))
        self.pub_ready.publish(Bool(data=ready))
        description = '; '.join(reasons)
        self.pub_reason.publish(String(data=description))

        status = DiagnosticStatus()
        status.name = 'learn2race/safety_monitor'
        status.level = (DiagnosticStatus.ERROR if faulted else
                        DiagnosticStatus.OK if ready else DiagnosticStatus.WARN)
        status.message = description or 'nominal'
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self.pub_diag.publish(array)



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
    node = SafetyMonitor()
    try:
        _spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub_ready.publish(Bool(data=False))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
