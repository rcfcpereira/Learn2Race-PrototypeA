#!/usr/bin/env python3
"""Arbitrate drive sources under a common telemetry and fault interlock.

Priority: competition stop, then teleoperation (only outside competition mode,
so the operator can take over), then armed or enabled autonomy.
Every source requires fresh safety/fault=false and safety/ready=true heartbeats.
The node publishes zero during startup, faults and stale supervision.
"""

# References (full entries: docs/references.md at the repository root):
#   [R3]  Learn2Race dissertation, requirements FR/NFR: FR10 command timeout and safe stop; drive-source priorities
#   [R10] ROS 2 Jazzy documentation: rclpy node
# More details: README.md of this package; safety_policy.py

import os
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from learn2race_safety.safety_policy import CommandGate, positive_seconds


class CmdVelMux(Node):

    def __init__(self):
        super().__init__('cmd_vel_mux')
        self.declare_parameter('competition_mode', True)
        self.declare_parameter('autonomy_enabled', False)
        self.declare_parameter('publish_hz', 25.0)
        self.declare_parameter('source_timeout_seconds', 0.5)
        self.declare_parameter('safety_timeout_seconds', 0.5)
        self.gate = CommandGate(
            competition_mode=bool(self.get_parameter('competition_mode').value),
            autonomy_enabled=bool(self.get_parameter('autonomy_enabled').value),
            source_timeout=self.get_parameter('source_timeout_seconds').value,
            safety_timeout=self.get_parameter('safety_timeout_seconds').value)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        for name in ('competition', 'autonomy', 'teleop'):
            self.create_subscription(
                Twist, f'{name}/cmd_vel',
                lambda msg, source=name: self._store(source, msg), 10)
        self.create_subscription(Bool, 'safety/arm', self._on_arm, 10)
        self.create_subscription(Bool, 'safety/fault', self._on_fault, 10)
        self.create_subscription(Bool, 'safety/ready', self._on_ready, 10)

        # Hardware watchdogs must keep expiring if ROS time pauses/jumps.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        hz = positive_seconds(self.get_parameter('publish_hz').value, 'publish_hz')
        self.create_timer(1.0 / hz, self._tick, clock=self.wall_clock)

    def _store(self, name, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.gate.sources.pop(name, None)
            self.pub.publish(Twist())
            self.get_logger().error(f'Rejected non-finite {name} velocity')
            return
        self.gate.store(name, msg, time.monotonic())
        if name == 'competition':
            # Forward stops at once rather than on the next tick.
            self._tick()

    def _on_arm(self, msg):
        if msg.data and not self.gate.healthy(time.monotonic()):
            self.get_logger().warn('Ignoring arm request while safety is not healthy')
            return
        if msg.data != self.gate.armed:
            self.get_logger().info(f'Arming state -> {msg.data}')
        self.gate.armed = bool(msg.data)

    def _on_fault(self, msg):
        self.gate.set_fault(msg.data, time.monotonic())
        if msg.data:
            self.pub.publish(Twist())

    def _on_ready(self, msg):
        self.gate.set_ready(msg.data, time.monotonic())
        if not msg.data:
            self.pub.publish(Twist())

    def _tick(self):
        selected = self.gate.select(time.monotonic())
        self.pub.publish(selected if selected is not None else Twist())



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
    node = CmdVelMux()
    try:
        _spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best effort: SIGINT may already have invalidated the context; the
        # firmware's CMD_TIMEOUT_MS stops the motors either way.
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
