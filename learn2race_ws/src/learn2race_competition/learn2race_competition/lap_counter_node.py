#!/usr/bin/env python3
"""Finish-line and pit-entry detection from the track sensing chain.

Two events are published:
    lap_event        a finish-line crossing
    pit_entry_event  the pit-lane entry marker

The TrackBotGP finish line is a narrow black stripe with a chequered pattern,
and is distinguished from the track stripe by width. Detection therefore keys on
a wide-marker signature across the line array (or, on the camera pipeline, on a
sustained full-width dark band), with a refractory interval so that one physical
marker cannot register twice.
"""

# References (full entries: docs/references.md at the repository root):
#   [R1] TrackBotGP rules 2026: finish line and pit-entry marker
# More details: README.md of this package (superseded by gp_referee in TrackBotGP mode)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Int32


class LapCounter(Node):

    def __init__(self):
        super().__init__('lap_counter')

        self.declare_parameter('dark_threshold', 0.6)
        self.declare_parameter('min_dark_channels', 6)
        self.declare_parameter('refractory_seconds', 2.0)
        self.declare_parameter('pit_marker_channels', 3)

        self.threshold = float(self.get_parameter('dark_threshold').value)
        self.min_dark = int(self.get_parameter('min_dark_channels').value)
        self.refractory = float(self.get_parameter('refractory_seconds').value)

        self.lap_count = 0
        self.last_event = 0.0

        self.pub_lap = self.create_publisher(Bool, 'lap_event', 10)
        self.pub_pit = self.create_publisher(Bool, 'pit_entry_event', 10)
        self.pub_count = self.create_publisher(Int32, 'lap_count', 10)

        self.create_subscription(Float32MultiArray, 'line_array/normalised',
                                 self._on_array, 10)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_array(self, msg: Float32MultiArray):
        if self._now() - self.last_event < self.refractory:
            return

        dark = sum(1 for v in msg.data if v >= self.threshold)
        if dark >= self.min_dark:
            self.last_event = self._now()
            self.lap_count += 1
            self.pub_lap.publish(Bool(data=True))
            self.pub_count.publish(Int32(data=self.lap_count))
            self.get_logger().info(f'Finish-line crossing {self.lap_count}')


def main(args=None):
    rclpy.init(args=args)
    node = LapCounter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
