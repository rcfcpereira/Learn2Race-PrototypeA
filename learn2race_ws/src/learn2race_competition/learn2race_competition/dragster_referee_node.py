#!/usr/bin/env python3
"""Start and finish events for a dragster run on Prototype A.

While the competition FSM is READY, the board's LTR303 light sensor learns the
ambient level and a sharp rise (the start light) publishes start_event. During
the run, the camera follower's band detection plus odometry find the finish
band and publish lap_event, which the FSM in dragster mode turns into a stop.
The robot must come to rest between the finish and the stop line, so the stop
is commanded early and the resting distance is logged for tuning.
"""

# References (full entries: docs/references.md at the repository root):
#   [R2]  Dragster rules: dragster start and finish
#   [R67] LTR-303ALS-01 datasheet: LTR303 start-light readings
# More details: README.md of this package; dragster_policy.py

import json
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String, UInt32

from learn2race_competition.dragster_policy import FinishDetector, StartLightDetector


class DragsterReferee(Node):

    def __init__(self):
        super().__init__('dragster_referee')
        self.declare_parameter('light_topic', 'light/visible_plus_ir_raw')
        self.declare_parameter('band_topic', 'line/band_detected')
        self.declare_parameter('start_rise_ratio', 1.5)
        self.declare_parameter('start_min_rise', 50.0)
        self.declare_parameter('start_confirm_samples', 2)
        self.declare_parameter('start_fall_factor', 0.05)   # ambient falls 20x slower than it rises
        self.declare_parameter('finish_ignore_m', 1.0)
        self.declare_parameter('finish_confirm_frames', 2)
        self.declare_parameter('finish_clear_frames', 3)
        self.declare_parameter('stop_band', 2)
        self.declare_parameter('finish_overrun_m', 0.15)
        self.declare_parameter('max_distance_m', 12.0)
        p = lambda name: self.get_parameter(name).value

        self.start = StartLightDetector(
            rise_ratio=p('start_rise_ratio'), min_rise=p('start_min_rise'),
            confirm_samples=p('start_confirm_samples'), fall_factor=p('start_fall_factor'))
        self.finish = FinishDetector(
            ignore_m=p('finish_ignore_m'), confirm_frames=p('finish_confirm_frames'),
            clear_frames=p('finish_clear_frames'), stop_band=p('stop_band'),
            overrun_m=p('finish_overrun_m'), max_distance_m=p('max_distance_m'))
        self.run_started = None
        self.stop_commanded_at = None     # odometry distance when lap_event was sent
        self.rest_logged = False
        self.last_moved = None
        self.state = None

        self.pub_start = self.create_publisher(Bool, 'start_event', 10)
        self.pub_lap = self.create_publisher(Bool, 'lap_event', 10)
        # Run progress for the web page: finish line, braking zone, stop, rest.
        self.pub_status = self.create_publisher(String, 'dragster/status', 10)
        self.finish_time = None
        self.rest_m = None              # distance at rest
        self.status_last = 0.0
        self.create_subscription(String, 'competition/state', self._on_state, 10)
        self.create_subscription(UInt32, p('light_topic'), self._on_light, qos_profile_sensor_data)
        self.create_subscription(Bool, p('band_topic'), self._on_band, 10)
        # Ready only with the camera follower up: on 2026-10-09 the start light
        # was flashed before it had started and the robot stood 6.4 s.
        self.follower_seen = 0.0
        self.create_subscription(Bool, 'line/found', self._on_follower, 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, qos_profile_sensor_data)
        self.get_logger().info('Dragster referee waiting for the start light.')

    def _on_state(self, msg):
        if msg.data == self.state:
            return
        if msg.data == 'READY':
            self.start.reset()
            self.finish.reset()
        elif msg.data == 'LAP1':
            self.finish.reset()
            self.stop_commanded_at, self.rest_logged = None, False
            self.finish_time = self.rest_m = None
            self.run_started = self.get_clock().now()
            self.get_logger().info('Run started; watching for the finish band.')
        self.state = msg.data
        self._publish_status(force=True)

    def _on_follower(self, msg):
        self.follower_seen = time.monotonic()

    def _on_light(self, msg):
        if time.monotonic() - self.follower_seen > 1.0:
            return                       # follower not up: no start yet
        if self.state == 'READY' and self.start.update(float(msg.data)):
            self.get_logger().info(
                f'Start light: {msg.data} vs ambient {self.start.baseline:.0f}')
            self.pub_start.publish(Bool(data=True))

    def _on_band(self, msg):
        if self.state == 'LAP1':
            before = len(self.finish.crossings)
            reason = self.finish.update_band(bool(msg.data))
            if len(self.finish.crossings) == 1 and before == 0:
                elapsed = (self.get_clock().now() - self.run_started).nanoseconds / 1e9
                self.get_logger().info(
                    f'FINISH LINE at {self.finish.crossings[0]:.2f} m, {elapsed:.2f} s')
                self.finish_time = elapsed
            if len(self.finish.crossings) != before:
                self._publish_status(force=True)
            self._report(reason)

    def _on_odom(self, msg):
        position = msg.pose.pose.position
        if self.state == 'LAP1':
            self._report(self.finish.update_odom(position.x, position.y))
        elif self.stop_commanded_at is not None and not self.rest_logged:
            self._log_rest(position)
        self._publish_status()

    def _publish_status(self, force=False):
        now = time.monotonic()
        if not force and now - self.status_last < 0.25:
            return
        self.status_last = now
        bands = self.finish.crossings
        finish = bands[0] if bands else None
        if self.rest_m is not None:
            phase = 'at rest'
        elif self.stop_commanded_at is not None:
            phase = 'braking'
        elif len(bands) >= 2:
            phase = 'stop line'
        elif finish is not None:
            phase = 'braking zone'
        elif self.state == 'LAP1':
            phase = 'running'
        else:
            phase = (self.state or '').lower()
        rnd = lambda v, n=3: None if v is None else round(v, n)
        self.pub_status.publish(String(data=json.dumps({
            'phase': phase,
            'distance': rnd(self.finish.distance),
            'bands': [round(b, 3) for b in bands],
            'finish_m': rnd(finish),
            'finish_time': rnd(self.finish_time, 2),
            'stop_line_m': rnd(bands[1]) if len(bands) > 1 else None,
            'stop_commanded_m': rnd(self.stop_commanded_at),
            'rest_m': rnd(self.rest_m),
            'past_finish_m': (None if self.rest_m is None or finish is None
                              else round(self.rest_m - finish, 3)),
            'braked_m': (None if self.rest_m is None or self.stop_commanded_at is None
                         else round(self.rest_m - self.stop_commanded_at, 3)),
        })))

    def _log_rest(self, position):
        # Keep measuring after the stop so braking distance can be tuned.
        before = self.finish.distance
        self.finish.update_odom(position.x, position.y)
        now = self.get_clock().now()
        if self.finish.distance - before > 0.002 or self.last_moved is None:
            self.last_moved = now
        elif (now - self.last_moved).nanoseconds > 5e8:
            self.rest_logged = True
            self.rest_m = self.finish.distance
            finish = self.finish.crossings[0] if self.finish.crossings else None
            past = (f', {self.finish.distance - finish:.2f} m past where the finish band '
                    'was confirmed') if finish is not None else ''
            self.get_logger().info(
                f'At rest: braked {self.finish.distance - self.stop_commanded_at:.2f} m '
                f'after the stop command{past}')

    def _report(self, reason):
        if reason:
            self.get_logger().info(f'Stop: {reason} ({self.finish.distance:.2f} m run)')
            self.stop_commanded_at = self.finish.distance
            self.last_moved = None
            self.pub_lap.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = DragsterReferee()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
