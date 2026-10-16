#!/usr/bin/env python3
"""Start, lap and pit events for a TrackBotGP race on Prototype A.

While the competition FSM is READY, the LTR303 light sensor learns the ambient
level and the white start light publishes start_event, as for the dragster.
During the race, the camera follower's band and fork detection plus odometry
drive a GpRaceTracker, which publishes:

    lap_event        each finish-line crossing (the FSM advances or finishes)
    pit_entry_event  the stop inside the pit lane (the FSM holds >= 5 s)
    pit_lane         True from taking the pit branch until the stop (LED yellow)
    line/branch_preference  the branch the follower takes at a fork

Every decision is logged with the distance into the lap, so a practice lap
gives the numbers to tune min_first_lap_m and the pit parameters.
"""

# References (full entries: docs/references.md at the repository root):
#   [R1]  TrackBotGP rules 2026: start light, laps, pit lane
#   [R67] LTR-303ALS-01 datasheet: LTR303 start-light readings
#   [R10] ROS 2 Jazzy documentation: rclpy node
# More details: README.md of this package; gp_policy.py

import os
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, String, UInt32

from learn2race_competition.dragster_policy import StartLightDetector
from learn2race_competition.gp_policy import GpRaceTracker


class GpReferee(Node):

    def __init__(self):
        super().__init__('gp_referee')
        self.declare_parameter('light_topic', 'light/visible_plus_ir_raw')
        self.declare_parameter('band_topic', 'line/band_detected')
        self.declare_parameter('fork_topic', 'line/fork_detected')
        self.declare_parameter('checker_topic', 'line/checker_detected')
        self.declare_parameter('pair_window_m', 0.0)
        self.declare_parameter('pit_needs_band', True)
        self.declare_parameter('preference_topic', 'line/branch_preference')
        self.declare_parameter('start_rise_ratio', 1.5)
        self.declare_parameter('start_min_rise', 50.0)
        self.declare_parameter('start_confirm_samples', 2)
        self.declare_parameter('start_fall_factor', 0.05)   # ambient falls 20x slower than it rises
        self.declare_parameter('total_laps', 3)
        self.declare_parameter('pit_after_lap', 2)
        self.declare_parameter('pit_side', 'auto')     # auto | left | right
        self.declare_parameter('start_ignore_m', 0.5)
        self.declare_parameter('min_first_lap_m', 2.0)
        self.declare_parameter('min_lap_fraction', 0.7)
        self.declare_parameter('pit_arm_fraction', 0.6)
        self.declare_parameter('pit_stop_after_m', 0.5)
        self.declare_parameter('pit_approach_speed', 0.15)
        self.declare_parameter('pit_arm_before_finish_m', 0.0)
        self.declare_parameter('pit_stop_grace_m', 0.5)
        self.declare_parameter('expected_lap_m', 0.0)   # 0: lap 1 is the reference
        self.declare_parameter('missed_finish_fraction', 1.35)
        self.declare_parameter('band_spin_radius_m', 0.03)
        self.declare_parameter('pit_exit_m', 1.0)
        self.declare_parameter('after_pit_min_fraction', 0.5)
        self.declare_parameter('max_final_lap_fraction', 1.3)
        self.declare_parameter('finish_overrun_m', 0.15)
        self.declare_parameter('band_confirm_frames', 2)
        self.declare_parameter('band_clear_frames', 3)
        self.declare_parameter('fork_confirm_frames', 2)
        p = lambda name: self.get_parameter(name).value

        self.start = StartLightDetector(
            rise_ratio=p('start_rise_ratio'), min_rise=p('start_min_rise'),
            confirm_samples=p('start_confirm_samples'), fall_factor=p('start_fall_factor'))
        self.race = GpRaceTracker(
            pit_side=p('pit_side'), total_laps=p('total_laps'),
            pit_after_lap=p('pit_after_lap'), start_ignore_m=p('start_ignore_m'),
            min_first_lap_m=p('min_first_lap_m'), min_lap_fraction=p('min_lap_fraction'),
            pit_arm_fraction=p('pit_arm_fraction'), pit_stop_after_m=p('pit_stop_after_m'),
            pit_approach_speed=p('pit_approach_speed'), pit_stop_grace_m=p('pit_stop_grace_m'),
            pit_arm_before_finish_m=p('pit_arm_before_finish_m'),
            expected_lap_m=p('expected_lap_m'),
            missed_finish_fraction=p('missed_finish_fraction'),
            band_spin_radius_m=p('band_spin_radius_m'), pair_window_m=p('pair_window_m'),
            pit_needs_band=p('pit_needs_band'),
            pit_exit_m=p('pit_exit_m'), after_pit_min_fraction=p('after_pit_min_fraction'),
            max_final_lap_fraction=p('max_final_lap_fraction'),
            finish_overrun_m=p('finish_overrun_m'), confirm_frames=p('band_confirm_frames'),
            clear_frames=p('band_clear_frames'), fork_confirm_frames=p('fork_confirm_frames'))
        self.state = None
        self.racing = False
        # Ready only with the camera follower up: on 2026-10-09 the start light
        # was flashed before it had started and the robot stood 6.4 s.
        self.follower_seen = 0.0
        self.race_started = self.lap_started = None

        self.pub_start = self.create_publisher(Bool, 'start_event', 10)
        self.pub_lap = self.create_publisher(Bool, 'lap_event', 10)
        self.pub_pit = self.create_publisher(Bool, 'pit_entry_event', 10)
        self.pub_pit_lane = self.create_publisher(Bool, 'pit_lane', 10)
        self.pub_pref = self.create_publisher(String, p('preference_topic'), 10)
        self.pub_speed = self.create_publisher(Float32, 'line/speed_limit', 10)
        self.create_subscription(Bool, 'line/found', self._on_line, 10)
        self.create_subscription(String, 'competition/state', self._on_state, 10)
        self.create_subscription(UInt32, p('light_topic'), self._on_light, qos_profile_sensor_data)
        self.create_subscription(Bool, p('band_topic'), self._on_band, 10)
        self.create_subscription(Bool, p('fork_topic'), self._on_fork, 10)
        self.create_subscription(Bool, p('checker_topic'), self._on_checker, 10)
        self.create_subscription(String, 'line/fork_side', self._on_fork_side, 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, qos_profile_sensor_data)
        # Republished steadily so a follower started later still gets it.
        self.create_timer(0.2, self._publish_preference)
        self.get_logger().info(
            'GP referee waiting for the start light; pit lane '
            + ('side from lap 1 (inside of the circuit)' if p('pit_side') == 'auto'
               else f"on the {p('pit_side')}")
            + (f", lap {p('expected_lap_m'):.2f} m." if p('expected_lap_m') > 0
               else ', lap length from lap 1.'))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_preference(self):
        # '' = no preference: the follower keeps the branch nearest its line.
        self.pub_pref.publish(String(data=self.race.preference or ''))
        self.pub_speed.publish(Float32(data=float(self.race.speed_limit if self.racing else 0.0)))

    def _on_state(self, msg):
        if msg.data == self.state:
            return
        previous, self.state = self.state, msg.data
        if msg.data == 'READY':
            self.start.reset()
            self.race.reset()
            self.racing = False
        elif msg.data == 'LAP1' and previous == 'READY':
            self.race.reset()
            self.racing = True
            self.lap_started = self.race_started = self._now()
            self.get_logger().info('Race started.')
        elif previous == 'PIT' and msg.data.startswith('LAP'):
            self.race.pit_released()
            self._log_lap_time()
            self._flush()
        elif msg.data in ('FINISH', 'SAFE_STOP'):
            self.racing = False
            self.pub_pit_lane.publish(Bool(data=False))
            if msg.data == 'FINISH' and self.race_started is not None:
                self.get_logger().info(
                    f'Race over in {self._now() - self.race_started:.2f} s; laps '
                    + ', '.join(f'{d:.2f} m' for d in self.race.lap_lengths))
        self._publish_preference()

    def _on_light(self, msg):
        if time.monotonic() - self.follower_seen > 1.0:
            return                       # follower not up: no start yet
        if self.state == 'READY' and self.start.update(float(msg.data)):
            self.get_logger().info(
                f'Start light: {msg.data} vs ambient {self.start.baseline:.0f}')
            self.pub_start.publish(Bool(data=True))

    def _on_band(self, msg):
        if self.racing:
            self._handle(self.race.update_band(bool(msg.data)))

    def _on_fork(self, msg):
        if self.racing:
            self._handle(self.race.update_fork(bool(msg.data)))

    def _on_fork_side(self, msg):
        self.race.update_fork_side(msg.data)

    def _on_checker(self, msg):
        if self.racing:
            self._handle(self.race.update_checker(bool(msg.data)))

    def _on_line(self, msg):
        self.follower_seen = time.monotonic()
        if self.racing:
            self._handle(self.race.update_line(bool(msg.data)))

    def _on_odom(self, msg):
        if self.racing:
            position, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self._handle(self.race.update_odom(position.x, position.y, yaw))

    def _log_lap_time(self):
        now = self._now()
        self.get_logger().info(f'Lap time {now - self.lap_started:.2f} s')
        self.lap_started = now

    def _flush(self):
        for note in self.race.notes:
            self.get_logger().info(note)
        self.race.notes.clear()

    def _handle(self, events):
        self._flush()
        for event in events:
            if event == 'lap':
                self._log_lap_time()
                self.pub_lap.publish(Bool(data=True))
            elif event == 'pit_lane':
                self.pub_pit_lane.publish(Bool(data=True))
                self._publish_preference()
            elif event == 'pit_stop':
                self.pub_pit_lane.publish(Bool(data=False))
                self.pub_pit.publish(Bool(data=True))
        if events:
            self._publish_preference()



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
    node = GpReferee()
    try:
        _spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
