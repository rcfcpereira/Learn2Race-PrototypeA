#!/usr/bin/env python3
"""Competition finite state machine for Learn2Race.

Implements the run phases required by the ISEP TrackBotGP and Dragster rules:
READY -> LAP1 -> LAP2 -> PIT -> LAP3 -> FINISH, with SAFE_STOP reachable from
every operational state.

Rule constraints enforced here
------------------------------
* No motion between the countdown button and the start signal (the mux lock is
  only released on a validated start event).
* Pit entry occurs before the finish line at the end of lap 2, is executed once
  per run, and holds stationary for at least the configured minimum time.
* The run terminates with a commanded stop after the third finish-line crossing.
* Any fault reported by the safety monitor forces SAFE_STOP.
"""

# References (full entries: docs/references.md at the repository root):
#   [R1]  TrackBotGP rules 2026: TrackBotGP run phases and pit hold
#   [R2]  Dragster rules: dragster run phases
#   [R50] Harel (1987) statecharts: finite state machine design
#   [R3]  Learn2Race dissertation, requirements FR/NFR: FR/NFR safety requirements
# More details: README.md of this package

import os
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist


class State(Enum):
    READY = 'READY'
    LAP1 = 'LAP1'
    LAP2 = 'LAP2'
    PIT = 'PIT'
    LAP3 = 'LAP3'
    FINISH = 'FINISH'
    SAFE_STOP = 'SAFE_STOP'


class CompetitionFsm(Node):

    def __init__(self):
        super().__init__('competition_fsm')

        self.declare_parameter('mode', 'trackbotgp')        # trackbotgp | dragster
        self.declare_parameter('total_laps', 3)
        self.declare_parameter('pit_after_lap', 2)
        self.declare_parameter('pit_min_seconds', 5.0)
        self.declare_parameter('lap_timeout_seconds', 60.0)
        self.declare_parameter('tick_hz', 20.0)

        self.mode = self.get_parameter('mode').value
        self.total_laps = int(self.get_parameter('total_laps').value)
        self.pit_after_lap = int(self.get_parameter('pit_after_lap').value)
        self.pit_min = float(self.get_parameter('pit_min_seconds').value)
        self.lap_timeout = float(self.get_parameter('lap_timeout_seconds').value)

        self.state = State.READY
        self.lap = 0
        self.pit_done = False
        self.pit_entered_at = None
        self.lap_started_at = None

        self.pub_state = self.create_publisher(String, 'competition/state', 10)
        self.pub_arm = self.create_publisher(Bool, 'safety/arm', 10)
        self.pub_stop = self.create_publisher(Twist, 'competition/cmd_vel', 10)

        self.create_subscription(Bool, 'start_event', self._on_start, 10)
        self.create_subscription(Bool, 'lap_event', self._on_lap, 10)
        self.create_subscription(Bool, 'pit_entry_event', self._on_pit_entry, 10)
        self.create_subscription(Bool, 'safety/fault', self._on_fault, 10)

        period = 1.0 / float(self.get_parameter('tick_hz').value)
        self.create_timer(period, self._tick)

        self._publish_state()
        self._set_armed(False)
        self.get_logger().info(f'Competition FSM ready in {self.mode} mode.')

    # ---------------------------------------------------------------- helpers

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.pub_state.publish(msg)

    def _set_armed(self, armed: bool):
        msg = Bool()
        msg.data = armed
        self.pub_arm.publish(msg)

    def _command_stop(self):
        self.pub_stop.publish(Twist())

    def _transition(self, new_state: State):
        if new_state is self.state:
            return
        self.get_logger().info(f'{self.state.value} -> {new_state.value}')
        self.state = new_state
        self._publish_state()

    # ------------------------------------------------------------- callbacks

    def _on_fault(self, msg: Bool):
        if msg.data and self.state is not State.SAFE_STOP:
            self.get_logger().error('Safety fault reported; entering SAFE_STOP.')
            self._enter_safe_stop()

    def _on_start(self, msg: Bool):
        if not msg.data or self.state is not State.READY:
            return
        self.lap = 1
        self.lap_started_at = self._now()
        self._set_armed(True)
        self._transition(State.LAP1)

    def _on_pit_entry(self, msg: Bool):
        if not msg.data:
            return
        if self.mode != 'trackbotgp' or self.pit_done:
            return
        if self.state is not State.LAP2:
            return
        self.pit_done = True
        self.pit_entered_at = self._now()
        self._command_stop()
        self._transition(State.PIT)

    def _on_lap(self, msg: Bool):
        """A finish-line crossing."""
        if not msg.data:
            return
        if self.state in (State.READY, State.PIT, State.FINISH, State.SAFE_STOP):
            return

        if self.mode == 'dragster':
            self._finish()
            return

        if self.lap >= self.total_laps:
            self._finish()
            return

        self.lap += 1
        self.lap_started_at = self._now()
        self._transition({1: State.LAP1, 2: State.LAP2, 3: State.LAP3}[self.lap])

    # ------------------------------------------------------------------ tick

    def _tick(self):
        if self.state is State.PIT:
            # Hold the stop: the mux expires a source after source_timeout,
            # so a single stop at pit entry lets armed autonomy drive off.
            self._command_stop()
            if self._now() - self.pit_entered_at >= self.pit_min:
                self.lap = 3
                self.lap_started_at = self._now()
                self._transition(State.LAP3)

        elif self.state in (State.LAP1, State.LAP2, State.LAP3):
            if self.lap_started_at is not None and \
                    self._now() - self.lap_started_at > self.lap_timeout:
                self.get_logger().error('Lap timeout exceeded; entering SAFE_STOP.')
                self._enter_safe_stop()

        elif self.state in (State.FINISH, State.SAFE_STOP):
            self._command_stop()

        self._publish_state()

    def _finish(self):
        self._command_stop()
        self._set_armed(False)
        self._transition(State.FINISH)

    def _enter_safe_stop(self):
        self._command_stop()
        self._set_armed(False)
        self._transition(State.SAFE_STOP)



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
    node = CompetitionFsm()
    try:
        _spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
