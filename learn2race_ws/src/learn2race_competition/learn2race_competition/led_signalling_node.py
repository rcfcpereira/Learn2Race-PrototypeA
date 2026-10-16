#!/usr/bin/env python3
"""TrackBotGP RGB LED signalling.

Maps the competition state onto the normative colour code of the TrackBotGP
rules. Every colour is a node parameter, so the mapping can be changed without
touching code if the organisers revise the code or a teaching activity needs a
different scheme.

Rule colour code:
    ready to start ......... green
    lap in progress ........ blue
    entering the pit lane .. yellow
    stopped in the pit lane  orange
    passing the finish line  three flashes on every lap_event and when
                             entering the pit lane (lap 2 ends there), then
                             yellow (colour
                             unspecified by the rules; the lap colour is
                             retained so that the pattern alone carries the
                             meaning)
    end of the race ........ white, flashing
"""

# References (full entries: docs/references.md at the repository root):
#   [R1] TrackBotGP rules 2026: RGB LED colour code, section V.3; lap times at the pit, II.7
# More details: README.md of this package

import os
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String, Bool


def _rgb(values):
    c = ColorRGBA()
    c.r, c.g, c.b = [float(v) / 255.0 for v in values[:3]]
    c.a = 1.0
    return c


class LedSignalling(Node):

    DEFAULTS = {
        'colour_ready':   [0, 255, 0],
        'colour_lap':     [0, 0, 255],
        'colour_pit_in':  [255, 200, 0],
        'colour_pit_hold': [255, 100, 0],
        'colour_finish':  [255, 255, 255],
        'colour_fault':   [255, 0, 0],
        'colour_off':     [0, 0, 0],
    }

    def __init__(self):
        super().__init__('led_signalling')

        for key, value in self.DEFAULTS.items():
            self.declare_parameter(key, value)
        self.declare_parameter('flash_hz', 4.0)
        self.declare_parameter('finish_flash_count', 3)

        self.state = 'READY'
        self.pit_lane = False
        self.lap_flashes_left = 0
        self.flash_on = False
        self.finish_flashes_left = 0

        self.pub = self.create_publisher(ColorRGBA, 'leds/set_color', 10)
        self.create_subscription(String, 'competition/state', self._on_state, 10)
        # The GP referee raises pit_lane from taking the pit branch until the
        # stop; the FSM's PIT state is the stop itself.
        self.create_subscription(Bool, 'pit_lane', self._on_pit_lane, 10)
        self.create_subscription(Bool, 'lap_event', self._on_lap, 10)
        # Green (ready) only once the camera follower is up, as the referees
        # accept the start light only then.
        self.follower_seen = 0.0
        self.create_subscription(Bool, 'line/found', self._on_follower, 10)
        # Dragster: the referee reports the finish line only here (its
        # lap_event is the stop line), so flash on the finish line from it.
        self.dragster_finish_seen = False
        self.create_subscription(String, 'dragster/status', self._on_dragster, 10)

        period = 1.0 / float(self.get_parameter('flash_hz').value)
        self.create_timer(period, self._tick)

    def _colour(self, key):
        return _rgb(self.get_parameter(key).value)

    def _on_pit_lane(self, msg: Bool):
        if msg.data and not self.pit_lane and self.state not in ('READY', 'SAFE_STOP'):
            # Lap 2 ends in the pit lane, which runs past the finish line; the
            # rules record it there ("or equivalent when in the pit lane",
            # II.7), so entering the pit lane gets the three finish-line
            # flashes, then yellow (V.3).
            self.lap_flashes_left = int(
                self.get_parameter('finish_flash_count').value) * 2
        self.pit_lane = msg.data

    def _on_dragster(self, msg: String):
        try:
            finish = json.loads(msg.data).get('finish_m')
        except ValueError:
            return
        if finish is None:
            self.dragster_finish_seen = False
        elif not self.dragster_finish_seen and self.state == 'LAP1':
            self.dragster_finish_seen = True
            self.lap_flashes_left = int(self.get_parameter('finish_flash_count').value) * 2

    def _on_follower(self, msg: Bool):
        self.follower_seen = time.monotonic()

    def _on_lap(self, msg: Bool):
        if msg.data and self.state not in ('READY', 'SAFE_STOP'):
            self.lap_flashes_left = int(
                self.get_parameter('finish_flash_count').value) * 2

    def _on_state(self, msg: String):
        if msg.data != self.state and msg.data == 'FINISH':
            # The last lap_event already started three flashes; let them run.
            self.finish_flashes_left = max(self.lap_flashes_left, int(
                self.get_parameter('finish_flash_count').value) * 2)
            self.lap_flashes_left = 0
        if msg.data != self.state and msg.data in ('PIT', 'READY', 'FINISH', 'SAFE_STOP'):
            self.pit_lane = False
        self.state = msg.data

    def _tick(self):
        self.flash_on = not self.flash_on

        if self.state == 'SAFE_STOP':
            self.pub.publish(self._colour('colour_fault') if self.flash_on
                             else self._colour('colour_off'))
            return

        if self.state == 'FINISH':
            if self.finish_flashes_left > 0:
                self.finish_flashes_left -= 1
                self.pub.publish(self._colour('colour_lap') if self.flash_on
                                 else self._colour('colour_off'))
            else:
                self.pub.publish(self._colour('colour_finish') if self.flash_on
                                 else self._colour('colour_off'))
            return

        if self.state == 'PIT':
            self.pub.publish(self._colour('colour_pit_hold'))
            return

        if self.lap_flashes_left > 0:
            self.lap_flashes_left -= 1
            self.pub.publish(self._colour('colour_lap') if self.flash_on
                             else self._colour('colour_off'))
            return

        if self.pit_lane:
            self.pub.publish(self._colour('colour_pit_in'))
            return

        if self.state == 'READY':
            ready = time.monotonic() - self.follower_seen <= 1.0
            self.pub.publish(self._colour('colour_ready' if ready else 'colour_off'))
            return

        self.pub.publish(self._colour('colour_lap'))



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
    node = LedSignalling()
    try:
        _spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
