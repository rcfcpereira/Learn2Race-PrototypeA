# References (full entries: docs/references.md at the repository root):
#   [R1] TrackBotGP rules 2026: race situations reproduced in the tests
# More details: gp_policy.py (the code under test)

import pytest

from learn2race_competition.gp_policy import GpRaceTracker


class Race:
    """Drives a tracker along a straight odometry path in 1 cm steps."""

    def __init__(self, **kwargs):
        self.t = GpRaceTracker(**kwargs)
        self.x = 0.0
        self.events = []
        self.t.update_odom(0.0, 0.0)

    def drive(self, metres):
        for _ in range(int(round(metres * 100))):
            self.x += 0.01
            self.events += [(e, round(self.t.distance, 2)) for e in self.t.update_odom(self.x, 0.0)]

    def band(self, frames_on=3, frames_off=4):
        for detected in [True] * frames_on + [False] * frames_off:
            self.events += [(e, round(self.t.distance, 2)) for e in self.t.update_band(detected)]

    def fork(self, frames=3):
        for detected in [True] * frames + [False]:
            self.events += [(e, round(self.t.distance, 2)) for e in self.t.update_fork(detected)]

    def checker(self, frames=3):
        for detected in [True] * frames + [False]:
            self.events += [(e, round(self.t.distance, 2)) for e in self.t.update_checker(detected)]

    def names(self):
        return [e for e, _ in self.events]


def full_race(r, lap=5.0):
    r.band()                    # the start line, right in front of the robot
    r.drive(lap)
    r.band()                    # end of lap 1
    r.drive(0.9 * lap)
    r.fork()                    # pit-lane entry
    r.drive(1.0)
    r.t.pit_released()
    r.drive(1.1 * lap)
    r.band()                    # end of lap 3
    r.drive(0.5)


def test_full_race_event_order():
    r = Race()
    full_race(r)
    assert r.names() == ['lap', 'pit_lane', 'pit_stop', 'lap']
    assert r.t.finished
    assert r.t.lap_lengths[0] == pytest.approx(5.15, abs=0.02)


def test_start_band_is_ignored():
    r = Race()
    r.band()
    r.drive(1.0)
    assert r.names() == []


def test_lap_event_waits_for_overrun():
    r = Race(finish_overrun_m=0.2)
    r.drive(5.0)
    r.band()
    assert r.names() == []
    r.drive(0.25)
    assert r.names() == ['lap']
    assert r.events[0][1] == pytest.approx(5.2, abs=0.015)


def test_corner_band_early_in_lap_is_ignored():
    r = Race()
    r.drive(5.0)
    r.band()
    r.drive(1.0)
    r.band()                    # a sharp corner 1 m into lap 2
    r.drive(0.5)
    assert r.names() == ['lap']


def test_preference_switches_for_pit_and_back():
    r = Race(pit_side='left')
    assert r.t.preference == 'right'
    r.drive(5.0)
    r.band()
    r.drive(0.2)
    assert r.t.preference == 'right'        # past the pit exit merge after the finish
    r.drive(3.2)
    assert r.t.preference == 'left'         # armed (0.6 x 5.15 m lap)
    r.fork()
    r.drive(1.0)
    assert r.t.preference == 'left'
    r.t.pit_released()
    r.drive(0.5)
    assert r.t.preference == 'left'         # pit exit: main continues on the pit side
    r.drive(1.0)
    assert r.t.preference == 'right'


def test_fork_before_arming_is_not_the_pit():
    r = Race()
    r.drive(5.0)
    r.band()
    r.drive(0.5)
    r.fork()                    # the pit exit merge, just after the finish
    r.drive(1.0)
    assert r.names() == ['lap']


def test_band_beside_pit_lane_is_ignored():
    r = Race()
    r.drive(5.0)
    r.band()
    r.drive(4.5)
    r.fork()
    r.drive(0.2)
    r.band()                    # the main finish line seen from the pit lane
    r.drive(0.5)
    r.t.pit_released()
    r.drive(0.3)
    r.band()
    r.drive(0.5)
    assert r.names() == ['lap', 'pit_lane', 'pit_stop']


def test_missed_pit_still_counts_laps():
    r = Race()
    r.drive(5.0)
    r.band()
    r.drive(5.0)
    r.band()                    # end of lap 2, pit never taken
    r.drive(0.5)
    assert r.names() == ['lap', 'lap']
    assert r.t.pit == 'missed'
    assert r.t.lap == 3


def test_final_lap_distance_limit_finishes():
    r = Race(max_final_lap_fraction=1.3)
    r.drive(5.0)
    r.band()
    r.drive(4.5)
    r.fork()
    r.drive(1.0)
    r.t.pit_released()
    r.drive(8.0)                # final finish line never seen
    assert r.names() == ['lap', 'pit_lane', 'pit_stop', 'lap']
    assert r.t.finished


def test_invalid_config_rejected():
    with pytest.raises(ValueError):
        GpRaceTracker(pit_side='up')
    with pytest.raises(ValueError):
        GpRaceTracker(pit_after_lap=3, total_laps=3)


def lap_with_turn(tracker, degrees, metres=5.0, steps=500):
    """Drive one lap that turns `degrees` in total, then cross the finish."""
    import math
    x = y = yaw = 0.0
    tracker.update_odom(x, y, yaw)
    for _ in range(steps):
        yaw += math.radians(degrees) / steps
        x += metres / steps * math.cos(yaw)
        y += metres / steps * math.sin(yaw)
        tracker.update_odom(x, y, yaw)
    for detected in [True] * 3 + [False] * 4:
        tracker.update_band(detected)
    for i in range(30):
        x += 0.01 * math.cos(yaw)
        y += 0.01 * math.sin(yaw)
        tracker.update_odom(x, y, yaw)


def test_auto_pit_side_from_lap_direction():
    ccw = GpRaceTracker(pit_side='auto')
    assert ccw.preference is None            # unknown in lap 1: keep nearest
    lap_with_turn(ccw, 360)
    assert ccw.pit_side == 'left' and ccw.preference == 'right'

    cw = GpRaceTracker(pit_side='auto')
    lap_with_turn(cw, -360)
    assert cw.pit_side == 'right' and cw.preference == 'left'


def test_auto_pit_side_figure_of_eight_falls_back_with_note():
    t = GpRaceTracker(pit_side='auto')
    lap_with_turn(t, 20)
    assert t.pit_side == 'left'
    assert any('figure of eight' in n for n in t.notes)


def test_pit_stop_waits_until_the_line_is_seen_in_the_pit_lane():
    r = Race()
    r.drive(5.0); r.band(); r.drive(4.5)
    r.fork()
    r.t.update_line(False)            # lost the line right after the fork
    r.drive(0.7)
    assert 'pit_stop' not in r.names()
    r.events += [(e, 0) for e in r.t.update_line(True)]
    assert r.names()[-1] == 'pit_stop'


def test_pit_stop_after_grace_even_if_the_line_stays_lost():
    r = Race(pit_stop_after_m=0.5, pit_stop_grace_m=0.5)
    r.drive(5.0); r.band(); r.drive(4.5)
    r.fork()
    r.t.update_line(False)
    r.drive(1.1)
    assert 'pit_stop' in r.names()


def test_speed_is_capped_from_arming_until_the_stop():
    r = Race(pit_approach_speed=0.15)
    r.drive(5.0); r.band()
    assert r.t.speed_limit == 0.0
    r.drive(3.5)
    assert r.t.speed_limit == 0.15              # armed
    r.fork(); r.drive(0.6)
    assert r.t.pit == 'stopped' and r.t.speed_limit == 0.0


def test_pit_armed_just_before_the_finish_ignores_an_earlier_split():
    # Practice track: 5.3 m lap, pit split 0.3 m before the finish line and
    # a split-like spot 1.1 m before it.
    r = Race(pit_arm_before_finish_m=0.7)
    r.drive(5.3); r.band()
    r.drive(4.2)
    r.fork()                                     # the false split
    assert r.t.pit == 'none' and 'pit_lane' not in r.names()
    r.drive(0.75)                                # 4.95+0.15 overrun: armed at 4.6
    assert r.t.pit == 'armed'
    r.fork()                                     # the real pit split
    assert 'pit_lane' in r.names()


def test_split_seen_just_after_the_finish_band_is_the_pit():
    # 2026-10-01 23:28: the branches separate in view next to the finish
    # stripe, so the band is confirmed 2 cm before the fork.
    r = Race()
    r.drive(5.0); r.band(); r.drive(4.4)          # armed
    r.band(frames_on=3, frames_off=0)             # finish stripe ahead...
    r.drive(0.02); r.fork()                       # ...and the pit split beside it
    r.drive(0.6)
    assert 'pit_lane' in r.names() and 'pit_stop' in r.names()
    assert r.names().count('lap') == 1 and r.t.lap == 2
    r.t.pit_released()
    assert r.t.lap == 3


def test_split_seen_after_the_lap_was_counted_is_not_the_pit():
    r = Race()
    r.drive(5.0); r.band(); r.drive(4.4)          # armed
    r.band()
    r.drive(0.3)                                  # lap counted after the overrun
    r.fork()
    assert 'pit_lane' not in r.names()
    assert r.t.pit == 'missed' and r.t.lap == 3



def test_band_seen_while_spinning_is_not_a_finish_line():
    import math
    t = GpRaceTracker()
    x = 0.0
    t.update_odom(x, 0.0, 0.0)
    for _ in range(500):                         # 5 m straight
        x += 0.01
        t.update_odom(x, 0.0, 0.0)
    events, yaw = [], 0.0
    for _ in range(10):                          # searching: spinning on the spot
        yaw += 0.1                               # 2 rad/s at 20 Hz
        t.update_odom(x, 0.0, yaw)
        events += t.update_band(True)            # the line sweeps across the view
    for _ in range(30):
        x += 0.01
        events += t.update_odom(x, 0.0, yaw)
    assert 'lap' not in events and t.lap == 1


def test_band_seen_driving_straight_still_counts():
    t = GpRaceTracker()
    x = 0.0
    t.update_odom(x, 0.0, 0.0)
    for _ in range(500):
        x += 0.01
        t.update_odom(x, 0.0, 0.0)
    events = []
    for d in [True] * 3 + [False] * 4:
        events += t.update_band(d)
    for _ in range(30):
        x += 0.01
        events += t.update_odom(x, 0.0, 0.0)
    assert 'lap' in events


def test_missed_first_finish_line_does_not_double_the_laps():
    # 2026-10-01 23:47: the robot went round the pit lane at the end of lap 1
    # (no finish line seen). Lap 1 measured 16.3 m and six laps were driven.
    r = Race(expected_lap_m=7.8)
    r.band()                                     # start line
    r.drive(7.8)                                 # finish line missed
    r.drive(0.35 * 7.8 + 0.01)                   # counted by distance
    assert r.names() == ['lap'] and r.t.lap == 2
    assert r.t.lap_lengths[0] == pytest.approx(7.8)
    r.drive(7.8 - 0.35 * 7.8 - 0.01 - 1.0)       # 1 m before lap 2's finish
    assert r.t.pit == 'armed'
    r.drive(0.7); r.fork(); r.drive(0.6)
    assert r.names()[-2:] == ['pit_lane', 'pit_stop']
    r.t.pit_released()
    r.drive(5.0); r.band(); r.drive(0.2)         # lap 3 finish
    assert r.names().count('lap') == 2 and r.t.finished


def test_fast_curve_does_not_hide_the_finish_line():
    t = GpRaceTracker(expected_lap_m=7.8)
    import math
    x = y = yaw = 0.0
    t.update_odom(x, y, yaw)
    events = []
    for _ in range(700):                         # 7 m: the lap so far
        x += 0.01
        t.update_odom(x, y, yaw)
    for i in range(10):                          # 0.3 m radius curve at 0.3 m/s, 20 Hz
        yaw += 0.05
        x += 0.015 * math.cos(yaw); y += 0.015 * math.sin(yaw)
        events += t.update_odom(x, y, yaw)
        events += t.update_band(i < 3)
    for _ in range(30):
        x += 0.01
        events += t.update_odom(x, y, yaw)
    assert 'lap' in events


def paired_race():
    return Race(expected_lap_m=7.9, pit_arm_before_finish_m=1.2, pair_window_m=0.5)


def test_lone_band_before_the_finish_is_not_a_lap():
    # 2026-10-02 00:02: a band 1.27 m before the finish line ended lap 1 at
    # 6.69 m and lap 3 before the finish.
    r = paired_race()
    r.band(); r.drive(6.69); r.band()            # no split beside it
    r.drive(1.0)
    assert 'lap' not in r.names()
    assert any('not the finish line' in n for n in r.t.notes)
    r.drive(0.1); r.fork(); r.drive(0.15); r.band(); r.drive(0.2)   # the real finish line
    assert r.names() == ['lap'] and r.t.lap_lengths[0] == pytest.approx(7.94 + 0.15, abs=0.02)


def test_band_then_split_is_a_finish_line():
    r = paired_race()
    r.band(); r.drive(7.8); r.band()
    r.drive(0.05); r.fork(); r.drive(0.2)
    assert r.names() == ['lap'] and r.t.lap == 2


def test_band_with_checker_is_a_finish_line():
    r = paired_race()
    r.band(); r.drive(7.8); r.band(); r.drive(0.05); r.checker(); r.drive(0.2)
    assert r.names() == ['lap']


def test_pit_needs_a_band_beside_the_split():
    r = paired_race()
    r.band(); r.drive(7.8); r.fork(); r.drive(0.1); r.band(); r.drive(0.2)   # lap 1
    assert r.t.lap == 2
    r.drive(6.8)                                   # armed at 6.7
    assert r.t.pit == 'armed'
    r.fork(); r.drive(0.6)                         # a split with no band: not the pit
    assert r.t.pit == 'armed' and 'pit_lane' not in r.names()
    r.drive(0.2); r.fork(); r.drive(0.25); r.band()   # split, then its finish band
    assert r.names()[-1] == 'pit_lane' and r.t.pit == 'lane'
    r.drive(0.3)
    assert r.names()[-1] == 'pit_stop'
    assert r.names().count('lap') == 1


def test_pit_split_right_after_the_band():
    r = paired_race()
    r.band(); r.drive(7.8); r.fork(); r.drive(0.1); r.band(); r.drive(0.2)
    r.drive(7.5); r.band(frames_on=3, frames_off=0); r.drive(0.02); r.fork()
    assert r.names()[-1] == 'pit_lane'
    r.drive(0.6)
    assert r.names()[-1] == 'pit_stop' and r.names().count('lap') == 1


def test_pit_without_band_when_not_required():
    r = Race(expected_lap_m=7.9, pit_arm_before_finish_m=1.2, pair_window_m=0.5, pit_needs_band=False)
    r.band(); r.drive(7.8); r.fork(); r.drive(0.1)
    r.band(frames_on=3, frames_off=0); r.checker(); r.band(frames_on=0); r.drive(0.2)   # the marker
    assert r.names() == ['lap']
    r.drive(7.5); r.fork()
    assert r.names()[-1] == 'pit_lane'
    r.drive(0.6)
    assert r.names()[-1] == 'pit_stop'


def test_pit_side_learned_from_the_split_before_the_lap1_finish():
    # Suzuka is a figure of eight: lap 1 turns ~0 deg net, so the turning
    # direction cannot tell the pit side; the split before the line can.
    r = Race(pit_side='auto', pair_window_m=0.5, pit_needs_band=False)
    r.band(); r.drive(6.0)
    r.t.update_fork_side('left'); r.fork(); r.drive(0.3)
    r.band(); r.drive(0.2)
    assert r.names() == ['lap'] and r.t.pit_side == 'left'
    assert r.t.preference == 'right'                 # main line on the right
