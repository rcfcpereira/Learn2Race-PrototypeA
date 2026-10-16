"""ROS-independent lap and pit decisions for a TrackBotGP race.

Inputs are the camera's transverse-band sightings (finish line), its fork
sightings (the pit lane splitting off) and odometry path length. Outputs are
events for the competition FSM:

    'lap'       the robot's front has crossed the finish line
    'pit_lane'  the robot has taken the pit-lane branch (LED yellow)
    'pit_stop'  the robot is inside the pit lane and must stop now

and ``preference``, the branch the line follower must take at a fork.

The pit lane is on the inside of the circuit, so with pit_side 'auto' it is
taken from the sign of the heading change over lap 1: an anticlockwise lap
(+360 deg) has the inside on the left, a clockwise lap on the right. A
figure-of-eight (Suzuka) turns ~0 deg net and needs pit_side set explicitly.

The robot starts just behind the finish line, so the band seen at the start
is ignored. A band only counts as a lap when most of a lap has been driven
since the last one, because a sharp corner can look like a band to the camera.
The first lap's length calibrates the rest: the pit is armed part way round
lap 2, the pit-side branch is taken at the next fork, and the stop is made a
fixed distance into the pit lane. The pit lane rejoins after the finish line,
so the next counted band is the end of the final lap.
"""

# References (full entries: docs/references.md at the repository root):
#   [R1]  TrackBotGP rules 2026: laps, pit stop before the finish line, >= 5 s stop, finish (II.6, III)
#   [R42] Siegwart et al., Autonomous Mobile Robots: odometry path length as the distance reference
# More details: README.md of this package

import math

LEFT, RIGHT, AUTO = 'left', 'right', 'auto'


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GpRaceTracker:

    def __init__(self, pit_side=LEFT, total_laps=3, pit_after_lap=2,
                 start_ignore_m=0.5, min_first_lap_m=2.0, min_lap_fraction=0.7,
                 pit_arm_fraction=0.6, pit_stop_after_m=0.5, pit_exit_m=1.0,
                 pit_approach_speed=0.15, pit_stop_grace_m=0.5, pit_arm_before_finish_m=0.0,
                 band_max_turn_per_update=0.04, band_turn_hold_updates=6,
                 band_spin_radius_m=0.03, expected_lap_m=0.0, missed_finish_fraction=1.35,
                 after_pit_min_fraction=0.5, max_final_lap_fraction=1.3,
                 finish_overrun_m=0.15, confirm_frames=2, clear_frames=3,
                 fork_confirm_frames=2, pair_window_m=0.0, pit_needs_band=True):
        if pit_side not in (LEFT, RIGHT, AUTO):
            raise ValueError("pit_side must be 'left', 'right' or 'auto'")
        if not 1 <= pit_after_lap < total_laps:
            raise ValueError('pit_after_lap must be before the last lap')
        if confirm_frames < 1 or clear_frames < 1 or fork_confirm_frames < 1:
            raise ValueError('frame counts must be >= 1')
        self.pit_side_setting = pit_side
        self.total_laps = int(total_laps)
        self.pit_after_lap = int(pit_after_lap)
        self.start_ignore_m = float(start_ignore_m)
        self.min_first_lap_m = float(min_first_lap_m)
        self.min_lap_fraction = float(min_lap_fraction)
        self.pit_arm_fraction = float(pit_arm_fraction)
        self.pit_stop_after_m = float(pit_stop_after_m)
        self.pit_approach_speed = float(pit_approach_speed)
        # > 0: arm the pit this far before the end of the lap (lap 1 length)
        # instead of at pit_arm_fraction. On the practice track the pit split
        # is 0.3 m before the finish line and another split-like spot sits
        # ~1.1 m before it, which 60% arming took for the pit (2026-10-01).
        self.pit_arm_before_finish_m = float(pit_arm_before_finish_m)
        # A finish line is crossed driving along the track. While the robot
        # spins (searching for a lost line, pivoting) the camera sweeps the
        # track line sideways across the view, where it looks just like a
        # finish line: a false lap 4.2 m into a 7.8 m lap (2026-10-01). Bands
        # are ignored while odometry turns faster than band_max_turn_per_update
        # rad per update (0.04 at 20 Hz = 0.8 rad/s) and for the hold after.
        self.band_max_turn = float(band_max_turn_per_update)
        self.band_turn_hold = int(band_turn_hold_updates)
        # Only a spin on the spot hides bands: a turn tighter than this radius
        # (searching pivots with no forward speed). A fast curve before the
        # finish line must not hide the finish line.
        self.band_spin_radius = float(band_spin_radius_m)
        # > 0: the track's lap length, the reference for every distance rule
        # instead of the measured lap 1. On 2026-10-01 23:47 lap 1's finish
        # line was missed, lap 1 measured 16.3 m (two laps), every later
        # finish line was "too early" and the robot drove six laps.
        self.expected_lap_m = float(expected_lap_m)
        # With a lap reference, a lap that runs this many lap lengths without
        # a finish line seen is counted anyway (the line was missed), and the
        # next lap is taken to have started one lap length after the last.
        self.missed_finish_fraction = float(missed_finish_fraction)
        self.pit_stop_grace_m = float(pit_stop_grace_m)
        self.pit_exit_m = float(pit_exit_m)
        self.after_pit_min_fraction = float(after_pit_min_fraction)
        self.max_final_lap_fraction = float(max_final_lap_fraction)
        self.finish_overrun_m = float(finish_overrun_m)
        self.confirm_frames = int(confirm_frames)
        self.clear_frames = int(clear_frames)
        self.fork_confirm_frames = int(fork_confirm_frames)
        # > 0: the finish line is a band with the pit split (or the chequered
        # marker) within this distance; the pit entry is a split with a band
        # within it. 0: any band is a finish line, any split the pit.
        self.pair_window = float(pair_window_m)
        # False: once armed, any split is the pit entry (the detector reports
        # the pit split's V as a split, never as a band, since 2026-10-02).
        self.pit_needs_band = bool(pit_needs_band)
        self.reset()

    def reset(self):
        self.distance = 0.0
        self.line_found = True
        self.last_xy = None
        self.last_yaw = None
        self.turned = 0.0               # unwrapped heading change since start
        self.turning = 0                # odometry updates left in which bands are ignored
        self.spin_band_run = 0
        self.pit_side = None if self.pit_side_setting == AUTO else self.pit_side_setting
        self.lap = 1
        self.lap_start = 0.0            # distance at the last counted crossing
        self.lap_lengths = []
        self.band_run = 0
        self.in_band = False
        self.pending_lap_at = None      # distance at which to emit 'lap'
        self.fork_run = 0
        self.checker_run = 0
        self.last_band_at = None        # distances of the last sightings
        self.last_fork_seen_at = None
        self.last_checker_at = None
        self.unpaired_band_at = None    # band waiting for its split or checker
        self.fork_side = None           # latest side of the branch not taken
        self.last_fork_side = None      # ...at the last confirmed split
        self.fork_candidate_at = None   # pit split waiting for its band
        self.pit = 'none'   # none | armed | lane | stopped | exit | done | missed
        self.fork_at = None
        self.pit_released_at = None
        self.finished = False
        self.notes = []                 # human-readable log lines for the node

    # ------------------------------------------------------------ properties

    @property
    def lap1_length(self):
        return self.lap_lengths[0] if self.lap_lengths else None

    @property
    def lap_ref(self):
        """Lap length the distance rules use: the track's, if given, else
        lap 1's once it is known."""
        if self.expected_lap_m > 0:
            return self.expected_lap_m
        return self.lap1_length

    @property
    def main_side(self):
        if self.pit_side is None:
            return None
        return RIGHT if self.pit_side == LEFT else LEFT

    @property
    def preference(self):
        """Branch for the follower: the pit side from arming until well clear
        of the pit exit (where the main line continues on the pit side), the
        main side everywhere else. None (keep the nearest branch) while the
        side is still unknown in lap 1."""
        if self.pit in ('armed', 'lane', 'stopped', 'exit'):
            return self.pit_side
        return self.main_side

    # ---------------------------------------------------------------- inputs

    def update_odom(self, x, y, yaw=None):
        moved = 0.0
        if math.isfinite(x) and math.isfinite(y):
            if self.last_xy is not None:
                moved = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
                self.distance += moved
            self.last_xy = (x, y)
        if yaw is not None and math.isfinite(yaw):
            if self.last_yaw is not None:
                step = _wrap(yaw - self.last_yaw)
                self.turned += step
                if abs(step) > self.band_max_turn and moved < self.band_spin_radius * abs(step):
                    self.turning = self.band_turn_hold
                elif self.turning > 0:
                    self.turning -= 1
            self.last_yaw = yaw
        return self._check()

    def update_band(self, detected):
        events = []
        if self.turning > 0 and not self.in_band:
            # Spinning: the line sweeps sideways and looks like a band.
            self.spin_band_run = self.spin_band_run + 1 if detected else 0
            if self.spin_band_run == self.confirm_frames and self.distance >= self.start_ignore_m:
                self.notes.append(f'band ignored at {self._lap_d():.2f} m into lap {self.lap} '
                                  '(robot spinning)')
            detected = False
        else:
            self.spin_band_run = 0
        if not self.in_band:
            self.band_run = self.band_run + 1 if detected else 0
            if self.band_run >= self.confirm_frames:
                self.in_band, self.band_run = True, 0
                events += self._on_band_confirmed()
        else:
            self.band_run = 0 if detected else self.band_run + 1
            if self.band_run >= self.clear_frames:
                self.in_band, self.band_run = False, 0
        events += self._check()
        return events

    def update_line(self, found):
        """Whether the camera sees the line this frame."""
        self.line_found = bool(found)
        return self._check()

    @property
    def speed_limit(self):
        """Speed cap for the follower (0 = none): slow from arming the pit
        until the stop, so the turn into the pit branch is not lost."""
        if self.pit in ('armed', 'lane'):
            return self.pit_approach_speed
        return 0.0

    def update_fork(self, detected):
        """The camera sees the line split (or two lines join).

        With pair_window_m > 0 a split counts as the pit entry only next to
        a finish band (the split is 0.3 m before the finish line); a fork
        that may follow a band still pending is allowed. On 2026-10-01 23:28
        the fork came 2 cm after the band, on 23:47 0.25 m before it. On
        2026-10-02 00:02 a split with no band beside it, 1.8 m before the
        finish line, was taken for the pit. The chequered marker's squares
        are never reported as a fork (the detector drops forks while it sees
        the checker)."""
        events = []
        self.fork_run = self.fork_run + 1 if detected else 0
        if self.fork_run != self.fork_confirm_frames:
            return events + self._check()
        self.last_fork_seen_at = self.distance
        self.last_fork_side = self.fork_side
        if self.pit == 'armed':
            if self.pair_window <= 0 or not self.pit_needs_band or self._near(self.last_band_at):
                events += self._enter_pit(self.distance)
            else:
                self.fork_candidate_at = self.distance
                self.notes.append(f'split at {self._lap_d():.2f} m into lap {self.lap} '
                                  'with no finish band beside it: not the pit yet')
        elif self.unpaired_band_at is not None and self._near(self.unpaired_band_at):
            self._accept_finish()
        return events + self._check()

    def update_fork_side(self, side):
        """Side ('left'/'right') of the branch the follower did not take."""
        if side in (LEFT, RIGHT):
            self.fork_side = side
        return []

    def update_checker(self, detected):
        """The camera sees the chequered part of the finish marker."""
        self.checker_run = self.checker_run + 1 if detected else 0
        if self.checker_run == self.confirm_frames:
            self.last_checker_at = self.distance
            if self.unpaired_band_at is not None and self._near(self.unpaired_band_at):
                self._accept_finish()
        return self._check()

    def _near(self, at):
        return at is not None and abs(self.distance - at) <= self.pair_window

    def _enter_pit(self, fork_at):
        self.pit, self.fork_at = 'lane', fork_at
        self.fork_candidate_at = None
        self.notes.append(f'pit-lane fork taken at {fork_at - self.lap_start:.2f} m into lap {self.lap}')
        if self.pending_lap_at is not None or self.unpaired_band_at is not None:
            # The pit lane bypasses the finish line: the lap ends at the
            # pit release instead.
            self.pending_lap_at = self.unpaired_band_at = None
            self.notes.append('finish band beside the split: lap counted at the pit exit')
        return ['pit_lane']

    def _accept_finish(self):
        """The band waiting for its split or checker is the finish line."""
        band_at, self.unpaired_band_at = self.unpaired_band_at, None
        self.pending_lap_at = band_at + self.finish_overrun_m

    def pit_released(self):
        """The FSM has left PIT after the minimum hold and started the next lap.

        The pit lane bypasses the finish line, so the lap ends here."""
        if self.pit == 'stopped':
            self.pit, self.pit_released_at = 'exit', self.distance
            self.lap_lengths.append(self.distance - self.lap_start)
            self.notes.append(f'lap {self.lap} done through the pit: {self.lap_lengths[-1]:.2f} m')
            self.lap_start = self.distance
            self.lap += 1

    # --------------------------------------------------------------- helpers

    def _lap_d(self):
        return self.distance - self.lap_start

    def _band_counts(self):
        if self.finished or self.pending_lap_at is not None:
            return False
        if self.lap == 1:
            first = max(self.start_ignore_m, self.min_first_lap_m)
            if self.expected_lap_m > 0:
                first = max(first, self.min_lap_fraction * self.expected_lap_m)
            return self._lap_d() >= first
        if self.pit in ('lane', 'stopped'):
            return False    # the pit lane runs alongside the finish line
        if self.pit_released_at is not None:
            return self.distance - self.pit_released_at >= \
                self.after_pit_min_fraction * self.lap_ref
        return self._lap_d() >= self.min_lap_fraction * self.lap_ref

    def _on_band_confirmed(self):
        self.last_band_at = self.distance
        if self.pit == 'armed' and self._near(self.fork_candidate_at):
            # The split came first (2026-10-01 23:47: 0.25 m before the band).
            return self._enter_pit(self.fork_candidate_at)
        if self._band_counts():
            if self.pair_window <= 0 or self._near(self.last_fork_seen_at) \
                    or self._near(self.last_checker_at):
                self.pending_lap_at = self.distance + self.finish_overrun_m
            else:
                # The finish line has the pit split 0.3 m before it and the
                # chequered marker beside it; a lone band is a corner or a
                # crossing until one of them turns up (2026-10-02 00:02:
                # bands 1.1-1.3 m before the finish line ended laps 1 and 3).
                self.unpaired_band_at = self.distance
        elif not self.finished and self.distance >= self.start_ignore_m:
            self.notes.append(f'band ignored at {self._lap_d():.2f} m into lap {self.lap} '
                              '(too early for a finish line)')
        return []

    def _decide_pit_side(self):
        # The lap is counted finish_overrun_m past the band, which is up to
        # pair_window_m past the split: look back that far.
        if self.last_fork_side is not None and self.last_fork_seen_at is not None                 and self.pair_window > 0 and self.distance - self.last_fork_seen_at                 <= self.pair_window + self.finish_overrun_m:
            # The pit lane splits off just before the finish line: the
            # branch not taken there is the pit lane. Works on a figure of
            # eight (Suzuka), where the turning direction cannot tell.
            self.pit_side = self.last_fork_side
            self.notes.append(f'pit lane split seen on the {self.pit_side} '
                              'before the lap 1 finish line: pit lane on the ' + self.pit_side)
            return
        degrees = math.degrees(self.turned)
        if abs(degrees) >= 180.0:
            self.pit_side = LEFT if degrees > 0 else RIGHT
            self.notes.append(f'lap 1 turned {degrees:+.0f} deg: '
                              f'{"anti" if degrees > 0 else ""}clockwise, pit lane on the {self.pit_side}')
        else:
            self.pit_side = LEFT
            self.notes.append(f'lap 1 turned only {degrees:+.0f} deg (figure of eight?): '
                              'cannot tell the inside, assuming pit lane on the left; '
                              'set pit_side explicitly for this track')

    def _count_lap(self, reason='', lap_start=None):
        end = self.distance if lap_start is None else lap_start
        length = end - self.lap_start
        self.unpaired_band_at = self.fork_candidate_at = None
        self.lap_lengths.append(length)
        self.notes.append(f'lap {self.lap} done: {length:.2f} m{reason}')
        if self.lap == 1 and self.pit_side is None:
            self._decide_pit_side()
        if self.lap == self.pit_after_lap and self.pit in ('armed', 'lane'):
            # 'lane' too: a robot really in the pit lane does not cross the
            # finish line, so the "fork" was not the pit entry. Without this
            # the pit stop fired in lap 3, where the FSM ignores it.
            self.pit = 'missed'
            self.notes.append('pit lane was not taken on this lap')
        self.lap_start = end
        if self.lap >= self.total_laps:
            self.finished = True
        else:
            self.lap += 1
        return ['lap']

    def _check(self):
        events = []
        if self.unpaired_band_at is not None \
                and self.distance - self.unpaired_band_at > self.pair_window:
            self.notes.append(f'band at {self.unpaired_band_at - self.lap_start:.2f} m into '
                              f'lap {self.lap} ignored: no split or checker within '
                              f'{self.pair_window:.2f} m, not the finish line')
            self.unpaired_band_at = None
        if self.fork_candidate_at is not None \
                and self.distance - self.fork_candidate_at > self.pair_window:
            self.fork_candidate_at = None
        if self.pending_lap_at is not None and self.distance >= self.pending_lap_at:
            self.pending_lap_at = None
            events += self._count_lap()
        if self.finished:
            return events
        ref = self.lap_ref
        if ref is None:
            return events
        if self.expected_lap_m > 0 and self.lap < self.total_laps \
                and self.pending_lap_at is None and self.pit not in ('lane', 'stopped') \
                and self._lap_d() >= self.missed_finish_fraction * ref:
            # The finish line was missed, or the robot went round the pit
            # lane, which bypasses it: count the lap where it should have been.
            events += self._count_lap(' (finish line not seen; distance limit)',
                                      lap_start=self.lap_start + ref)
            if self.finished:
                return events
        arm_at = (ref - self.pit_arm_before_finish_m if self.pit_arm_before_finish_m > 0
                  else self.pit_arm_fraction * ref)
        if self.pit == 'none' and self.lap == self.pit_after_lap \
                and self._lap_d() >= arm_at:
            self.pit = 'armed'
            self.notes.append(f'pit armed at {self._lap_d():.2f} m into lap {self.lap}')
        # Stop in the pit lane only while the camera sees the line there: on
        # 2026-10-01 the robot lost the line right after the fork and the
        # distance-only rule stopped it off the pit lane. If the line stays
        # lost for pit_stop_grace_m more, stop anyway.
        into_pit = self.distance - self.fork_at if self.pit == 'lane' else 0.0
        if self.pit == 'lane' and into_pit >= self.pit_stop_after_m and (
                self.line_found or into_pit >= self.pit_stop_after_m + self.pit_stop_grace_m):
            self.pit = 'stopped'
            self.notes.append(f'pit stop {self.distance - self.fork_at:.2f} m into the pit lane')
            events.append('pit_stop')
        if self.pit == 'exit' and self.distance - self.pit_released_at >= self.pit_exit_m:
            self.pit = 'done'
        if self.lap == self.total_laps and self.pending_lap_at is None:
            since = self.pit_released_at if self.pit_released_at is not None else self.lap_start
            if self.distance - since >= self.max_final_lap_fraction * ref:
                events += self._count_lap(' (finish line not seen; distance limit)')
        return events
