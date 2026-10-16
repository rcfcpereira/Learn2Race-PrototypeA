"""ROS-independent start-light and finish-line decisions for dragster runs."""

# References (full entries: docs/references.md at the repository root):
#   [R2] Dragster rules: dragster start and finish
# More details: README.md of this package

import math


class StartLightDetector:
    """Trigger once when the light rises clearly above the learned ambient level.

    The baseline is an exponential average of samples that are not part of a
    candidate rise, so a slowly changing room light is tracked but the start
    light itself never raises the baseline it is compared against.

    The baseline falls ``fall_factor`` times more slowly than it rises. With a
    symmetric average, shading the sensor (a hand, the room lights off) pulled
    the baseline down to 9-32 counts within seconds, and the return of normal
    room light (~92 counts) then passed for the start light: 4 false starts in
    3.5 min of deliberate disturbances (2026-10-08). A slow fall keeps the
    reference at the room level through short shadows; a venue that really
    gets darker only makes the start harder, never a false one.
    """

    def __init__(self, rise_ratio=1.5, min_rise=50.0, confirm_samples=2,
                 min_baseline_samples=10, baseline_alpha=0.1, fall_factor=0.05):
        if rise_ratio <= 1.0 or min_rise <= 0 or confirm_samples < 1:
            raise ValueError('rise_ratio > 1, min_rise > 0 and confirm_samples >= 1 required')
        self.rise_ratio = float(rise_ratio)
        self.min_rise = float(min_rise)
        self.confirm_samples = int(confirm_samples)
        self.min_baseline_samples = int(min_baseline_samples)
        self.alpha = float(baseline_alpha)
        self.fall_factor = float(fall_factor)
        self.reset()

    def reset(self):
        self.baseline = None
        self.samples = 0
        self.run = 0
        self.triggered = False

    def update(self, value):
        """Feed one raw sample; return True exactly once, on the start light."""
        if self.triggered or not math.isfinite(value):
            return False
        if self.baseline is None:
            self.baseline, self.samples = float(value), 1
            return False
        rising = (self.samples >= self.min_baseline_samples
                  and value >= self.baseline * self.rise_ratio
                  and value - self.baseline >= self.min_rise)
        if rising:
            self.run += 1
            if self.run >= self.confirm_samples:
                self.triggered = True
                return True
            return False
        self.run = 0
        self.samples += 1
        alpha = self.alpha if value >= self.baseline else self.alpha * self.fall_factor
        self.baseline += alpha * (value - self.baseline)
        return False


class FinishDetector:
    """Decide where to stop from camera band sightings and distance travelled.

    Bands in the first ``ignore_m`` are the start line and are ignored. After
    that, each band counts once: it must be seen for ``confirm_frames`` and
    then lost for ``clear_frames`` before the next can count. The first band is
    the finish line; the run stops on band number ``stop_band`` (the stop line
    after the finish). The camera sees a band before the robot reaches it, so
    the stop is declared ``overrun_m`` later. ``max_distance_m`` ends the run
    even if a band is missed.
    """

    def __init__(self, ignore_m=1.0, confirm_frames=2, clear_frames=3, stop_band=2,
                 overrun_m=0.15, max_distance_m=12.0):
        if not 0 <= ignore_m < max_distance_m or overrun_m < 0                 or confirm_frames < 1 or clear_frames < 1 or stop_band < 1:
            raise ValueError('invalid finish detector limits')
        self.ignore_m = float(ignore_m)
        self.confirm_frames = int(confirm_frames)
        self.clear_frames = int(clear_frames)
        self.stop_band = int(stop_band)
        self.overrun_m = float(overrun_m)
        self.max_distance_m = float(max_distance_m)
        self.reset()

    def reset(self):
        self.distance = 0.0
        self.last_xy = None
        self.run = 0
        self.in_band = False
        self.crossings = []           # distance at which each band was confirmed
        self.finished = False

    def update_odom(self, x, y):
        """Accumulate path length; return a reason string once the run is over."""
        if self.last_xy is not None and math.isfinite(x) and math.isfinite(y):
            self.distance += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        if math.isfinite(x) and math.isfinite(y):
            self.last_xy = (x, y)
        return self._check()

    def update_band(self, detected):
        if self.distance >= self.ignore_m and len(self.crossings) < self.stop_band:
            if not self.in_band:
                self.run = self.run + 1 if detected else 0
                if self.run >= self.confirm_frames:
                    self.in_band, self.run = True, 0
                    self.crossings.append(self.distance)
            else:
                self.run = 0 if detected else self.run + 1
                if self.run >= self.clear_frames:
                    self.in_band, self.run = False, 0
        return self._check()

    def _check(self):
        if self.finished:
            return None
        if len(self.crossings) >= self.stop_band                 and self.distance >= self.crossings[-1] + self.overrun_m:
            self.finished = True
            return f'stop band {self.stop_band} at {self.crossings[-1]:.2f} m'
        if self.distance >= self.max_distance_m:
            self.finished = True
            return (f'only {len(self.crossings)} of {self.stop_band} bands by '
                    f'{self.max_distance_m:.1f} m; stopping')
        return None
