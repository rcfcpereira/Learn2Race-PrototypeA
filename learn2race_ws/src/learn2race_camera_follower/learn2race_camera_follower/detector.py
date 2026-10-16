"""Pure image-processing utilities for line detection."""

# References (full entries: docs/references.md at the repository root):
#   [R30] OpenCV 4.x: blur, max-filter background, morphology, connected components, fitLine
#   [R32] Gonzalez & Woods, Digital Image Processing: local (relative) thresholding and mask clean-up
#   [R31] Otsu (1979) thresholding: per-blob threshold in the shadow split and the 'otsu' mode
#   [R33] Suzuki & Abe (1985) contours: contour extraction (cv2.findContours)
#   [R41] Coulter (1992) pure pursuit: the 'pursuit' steering mode
#   [R1]  TrackBotGP rules 2026: track geometry: 20 mm line, finish band + chequered marker, pit split
# More details: README.md of this package (Detector)

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import math

import cv2
import numpy as np


Array = np.ndarray


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class DetectorConfig:
    # The upper rows see farthest ahead (~23 cm vs ~16 cm at 0.60), which is
    # where a hairpin first appears. Safe now that the look-ahead follows only
    # the line's own shape; raise it again if other track parts get in.
    roi_top_ratio: float = 0.30
    roi_bottom_ratio: float = 0.95
    black_threshold: int = 70
    # 'otsu' picks the threshold per frame from the image itself, so it follows
    # changes in room lighting; 'fixed' uses black_threshold as before. A fixed
    # absolute threshold that works under one ceiling fails under the next.
    threshold_mode: str = "relative"
    # 'relative' (default since 2026-09-30): a pixel is line when it is much
    # darker than the floor AROUND it, not than a single threshold. The
    # robot's own shadow on the floor is wide and only somewhat darker, so a
    # per-frame Otsu threshold landed between shadow and floor and the robot
    # followed its shadow. The local floor brightness is a maximum filter
    # wider than the line (it ignores the line but follows shadows), and the
    # line is what falls below line_ratio of it.
    line_ratio: float = 0.5
    # Two-level threshold (off: 0.0). On the 2026-09-30 video it pulled the
    # robot's shadow into the line wherever the two touched.
    line_ratio_weak: float = 0.0
    # Split a dark shadow off the line when they merge (see
    # _drop_attached_shadow): the two darkness levels must differ by
    # shadow_separation and the lighter part must be at least
    # shadow_area_ratio times the darker part.
    # Chequered finish marker: rows of >= 3 similar squares spanning at least
    # checker_min_span of the width, over at least checker_min_rows rows.
    checker_min_span: float = 0.2
    checker_min_rows: int = 8
    split_shadow: bool = True
    shadow_separation: float = 0.12
    shadow_area_ratio: float = 0.35      # replay of both 2026-09-30 videos
    # ...and the lighter part must be wider (per row) than this many times
    # the darker part: a shadow patch, not the lit end of the tape.
    shadow_min_width_ratio: float = 1.6
    # Closing of the line mask: fills bites that shiny streaks on the tape
    # take out of the line. Works on line pixels only, so it cannot add shadow.
    close_px: int = 5
    # Wider than the line (~55 px near the robot) AND than the finish stripe
    # seen close up, or the floor estimate darkens inside them.
    background_kernel_px: int = 129
    # Minimum spread between dark and bright pixels in the ROI for a line to be
    # considered present. Without it, Otsu will happily split a blank floor into
    # two classes of noise and report a line that is not there.
    min_contrast: int = 40
    min_contour_area_ratio: float = 0.003
    # Distance ahead at which the line position is measured. 125 mm reproduces
    # the point the original code steered on by accident, near the top of the
    # ROI. The previous default of 50 mm was never effective, because a bug in
    # the row mapping clamped every value to the same row; once that bug was
    # fixed, 50 mm moved the steering point to the bottom of the ROI, closest to
    # the robot, and the robot began running off curves. Tune this with the
    # robot on the track: longer is smoother and cuts corners, shorter tracks
    # tightly but reacts late.
    # Since the camera was re-aimed (2026-09-29, ~32 deg down) the mm mapping
    # no longer matches the floor; 125 is the row ~3/4 down the image. The
    # simulator preferred 85 (bottom row), but on the track that reacted too
    # late: the robot drove straight past a hook before the bend reached it.
    target_line_distance_mm: float = 125.0
    # Measured on Prototype A 2026-09-29: the bottom image row is 70 mm ahead.
    image_start_distance_mm: float = 70.0
    image_depth_mm: float = 220.0
    target_row_band_px: int = 6
    # Rows above the target row at which the curve term reads the line. The
    # camera now sees to the horizon; aiming at the farthest point cut corners
    # so much that the robot met hairpins from the inside.
    curve_lookahead_px: int = 45
    curve_gain: float = 0.55
    # The followed line must reach this fraction of the way from the target
    # row down to the bottom of the window: the line under the robot, not a
    # wall or another track section farther away.
    near_reach_ratio: float = 0.3
    # Hairpin: the line peaks and comes back down on one side. When the peak
    # is within this many rows above the target row (or both legs already
    # cross it), turn at hairpin_error towards the returning leg.
    hairpin_trigger_px: int = 70
    hairpin_error: float = 1.0
    # Hairpin latch: once a hairpin is seen, keep turning that way for up to
    # this many frames (~1 s at 42 Hz) after the last sighting, unless a long
    # line straight ahead shows the robot is out of it. Straddling the apex,
    # the camera sees an arch with both legs leaving the bottom, and the
    # "nearer leg" that decides the side flips as the robot drifts: on Suzuka
    # (2026-10-08 02:11) right -> left within 20 ms, then a hard left turn off
    # the inside of a right hairpin. While latched the full hairpin turn is
    # held: clamping a mis-read to zero drove the robot straight across the
    # apex for 0.4 s (2026-10-08 22:10). 0 disables the latch.
    hairpin_latch_frames: int = 0     # Suzuka: 30 (track file)
    hairpin_release_error: float = 0.35
    # A crossing or fork cancels the latch, and a "hairpin" seen this many
    # frames after a crossing is not latched: met at an angle, the robot's
    # leg and one arm of the X join above like an arch. Latched, that held a
    # full left turn through the Suzuka crossing (2026-10-08 22:48).
    hairpin_after_crossing_frames: int = 0   # Suzuka: 15 (track file)
    # At a split, take the preferred branch (the referee's main line) rather
    # than the one straight on, unless a crossing was seen within
    # split_after_crossing_frames: close up, the crossing looks like a split
    # and there straight on is right. Suzuka's pit lane leaves straight on and
    # the main line curves right: straight on took the pit in lap 1, missing
    # the finish line (2026-10-08 22:52). Off by default (Portimao, Monza).
    # A run on the steering row wider than this many line widths is two lines
    # touching: follow the part nearest the line followed so far.
    merged_width_ratio: float = 0.0   # 0 = off; Suzuka: 1.6 (track file)
    # ... only while the line followed runs near-straight ahead (|dx/drow|):
    # a single line crossing the row at a steep diagonal in a curve is also
    # wide, and holding it back there would cut the turn.
    merged_max_slope: float = 0.6
    # No bend (slowdown, forced turn) reported this many frames after a crossing.
    bend_after_crossing_frames: int = 0      # Suzuka: 40 (track file)
    split_prefer_main: bool = False
    split_after_crossing_frames: int = 60
    # 'pursuit': aim at the point of the traced line pursuit_lookahead_px
    # from the robot and steer by the angle to it (pure pursuit), which stays
    # right when the robot is at a large angle to the line. 'offset': the
    # older centre offset + curve_gain * look-ahead offset.
    steering_mode: str = "offset"
    pursuit_lookahead_px: float = 110.0
    # The robot turns about a point behind the bottom image row; in pixels.
    pivot_below_image_px: float = 40.0
    # Steering error per radian of aim angle: pi/4 (45 deg) gives 1.0.
    pursuit_error_per_rad: float = 4.0 / math.pi
    # Contours wider than this fraction of the image are treated as a
    # transverse band (a finish line) rather than as the track stripe.
    max_line_width_ratio: float = 0.50
    # A finish line crosses the track: it must reach this fraction of the
    # image width to BOTH sides of the centre. A sharp corner can be as wide,
    # but extends to one side only, and must be followed, not coasted over.
    band_min_side_ratio: float = 0.15
    # A finish line is a cross: the track stripe continues through it at the
    # same place below and above. In an S bend the stripe also runs across the
    # image, but it enters low on one side and leaves high on the other. The
    # continuations must line up within this fraction of the width.
    band_align_ratio: float = 0.15
    # A band needs the stripe visible both below and above it (see
    # _classify_wide). False restores the older rule: a wide shape with one
    # continuation in view is a band too.
    band_needs_cross: bool = True
    # Dragster: the line ends in a T at the finish line. A bar reaching out on
    # both sides of the line, with the line coming in from below and nothing
    # beyond, counts as a band. Off for the race tracks (no T there).
    band_t_end: bool = False
    # Runs of dark columns in the target band narrower than this are noise;
    # two or more wider runs are a fork (the pit lane splitting off or
    # joining), and the branch is chosen by the caller's preference.
    fork_min_run_px: int = 4
    # detect(commit=True): after the last frame with a split, stay committed
    # to the preferred branch for this many frames (~0.75 s at 40 fps); a
    # lone line less than fork_commit_px towards the branch from the main
    # line's last position is then the main line, being left.
    fork_commit_frames: int = 30
    fork_commit_px: float = 25.0
    # Straight on at splits and crossings: the direction of the line followed
    # is measured over straight_depth_px rows above the target row on plain
    # frames and trusted for straight_memory_frames frames (~0.5 s at 40 fps).
    straight_depth_px: int = 35
    straight_memory_frames: int = 20
    # A branch is straight on only if it beats the next by this much (~10 deg);
    # at a symmetric split the old choice (preference, nearest) stands.
    straight_margin_rad: float = 0.17
    # A crossing's straight way through must lie on the stripe for at least
    # this fraction of its length.
    crossing_on_stripe: float = 0.85
    # A sharp bend: the line, traced to the top of the window, runs flatter
    # than bend_slope (|dx/drow|; 1.0 = 45 deg) over bend_span_px rows, runs
    # across (bend_width_ratio times its width at the target row) or leaves
    # through a side.
    bend_slope: float = 1.0
    bend_span_px: int = 10
    bend_width_ratio: float = 2.5


@dataclass
class DetectionResult:
    found: bool
    state: str
    image_width: int
    image_height: int
    roi_top: int
    roi_bottom: int
    target_row_roi: int
    target_row_abs: int
    contour_area: float = 0.0
    line_x: Optional[float] = None
    center_error: float = 0.0
    curve_error: float = 0.0
    error: float = 0.0
    contour: Optional[Array] = None
    mask: Optional[Array] = None
    band_detected: bool = False
    band_width_ratio: float = 0.0
    checker_detected: bool = False
    fork_detected: bool = False
    fork_ahead: bool = False
    # Where a tight bend takes the line out of view, in full-image pixels:
    # the point where it leaves through the side (curve_exit) or the top of
    # a hairpin arch. Used to drive to the bend before pivoting.
    hook_px: Optional[tuple] = None
    # Far end of the followed line in view (full-image pixels): where it
    # ends, or leaves the image. Steering can aim at it (extreme-point mode).
    extreme_px: Optional[tuple] = None
    branch_count: int = 0
    # At a split: the side of the branch NOT taken ('left'/'right', '' if
    # unknown). Driving the main line, that is the pit lane's side.
    other_branch_side: str = ""
    # A sharp bend ahead of the followed line (rows above the target row
    # where it turns sideways), and its side; None when the line runs on.
    bend_px: Optional[float] = None
    bend_side: str = ""
    state_hint: str = ""


class LineDetector:
    def __init__(self, config: DetectorConfig):
        self.config = config
        self.last_line_x: Optional[float] = None
        self.last_error: float = 0.0
        self._runs_cache = {}             # id(array) -> (array, row runs), per frame
        self.commit_left = 0              # frames left committed to a branch
        self.commit_x = 0.0               # where the committed branch was
        self.track_slope: Optional[float] = None   # dx/drow of the line followed
        self.track_slope_age = 0          # frames since track_slope was measured
        self.hairpin_side: Optional[str] = None    # latched hairpin direction
        self.hairpin_left = 0             # frames the latch still holds
        self.crossing_recent = 0          # frames since a crossing, counting down
        self.frames_since_crossing = 10 ** 6
        self.line_width_px: Optional[float] = None  # one line's width on the steering row
        self.plain_slope: Optional[float] = None    # track_slope from plain tracking only
        self.plain_slope_age = 0

    def _roi_bounds(self, image_height: int) -> tuple[int, int]:
        top = int(clamp(self.config.roi_top_ratio, 0.0, 0.98) * image_height)
        bottom = int(clamp(self.config.roi_bottom_ratio, 0.02, 1.0) * image_height)
        if bottom <= top:
            bottom = min(image_height, top + 1)
        return top, bottom

    def _target_row_in_roi(self, roi_height: int, full_height: int, roi_top: int) -> int:
        if roi_height <= 0 or full_height <= 1:
            return 0
        lookahead_mm = self.config.target_line_distance_mm - self.config.image_start_distance_mm
        image_depth_mm = max(1.0, self.config.image_depth_mm)
        row_ratio = clamp(lookahead_mm / image_depth_mm, 0.0, 1.0)

        # Image row 0 is the TOP of the frame, which for a forward-and-down
        # looking camera is the FARTHEST point in view. Distance therefore
        # decreases as the row index grows, so the ratio must be inverted.
        #
        # Without the inversion, a short target distance maps to a row near the
        # top of the frame, which falls above the ROI and clamps to its first
        # row. The effect is that target_line_distance_mm has no influence at
        # all: every value produces the same target row.
        target_full_row = int(round((1.0 - row_ratio) * (full_height - 1)))
        return int(clamp(target_full_row - roi_top, 0, roi_height - 1))

    def _row_runs(self, full):
        """{row: [(start, end), ...]} of the stripe pixels in every row, in
        one vectorised pass (per-row _column_runs calls were two thirds of
        the detector's time). Cached for the frame being processed."""
        hit = self._runs_cache.get(id(full))
        if hit is not None and hit[0] is full:
            return hit[1]
        padded = np.zeros((full.shape[0], full.shape[1] + 2), dtype=np.int8)
        padded[:, 1:-1] = full > 0
        rows, cols = np.nonzero(np.diff(padded, axis=1))
        out = {}
        for r, a, b in zip(rows[0::2].tolist(), cols[0::2].tolist(), cols[1::2].tolist()):
            out.setdefault(r, []).append((a, b))
        self._runs_cache[id(full)] = (full, out)    # holds `full`: its id stays unique
        return out

    def _trace_ahead(self, full, target_row, start_x, preference, window, step=2):
        """Follow the stripe upwards (farther ahead) from the target row.

        Returns (far_x, fork_ahead, end_row, picks): far_x is where the
        followed branch is `window` rows above the target row, or the side
        edge if it leaves the image before that (None if nothing lies ahead);
        end_row is the last row reached if the stripe ENDS there (a peak or
        the end of the visible line), None if it runs to the top or an edge;
        picks maps each traced row to its (start, end) run.

        Only the stripe's own connected shape is followed, row by row. At a
        split the preferred side is taken; with no preference, the branch
        that continues the line where it was before the split (straight on),
        so a pit lane peeling off to the side is not mistaken for a curve.
        """
        width = full.shape[1]
        slack = 3 * step            # a slanted stripe moves a little per row
        cur, ref, base_w, fork = None, start_x, None, False
        far, picks, end_row = None, {}, None
        row_runs = self._row_runs(full)
        for row in range(min(target_row, full.shape[0] - 1), -1, -step):
            runs = [r for r in row_runs.get(row, ()) if r[1] - r[0] >= 2]
            if cur is None:
                if not runs:
                    continue
                cands = runs
            else:
                cands = [r for r in runs if r[0] <= cur[1] + slack and r[1] >= cur[0] - slack]
                if not cands:
                    end_row = row + step
                    break
            if cur is not None and len(cands) > 1:
                fork = True
                if preference == "left":
                    pick = min(cands, key=lambda r: r[0])
                elif preference == "right":
                    pick = max(cands, key=lambda r: r[1])
                else:
                    pick = min(cands, key=lambda r: abs((r[0] + r[1]) / 2.0 - ref))
            else:
                pick = min(cands, key=lambda r: abs((r[0] + r[1]) / 2.0 - ref))
            x0, x1 = pick
            picks[row] = pick
            within = target_row - row <= window
            if x0 <= 1 and x1 < width - 2:
                return (0.0 if within or far is None else far), fork, None, picks
            if x1 >= width - 2 and x0 > 1:
                return (float(width - 1) if within or far is None else far), fork, None, picks
            if base_w is None:
                base_w = x1 - x0
            # Inside the widened blob where two branches have not separated
            # yet, keep the line's previous position as the reference.
            if len(cands) == 1 and (x1 - x0) <= 1.6 * max(base_w, 4):
                ref = (x0 + x1) / 2.0
            if within:
                far = (x0 + x1) / 2.0
            cur = pick
        return far, fork, end_row, picks

    @staticmethod
    def _border_ends(full):
        """Where a stripe leaves the window: the midpoints (x, y) of the runs
        of stripe pixels along the window's border, in order round it. A
        line or a curve has 2 ends, a split (Y) 3, a crossing (X) 4."""
        h, w = full.shape
        xs = np.concatenate([np.arange(w), np.full(h - 1, w - 1), np.arange(w - 2, -1, -1),
                             np.zeros(h - 2, dtype=int)])
        ys = np.concatenate([np.zeros(w, dtype=int), np.arange(1, h), np.full(w - 1, h - 1),
                             np.arange(h - 2, 0, -1)])
        ring = full[ys, xs] > 0
        if ring.all() or not ring.any():
            return []
        # Start the walk at a gap so that no run wraps round the end.
        shift = int(np.flatnonzero(~ring)[0])
        ring, xs, ys = (np.roll(a, -shift) for a in (ring, xs, ys))
        ends = []
        for a, b in LineDetector._column_runs(ring):
            m = (a + b - 1) // 2
            ends.append((float(xs[m]), float(ys[m])))
        return ends

    def _crossing_aim(self, full, row):
        """At a crossing (X) the line continues straight through: x on `row`
        of the straight line from the end the robot is on (nearest the
        bottom centre) to the opposite end round the border, or None if the
        stripe is not a crossing. Suzuka is a figure of eight."""
        ends = self._border_ends(full)
        if len(ends) < 4:
            return None
        h, w = full.shape
        ours = min(range(len(ends)),
                   key=lambda i: math.hypot(ends[i][0] - w / 2.0, ends[i][1] - (h - 1)))
        (xa, ya), (xb, yb) = ends[ours], ends[(ours + len(ends) // 2) % len(ends)]
        # An X is two straight lines: the way through from our end to the
        # opposite one lies on the stripe all along. A curve or an S bend
        # leaving the window four times does not (Suzuka 2026-10-02 19:01:
        # curves taken for the crossing were driven straight into).
        if not self._on_stripe(full, (xa, ya), (xb, yb)):
            return None
        if abs(yb - ya) < 1e-6:
            return (xa + xb) / 2.0
        return xa + (row - ya) * (xb - xa) / (yb - ya)

    def _on_stripe(self, full, a, b, samples=40):
        """Whether the straight segment a-b runs on the stripe (at least
        crossing_on_stripe of points along it, ends excluded)."""
        h, w = full.shape
        t = np.linspace(0.05, 0.95, samples)
        xs = np.clip(np.round(a[0] + t * (b[0] - a[0])).astype(int), 0, w - 1)
        ys = np.clip(np.round(a[1] + t * (b[1] - a[1])).astype(int), 0, h - 1)
        return float(np.mean(full[ys, xs] > 0)) >= self.config.crossing_on_stripe

    def _straight_to_top(self, full):
        """A crossing seen far ahead, its upper legs run together at the top
        edge: three ends, one of them on the top edge, and the straight way
        from our end to it lies on the stripe. A hairpin arch reaching the
        top edge is not straight through: its inside is floor."""
        ends = self._border_ends(full)
        h, w = full.shape
        if len(ends) != 3:
            return False
        tops = [e for e in ends if e[1] <= 0.5]
        if len(tops) != 1:
            return False
        ours = min(ends, key=lambda e: math.hypot(e[0] - w / 2.0, e[1] - (h - 1)))
        return ours[1] >= h - 1.5 and self._on_stripe(full, ours, tops[0])

    @staticmethod
    def _picks_slope(picks, row, depth):
        """dx/drow of a traced leg over `depth` rows above `row`, or None."""
        rows = [k for k in picks if 0 <= row - k <= depth]
        if len(rows) < 4:
            return None
        xs = [(picks[k][0] + picks[k][1]) / 2.0 for k in rows]
        return float(np.polyfit(rows, xs, 1)[0])

    def _incoming_slope(self, fills, row, leg_w):
        """dx/drow of the leg the robot is on, measured in this frame: the
        stripe at the bottom edge nearest the image centre, traced up until
        it widens into a junction. None if too little of it is in view, or
        if at the bottom it is already much wider than one leg (the junction
        is at the edge and the legs are merged there)."""
        best = None
        for full in fills.values():
            h, w = full.shape
            for a, b in self._column_runs(full[h - 1] > 0):
                if b - a >= 3:
                    d = abs((a + b) / 2.0 - w / 2.0)
                    if best is None or d < best[0]:
                        best = (d, full, (a + b) / 2.0, b - a)
        if best is None:
            return None
        _, full, x, w0 = best
        if w0 > 1.5 * max(leg_w, 4.0):
            return None
        h = full.shape[0]
        _, _, _, picks = self._trace_ahead(full, h - 1, x, None, h - 1 - row)
        leg = {}
        for k in sorted(picks, reverse=True):          # from the bottom up
            x0, x1 = picks[k]
            # Stop where it starts to widen: a stem whose legs are still
            # merged gives the average of both directions.
            if x1 - x0 > 1.25 * max(w0, 4) or k < row:
                break
            leg[k] = picks[k]
        return self._picks_slope(leg, h - 1, h - 1 - row) if len(leg) >= 10 else None

    def _pick_straight(self, fills, row, legs, leg_w):
        """Of `legs` ((x0, x1, full) on `row`, branches going up from there),
        the one running most nearly in the direction of the line followed:
        the robot's own leg below the junction in this frame, else the
        direction remembered from recent frames, else the robot's heading
        (straight up the image: it follows its line). None if no leg is
        clearly (straight_margin_rad) straighter than the others.

        Near the robot the junction is often at the bottom edge, with no leg
        of its own below it in view; on Suzuka (2026-10-02 02:16) the robot
        also met the crossing just out of a curve, with nothing in memory."""
        reference = self._incoming_slope(fills, row, leg_w)
        if reference is None:
            fresh = (self.track_slope is not None
                     and self.track_slope_age <= self.config.straight_memory_frames)
            reference = self.track_slope if fresh else 0.0
        depth = int(self.config.straight_depth_px)
        scored = []
        for i, (x0, x1, full) in enumerate(legs):
            _, _, _, picks = self._trace_ahead(full, row, (x0 + x1) / 2.0, None, depth)
            slope = self._picks_slope(picks, row, depth)
            if slope is not None:
                scored.append((abs(math.atan(slope) - math.atan(reference)), i, slope))
            elif len(picks) < 4:
                # Leaves the image sideways within a few rows: not straight on.
                scored.append((math.pi / 2, i, None))
        if not scored or scored[0][2] is None and len(scored) == 1:
            return None
        scored.sort(key=lambda t: t[0])
        if scored[0][2] is None:
            return None
        if len(scored) > 1 and scored[1][0] - scored[0][0] < self.config.straight_margin_rad:
            return None                                  # no clear straight on
        self.track_slope, self.track_slope_age = scored[0][2], 0   # fresh through the junction
        return scored[0][1]

    def _straightest(self, runs, row, shape):
        """The run on `row` that continues the line straight on (see
        _pick_straight), or None."""
        fills = {}
        for r in runs:
            if id(r[2]) not in fills:
                fills[id(r[2])] = self._fill(r[2], shape)
        leg_w = float(np.median([r[1] - r[0] for r in runs]))
        i = self._pick_straight(fills, row, [(r[0], r[1], fills[id(r[2])]) for r in runs], leg_w)
        return None if i is None else runs[i]

    def _bend_ahead(self, full, target_row, line_x, preference, picks=None):
        """(rows ahead, side) of the first sharp bend of the followed line
        above the target row, or (None, ''). 2026-10-02 19:10 (Suzuka): every
        loss was a 5-6 cm right hairpin whose bend showed at the top of the
        view ~0.5 s ahead, while the robot ran the straight-looking leg
        below it at full speed and reached the bend too fast to turn."""
        if picks is None:
            _, _, _, picks = self._trace_ahead(full, target_row, line_x, preference, target_row)
        rows = sorted(picks, reverse=True)                # from the target row up
        if len(rows) < 3:
            return None, ""
        w = full.shape[1]
        base = max(4, picks[rows[0]][1] - picks[rows[0]][0])
        span = int(self.config.bend_span_px)
        prev_c = None
        for k in rows:
            x0, x1 = picks[k]
            c = (x0 + x1) / 2.0
            if x1 >= w - 2 and x0 > 1:
                return float(target_row - k), "right"
            if x0 <= 1 and x1 < w - 2:
                return float(target_row - k), "left"
            if x1 - x0 > self.config.bend_width_ratio * base and prev_c is not None:
                return float(target_row - k), ("right" if c > prev_c else "left")
            if k + span in picks:
                b0, b1 = picks[k + span]
                slope = (c - (b0 + b1) / 2.0) / span
                if abs(slope) > self.config.bend_slope:
                    return float(target_row - k), ("right" if slope > 0 else "left")
            prev_c = c
        return None, ""

    def _returning_leg_side(self, full, target_row, end_row, picks, step=2):
        """'left'/'right' if, below a peak at end_row, the stripe has a second
        leg coming back down beside the traced one (a hairpin), else None."""
        if end_row is None or target_row - end_row > self.config.hairpin_trigger_px:
            return None
        rows = full[end_row:target_row].copy()
        traced = []
        pad = 3 * step
        for row, (x0, x1) in picks.items():
            if end_row <= row < target_row:
                r = row - end_row
                rows[r:r + step, max(0, x0 - pad):x1 + pad] = 0
                traced.append((x0 + x1) / 2.0)
        ys, xs = np.nonzero(rows)
        if xs.size < 40 or not traced:
            return None
        return "right" if xs.mean() > np.mean(traced) else "left"

    def _joined_below(self, full, row, a, b):
        """Whether runs a and b in `row` connect at or below that row (a Y
        fork with its stem below) rather than only above (a hairpin arch)."""
        _, labels = cv2.connectedComponents((full[row:] > 0).astype(np.uint8))
        la = labels[0, (a[0] + a[1]) // 2]
        lb = labels[0, (b[0] + b[1]) // 2]
        return la != 0 and la == lb

    def _pursuit_error(self, picks, pivot):
        """Pure pursuit on the traced line: aim at the first point, going
        forward along the line from the point nearest the robot, that is
        pursuit_lookahead_px away (the nearest point if the line is farther
        than that, the far end if the line ends sooner), and steer by the
        angle to it."""
        pts = [((x0 + x1) / 2.0, float(row)) for row, (x0, x1) in sorted(picks.items(), reverse=True)]
        dist = [math.hypot(px - pivot[0], pivot[1] - py) for px, py in pts]
        near = min(range(len(pts)), key=dist.__getitem__)
        lookahead = self.config.pursuit_lookahead_px
        aim = pts[near]
        if dist[near] < lookahead:
            aim = next((pts[j] for j in range(near, len(pts)) if dist[j] >= lookahead), pts[-1])
        angle = math.atan2(aim[0] - pivot[0], pivot[1] - aim[1])
        return clamp(angle * self.config.pursuit_error_per_rad, -1.2, 1.2)

    def _checker_present(self, mask: Array, step: int = 2) -> bool:
        """Whether the chequered rectangle of a finish marker is in view.

        A chequered row holds at least three dark pieces of similar width
        (the squares; the track line crossing it may be one odd piece),
        spread over part of the width. The pattern must last over
        checker_min_rows consecutive rows, but not most of the window:
        parallel track lines would also give regular pieces, all the way up.
        """
        h, w = mask.shape[:2]
        rows = mask[::step] > 0
        # Pieces per row, all rows at once; only rows with >= 3 pieces can be
        # chequered, and usually there are none (just the track line).
        padded = np.zeros((rows.shape[0], w + 2), dtype=np.int8)
        padded[:, 1:-1] = rows
        starts = (np.diff(padded, axis=1) == 1).sum(axis=1)
        if not (starts >= 3).any():
            return False
        streak = best = 0
        for k in range(rows.shape[0]):
            if starts[k] < 3:
                streak = 0
                continue
            runs = [r for r in self._column_runs(rows[k]) if r[1] - r[0] >= 3]
            chequered = False
            if len(runs) >= 3:
                widths = np.array([b - a for a, b in runs], dtype=float)
                med = float(np.median(widths))
                good = [r for r, wd in zip(runs, widths) if 0.5 * med <= wd <= 1.8 * med]
                if len(good) >= 3:
                    gaps = np.array([good[i + 1][0] - good[i][1] for i in range(len(good) - 1)], float)
                    span = good[-1][1] - good[0][0]
                    chequered = (span >= self.config.checker_min_span * w and gaps.min() >= 3
                                 and gaps.std() <= 0.6 * gaps.mean())
            streak = streak + 1 if chequered else 0
            best = max(best, streak)
        rows = best * step
        return self.config.checker_min_rows <= rows <= 0.8 * h

    def _drop_attached_shadow(self, mask: Array, ratio: Array) -> Array:
        """Remove a dark shadow merged with the line into one blob.

        Under a single strong lamp the robot's own shadow is almost as dark
        as the tape (0.33-0.41 of the floor vs 0.06-0.2 on 2026-09-30) and
        merges with it. Such a blob holds two separate darkness levels, and
        its lighter part is large: a wide patch, not a sliver. Shiny streaks
        on the tape are also a lighter level, but narrow and small next to
        the tape. Only when both hold is the lighter level dropped.
        """
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
        min_area = max(50, int(0.01 * mask.size))
        out = mask.copy()
        for i in range(1, count):
            if stats[i, cv2.CC_STAT_AREA] < min_area:
                continue
            x, y, bw, bh = (int(v) for v in stats[i, :4])
            box = (slice(y, y + bh), slice(x, x + bw))
            sel_box = labels[box] == i
            vals = ratio[box][sel_box]
            if float(vals.max() - vals.min()) < self.config.shadow_separation:
                continue                       # one darkness level only
            scaled = np.clip(vals * 255.0, 0, 255).astype(np.uint8).reshape(-1, 1)
            t, _ = cv2.threshold(scaled, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            t /= 255.0
            dark, light = vals[vals < t], vals[vals >= t]
            if dark.size == 0 or light.size == 0:
                continue
            separated = light.mean() - dark.mean() >= self.config.shadow_separation
            big_light = light.size >= self.config.shadow_area_ratio * dark.size
            if separated and big_light and dark.mean() < 0.3:
                drop = sel_box & (ratio[box] >= t)
                keep = sel_box & ~drop
                # A shadow is a wide patch; the lighter part of the TAPE
                # (near the robot, where the track light glints on it) has
                # the tape's width. Dropping that cut the line short above
                # the steering row: false hairpins and lost hairpins on
                # 2026-10-02 19:10 (Suzuka).
                drop_w = np.count_nonzero(drop, axis=1)
                keep_w = np.count_nonzero(keep, axis=1)
                if drop_w.any() and keep_w.any() and \
                        np.median(drop_w[drop_w > 0]) <= self.config.shadow_min_width_ratio \
                        * np.median(keep_w[keep_w > 0]):
                    continue
                out[box][drop] = False
        return out

    def _relative_mask(self, blurred: Array) -> Array:
        """Line mask from darkness relative to the local floor brightness."""
        h, w = blurred.shape[:2]
        small = cv2.resize(blurred, (max(1, w // 4), max(1, h // 4)), interpolation=cv2.INTER_AREA)
        k = max(3, int(self.config.background_kernel_px) // 4) | 1
        floor = cv2.dilate(small, np.ones((k, k), np.uint8))
        floor = cv2.blur(floor, (k, k))
        floor = cv2.resize(floor, (w, h), interpolation=cv2.INTER_LINEAR)
        ratio = blurred.astype(np.float32) / np.maximum(floor.astype(np.float32), 1.0)
        strong = ratio < float(self.config.line_ratio)
        if self.config.split_shadow:
            strong = self._drop_attached_shadow(strong, ratio)
        weak_limit = float(self.config.line_ratio_weak)
        if weak_limit <= float(self.config.line_ratio):
            return (strong * 255).astype(np.uint8)
        # Hysteresis: the tape's shiny streaks (0.3-0.55 of the floor on the
        # track, 2026-09-30 video) bit notches out of the line at a single
        # cut-off, while raising that cut-off let the shadow (~0.7) in. Pixels
        # up to line_ratio_weak count only where connected to certain line.
        weak = (ratio < weak_limit).astype(np.uint8)
        count, labels = cv2.connectedComponents(weak, connectivity=8)
        if count <= 1:
            return (strong * 255).astype(np.uint8)
        keep = np.zeros(count, dtype=bool)
        keep[np.unique(labels[strong])] = True
        keep[0] = False
        return (keep[labels] * 255).astype(np.uint8)

    @staticmethod
    def _fill(contour, shape):
        full = np.zeros(shape, dtype=np.uint8)
        cv2.drawContours(full, [contour], -1, 255, thickness=cv2.FILLED)
        return full

    def _main_branch(self, preference):
        """Take the preferred (main-line) branch at a split rather than the
        one straight on: split_prefer_main, and no crossing seen lately."""
        return (self.config.split_prefer_main and preference in ("left", "right")
                and self.frames_since_crossing > self.config.split_after_crossing_frames)

    def _t_ahead(self, contours, shape, y1, bar_ratio=0.8):
        """Dragster finish T seen ahead: one stripe that reaches the bottom of
        the window (the line under the robot) and widens into a bar across at
        least bar_ratio of the width somewhere above y1, with no line (more
        than a blurred row or two) beyond the bar."""
        h, w = shape
        for contour in contours:
            if cv2.contourArea(contour) < 0.01 * h * w:
                continue
            full = np.zeros(shape, dtype=np.uint8)
            cv2.drawContours(full, [contour], -1, 255, thickness=cv2.FILLED)
            widths = np.count_nonzero(full, axis=1)
            if widths[h - 1] == 0 or widths[h - 1] > 0.5 * w:
                continue                      # not the line under the robot
            bar = np.flatnonzero(widths[:y1] >= bar_ratio * w)
            if bar.size < 3:
                continue
            beyond = np.count_nonzero((widths[:bar[0]] > 0) & (widths[:bar[0]] < bar_ratio * w))
            if beyond < 3:
                return True
        return False

    def _classify_wide(self, contour, shape, y0, y1, rows=8, preference=None, commit=False):
        """(is_finish_line, x of the stripe ahead or None, is_split) for a
        contour that spans most of the width in the target band."""
        full = np.zeros(shape, dtype=np.uint8)
        cv2.drawContours(full, [contour], -1, 255, thickness=cv2.FILLED)
        widths = np.count_nonzero(full, axis=1)
        narrow = (widths > 0) & (widths <= self.config.max_line_width_ratio * shape[1])
        above = np.flatnonzero(narrow[:y0])[-rows:]          # farther ahead
        below = y1 + np.flatnonzero(narrow[y1:])[:rows]      # nearer the robot

        def centre(sel):
            if sel.size == 0:
                return None
            ys, xs = np.nonzero(full[sel])
            return float(xs.mean()) if xs.size else None

        x_above, x_below = centre(above), centre(below)
        # (A row or two "beyond" are the bar's blurred far edge, not a line.)
        if self.config.band_t_end and above.size < 3 and below.size:
            # The finish T (dragster, 2026-10-09: read as a split or a
            # hairpin, the follower took the bar's left arm and followed the
            # track's outer edge to 11 m). Nothing beyond the bar, one stem at
            # the bottom of the image (a hairpin apex has two legs there; the
            # blurred lower edge of a close bar can split in two just below
            # it, so the bottom-most rows decide), and the bar reaches out on
            # both sides of the stem.
            stem_rows = y1 + np.flatnonzero(narrow[y1:])
            stems = [r for r in self._column_runs(full[stem_rows[-1]] > 0) if r[1] - r[0] >= 3]
            if len(stems) == 1:
                stem_x = (stems[0][0] + stems[0][1]) / 2.0
                stem_w = float(stems[0][1] - stems[0][0])
                xs = np.flatnonzero(full[y0:y1].any(axis=0))
                side = max(0.0, self.config.band_min_side_ratio) * shape[1]
                if (xs.size and xs[0] <= stem_x - side and xs[-1] >= stem_x + side
                        and xs[-1] - xs[0] >= 2.0 * max(1.0, stem_w)):
                    return True, None, False
        # Two separate legs below: the apex of a hairpin seen head-on, not a
        # finish line (whose track stripe continues as ONE stripe below it).
        if below.size:
            legs = self._column_runs(full[below[0]] > 0)
            if len([r for r in legs if r[1] - r[0] >= 3]) >= 2:
                return False, None, False
        # Two separate legs ahead: the pit lane splitting off (a V with its
        # stem near the robot), not a finish line. Calling it a band made the
        # follower coast straight past the pit entry (2026-10-02 00:28).
        if above.size:
            legs = [r for r in self._column_runs(full[above[-1]] > 0) if r[1] - r[0] >= 3]
            if len(legs) >= 2:
                straight = None
                if not (commit and preference in ("left", "right")) and not self._main_branch(preference):
                    leg_w = float(np.median([r[1] - r[0] for r in legs]))
                    straight = self._pick_straight({0: full}, int(above[-1]),
                                                   [(r[0], r[1], full) for r in legs], leg_w)
                if straight is not None:
                    leg = legs[straight]
                elif preference == "left":
                    leg = min(legs, key=lambda r: r[0])
                elif preference == "right":
                    leg = max(legs, key=lambda r: r[1])
                else:
                    ref = x_below if x_below is not None else shape[1] / 2.0
                    leg = min(legs, key=lambda r: abs((r[0] + r[1]) / 2.0 - ref))
                return False, (leg[0] + leg[1]) / 2.0, True
        # A finish line is a cross: the stripe continues below AND above it,
        # in line. With a continuation missing it is a wide curve (on the
        # practice track four curves a lap were taken for bands, and the
        # follower coasted straight through them); the finish marker there
        # is also told by its chequered part (_checker_present).
        if (self.config.band_needs_cross
                and (x_above is None or x_below is None)):
            return False, x_above, False
        s_bend = (x_above is not None and x_below is not None
                  and abs(x_above - x_below) > self.config.band_align_ratio * shape[1])
        if self.config.band_needs_cross and not s_bend:
            # The stripe of a cross reaches out on BOTH sides of the line
            # through it; a curve leaves the line to one side only.
            xs = np.flatnonzero(full[y0:y1].any(axis=0))
            line_x = (x_above + x_below) / 2.0
            side = max(0.0, self.config.band_min_side_ratio) * shape[1]
            if xs.size == 0 or xs[0] > line_x - side or xs[-1] < line_x + side:
                return False, x_above, False
            # And it is much wider than the (thin) line through it. A gentle
            # diagonal curve only just reaches the wide limit and is about as
            # wide just above and below (169 vs ~130 px on 2026-10-02).
            line_w = float(np.median(widths[np.concatenate((above, below))]))
            if line_w > 0.35 * shape[1] or xs[-1] - xs[0] < 2.0 * line_w:
                return False, x_above, False
        return not s_bend, x_above, False

    def _crosses_centre(self, columns: Array) -> bool:
        xs = np.flatnonzero(columns)
        if xs.size == 0:
            return False
        centre = columns.size / 2.0
        side = max(0.0, self.config.band_min_side_ratio) * columns.size
        return xs[0] <= centre - side and xs[-1] >= centre + side

    def _line_center_x_from_mask(self, mask: Array, target_row: int) -> float:
        band_px = max(1, int(self.config.target_row_band_px))
        y0 = int(clamp(target_row - band_px, 0, mask.shape[0] - 1))
        y1 = int(clamp(target_row + band_px + 1, 1, mask.shape[0]))
        band = mask[y0:y1, :]
        moments = cv2.moments(band, binaryImage=True)
        if moments["m00"] > 0.0:
            return float(moments["m10"] / moments["m00"])
        return mask.shape[1] / 2.0

    @staticmethod
    def _column_runs(columns: Array) -> list[tuple[int, int]]:
        """[start, end) of each run of True in a 1-D boolean array."""
        padded = np.concatenate(([False], columns, [False])).astype(np.int8)
        edges = np.flatnonzero(np.diff(padded))
        return [(int(a), int(b)) for a, b in zip(edges[::2], edges[1::2])]

    def _choose_branch(self, runs, preference, width, expected_x=None):
        if preference == "left":
            return min(runs, key=lambda r: r[0])
        if preference == "right":
            return max(runs, key=lambda r: r[1])
        # No preference: stay on the branch closest to where the line is
        # expected (from the remembered line, if the caller has one), else to
        # where it was in the last frame.
        ref = expected_x if expected_x is not None else (
            self.last_line_x if self.last_line_x is not None else width / 2.0)
        return min(runs, key=lambda r: abs((r[0] + r[1]) / 2.0 - ref))

    @staticmethod
    def _x_on_contour_fit(contour: Array, row_y: int) -> float:
        vx, vy, x0, y0 = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
        vx_f = float(vx[0])
        vy_f = float(vy[0])
        x0_f = float(x0[0])
        y0_f = float(y0[0])
        if abs(vy_f) < 1e-6:
            return x0_f
        return x0_f + ((float(row_y) - y0_f) * (vx_f / vy_f))

    def detect(self, gray: Array, preference: Optional[str] = None,
               expected_x: Optional[float] = None, commit: bool = False) -> DetectionResult:
        """Find the line. preference ('left', 'right' or None) picks the
        branch when the line forks; expected_x (image column on the target
        row, e.g. from the remembered line) keeps the choice on the line
        being followed when another line is also in view."""
        # Taking the main line at a split (split_prefer_main) commits to it
        # as the pit approach does: once the legs merge on the steering row
        # the branch is lost, and on Suzuka the follower flipped between legs
        # and left on the pit lane (2026-10-08 23:34).
        commit = commit or self._main_branch(preference)
        if gray is None or gray.size == 0:
            return DetectionResult(
                found=False,
                state="empty_image",
                image_width=0,
                image_height=0,
                roi_top=0,
                roi_bottom=0,
                target_row_roi=0,
                target_row_abs=0,
            )

        image_height, image_width = gray.shape[:2]
        self._runs_cache = {}
        top, bottom = self._roi_bounds(image_height)
        self.track_slope_age += 1
        self.plain_slope_age += 1
        roi = gray[top:bottom, :]

        blurred = cv2.GaussianBlur(roi, (5, 5), 0)

        # Contrast check on a subsample: cheap, and enough to tell a line on a
        # floor from a blank floor.
        sample = blurred[::2, ::2]
        # 0.5/99.5, not 5/95: a line covering under 5% of the window (far
        # away, or only its end in view) must still count as contrast; with
        # the taller window that was the usual case in tight bends.
        low, high = np.percentile(sample, (0.5, 99.5))
        contrast = float(high - low)

        if self.config.threshold_mode == "relative":
            if contrast < self.config.min_contrast:
                target_row = self._target_row_in_roi(roi.shape[0], image_height, top)
                return DetectionResult(
                    found=False, state="low_contrast",
                    image_width=image_width, image_height=image_height,
                    roi_top=top, roi_bottom=bottom,
                    target_row_roi=target_row, target_row_abs=top + target_row)
            mask = self._relative_mask(blurred)
        elif self.config.threshold_mode == "otsu":
            if contrast < self.config.min_contrast:
                target_row = self._target_row_in_roi(roi.shape[0], image_height, top)
                return DetectionResult(
                    found=False, state="low_contrast",
                    image_width=image_width, image_height=image_height,
                    roi_top=top, roi_bottom=bottom,
                    target_row_roi=target_row, target_row_abs=top + target_row)
            _, mask = cv2.threshold(blurred, 0, 255,
                                    cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        else:
            _, mask = cv2.threshold(blurred, int(self.config.black_threshold), 255,
                                    cv2.THRESH_BINARY_INV)
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        close = max(3, int(self.config.close_px)) | 1
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close, close)))

        target_row = self._target_row_in_roi(roi.shape[0], image_height, top)
        result = DetectionResult(
            found=False,
            state="line_lost",
            image_width=image_width,
            image_height=image_height,
            roi_top=top,
            roi_bottom=bottom,
            target_row_roi=target_row,
            target_row_abs=top + target_row,
            mask=mask,
        )

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.hairpin_left = max(0, self.hairpin_left - 1)   # no line: the latch runs down
            return result

        roi_area = float(max(1, roi.shape[0] * roi.shape[1]))
        min_area = roi_area * max(0.0, self.config.min_contour_area_ratio)

        # Classify each candidate by how much of the image WIDTH it covers
        # within the band of rows around the target row, not by its bounding
        # box. A curve crossing the ROI diagonally has a wide bounding box but
        # covers only a narrow run of columns in any single band of rows,
        # whereas a transverse finish line covers most of the width in every
        # row. Classifying by bounding box rejected curves as finish lines,
        # which is exactly where the robot was losing the line.
        band_px = max(1, int(self.config.target_row_band_px))
        y0 = int(clamp(target_row - band_px, 0, mask.shape[0] - 1))
        y1 = int(clamp(target_row + band_px + 1, y0 + 1, mask.shape[0]))
        width = mask.shape[1]

        stripes = []                       # (coverage, contour, band_mask)
        aim = {}                           # id(contour) -> x of the line ahead
        split_ahead = False
        best_band_ratio = 0.0
        for contour in contours:
            if cv2.contourArea(contour) < min_area:
                continue
            band_mask = np.zeros((y1 - y0, width), dtype=np.uint8)
            cv2.drawContours(band_mask, [contour], -1, 255,
                             thickness=cv2.FILLED, offset=(0, -y0))
            columns = band_mask.any(axis=0)
            coverage = float(np.count_nonzero(columns)) / width
            if coverage > self.config.max_line_width_ratio and self._crosses_centre(columns):
                is_band, ahead_x, is_split = self._classify_wide(
                    contour, mask.shape, y0, y1, preference=preference, commit=commit)
                if is_band:
                    best_band_ratio = max(best_band_ratio, coverage)
                    continue
                # An S bend, a corner or a split, not a finish line: follow
                # it, aiming at where the stripe (the preferred leg at a
                # split) continues ahead of the wide part.
                split_ahead = split_ahead or is_split
                if ahead_x is not None:
                    aim[id(contour)] = ahead_x
            stripes.append((coverage, contour, band_mask))

        if best_band_ratio > 0.0:
            result.band_detected = True
            result.band_width_ratio = best_band_ratio
        if self.config.band_t_end and not result.band_detected and self._t_ahead(contours, mask.shape, y1):
            # The finish T ahead of the steering row: at 1.0 m/s it crosses
            # the row in a frame or two, too briefly to rely on (2026-10-09
            # 23:47: frame 5884 showed it, 5888 was past it).
            result.band_detected = True
            result.band_width_ratio = 1.0
        if self._checker_present(mask):
            # The chequered part of the finish marker: a finish line whether
            # or not the solid stripe next to it was seen as a band, and never
            # a fork, although its squares cut the target rows into pieces.
            result.checker_detected = True
            result.band_detected = True

        if not stripes:
            result.state = "band_only" if result.band_detected else "line_too_small"
            self.hairpin_left = max(0, self.hairpin_left - 1)   # no line: the latch runs down
            return result

        # Prefer a stripe that actually crosses the target band; among those,
        # the largest. A stripe that misses the band is still usable through
        # its fitted line, which is better than steering straight ahead.
        #
        # Each separate run of dark columns in the target band is a branch.
        # A Y-shaped fork is ONE contour with two runs, so runs are counted
        # per column, not per contour.
        #
        # Only a stripe that comes down towards the robot is the line being
        # followed; a wall or another track section higher in the view is not.
        roi_h = mask.shape[0]
        near_row = target_row + self.config.near_reach_ratio * (roi_h - 1 - target_row)

        def bottom(contour):
            return int(contour[:, 0, 1].max())

        own = [st for st in stripes if bottom(st[1]) >= near_row]
        stripes = own or stripes

        min_run = max(1, int(self.config.fork_min_run_px))
        runs = []                          # (start, end, contour, band_mask)
        for coverage, contour, band_mask in stripes:
            if coverage <= 0.0:
                continue
            for x0, x1 in self._column_runs(band_mask.any(axis=0)):
                if x1 - x0 >= min_run:
                    runs.append((x0, x1, contour, band_mask))

        full = None
        hairpin = None
        crossing = False
        steer_picks = None
        if runs:
            nearest = self._choose_branch(runs, None, width, expected_x)
            same = [r for r in runs if r[2] is nearest[2] and r is not nearest]
            cross_x = None
            if same:
                full = self._fill(nearest[2], mask.shape)
                cross_x = self._crossing_aim(full, target_row)
                if cross_x is not None:
                    # A crossing, not a split or a hairpin: go straight
                    # across, whatever the branch preference.
                    crossing = True
                    nearest = min([nearest] + same, key=lambda r: abs((r[0] + r[1]) / 2.0 - cross_x))
                elif not any(self._joined_below(full, y1 - 1, nearest[:2], r[:2]) for r in same):
                    if not self._straight_to_top(full):
                        # Two legs of one stripe joined only ahead: an arch,
                        # i.e. a hairpin. Stay on our leg and turn towards
                        # the other.
                        other = min(same, key=lambda r: abs(r[0] - nearest[0]))
                        hairpin = "right" if other[0] > nearest[0] else "left"
                    else:
                        # Joined ahead but going on to the top edge: not an
                        # arch but a crossing whose far legs run into each
                        # other at the edge (Suzuka); a split has its stem
                        # below. Go straight on.
                        crossing = True
            # Two or more branches and not committed to the pit: straight on,
            # the branch continuing the direction of the line followed so
            # far. At a crossing that is the far side of the X; at the pit
            # split it is the main line. Seen near the robot, a crossing's
            # lower legs merge at the image edge and it looks just like a
            # split (Suzuka, 2026-10-02 02:09), and the nearest branch was
            # often the crossing line.
            straight = None
            main_branch = not crossing and self._main_branch(preference)
            if (not hairpin and len(runs) > 1 and not main_branch
                    and not (commit and preference in ("left", "right"))):
                straight = self._straightest(runs, target_row, mask.shape)
            if straight is not None:
                chosen = straight
            elif hairpin or crossing:
                chosen = nearest
            else:
                chosen = self._choose_branch(runs, preference, width, expected_x)
            x0, x1, best_contour, best_band_mask = chosen
            if id(best_contour) in aim:
                line_x = aim[id(best_contour)]
                result.state_hint = "split" if split_ahead else "s_bend"
            else:
                moments = cv2.moments(best_band_mask[:, x0:x1], binaryImage=True)
                line_x = x0 + float(moments["m10"] / moments["m00"])
                run_w = float(x1 - x0)
                if (len(runs) == 1 and not crossing and self.line_width_px
                        and self.config.merged_width_ratio > 0
                        and run_w > self.config.merged_width_ratio * self.line_width_px
                        and self.last_line_x is not None
                        and self.plain_slope is not None and self.plain_slope_age < 15
                        and abs(self.plain_slope) < self.config.merged_max_slope):
                    # Two lines touching on the steering row (a merge or a
                    # junction close up): stay on the line-wide part nearest
                    # our line, not the middle of both. The pit lane joining
                    # Suzuka's main line made one 158 px run; its centroid
                    # led onto the pit exit, backwards (2026-10-08 23:01).
                    half_w = self.line_width_px / 2.0
                    line_x = clamp(self.last_line_x, x0 + half_w, x1 - half_w)
                elif len(runs) == 1 and not crossing and (
                        self.line_width_px is None or run_w < 1.3 * self.line_width_px):
                    self.line_width_px = (run_w if self.line_width_px is None
                                          else 0.9 * self.line_width_px + 0.1 * run_w)
            result.branch_count = len(runs)
            result.fork_detected = ((len(runs) > 1 or split_ahead) and hairpin is None
                                    and not crossing and not result.checker_detected)
            if crossing:
                result.state_hint = "crossing"
            elif result.fork_detected and len(runs) > 1:
                others = [r for r in runs if r is not chosen]
                mid = (x0 + x1) / 2.0
                result.other_branch_side = (
                    "right" if np.mean([(r[0] + r[1]) / 2.0 for r in others]) > mid else "left")
        else:
            _, best_contour, _ = max(stripes, key=lambda s: (bottom(s[1]), cv2.contourArea(s[1])))
            bx, by, bw, bh = cv2.boundingRect(best_contour)
            exits_left = bx <= 1
            exits_right = bx + bw >= width - 1
            if by > target_row + band_px and not (exits_left or exits_right):
                # Only a piece of line behind the target row (e.g. the bottom
                # of a hairpin the robot is turning through): keep turning
                # the way it was, rather than extrapolating a fitted line.
                line_x = None
                result.state_hint = "hold_turn"
            elif exits_left != exits_right:
                # In a tight curve the line leaves through the side of the image
                # before it reaches the target row. A straight line fitted through
                # a curved stripe can point anywhere; the exit side cannot, so
                # steer as hard as the error allows towards it.
                line_x = 0.0 if exits_left else float(width - 1)
                result.state_hint = "curve_exit"
                pts = best_contour[:, 0, :]
                edge = pts[pts[:, 0] <= 1] if exits_left else pts[pts[:, 0] >= width - 2]
                if edge.size:
                    result.hook_px = (float(line_x), float(top + edge[:, 1].mean()))
            else:
                # Extrapolate the fitted line to the target row, kept inside
                # the image: a near-horizontal fit extrapolates to nonsense.
                line_x = clamp(self._x_on_contour_fit(best_contour, target_row),
                               0.0, float(width - 1))
        contour_area = float(cv2.contourArea(best_contour))
        half = max(1.0, image_width / 2.0)
        if line_x is None:
            line_x = self.last_line_x if self.last_line_x is not None else half
        center_error = (line_x - half) / half

        curve_error = 0.0
        if not result.state_hint and hairpin is None:
            # Traced from the chosen branch, so at a fork the curve term also
            # follows that branch rather than the average of both arms.
            if full is None:
                full = self._fill(best_contour, mask.shape)
            window = max(1, int(self.config.curve_lookahead_px))
            # A crossing ahead: trace straight on through it (no preference).
            ahead_crossing = self._crossing_aim(full, target_row) is not None
            ahead_x, result.fork_ahead, end_row, picks = self._trace_ahead(
                full, target_row, line_x, None if ahead_crossing else preference, window)
            if not ahead_crossing:
                steer_picks = picks         # the bend check traces the same way
            if ahead_crossing:
                result.fork_ahead = False
            else:
                hairpin = self._returning_leg_side(full, target_row, end_row, picks)
            if hairpin is None:
                slope = self._picks_slope(picks, target_row, int(self.config.straight_depth_px))
                if slope is not None:
                    self.track_slope, self.track_slope_age = slope, 0
                    self.plain_slope, self.plain_slope_age = slope, 0
            if picks:
                far_row = min(picks)
                fx0, fx1 = picks[far_row]
                ex = 0.0 if fx0 <= 1 and fx1 < width - 2 else (
                    float(width - 1) if fx1 >= width - 2 and fx0 > 1 else (fx0 + fx1) / 2.0)
                result.extreme_px = (ex, float(top + far_row))
            if ahead_x is None:
                ahead_row = max(0, target_row - window)
                ahead_x = clamp(self._x_on_contour_fit(best_contour, ahead_row)
                                - (self._x_on_contour_fit(best_contour, target_row) - line_x),
                                0.0, float(width - 1))
            curve_error = (ahead_x - line_x) / half
        total_error = center_error + (float(self.config.curve_gain) * curve_error)
        if (self.config.steering_mode == "pursuit" and not result.state_hint
                and hairpin is None and picks):
            pivot = (half, (image_height - top) + self.config.pivot_below_image_px)
            total_error = self._pursuit_error(picks, pivot)
            curve_error = (total_error - center_error) / max(1e-6, self.config.curve_gain)

        self.frames_since_crossing = 0 if crossing else self.frames_since_crossing + 1
        if crossing or result.fork_detected:
            self.hairpin_left = 0
            if crossing:
                self.crossing_recent = int(self.config.hairpin_after_crossing_frames)
        elif self.crossing_recent > 0:
            self.crossing_recent -= 1
        latched = self.hairpin_left > 0 and self.hairpin_side is not None
        if (hairpin is not None and self.config.hairpin_after_crossing_frames > 0
                and (crossing or self.crossing_recent > 0)):
            # Two arms of an X seen at an angle, not a hairpin: steer on the
            # nearest leg. Even one frame of the hairpin turn (+1.8 rad/s)
            # rotated the robot ~30 deg on top of the Suzuka crossing and it
            # left on the wrong arm (2026-10-08 23:08).
            hairpin = None
            if result.state_hint.startswith("hairpin"):
                result.state_hint = ""
        elif hairpin is not None:
            if latched and hairpin != self.hairpin_side:
                hairpin = self.hairpin_side          # the arch's other leg: same hairpin
            self.hairpin_side = hairpin
            self.hairpin_left = int(self.config.hairpin_latch_frames)
        elif latched:
            if full is None:
                full = self._fill(best_contour, mask.shape)
            ends = self._border_ends(full)
            if not (len(ends) == 2 and min(e[1] for e in ends) >= 0.5 * full.shape[0]):
                # Still an arch (both ends in the lower half: the line comes
                # back towards the robot) holds the latch; leaving the pit
                # box on Suzuka the robot needed more than 30 frames to turn
                # out of one (2026-10-08 22:17).
                self.hairpin_left -= 1
            out_of_it = (not result.state_hint and picks
                         and min(picks) <= 0.25 * target_row
                         and abs(center_error) < self.config.hairpin_release_error)
            if out_of_it:
                self.hairpin_left = 0
        if hairpin is not None:
            pts = best_contour[:, 0, :]
            apex = pts[pts[:, 1] == pts[:, 1].min()]
            result.hook_px = (float(apex[:, 0].mean()), float(top + apex[0, 1]))
            total_error = self.config.hairpin_error * (1.0 if hairpin == "right" else -1.0)
            curve_error = (total_error - center_error) / max(1e-6, self.config.curve_gain)
            result.state_hint = f"hairpin_{hairpin}"
        elif result.state_hint == "hold_turn":
            total_error = self.config.hairpin_error * (1.0 if self.last_error >= 0 else -1.0)
        if hairpin is None and self.hairpin_left > 0:
            # Inside a latched hairpin: keep the full turn towards it until a
            # line straight ahead releases the latch (above).
            sign = 1.0 if self.hairpin_side == "right" else -1.0
            if total_error * sign < self.config.hairpin_error:
                total_error = sign * self.config.hairpin_error
                curve_error = (total_error - center_error) / max(1e-6, self.config.curve_gain)
            result.state_hint = f"hairpin_{self.hairpin_side}"

        # Committing to the preferred branch (the pit entry). The pit branch
        # leaves the main line at ~35 deg right in front of the camera; on
        # 2026-10-02 01:29 it slid out of the side of the image within 0.4 s,
        # before the robot had turned, and the follower went back to the main
        # line, the only line left in view. While committed, a lone line at
        # (or beyond) the main line's last position is the line being left:
        # turn towards the branch instead. The main line's position is
        # tracked, not the branch's: once the robot turns, the branch moves
        # across the image towards the main line's side, and comparing with
        # the branch's old position took it for the main line (2026-10-02
        # 01:39, Monza: the robot kept turning past the branch).
        if commit and preference in ("left", "right"):
            sign = 1.0 if preference == "right" else -1.0
            if result.fork_detected:
                self.commit_left = int(self.config.fork_commit_frames)
                if len(runs) > 1:
                    main = (min if sign > 0 else max)(runs, key=lambda r: (r[0] + r[1]) / 2.0)
                    self.commit_x = (main[0] + main[1]) / 2.0
            elif self.commit_left > 0:
                self.commit_left -= 1
                if len(runs) == 1:
                    lone_x = (runs[0][0] + runs[0][1]) / 2.0
                    if (lone_x - self.commit_x) * sign <= float(self.config.fork_commit_px):
                        self.commit_x = lone_x       # the main line, drifting as the robot turns
                        total_error = sign * self.config.hairpin_error
                        curve_error = (total_error - center_error) / max(1e-6, self.config.curve_gain)
                        result.state_hint = "fork_commit"
                    else:
                        self.commit_left = 0         # on the branch: done
        else:
            self.commit_left = 0

        if result.extreme_px is None and result.hook_px is not None:
            result.extreme_px = result.hook_px
        if full is None:
            full = self._fill(best_contour, mask.shape)
        result.bend_px, result.bend_side = self._bend_ahead(full, target_row, line_x, preference,
                                                            steer_picks)
        if (0 < self.config.bend_after_crossing_frames
                and self.frames_since_crossing <= self.config.bend_after_crossing_frames):
            # The arms of a crossing are not a bend: no slowdown to a pivot
            # and no forced turn on top of the X.
            result.bend_px, result.bend_side = None, ""
        self.last_line_x = line_x
        self.last_error = total_error
        result.found = True
        if result.fork_detected:
            result.state = f"fork_{preference or 'nearest'}"
        else:
            result.state = result.state_hint or "tracking"
        result.contour_area = contour_area
        result.line_x = line_x
        result.center_error = center_error
        result.curve_error = curve_error
        result.error = total_error
        result.contour = best_contour
        return result


def draw_preview(bgr: Array, result: DetectionResult) -> Array:
    preview = bgr.copy()
    if result.image_width <= 0 or result.image_height <= 0:
        return preview

    cv2.rectangle(
        preview,
        (0, result.roi_top),
        (result.image_width - 1, max(result.roi_top, result.roi_bottom - 1)),
        (255, 200, 0),
        1,
    )

    image_center_x = result.image_width // 2
    cv2.line(
        preview,
        (0, result.target_row_abs),
        (result.image_width - 1, result.target_row_abs),
        (80, 80, 80),
        1,
    )
    cv2.circle(preview, (image_center_x, result.target_row_abs), 5, (255, 0, 0), -1)

    if result.found and result.line_x is not None:
        cv2.circle(preview, (int(round(result.line_x)), result.target_row_abs), 5, (0, 0, 255), -1)
        cv2.line(
            preview,
            (image_center_x, result.target_row_abs),
            (int(round(result.line_x)), result.target_row_abs),
            (0, 0, 255),
            2,
        )
        if result.contour is not None:
            contour_abs = result.contour.copy()
            contour_abs[:, 0, 1] += result.roi_top
            cv2.drawContours(preview, [contour_abs], -1, (0, 255, 0), 2)

    cv2.putText(
        preview,
        f"state={result.state} err={result.error:+.3f}",
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return preview
