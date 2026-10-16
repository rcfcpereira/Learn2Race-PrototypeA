"""Remembered line for a forward-looking camera.

The camera's nearest visible ground is well ahead of the wheels (~13 cm on
Prototype A with the camera pitched ~32 deg down), so steering on what it
sees now always aims that far ahead and must cut any bend tighter than that.
Instead, each frame's line point on the steering row is converted to floor
coordinates and stored in the odometry frame; the robot then steers by pure
pursuit on the stored line a short distance ahead of its wheels, which it
reaches a fraction of a second after the camera saw it. The stored line also
carries the robot through a hairpin after the line has left the camera view.
"""

# References (full entries: docs/references.md at the repository root):
#   [R35] Hartley & Zisserman, Multiple View Geometry: pinhole projection between image and ground plane
#   [R42] Siegwart et al., Autonomous Mobile Robots: odometry dead reckoning of the remembered path
#   [R41] Coulter (1992) pure pursuit: pure-pursuit aim point
# More details: README.md of this package; tools/follower_sim.py (only used there)

from __future__ import annotations

import math

import numpy as np
from dataclasses import dataclass


@dataclass(frozen=True)
class GroundModel:
    """Pinhole camera over a flat floor. Robot frame: x forward from the
    wheel axle, y to the left. Values measured on Prototype A 2026-09-29
    from the width of the 20 mm line across the image."""
    width: int = 320
    height: int = 240
    hfov_deg: float = 62.2
    camera_height_m: float = 0.075      # measured on the robot 2026-09-29
    pitch_deg: float = 32.0
    camera_forward_m: float = 0.07       # camera ahead of the wheel axle

    def to_ground(self, u: float, v: float):
        """Floor point (x, y) in the robot frame seen at pixel (u, v), or
        None above the horizon."""
        f = (self.width / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)
        xc, yc = (u - self.width / 2.0) / f, (v - self.height / 2.0) / f
        p = math.radians(self.pitch_deg)
        fwd = math.cos(p) - yc * math.sin(p)
        up = -math.sin(p) - yc * math.cos(p)
        if up >= -1e-6:
            return None
        t = self.camera_height_m / -up
        return self.camera_forward_m + fwd * t, -xc * t

    def to_pixel(self, x: float, y: float):
        """Pixel (u, v) where the floor point (x, y) of the robot frame
        appears, or None if it is behind the camera."""
        f = (self.width / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)
        p = math.radians(self.pitch_deg)
        fwd, left, up = x - self.camera_forward_m, y, -self.camera_height_m
        zc = fwd * math.cos(p) - up * math.sin(p)
        yc = -fwd * math.sin(p) - up * math.cos(p)
        if zc <= 1e-6:
            return None
        return self.width / 2.0 + f * (-left) / zc, self.height / 2.0 + f * yc / zc


def _to_world(pose, point):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return x + c * point[0] - s * point[1], y + s * point[0] + c * point[1]


def _to_robot(pose, point):
    x, y, th = pose
    dx, dy = point[0] - x, point[1] - y
    c, s = math.cos(th), math.sin(th)
    return c * dx + s * dy, -s * dx + c * dy


class PathMemory:
    """Line points in the odometry frame, in the order they were driven."""

    def __init__(self, spacing_m=0.005, max_jump_m=0.04, keep_m=0.6, patience=12):
        self.spacing, self.max_jump, self.keep = spacing_m, max_jump_m, keep_m
        self.patience = patience
        self.points: list[tuple[float, float]] = []
        self.rejected = 0

    def clear(self):
        self.points.clear()
        self.rejected = 0

    def last_in_robot(self, pose):
        """The most recently stored point in the robot frame, or None."""
        return _to_robot(pose, self.points[-1]) if self.points else None

    def add(self, pose, point_robot):
        """Store the line point seen now (robot frame) at odometry pose."""
        p = _to_world(pose, point_robot)
        if self.points:
            last = self.points[-1]
            gap = math.hypot(p[0] - last[0], p[1] - last[1])
            if gap < self.spacing:
                return
            if gap > self.max_jump:
                # Not the continuation of the stored line: another line in
                # view, or noise. Ignore it; only if it persists (the robot
                # really is somewhere else now) start again from here.
                self.rejected += 1
                if self.rejected < self.patience:
                    return
                self.points.clear()
        self.rejected = 0
        self.points.append(p)
        # Forget what is well behind the robot.
        while len(self.points) > 2:
            bx, _ = _to_robot(pose, self.points[0])
            if bx < -self.keep / 3 or len(self.points) > 400:
                self.points.pop(0)
            else:
                break

    def pursuit(self, pose, lookahead_m):
        """(curvature, remaining_m) to steer by, or None if the stored line
        does not reach far enough ahead of the wheels to use.

        Pure pursuit on the stored line: the first stored point, going
        forward from the one nearest the robot, at lookahead_m from the
        wheel axle; curvature = 2*y / d^2. remaining_m is how far the stored
        line still extends ahead of the robot.
        """
        if len(self.points) < 3:
            return None
        local = [_to_robot(pose, p) for p in self.points]
        dist = [math.hypot(px, py) for px, py in local]
        near = min(range(len(local)), key=dist.__getitem__)
        aim = next((j for j in range(near, len(local)) if dist[j] >= lookahead_m), None)
        remaining = dist[-1]
        if aim is None or local[aim][0] <= 0.0:
            return None
        px, py = local[aim]
        d2 = px * px + py * py
        return 2.0 * py / d2, remaining

    def is_driven(self, pose, point_robot, radius_m=0.03, frontier_m=0.06):
        """Whether a line point seen now lies on the stored line already
        driven (not its newest frontier_m). Pivoting at a hairpin apex brings
        the leg the robot came along back into view; it must not be taken
        as the way on."""
        if len(self.points) < 3:
            return False
        p = _to_world(pose, point_robot)
        length, cut = 0.0, 0
        for i in range(len(self.points) - 1, 0, -1):
            a, b = self.points[i], self.points[i - 1]
            length += math.hypot(a[0] - b[0], a[1] - b[1])
            if length >= frontier_m:
                cut = i - 1
                break
        return any(math.hypot(p[0] - q[0], p[1] - q[1]) < radius_m for q in self.points[:cut + 1]) \
            if cut > 0 else False

    def tail_curvature(self, span_m=0.08):
        """Signed curvature (1/m, left positive) of the last span_m of the
        stored line, from the circle through its first, middle and last
        points; None if the stored line is shorter than half the span."""
        if len(self.points) < 3:
            return None
        tail = [self.points[-1]]
        length = 0.0
        for p in reversed(self.points[:-1]):
            length += math.hypot(p[0] - tail[-1][0], p[1] - tail[-1][1])
            tail.append(p)
            if length >= span_m:
                break
        if length < span_m / 2 or len(tail) < 3:
            return None
        a, b, c = tail[-1], tail[len(tail) // 2], tail[0]     # oldest -> newest
        cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        ab = math.hypot(b[0] - a[0], b[1] - a[1])
        bc = math.hypot(c[0] - b[0], c[1] - b[1])
        ca = math.hypot(a[0] - c[0], a[1] - c[1])
        if ab * bc * ca < 1e-9:
            return 0.0
        return 2.0 * cross / (ab * bc * ca)


class MemoryPilot:
    """Steering and speed from the remembered line.

    update() returns (linear, angular) or None when the memory cannot steer
    (nothing stored yet, or blind for too long): the caller then falls back
    to steering on the image alone, or to searching.
    """

    # Detector states whose line_x is a real line point on the target row.
    REAL_POINT = ("tracking", "fork_nearest", "fork_left", "fork_right",
                  "hairpin_left", "hairpin_right")

    def __init__(self, ground: GroundModel, lookahead_m=0.07, max_speed=0.3,
                 min_speed=0.05, max_turn=1.8, lateral_accel=0.4,
                 blind_speed=0.08, blind_max_m=0.0, slowdown=1.5):
        self.ground = ground
        self.memory = PathMemory()
        self.lookahead = lookahead_m
        self.max_v, self.min_v, self.max_w = max_speed, min_speed, max_turn
        self.lat_acc, self.blind_v, self.blind_max = lateral_accel, blind_speed, blind_max_m
        self.slowdown = slowdown
        self.blind = 0.0                  # distance driven since the last stored point
        self.seen_driven = False          # this frame's line is one already driven
        self.last_pose = None
        self.mode = "none"

    def line_side(self, pose):
        """+1 if the newest stored point is left of the robot, -1 if right,
        0 if nothing is stored: where to search when the line is lost."""
        last = self.memory.last_in_robot(pose)
        if last is None or abs(last[1]) < 0.005:
            return 0
        return 1 if last[1] > 0 else -1

    def expected_x(self, pose):
        """Image column where the line should be on the target row, from the
        newest stored point, to keep the detector on the followed line."""
        last = self.memory.last_in_robot(pose)
        if last is None:
            return None
        px = self.ground.to_pixel(*last)
        if px is None or not (0 <= px[0] < self.ground.width and 0 <= px[1] < self.ground.height):
            return None
        return px[0]

    def update(self, pose, result):
        if self.last_pose is not None:
            self.blind += math.hypot(pose[0] - self.last_pose[0], pose[1] - self.last_pose[1])
        self.last_pose = pose
        stored_before = len(self.memory.points), self.memory.points[-1] if self.memory.points else None
        self.seen_driven = False
        if result.found and result.state in self.REAL_POINT and result.line_x is not None:
            pt = self.ground.to_ground(result.line_x, result.target_row_abs)
            if pt is not None:
                if self.memory.is_driven(pose, pt):
                    # The line already driven (seen again while pivoting at a
                    # hairpin): neither store nor follow it.
                    self.seen_driven = True
                else:
                    self.memory.add(pose, pt)
        if self.memory.points and (len(self.memory.points), self.memory.points[-1]) != stored_before:
            self.blind = 0.0

        steer = self.memory.pursuit(pose, self.lookahead)
        if steer is not None:
            kappa = steer[0]
            self.mode = "memory"
        else:
            # The stored line ends before the look-ahead: the line has left
            # the camera view (a hairpin). Optionally keep the curvature the
            # line had at its end (blind_max_m > 0). Off by default: in the
            # simulator the stored tail is mostly the straight entry of the
            # hairpin, so it underestimates the bend (0.9 vs 6.7 1/m) and
            # searching towards line_side() recovers better.
            if result.found:
                # The camera sees the line: steering on the image is better
                # than guessing.
                self.mode = "none"
                return None
            kappa = self.memory.tail_curvature()
            if kappa is None or self.blind > self.blind_max:
                self.mode = "none"
                return None
            self.mode = "blind"

        # Speed from what the camera sees ahead, as the image-only law does:
        # slow for a bend before reaching it, crawl while the line is out of
        # view. Also capped by the bend being driven now.
        err = abs(result.error) if result.found else 1.0
        v = self.min_v + (self.max_v - self.min_v) * (1.0 - min(1.0, self.slowdown * err))
        if abs(kappa) > 1e-6:
            v = min(v, math.sqrt(self.lat_acc / abs(kappa)), self.max_w / abs(kappa))
        if self.mode == "blind":
            v = self.blind_v
        v = max(self.min_v, min(self.max_v, v))
        w = max(-self.max_w, min(self.max_w, v * kappa))
        return v, w


class HookPivot:
    """Drive to a tight bend remembered by odometry, then pivot through it.

    A hairpin tighter than the robot is wide (the practice track's are
    ~5-7 cm in radius) takes the line out of the camera view well before the
    wheels get there. When the detector reports where the line leaves the
    view (result.hook_px), that floor point is stored in the odometry frame.
    The robot keeps following, slowly, until its axle reaches the point, then
    pivots towards the bend's side. While pivoting it ignores the line it has
    already driven (the path memory knows it) and stops at the first other
    line on the turning side or ahead: the leg that leads on.
    """

    HOOK_STATES = ("curve_exit", "hairpin_left", "hairpin_right")

    def __init__(self, ground: GroundModel, memory: PathMemory, approach_speed=0.10,
                 turn_speed=1.5, arm_within_m=0.35, pivot_at_m=0.0, min_turn_deg=45.0,
                 max_turn_deg=300.0, accept_margin=0.10, max_radius_m=1.0):
        self.ground, self.memory = ground, memory
        self.approach_v, self.turn_w = approach_speed, turn_speed
        self.arm_within, self.pivot_at = arm_within_m, pivot_at_m
        self.min_turn, self.max_turn = math.radians(min_turn_deg), math.radians(max_turn_deg)
        self.margin = accept_margin
        self.max_radius = max_radius_m
        self.reset()

    def reset(self):
        self.target = None          # hook point, odometry frame
        self.side = None            # 'left' / 'right'
        self.pivoting = False
        self.turned = 0.0
        self.last_yaw = None
        self.state = "idle"
        self.speed_cap = None       # while approaching: cap for the caller's speed

    def bend_radius(self, result):
        """Radius (m) of the circle best fitting the visible line's centre on
        the floor, or None. Meant to tell a hairpin from an ordinary curve, but
        in the simulator it reads 30-50% tight on the short arc in view and
        chicanes (12-15 cm: 4-9 cm read) overlap hairpins (6.5 cm: 2-6 cm
        read), so max_radius_m defaults to 1.0 (filter off)."""
        if result.contour is None or result.mask is None:
            return None
        import cv2
        full = np.zeros(result.mask.shape, np.uint8)
        cv2.drawContours(full, [result.contour], -1, 255, thickness=cv2.FILLED)
        pts = []
        for v in range(full.shape[0] - 1, -1, -3):
            xs = np.flatnonzero(full[v])
            if xs.size and xs[-1] - xs[0] + 1 == xs.size:        # one run: a centre
                g = self.ground.to_ground((xs[0] + xs[-1]) / 2.0, v + result.roi_top)
                if g is not None:
                    pts.append(g)
        if len(pts) < 6:
            return None
        g = np.array(pts)
        a = np.c_[2 * g[:, 0], 2 * g[:, 1], np.ones(len(g))]
        b = (g ** 2).sum(axis=1)
        (cx, cy, c), *_ = np.linalg.lstsq(a, b, rcond=None)
        r2 = c + cx * cx + cy * cy
        return math.sqrt(r2) if r2 > 0 else None

    @staticmethod
    def _side(result):
        if result.state.startswith("hairpin_"):
            return result.state.split("_", 1)[1]
        return "right" if result.error > 0 else "left"

    def update(self, pose, result):
        """(linear, angular) while approaching or pivoting, else None."""
        if not self.pivoting and result.found and result.state in self.HOOK_STATES \
                and result.hook_px is not None:
            pt = self.ground.to_ground(*result.hook_px)
            radius = self.bend_radius(result) if self.target is None else 0.0
            if pt is not None and 0.0 < pt[0] <= self.arm_within                     and radius is not None and radius <= self.max_radius:
                new = _to_world(pose, pt)
                # Refine with nearer sightings of the same bend only; a line
                # farther down the track leaving the view must not move the
                # target away from the bend the robot is about to reach.
                if self.target is None or math.hypot(new[0] - self.target[0],
                                                     new[1] - self.target[1]) < 0.06:
                    self.target = new
                    self.side = self._side(result)
        if self.target is None:
            self.state = "idle"
            return None

        if not self.pivoting:
            tx, _ = _to_robot(pose, self.target)
            if tx > self.pivot_at:
                # Approach: straight on towards the bend, slowly; following
                # the line here would turn early and cut inside the hairpin.
                self.state = "approach"
                return self.approach_v, 0.0
            if result.found and result.state == "tracking" and abs(result.error) < 0.4:
                # At the bend and the line is comfortably in view: an
                # ordinary curve, already being followed. No pivot.
                self.reset()
                return None
            self.pivoting, self.turned, self.last_yaw = True, 0.0, pose[2]

        # Pivot, accumulating the angle turned from odometry.
        d = math.atan2(math.sin(pose[2] - self.last_yaw), math.cos(pose[2] - self.last_yaw))
        self.turned += abs(d)
        self.last_yaw = pose[2]
        self.state = "pivot"
        if self.turned >= self.min_turn and result.found and result.line_x is not None \
                and not result.state.startswith("hairpin"):
            half = self.ground.width / 2.0
            pt = self.ground.to_ground(result.line_x, result.target_row_abs)
            driven = pt is not None and self.memory.is_driven(pose, pt, radius_m=0.04, frontier_m=0.0)
            on_side = (result.line_x >= half * (1 - self.margin) if self.side == "right"
                       else result.line_x <= half * (1 + self.margin))
            if not driven and on_side:
                self.reset()
                return None
        if self.turned >= self.max_turn:
            self.reset()
            return None
        return 0.0, (self.turn_w if self.side == "left" else -self.turn_w)
