#!/usr/bin/env python3
"""Closed-loop simulator for the camera line follower (Prototype A).

Renders what the robot's camera would see on a virtual track, runs the real
LineDetector and the follower node's control law (PID, speed rule, line-loss
search) at the camera rate with actuation delay, and moves a unicycle robot.
Used to tune the follower offline instead of one track run per change.

Camera model measured from a recording of the preview on 2026-09-29: a 20 mm
line is ~54 px wide at the bottom row and narrows linearly towards a horizon
just above the image, giving a camera ~90 mm above the floor pitched ~32 deg
down with the standard IMX219 lens (62 deg horizontal field).

    python3 tools/follower_sim.py                 # all test tracks, defaults
    python3 tools/follower_sim.py --speed 0.4 --track hairpins --save out.png
"""

# References (full entries: docs/references.md at the repository root):
#   [R35] Hartley & Zisserman, Multiple View Geometry: camera model fitted to the robot's camera
#   [R42] Siegwart et al., Autonomous Mobile Robots: unicycle kinematics with actuation delay
#   [R30] OpenCV 4.x: rendering the virtual camera image
# More details: README.md of this package; detector.py, line_follower_node.py

from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import replace

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from learn2race_camera_follower.control import PIDConfig, PIDController, UTurn, UTurnConfig  # noqa: E402
from learn2race_camera_follower.detector import DetectorConfig, LineDetector, clamp  # noqa: E402
from learn2race_camera_follower.path_memory import GroundModel, HookPivot, MemoryPilot  # noqa: E402

MM = 0.001
# Pivot farther than this from its section of line: the robot (~150 mm wide)
# has left the line entirely (a rules penalty) or cut across to another leg.
OFF_TRACK = 0.075


# ---------------------------------------------------------------- track

def build_track(commands, start=(0.3, 0.3), heading=0.0, step=0.005):
    """Centre line from ('S', length) and ('L'|'R', radius, degrees) commands."""
    x, y, th = start[0], start[1], heading
    pts = [(x, y)]
    for cmd in commands:
        if cmd[0] == "S":
            n = max(1, int(cmd[1] / step))
            for _ in range(n):
                x += step * math.cos(th)
                y += step * math.sin(th)
                pts.append((x, y))
        else:
            radius, deg = cmd[1], cmd[2]
            sign = 1.0 if cmd[0] == "L" else -1.0
            n = max(1, int(radius * math.radians(deg) / step))
            dth = sign * math.radians(deg) / n
            for _ in range(n):
                th += dth
                x += radius * abs(dth) * math.cos(th - dth / 2)
                y += radius * abs(dth) * math.sin(th - dth / 2)
                pts.append((x, y))
    return np.array(pts)


TRACKS = {
    # gentle and 90 degree curves
    "curves": [("S", 0.6), ("R", 0.30, 90), ("S", 0.3), ("L", 0.25, 90), ("S", 0.3),
               ("R", 0.20, 120), ("S", 0.4), ("S", 0.4)],
    # tight chicanes / S bends as on Portimao
    "chicanes": [("S", 0.5), ("R", 0.15, 70), ("L", 0.15, 140), ("R", 0.15, 70), ("S", 0.3),
                 ("L", 0.12, 60), ("R", 0.12, 60), ("S", 0.4), ("S", 0.4)],
    # hairpins of decreasing radius
    # hairpins as measured on the practice track (apex radius ~5-7 cm): the
    # robot (167 mm track) can only pivot through these
    "tight": [("S", 0.5), ("R", 0.065, 180), ("S", 0.4), ("L", 0.06, 180), ("S", 0.4),
              ("R", 0.15, 90), ("L", 0.07, 180), ("S", 0.4)],
    "hairpins": [("S", 0.5), ("R", 0.20, 180), ("S", 0.4), ("L", 0.15, 180), ("S", 0.4),
                 ("R", 0.12, 180), ("S", 0.4), ("L", 0.10, 180), ("S", 0.4), ("S", 0.4)],
}


class World:
    """Top-down image of the track (1 mm per pixel) sampled by the camera."""

    def __init__(self, centre, line_width=0.020, margin=0.4):
        self.origin = centre.min(axis=0) - margin
        size = centre.max(axis=0) + margin - self.origin
        self.scale = 1.0 / MM
        w, h = int(size[0] * self.scale) + 1, int(size[1] * self.scale) + 1
        self.map = np.full((h, w), 215, np.uint8)            # light floor
        px = ((centre - self.origin) * self.scale).astype(np.int32)
        cv2.polylines(self.map, [px.reshape(-1, 1, 2)], False, 25,
                      thickness=int(line_width * self.scale), lineType=cv2.LINE_AA)
        self.centre = centre
        seg = np.diff(centre, axis=0)
        self.arclen = np.concatenate(([0.0], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))))

    def sample(self, gx, gy):
        u = ((gx - self.origin[0]) * self.scale).astype(np.int32)
        v = ((gy - self.origin[1]) * self.scale).astype(np.int32)
        inside = (u >= 0) & (v >= 0) & (u < self.map.shape[1]) & (v < self.map.shape[0])
        out = np.full(gx.shape, 215, np.uint8)
        out[inside] = self.map[v[inside], u[inside]]
        return out

    def nearest(self, x, y, near_s=None, window=0.25):
        """Distance to the centre line and arc length there. With near_s, only
        the part of the line within `window` metres ahead/behind counts, so a
        shortcut across a hairpin to the other leg is not "on the line"."""
        d = np.hypot(self.centre[:, 0] - x, self.centre[:, 1] - y)
        if near_s is not None:
            d = np.where(np.abs(self.arclen - near_s) <= window, d, np.inf)
        i = int(np.argmin(d))
        return float(d[i]), float(self.arclen[i])


class Camera:
    def __init__(self, width=320, height=240, hfov_deg=62.2, height_m=0.075,
                 pitch_deg=32.0, forward_m=0.07):
        self.w, self.h = width, height
        f = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
        u, v = np.meshgrid(np.arange(width) + 0.5, np.arange(height) + 0.5)
        # camera frame: x right, y down, z forward -> robot frame: x fwd, y left, z up
        xc, yc, zc = (u - width / 2) / f, (v - height / 2) / f, np.ones_like(u)
        p = math.radians(pitch_deg)
        fwd = zc * math.cos(p) - yc * math.sin(p)
        up = -zc * math.sin(p) - yc * math.cos(p)
        left = -xc
        self.hit = up < -1e-6
        t = np.where(self.hit, height_m / np.maximum(-up, 1e-6), 0.0)
        self.gx = forward_m + fwd * t                          # ground point, robot frame
        self.gy = left * t
        self.far = t * np.hypot(fwd, left) > 1.5

    def render(self, world, x, y, th, rng):
        c, s = math.cos(th), math.sin(th)
        wx = x + c * self.gx - s * self.gy
        wy = y + s * self.gx + c * self.gy
        img = world.sample(wx, wy).astype(np.float32)
        img[~self.hit | self.far] = 120                          # wall / far room
        img = cv2.GaussianBlur(img, (3, 3), 0) + rng.normal(0, 4, img.shape)
        return np.clip(img, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------- controller

class Follower:
    """The control law of line_follower_node, without ROS."""

    def __init__(self, det_cfg, max_speed, p):
        self.det = LineDetector(det_cfg)
        self.pid = PIDController(PIDConfig(kp=p.kp, ki=0.0, kd=p.kd, integral_limit=0.6))
        self.max_v, self.min_v, self.max_w = max_speed, 0.05, 1.8
        self.lost_w, self.slow = p.lost_turn, p.slowdown
        self.last_seen, self.search_t = 0.0, None
        self.states = {}
        self.uturn = UTurn(UTurnConfig(enabled=p.uturn))
        self.extreme = p.extreme
        self.pivot_deg = p.pivot_deg
        self.xground = GroundModel(pitch_deg=p.cam_pitch)

    def step(self, gray, now, pose=None):
        r = self.det.detect(gray, self.uturn.preference(now))
        self.states[r.state] = self.states.get(r.state, 0) + 1
        if self.extreme and r.found and r.extreme_px is not None:
            g = self.xground.to_ground(*r.extreme_px)
            if g is not None:
                ang = math.atan2(g[1], g[0])          # left positive, from the axle
                if abs(ang) > math.radians(self.pivot_deg):
                    # The line's far end is well off to one side: stop and
                    # rotate towards it before driving on.
                    self.states["pivot_to_end"] = self.states.get("pivot_to_end", 0) + 1
                    self.pid.reset()
                    self.last_seen = -ang
                    return 0.0, clamp(2.0 * ang, -self.max_w, self.max_w), r
        pivot = self.uturn.update(r, now)
        if pivot is not None:
            self.states["uturn"] = self.states.get("uturn", 0) + 1
            self.pid.reset()
            return pivot[0], pivot[1], r
        return self.step_result(r, now)

    def step_result(self, r, now):
        if r.found:
            out = self.pid.update(r.error, now)
            w = clamp(-out, -self.max_w, self.max_w)
            v = self.min_v + (self.max_v - self.min_v) * (1.0 - min(1.0, self.slow * abs(r.error)))
            self.last_seen = r.error if abs(r.error) > 1e-4 else self.last_seen
            self.search_t = None
            return clamp(v, self.min_v, self.max_v), w, r
        self.pid.reset()
        if abs(self.last_seen) > 0.05:
            self.search_t = self.search_t if self.search_t is not None else now
            if now - self.search_t <= 3.0:
                return 0.0, -math.copysign(self.lost_w, self.last_seen), r
        return 0.0, 0.0, r


class MemoryFollower(Follower):
    """Steers with MemoryPilot on the remembered line; the image-only law
    of the base class when the memory cannot steer."""

    def __init__(self, det_cfg, max_speed, p):
        super().__init__(det_cfg, max_speed, p)
        ground = GroundModel(pitch_deg=32.0 + p.pitch_err, camera_height_m=0.075 + p.height_err,
                             camera_forward_m=0.07 + p.forward_err)
        self.pilot = MemoryPilot(ground, lookahead_m=p.mem_lookahead, max_speed=max_speed,
                                 max_turn=self.max_w, lateral_accel=p.lat_acc)
        self.hook = HookPivot(ground, self.pilot.memory) if p.hook else None

    def step(self, gray, now, pose=None):
        r = self.det.detect(gray, None, self.pilot.expected_x(pose))
        self.states[r.state] = self.states.get(r.state, 0) + 1
        cmd = self.pilot.update(pose, r)
        if self.hook is not None:
            hk = self.hook.update(pose, r)
            if hk is not None:
                self.states["hook_" + self.hook.state] = self.states.get("hook_" + self.hook.state, 0) + 1
                self.pid.reset()
                return hk[0], hk[1], r
        if cmd is not None:
            self.states[self.pilot.mode] = self.states.get(self.pilot.mode, 0) + 1
            if r.found:
                self.last_seen = r.error if abs(r.error) > 1e-4 else self.last_seen
            self.search_t = None
            return cmd[0], cmd[1], r
        if self.pilot.seen_driven:
            # Only the line already driven is in view: keep searching.
            r.found = False
            self.states["driven"] = self.states.get("driven", 0) + 1
        if not r.found:
            side = self.pilot.line_side(pose)
            if side:
                # The line went left (+1) or right (-1): error has the
                # opposite sign convention (positive = line to the right).
                self.last_seen = -0.5 * side
        return self.step_result(r, now)


def run(track_name, det_cfg, speed, p, save=None, seed=1, memory=False):
    centre = build_track(TRACKS[track_name])
    world, cam = World(centre), Camera(pitch_deg=p.cam_pitch)
    rng = np.random.default_rng(seed)
    ctrl = (MemoryFollower if memory else Follower)(det_cfg, speed, p)
    x, y = centre[0]
    th = math.atan2(centre[5, 1] - centre[0, 1], centre[5, 0] - centre[0, 0])
    v = w = 0.0
    dt, delay_steps, tau = 1.0 / 40.0, 2, 0.08
    queue = [(0.0, 0.0)] * delay_steps
    ox, oy, oth = x, y, th             # odometry estimate
    total = world.arclen[-1]
    worst, progress, path = 0.0, 0.0, []
    t = 0.0
    while t < 60.0:
        img = cam.render(world, x, y, th, rng)
        cv, cw, _ = ctrl.step(img, t, (ox, oy, oth))
        queue.append((cv, cw))
        tv, tw = queue.pop(0)
        v += (tv - v) * dt / tau
        w += (tw - w) * dt / tau
        th += w * dt
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        oth += w * p.odom_turn * dt
        ox += v * p.odom_scale * math.cos(oth) * dt
        oy += v * p.odom_scale * math.sin(oth) * dt
        t += dt
        # the camera's view point (just ahead of the pivot) must stay near the line
        # Rules: off the track only when the robot is entirely to one side of
        # the line. The body (~150 mm wide) must overlap some part of the line
        # between just behind it and a little further on (cutting a hairpin
        # whose legs are closer than the body is wide is allowed).
        d, s = world.nearest(x, y, near_s=progress + 0.2, window=0.45)
        progress = max(progress, s)
        worst = max(worst, d)
        path.append((x, y))
        if d > OFF_TRACK or progress > total - 0.25:
            break
    ok = progress > total - 0.3 and worst <= OFF_TRACK
    if save:
        img = cv2.cvtColor(world.map, cv2.COLOR_GRAY2BGR)
        pts = ((np.array(path) - world.origin) * world.scale).astype(np.int32)
        cv2.polylines(img, [pts.reshape(-1, 1, 2)], False, (0, 0, 255) if not ok else (0, 160, 0), 6)
        img = cv2.flip(img, 0)
        scale = 900 / max(img.shape)
        cv2.imwrite(save, cv2.resize(img, None, fx=scale, fy=scale))
    return ok, progress / total, worst, t, ctrl.states


class Params:
    def __init__(self, **kw):
        self.kp, self.kd, self.lost_turn, self.slowdown = 1.8, 0.25, 1.2, 1.5
        self.mem_lookahead = 0.07
        self.uturn = True
        self.hook = True
        self.cam_pitch = 32.0
        self.extreme = False
        self.pivot_deg = 40.0
        self.lat_acc = 0.4
        # model / odometry errors the controller does not know about
        self.pitch_err = self.height_err = self.forward_err = 0.0
        self.odom_scale = 1.0          # odometry distance / true distance
        self.odom_turn = 1.0           # odometry rotation / true rotation
        self.__dict__.update(kw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--track", default="all", choices=["all", *TRACKS])
    ap.add_argument("--speed", type=float, default=0.3)
    ap.add_argument("--mode", default=None, help="steering_mode override (pursuit|offset)")
    ap.add_argument("--save", default=None)
    ap.add_argument("--memory", action="store_true", help="steer on the remembered line")
    ap.add_argument("--look", type=float, default=0.07, help="memory pursuit look-ahead (m)")
    ap.add_argument("--errors", action="store_true",
                    help="pitch +3 deg, height +10 mm, camera 2 cm farther, odometry 3%% long, turns 5%% short")
    args = ap.parse_args()
    cfg = DetectorConfig()
    if args.mode:
        cfg = replace(cfg, steering_mode=args.mode)
    names = list(TRACKS) if args.track == "all" else [args.track]
    for name in names:
        save = args.save.replace(".png", f"_{name}.png") if args.save else None
        extra = dict(pitch_err=3.0, height_err=0.010, forward_err=0.02, odom_scale=1.03,
                     odom_turn=0.95) if args.errors else {}
        ok, frac, worst, t, states = run(name, cfg, args.speed, Params(mem_lookahead=args.look, **extra),
                                         save, memory=args.memory)
        top = ", ".join(f"{k}:{v}" for k, v in sorted(states.items(), key=lambda kv: -kv[1])[:4])
        print(f"{name:9s} {'OK  ' if ok else 'FAIL'} {frac * 100:5.1f}% of track, worst "
              f"{worst * 1000:4.0f} mm off, {t:5.1f} s | {top}")


if __name__ == "__main__":
    main()
