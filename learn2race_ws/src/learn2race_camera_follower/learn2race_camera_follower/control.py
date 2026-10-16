"""Control helpers for the line follower."""

# References (full entries: docs/references.md at the repository root):
#   [R40] Astrom & Murray, Feedback Systems (PID): PID controller with derivative and integral limit
#   [R42] Siegwart et al., Autonomous Mobile Robots: differential-drive turning
# More details: README.md of this package; line_follower_node.py

from __future__ import annotations

from dataclasses import dataclass

from .detector import clamp


@dataclass(frozen=True)
class PIDConfig:
    kp: float = 1.8
    ki: float = 0.0
    kd: float = 0.25
    integral_limit: float = 0.6


class PIDController:
    def __init__(self, config: PIDConfig):
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time_s: float | None = None

    def update(self, error: float, now_s: float) -> float:
        derivative = 0.0
        if self.last_time_s is not None:
            dt = max(1e-3, now_s - self.last_time_s)
            self.integral += error * dt
            self.integral = clamp(
                self.integral,
                -abs(self.config.integral_limit),
                abs(self.config.integral_limit),
            )
            derivative = (error - self.last_error) / dt

        self.last_error = error
        self.last_time_s = now_s
        return (
            (self.config.kp * error)
            + (self.config.ki * self.integral)
            + (self.config.kd * derivative)
        )


@dataclass(frozen=True)
class UTurnConfig:
    enabled: bool = True
    turn_speed: float = 1.5       # rad/s while pivoting
    min_s: float = 0.35           # pivot at least this long (~45 deg) first
    max_s: float = 4.0            # then give up and let the normal search run
    accept_margin: float = 0.10   # returning leg must be this far to the turn side
    hold_s: float = 0.6           # prefer the turn side at splits afterwards
    loss_window_s: float = 0.3    # line lost this soon after a hook also triggers


class UTurn:
    """Pivot through a U-turn tighter than the robot can drive.

    The practice-track hairpins are ~5-7 cm in radius: the two legs are
    closer together than the robot is wide, and the returning leg is outside
    the camera view until the robot has turned. When the line hooks out of the
    image to one side (curve_exit, a hairpin arch) the robot stops and pivots
    towards that side. It ignores the leg it came along, which during the
    pivot lies on the other side of the image, and stops as soon as a line
    appears on the turning side: the returning leg. The body keeps covering
    the line throughout, so this is not leaving the track.
    """

    TRIGGERS = ("curve_exit", "hairpin_left", "hairpin_right")

    def __init__(self, config: UTurnConfig):
        self.config = config
        self.side: str | None = None          # 'left' / 'right' while pivoting
        self.started = 0.0
        self.prefer_side: str | None = None
        self.prefer_until = 0.0
        self.last_hook: tuple[float, str] | None = None

    @property
    def active(self) -> bool:
        return self.side is not None

    def preference(self, now: float) -> str | None:
        return self.prefer_side if now < self.prefer_until else None

    @staticmethod
    def _side_of(result) -> str:
        if result.state.startswith("hairpin_"):
            return result.state.split("_", 1)[1]
        return "right" if result.error > 0 else "left"

    def update(self, result, now: float):
        """(linear, angular) while pivoting, else None (steer normally)."""
        cfg = self.config
        if not cfg.enabled:
            return None
        if self.side is None:
            hook = result.found and result.state in self.TRIGGERS
            lost_after_hook = (not result.found and self.last_hook is not None
                               and now - self.last_hook[0] <= cfg.loss_window_s)
            if hook:
                self.last_hook = (now, self._side_of(result))
            if not (hook or lost_after_hook):
                return None
            self.side = self._side_of(result) if hook else self.last_hook[1]
            self.started = now
        elapsed = now - self.started
        half = result.image_width / 2.0 if result.image_width else 160.0
        on_turn_side = result.found and result.line_x is not None and (
            result.line_x > half * (1 + cfg.accept_margin) if self.side == "right"
            else result.line_x < half * (1 - cfg.accept_margin))
        if elapsed > cfg.max_s or (elapsed >= cfg.min_s and on_turn_side
                                   and not result.state.startswith("hairpin")):
            self.prefer_side, self.prefer_until = self.side, now + cfg.hold_s
            self.side, self.last_hook = None, None
            return None
        # Positive angular.z turns left.
        return 0.0, (cfg.turn_speed if self.side == "left" else -cfg.turn_speed)
