"""ROS-independent command arbitration and telemetry watchdog policies."""

# References (full entries: docs/references.md at the repository root):
#   [R3] Learn2Race dissertation, requirements FR/NFR: arbitration and watchdog rules
# More details: README.md of this package

import math


def positive_seconds(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return value


class CommandGate:
    """Require live, healthy safety supervision before forwarding any source."""

    def __init__(self, competition_mode=True, autonomy_enabled=False,
                 source_timeout=0.5, safety_timeout=0.5):
        self.competition_mode = competition_mode
        self.autonomy_enabled = autonomy_enabled
        self.source_timeout = positive_seconds(source_timeout, 'source_timeout')
        self.safety_timeout = positive_seconds(safety_timeout, 'safety_timeout')
        self.armed = False
        self.fault = True
        self.ready = False
        self.fault_stamp = None
        self.ready_stamp = None
        self.sources = {}

    def set_fault(self, fault, now):
        self.fault, self.fault_stamp = bool(fault), now
        if self.fault:
            self._inhibit()

    def set_ready(self, ready, now):
        self.ready, self.ready_stamp = bool(ready), now
        if not self.ready:
            self._inhibit()

    def _inhibit(self):
        # Clearing a fault must not resume autonomy: it needs a fresh arm.
        self.armed = False
        self.sources.clear()

    def healthy(self, now):
        return (not self.fault and self.ready
                and self.fault_stamp is not None and self.ready_stamp is not None
                and 0 <= now - self.fault_stamp <= self.safety_timeout
                and 0 <= now - self.ready_stamp <= self.safety_timeout)

    def store(self, name, command, now):
        # Do not replay movements queued during a fault when it is cleared.
        if self.healthy(now):
            self.sources[name] = (command, now)

    def select(self, now):
        if not self.healthy(now):
            self._inhibit()
            return None
        allowed = ['competition']
        # Outside competition the operator's deadman-held gamepad overrides
        # autonomy, so a misbehaving follower can be taken over at once.
        if not self.competition_mode:
            allowed.append('teleop')
        if self.armed or (self.autonomy_enabled and not self.competition_mode):
            allowed.append('autonomy')
        for name in allowed:
            command, stamp = self.sources.get(name, (None, 0))
            if command is not None and 0 <= now - stamp <= self.source_timeout:
                return command
        return None


class TelemetryWatchdog:
    """Delay latching missing sensors at boot while keeping motion inhibited."""

    def __init__(self, timeouts, started, startup_grace=15.0):
        self.timeouts = {key: positive_seconds(value, key)
                         for key, value in timeouts.items()}
        self.last = {key: None for key in self.timeouts}
        self.started = started
        self.grace = float(startup_grace)
        if not math.isfinite(self.grace) or self.grace < 0:
            raise ValueError('startup_grace must be finite and nonnegative')
        self.fault_latched = False
        self.latched_reasons = []

    def mark(self, key, now):
        if key in self.last:
            self.last[key] = now

    def reset(self):
        # Preserve timestamps/deadline: reset cannot mask a missing sensor.
        self.fault_latched = False
        self.latched_reasons = []

    def check(self, now):
        problems, faults = [], []
        for key, timeout in self.timeouts.items():
            stamp = self.last[key]
            if stamp is None:
                reason = f'{key} never received'
                problems.append(reason)
                if now - self.started >= self.grace:
                    faults.append(reason)
            elif not 0 <= now - stamp <= timeout:
                reason = f'{key} stale ({now - stamp:.2f} s)'
                problems.append(reason)
                faults.append(reason)
        if faults:
            self.fault_latched = True
            self.latched_reasons = faults
        ready = not problems and not self.fault_latched
        return ready, self.fault_latched, problems or self.latched_reasons
