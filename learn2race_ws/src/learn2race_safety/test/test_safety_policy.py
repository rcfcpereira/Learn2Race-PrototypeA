# References (full entries: docs/references.md at the repository root):
#   [R3] Learn2Race dissertation, requirements FR/NFR: safety rules under test
# More details: safety_policy.py (the code under test)

import pytest

from learn2race_safety.safety_policy import CommandGate, TelemetryWatchdog


def healthy_gate(**kwargs):
    gate = CommandGate(**kwargs)
    gate.set_fault(False, 0.0)
    gate.set_ready(True, 0.0)
    return gate


def test_gate_blocks_until_both_heartbeats_arrive():
    gate = CommandGate(competition_mode=False)
    gate.store('teleop', 'go', 0.0)
    assert gate.select(0.0) is None
    gate.set_fault(False, 0.0)
    gate.store('teleop', 'go', 0.0)
    assert gate.select(0.0) is None


def test_gate_expires_stale_supervision_and_sources():
    gate = healthy_gate(competition_mode=False)
    gate.store('teleop', 'go', 0.4)
    assert gate.select(0.4) == 'go'
    assert gate.select(0.6) is None  # heartbeats older than safety_timeout


def test_competition_mode_blocks_teleop_and_unarmed_autonomy():
    gate = healthy_gate(autonomy_enabled=True)
    gate.store('teleop', 'joy', 0.0)
    gate.store('autonomy', 'auto', 0.0)
    assert gate.select(0.0) is None
    gate.armed = True
    assert gate.select(0.0) == 'auto'


def test_competition_stop_has_priority():
    gate = healthy_gate()
    gate.armed = True
    gate.store('autonomy', 'auto', 0.0)
    gate.store('competition', 'stop', 0.0)
    assert gate.select(0.0) == 'stop'


def test_teleop_overrides_autonomy_outside_competition():
    gate = healthy_gate(competition_mode=False, autonomy_enabled=True)
    gate.store('autonomy', 'auto', 0.0)
    assert gate.select(0.0) == 'auto'
    gate.store('teleop', 'joy', 0.1)
    assert gate.select(0.1) == 'joy'


def test_released_gamepad_returns_control_to_autonomy():
    gate = healthy_gate(competition_mode=False, autonomy_enabled=True)
    gate.set_fault(False, 0.6)
    gate.set_ready(True, 0.6)
    gate.store('teleop', 'joy', 0.0)
    gate.store('autonomy', 'auto', 0.6)
    assert gate.select(0.6) == 'auto'  # teleop older than source_timeout


def test_fault_disarms_and_drops_queued_commands():
    gate = healthy_gate()
    gate.armed = True
    gate.store('autonomy', 'auto', 0.0)
    gate.set_fault(True, 0.1)
    gate.store('autonomy', 'queued', 0.1)
    gate.set_fault(False, 0.2)
    gate.set_ready(True, 0.2)
    gate.store('autonomy', 'auto', 0.2)
    assert not gate.armed
    assert gate.select(0.2) is None


def test_stale_supervision_disarms():
    gate = healthy_gate()
    gate.armed = True
    assert gate.select(1.0) is None
    assert not gate.armed


@pytest.mark.parametrize('value', [0, -1, float('nan'), float('inf')])
def test_gate_rejects_invalid_timeouts(value):
    with pytest.raises(ValueError):
        CommandGate(source_timeout=value)


def test_watchdog_waits_during_grace_then_latches():
    dog = TelemetryWatchdog({'odom': 0.5}, started=0.0, startup_grace=5.0)
    assert dog.check(1.0) == (False, False, ['odom never received'])
    ready, faulted, _ = dog.check(5.0)
    assert (ready, faulted) == (False, True)


def test_watchdog_latch_survives_recovery_until_reset():
    dog = TelemetryWatchdog({'odom': 0.5}, started=0.0, startup_grace=0.0)
    dog.mark('odom', 0.0)
    assert dog.check(0.1)[:2] == (True, False)
    assert dog.check(1.0)[:2] == (False, True)
    dog.mark('odom', 1.1)
    assert dog.check(1.2)[:2] == (False, True)
    dog.reset()
    assert dog.check(1.2)[:2] == (True, False)


def test_watchdog_reset_cannot_mask_missing_sensor():
    dog = TelemetryWatchdog({'imu': 1.0}, started=0.0, startup_grace=0.0)
    dog.mark('imu', 0.0)
    dog.check(2.0)
    dog.reset()
    assert dog.check(2.0)[:2] == (False, True)
