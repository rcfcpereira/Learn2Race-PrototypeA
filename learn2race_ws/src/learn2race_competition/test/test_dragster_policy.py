# References (full entries: docs/references.md at the repository root):
#   [R2] Dragster rules: dragster situations reproduced in the tests
# More details: dragster_policy.py (the code under test)

import pytest

from learn2race_competition.dragster_policy import FinishDetector, StartLightDetector


def ambient(detector, value=200, count=12):
    for _ in range(count):
        assert not detector.update(value)


def test_start_light_triggers_once_after_confirmation():
    d = StartLightDetector(rise_ratio=1.5, min_rise=50, confirm_samples=2)
    ambient(d)
    assert not d.update(400)
    assert d.update(400)
    assert not d.update(400)


def test_single_flash_does_not_start():
    d = StartLightDetector(confirm_samples=2)
    ambient(d)
    assert not d.update(400)
    assert not d.update(200)
    assert not d.update(400)


def test_no_start_before_baseline_learned():
    d = StartLightDetector(confirm_samples=1, min_baseline_samples=10)
    ambient(d, count=3)
    assert not d.update(1000)


def test_small_absolute_rise_in_dark_room_is_ignored():
    d = StartLightDetector(rise_ratio=1.5, min_rise=50, confirm_samples=1)
    ambient(d, value=10)
    assert not d.update(40)          # 4x but only +30 counts


def test_start_light_does_not_raise_its_own_baseline():
    d = StartLightDetector(confirm_samples=3)
    ambient(d)
    d.update(400)
    d.update(400)
    assert d.baseline == pytest.approx(200)


def drive(detector, start, stop, step=0.05, band=lambda d: False):
    reason, d = None, start
    while d <= stop + 1e-9 and reason is None:
        reason = detector.update_odom(d, 0.0) or detector.update_band(band(d))
        d += step
    return reason


# start band at 0 m, finish band at 10 m, stop band at 11 m (each 0.3 m seen)
TRACK = lambda d: d < 0.2 or 9.8 <= d <= 10.1 or 10.8 <= d <= 11.1


def test_stops_on_second_band_after_finish():
    f = FinishDetector(ignore_m=1.0, stop_band=2, overrun_m=0.15, max_distance_m=12)
    reason = drive(f, 0.0, 12.0, band=TRACK)
    assert reason.startswith('stop band 2')
    assert f.crossings[0] == pytest.approx(9.85, abs=0.06)
    assert f.crossings[1] == pytest.approx(10.85, abs=0.06)
    assert f.distance == pytest.approx(f.crossings[1] + 0.15, abs=0.06)


def test_one_wide_band_counts_once():
    f = FinishDetector(ignore_m=1.0, stop_band=2, max_distance_m=12)
    reason = drive(f, 0.0, 12.0, band=lambda d: 5.0 <= d <= 6.0)
    assert len(f.crossings) == 1
    assert 'only 1 of 2' in reason


def test_overrun_guard_stops_without_band():
    f = FinishDetector(max_distance_m=12.0)
    reason = drive(f, 0.0, 13.0)
    assert 'only 0 of 2' in reason
    assert f.distance == pytest.approx(12.0, abs=0.06)


def test_single_frame_glitch_is_not_a_band():
    f = FinishDetector(ignore_m=1.0, confirm_frames=2)
    drive(f, 0.0, 5.0, band=lambda d: abs(d - 3.0) < 0.01)
    assert f.crossings == []


def test_stop_reported_only_once():
    f = FinishDetector(max_distance_m=2.0, ignore_m=0.5)
    assert drive(f, 0.0, 2.5) is not None
    assert f.update_odom(3.0, 0.0) is None


def test_stop_commanded_at_finish_band_for_zone_stop():
    f = FinishDetector(ignore_m=1.0, stop_band=1, overrun_m=0.0, max_distance_m=11)
    reason = drive(f, 0.0, 11.0, band=TRACK)
    assert reason.startswith('stop band 1')
    assert f.distance == pytest.approx(9.85, abs=0.06)   # before the stop line at 10.8


def test_shading_the_sensor_is_not_a_start():
    """2026-10-08: a hand over the sensor pulled the ambient level down to
    9-32 counts, and normal room light (~92) then started the race."""
    from learn2race_competition.dragster_policy import StartLightDetector
    d = StartLightDetector(rise_ratio=1.5, min_rise=50.0, confirm_samples=2)
    trace = [92] * 50 + [5] * 40 + [92] * 30 + [0] * 25 + [91] * 30
    assert not any(d.update(v) for v in trace)
    # ...while the start light still triggers.
    assert any(d.update(v) for v in [300] * 3)


def test_real_start_after_room_light_change():
    from learn2race_competition.dragster_policy import StartLightDetector
    d = StartLightDetector(rise_ratio=1.5, min_rise=50.0, confirm_samples=2)
    for v in [210] * 40:
        assert not d.update(v)
    assert any(d.update(v) for v in [330] * 3)      # +120 counts, 1.57 x (a recorded race start)
