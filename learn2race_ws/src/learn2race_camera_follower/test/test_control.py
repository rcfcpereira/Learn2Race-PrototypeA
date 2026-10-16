# References (full entries: docs/references.md at the repository root):
#   [R40] Astrom & Murray, Feedback Systems (PID): PID behaviour under test
# More details: control.py (the code under test)

from learn2race_camera_follower.control import PIDConfig, PIDController


def test_pid_zero_error_stays_zero():
    controller = PIDController(PIDConfig())
    assert controller.update(0.0, 1.0) == 0.0
    assert controller.update(0.0, 1.1) == 0.0


def test_pid_positive_error_gives_positive_output():
    controller = PIDController(PIDConfig(kp=2.0, ki=0.0, kd=0.0))
    output = controller.update(0.3, 1.0)
    assert output > 0.0
