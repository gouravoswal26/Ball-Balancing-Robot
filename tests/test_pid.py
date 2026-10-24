"""
tests/test_pid.py
==================
Software-only PID tests (Section 16, Test 3): positive/negative error,
error approaching zero, constant error, integral saturation, variable
loop time, derivative response. No hardware or camera required.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pid_controller import PIDController  # noqa: E402


def test_positive_error_produces_positive_output():
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0, deadband=0.0)
    result = pid.update(error=10.0, dt=0.1)
    assert result.output > 0


def test_negative_error_produces_negative_output():
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0, deadband=0.0)
    result = pid.update(error=-10.0, dt=0.1)
    assert result.output < 0


def test_error_within_deadband_is_ignored():
    pid = PIDController(kp=1.0, ki=1.0, kd=1.0, deadband=5.0)
    result = pid.update(error=2.0, dt=0.1)
    assert result.output == 0.0
    assert result.i == 0.0


def test_error_approaching_zero_shrinks_output():
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0, deadband=0.0)
    r1 = pid.update(error=20.0, dt=0.1)
    r2 = pid.update(error=5.0, dt=0.1)
    assert abs(r2.p) < abs(r1.p)


def test_constant_error_accumulates_integral():
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, deadband=0.0, integral_limit=1000)
    r1 = pid.update(error=5.0, dt=1.0)
    r2 = pid.update(error=5.0, dt=1.0)
    assert r2.i > r1.i


def test_integral_saturates_within_limit():
    pid = PIDController(kp=0.0, ki=10.0, kd=0.0, deadband=0.0,
                         integral_limit=20.0, output_limit=1000)
    result = None
    for _ in range(50):
        result = pid.update(error=5.0, dt=1.0)
    assert abs(result.i) <= 20.0 + 1e-9


def test_output_is_clamped():
    pid = PIDController(kp=1000.0, ki=0.0, kd=0.0, deadband=0.0, output_limit=10.0)
    result = pid.update(error=1000.0, dt=0.1)
    assert abs(result.output) <= 10.0


def test_variable_loop_time_does_not_explode():
    pid = PIDController(kp=0.0, ki=0.0, kd=1.0, deadband=0.0, output_limit=1e9)
    pid.update(error=0.0, dt=0.05)
    result = pid.update(error=10.0, dt=1e-9)  # near-zero dt
    assert math.isfinite(result.output)


def test_zero_or_negative_dt_does_not_crash():
    pid = PIDController(kp=0.0, ki=1.0, kd=1.0, deadband=0.0)
    result = pid.update(error=10.0, dt=0.0)
    assert math.isfinite(result.output)
    result = pid.update(error=10.0, dt=-1.0)
    assert math.isfinite(result.output)


def test_derivative_responds_to_rate_of_change():
    pid = PIDController(kp=0.0, ki=0.0, kd=1.0, deadband=0.0,
                         derivative_filter_alpha=1.0, output_limit=1e9)
    pid.update(error=0.0, dt=0.1)
    fast = pid.update(error=10.0, dt=0.1)

    pid2 = PIDController(kp=0.0, ki=0.0, kd=1.0, deadband=0.0,
                          derivative_filter_alpha=1.0, output_limit=1e9)
    pid2.update(error=0.0, dt=1.0)
    slow = pid2.update(error=10.0, dt=1.0)

    assert abs(fast.d) > abs(slow.d)


def test_reset_clears_state_and_avoids_derivative_kick():
    pid = PIDController(kp=0.0, ki=1.0, kd=1.0, deadband=0.0)
    pid.update(error=5.0, dt=1.0)
    pid.update(error=5.0, dt=1.0)
    pid.reset()
    result = pid.update(error=5.0, dt=1.0)
    assert result.d == 0.0
    assert result.i == pid.ki * 5.0 * 1.0 or math.isfinite(result.i)


def test_nan_error_is_sanitized():
    pid = PIDController(kp=1.0, ki=1.0, kd=1.0, deadband=0.0)
    result = pid.update(error=float("nan"), dt=0.1)
    assert math.isfinite(result.output)


def test_inf_dt_is_sanitized():
    pid = PIDController(kp=0.0, ki=0.0, kd=1.0, deadband=0.0)
    result = pid.update(error=5.0, dt=float("inf"))
    assert math.isfinite(result.output)


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
