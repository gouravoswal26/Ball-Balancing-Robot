"""
tests/test_servo_mapping.py
=============================
Software-only servo-mapping tests (Section 16, Test 4/2 overlap):
per-servo center/direction handling, output clamping, per-update rate
limiting, and NaN protection. Uses the SimulatedPCA9685 driver, so no
hardware or I2C bus is required.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import config  # noqa: E402
from servo_controller import ServoController  # noqa: E402

TEST_SERVO_CONFIG = [
    dict(channel=0, center_us=1500, direction=1, min_us=1300, max_us=1700),
    dict(channel=1, center_us=1550, direction=1, min_us=1350, max_us=1750),
    dict(channel=2, center_us=1500, direction=-1, min_us=1300, max_us=1700),
]


def make_controller():
    return ServoController(servo_config=TEST_SERVO_CONFIG, use_hardware=False)


def test_neutral_positions_match_config_centers():
    sc = make_controller()
    sc.move_to_neutral()
    assert [s.current_us for s in sc.servos] == [1500, 1550, 1500]


def test_direction_flip_inverts_delta_sign():
    config.IK_MODEL = "linear"
    sc = make_controller()
    sc.apply_ik_output([50, 50, 50])
    assert sc.servos[0].current_us > 1500      # direction=+1
    assert sc.servos[2].current_us < 1500      # direction=-1


def test_output_is_clamped_to_configured_range():
    config.IK_MODEL = "linear"
    sc = make_controller()
    for _ in range(50):  # enough iterations to walk past the rate limit
        sc.apply_ik_output([100000, -100000, 100000])
    for s in sc.servos:
        assert s.min_us <= s.current_us <= s.max_us


def test_rate_limit_caps_single_step_change():
    config.IK_MODEL = "linear"
    sc = make_controller()
    sc.apply_ik_output([0, 0, 0])
    sc.apply_ik_output([500, 500, 500])  # large jump attempt
    step = abs(sc.servos[0].current_us - 1500)
    assert step <= config.MAX_SERVO_STEP_US + 1e-6


def test_repeated_updates_eventually_reach_clamped_target():
    config.IK_MODEL = "linear"
    sc = make_controller()
    for _ in range(20):
        sc.apply_ik_output([1000, 0, 0])  # will clamp to +SAFETY_LIMIT_US
    expected_target = min(1500 + config.SAFETY_LIMIT_US, 1700)
    assert abs(sc.servos[0].current_us - expected_target) < 1e-6


def test_nan_input_does_not_crash_and_holds_position():
    config.IK_MODEL = "linear"
    sc = make_controller()
    sc.apply_ik_output([float("nan"), 0, 0])
    assert sc.servos[0].current_us == sc.servos[0].center_us


def test_wrong_length_ik_output_raises():
    config.IK_MODEL = "linear"
    sc = make_controller()
    try:
        sc.apply_ik_output([0, 0])
    except ValueError:
        pass
    else:
        assert False, "expected ValueError for mismatched IK output length"


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
