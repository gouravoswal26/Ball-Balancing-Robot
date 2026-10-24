"""
tests/test_kinematics.py
=========================
Software-only IK tests (Section 16, Test 4): neutral orientation, small
positive/negative roll, small positive/negative pitch, and invalid /
unreachable poses -- for both the "linear" and "analytical" IK models.

IMPORTANT: these tests check internal consistency of the math (does
neutral give ~zero output, is positive/negative symmetric, are
unreachable poses rejected) -- they do NOT and cannot confirm the
"analytical" model matches the real mechanism. That still has to be
checked against the CAD model / robot photo and cross-referenced with
calibration.py test-direction on the physical robot.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import config  # noqa: E402
from inverse_kinematics import inverse_kinematics, IKUnreachable  # noqa: E402


def setup_function(_):
    # Each test sets config.IK_MODEL explicitly rather than relying on
    # whatever the previous test left behind.
    pass


def test_neutral_pose_linear_model_gives_zero_deltas():
    config.IK_MODEL = "linear"
    values = inverse_kinematics(0.0, 0.0)
    assert len(values) == 3
    assert all(abs(v) < 1e-9 for v in values)


def test_small_positive_pitch_linear_model_nonzero():
    config.IK_MODEL = "linear"
    values = inverse_kinematics(0.0, 5.0)
    assert any(abs(v) > 0 for v in values)


def test_positive_and_negative_pitch_are_antisymmetric():
    config.IK_MODEL = "linear"
    pos = inverse_kinematics(0.0, 5.0)
    neg = inverse_kinematics(0.0, -5.0)
    for p, n in zip(pos, neg):
        assert math.isclose(p, -n, abs_tol=1e-9)


def test_positive_and_negative_roll_are_antisymmetric():
    config.IK_MODEL = "linear"
    pos = inverse_kinematics(5.0, 0.0)
    neg = inverse_kinematics(-5.0, 0.0)
    for p, n in zip(pos, neg):
        assert math.isclose(p, -n, abs_tol=1e-9)


def test_yaw_rejected_in_linear_model():
    config.IK_MODEL = "linear"
    try:
        inverse_kinematics(0.0, 0.0, yaw_deg=5.0)
    except IKUnreachable:
        pass
    else:
        assert False, "expected IKUnreachable for nonzero yaw in linear mode"


def test_nan_input_rejected():
    config.IK_MODEL = "linear"
    try:
        inverse_kinematics(float("nan"), 0.0)
    except IKUnreachable:
        pass
    else:
        assert False, "expected IKUnreachable for NaN input"


def test_analytical_neutral_pose_is_handled_without_crashing():
    """IMPORTANT FINDING, not a bug in this test: with the current
    PLACEHOLDER geometry (l1=45, l2=81, H=145 from the project notes,
    plus a guessed base_radius), this leg model's maximum reach is only
    l1+l2=126mm -- less than H=145mm -- so even the *neutral* pose comes
    back IKUnreachable. That is a real, useful signal: it means at least
    one of {l1, l2, H, the assumed single-hinge-plus-rod mechanism} does
    not mean what was assumed (e.g. H may include a fixed standoff not
    part of the l1/l2 linkage, or the real mechanism differs). This is
    exactly why config.IK_MODEL defaults to "linear" and why GEOMETRY
    must be corrected from the CAD model / robot photo before switching
    to "analytical". This test only confirms the failure is reported
    cleanly as IKUnreachable rather than silently returning a wrong or
    NaN angle -- it is EXPECTED to keep hitting that branch until
    config.GEOMETRY is replaced with verified measurements.
    """
    config.IK_MODEL = "analytical"
    try:
        values = inverse_kinematics(0.0, 0.0)
    except IKUnreachable:
        return
    for v in values:
        assert math.isfinite(v)
        assert abs(v) < 1.0  # should sit near the horn's neutral angle


def test_analytical_small_tilts_do_not_crash_or_produce_nan():
    """See test_analytical_neutral_pose_is_handled_without_crashing --
    with the current placeholder geometry these are also expected to
    raise IKUnreachable. What matters here is that every case either
    raises that specific, documented exception or returns finite
    numbers -- never a crash, never NaN/inf reaching a servo."""
    config.IK_MODEL = "analytical"
    for roll, pitch in [(3.0, 0.0), (-3.0, 0.0), (0.0, 3.0), (0.0, -3.0)]:
        try:
            values = inverse_kinematics(roll, pitch)
        except IKUnreachable:
            continue
        assert len(values) == 3
        assert all(math.isfinite(v) for v in values)


def test_analytical_extreme_pose_handled_safely():
    config.IK_MODEL = "analytical"
    try:
        values = inverse_kinematics(44.0, 44.0)
    except IKUnreachable:
        return  # acceptable: extreme combined tilt exceeds this geometry
    assert all(math.isfinite(v) for v in values)


def test_analytical_beyond_sanity_backstop_raises():
    config.IK_MODEL = "analytical"
    try:
        inverse_kinematics(60.0, 0.0)
    except IKUnreachable:
        pass
    else:
        assert False, "expected IKUnreachable beyond the +/-45 degree backstop"


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
