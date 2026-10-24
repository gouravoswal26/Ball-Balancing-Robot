"""
inverse_kinematics.py
======================
Converts a desired platform orientation (roll, pitch[, yaw]) into three
per-servo commands.

*** GEOMETRY STATUS: UNVERIFIED PLACEHOLDER ***
config.GEOMETRY currently holds numbers carried over from project notes
(l1=45mm, l2=81mm, platform=165mm, H=145mm) plus a base_radius that was
never actually specified anywhere and had to be guessed. Per the project
brief ("do not invent a generic Stewart platform equation and assume it
matches this mechanism"), this module does NOT quietly trust that
geometry. Instead it offers two strategies:

1. "linear"  (config.IK_MODEL = "linear", the default)
   -----------------------------------------------------------------
   A calibrated local mapping, not a geometric model at all. It assumes
   only that the three servos are mounted 120 degrees apart around a
   platform that tilts in roll/pitch -- the one geometric fact from the
   photo/description we're actually confident in -- and that, near the
   neutral pose, each servo's contribution to a commanded tilt is
   proportional to the projection of the (roll, pitch) tilt vector onto
   that servo's mounting angle:

       delta_i = (pitch*cos(phi_i) + roll*sin(phi_i)) * LINEAR_IK_US_PER_DEG

   This is exactly the "validated alternative ... such as ... a
   calibrated local mapping" the spec allows when full analytical IK
   isn't ready. It needs no unverified length measurements -- only a
   single empirical gain you tune during bring-up (README "Tuning
   procedure"). Use this for everything up to and including full
   closed-loop testing; it is deliberately the default.

2. "analytical" (config.IK_MODEL = "analytical")
   -----------------------------------------------------------------
   A full geometric solution for the mechanism this project most likely
   uses: three legs, each a servo horn (length l1) rotating in a
   vertical plane about a horizontal axis at the base, connected by a
   passive, double-ball-jointed rod (length l2) to an attachment point
   on the platform -- i.e. a 3-leg RSS parallel platform, the same leg
   equation used in delta-robot kinematics. This is a *placeholder
   hypothesis*, not a confirmed model: do not switch to it until
   config.GEOMETRY has been checked against the CAD model / robot photo
   (see calibration.py and README.md). If the real mechanism turns out
   to be different -- e.g. a direct-drive gimbal, a slider-crank, or the
   servo axis not horizontal -- this model's math is internally
   consistent but will answer a question about the wrong mechanism.

Both strategies raise IKUnreachable if the requested pose has no valid
solution (analytical) or is out of range (linear, defensively).
"""

import math

import config


class IKUnreachable(ValueError):
    """Raised when a requested (roll, pitch, yaw) has no valid solution
    for the current geometry / model, or when an input is invalid."""


# ---------------------------------------------------------------------------
# small vector/matrix helpers (kept dependency-free -- no numpy needed
# just for 3x3 rotations)
# ---------------------------------------------------------------------------

def _rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    """Intrinsic rotation R = Rz(yaw) * Ry(pitch) * Rx(roll).

    Convention: roll = rotation about the platform's local X axis,
    pitch = about Y, yaw = about Z. This is a CONVENTION CHOICE, not a
    measured fact -- verify it matches whatever "positive roll" /
    "positive pitch" means in your camera-to-platform calibration
    (Section 3 / Section 11), and flip ROLL_CONTROL_SIGN /
    PITCH_CONTROL_SIGN in config.py rather than editing this function if
    the sense turns out to be backwards.
    """
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]

    return _matmul(_matmul(rz, ry), rx)


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _matvec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _sub(a, b):
    return [x - y for x, y in zip(a, b)]


# ---------------------------------------------------------------------------
# Strategy 1: calibrated linear mapping
# ---------------------------------------------------------------------------

def _inverse_kinematics_linear(roll_deg, pitch_deg, yaw_deg=0.0):
    if yaw_deg:
        raise IKUnreachable("the 'linear' IK model does not support yaw; "
                             "pass yaw_deg=0.0 or switch config.IK_MODEL "
                             "to 'analytical'")

    deltas_us = []
    for leg_angle_deg in config.GEOMETRY.leg_mount_angles_deg:
        phi = math.radians(leg_angle_deg)
        effective_tilt_deg = pitch_deg * math.cos(phi) + roll_deg * math.sin(phi)
        deltas_us.append(effective_tilt_deg * config.LINEAR_IK_US_PER_DEG)

    return deltas_us   # 3 microsecond deltas, relative to each servo's center_us


# ---------------------------------------------------------------------------
# Strategy 2: analytical 3-leg RSS geometric model
# ---------------------------------------------------------------------------

def _leg_positions():
    """Base pivot points B_i (world/base frame) and the platform
    attachment points' *local* (platform-frame) coordinates P_i_local,
    for all 3 legs, from config.GEOMETRY."""
    g = config.GEOMETRY
    bases = []
    platform_local = []
    for base_angle_deg in g.leg_mount_angles_deg:
        theta_b = math.radians(base_angle_deg)
        theta_p = math.radians(base_angle_deg + g.platform_angle_offset_deg)
        bases.append((g.base_radius * math.cos(theta_b),
                      g.base_radius * math.sin(theta_b),
                      0.0))
        platform_local.append((g.platform_radius * math.cos(theta_p),
                               g.platform_radius * math.sin(theta_p),
                               0.0))
    return bases, platform_local


def _solve_leg_angle(base_point, platform_point_world, leg_angle_deg):
    """Solve for the servo horn angle theta such that a rod of length l2
    reaches from the horn tip -- which sweeps a circle of radius l1 in
    the vertical plane containing this leg's radial direction -- to the
    known platform attachment point.

    With
        u = outward radial unit vector at this leg's base angle
        w = (0, 0, 1)
        A(theta) = base_point + l1 * (cos(theta)*u + sin(theta)*w)
    solving |platform_point_world - A(theta)| = l2 expands to
        a*cos(theta) + b*sin(theta) = k
    with a, b, k as below -- a standard sinusoid-equals-constant equation
    with closed form theta = atan2(b, a) +/- acos(k / hypot(a, b)).
    This is the same leg equation used in delta-robot inverse kinematics;
    it is geometrically correct FOR THIS ASSUMED MECHANISM -- see the
    module docstring for why that assumption itself is unverified.
    """
    g = config.GEOMETRY
    theta_b = math.radians(leg_angle_deg)
    u = (math.cos(theta_b), math.sin(theta_b), 0.0)
    w = (0.0, 0.0, 1.0)

    d = _sub(platform_point_world, base_point)
    a = _dot(d, u)
    b = _dot(d, w)
    r = math.hypot(a, b)
    if r < 1e-9:
        raise IKUnreachable(f"degenerate leg geometry at {leg_angle_deg} deg")

    k = (_dot(d, d) + g.l1 ** 2 - g.l2 ** 2) / (2.0 * g.l1)
    ratio = k / r
    if not (-1.0 <= ratio <= 1.0):
        raise IKUnreachable(
            f"pose unreachable for leg at {leg_angle_deg} deg "
            f"(|k/r|={abs(ratio):.3f} > 1); reduce roll/pitch or re-check "
            f"geometry (l1/l2/base_radius/platform_radius/H)")

    phase = math.atan2(b, a)
    acos_term = math.acos(ratio)

    # Two mathematical solutions ("elbow up" / "elbow down"). Pick the
    # one closer to zero (neutral) as the physically sensible branch for
    # a horn that sits roughly horizontal at rest -- VERIFY against the
    # real mechanism; if the servo horn actually rests closer to
    # vertical, this branch choice needs to change.
    theta1 = phase + acos_term
    theta2 = phase - acos_term
    theta = theta1 if abs(theta1) <= abs(theta2) else theta2

    return math.degrees(theta)


def _inverse_kinematics_analytical(roll_deg, pitch_deg, yaw_deg=0.0):
    g = config.GEOMETRY
    if abs(roll_deg) > 45 or abs(pitch_deg) > 45:
        # Sanity backstop, independent of MAX_ROLL_DEG/MAX_PITCH_DEG --
        # this model was never meant to be evaluated far outside the
        # small-tilt balancing regime.
        raise IKUnreachable("roll/pitch magnitude implausibly large for "
                             "this mechanism; refusing to solve")

    rotation = _rotation_matrix(roll_deg, pitch_deg, yaw_deg)
    bases, platform_local = _leg_positions()

    angles = []
    for base_point, p_local, leg_angle_deg in zip(bases, platform_local,
                                                    g.leg_mount_angles_deg):
        p_world_rel = _matvec(rotation, p_local)
        p_world = (p_world_rel[0], p_world_rel[1], p_world_rel[2] + g.H)
        angles.append(_solve_leg_angle(base_point, p_world, leg_angle_deg))

    return angles   # 3 servo angles in degrees, 0 = neutral horn position


# ---------------------------------------------------------------------------
# Public dispatcher
# ---------------------------------------------------------------------------

def inverse_kinematics(roll_deg, pitch_deg, yaw_deg=0.0):
    """
    Returns a list of 3 values, one per servo, in leg order matching
    config.GEOMETRY.leg_mount_angles_deg / config.SERVO_CONFIG:

      - config.IK_MODEL == "linear":     microsecond deltas from each
                                          servo's configured center_us
      - config.IK_MODEL == "analytical": angles in degrees from each
                                          servo's neutral horn position

    Raises IKUnreachable if the pose can't be solved (analytical mode),
    if yaw is requested in linear mode, or if an input isn't finite.
    """
    for value, name in ((roll_deg, "roll_deg"), (pitch_deg, "pitch_deg"),
                        (yaw_deg, "yaw_deg")):
        if not math.isfinite(value):
            raise IKUnreachable(f"{name}={value!r} is not a finite number")

    if config.IK_MODEL == "linear":
        return _inverse_kinematics_linear(roll_deg, pitch_deg, yaw_deg)
    elif config.IK_MODEL == "analytical":
        return _inverse_kinematics_analytical(roll_deg, pitch_deg, yaw_deg)
    else:
        raise ValueError(f"Unknown config.IK_MODEL: {config.IK_MODEL!r}")
