"""
config.py
=========
Central configuration for the ball-balancing robot. Every other module
reads its tunable numbers from here (as `import config; config.X`, not
`from config import X`) so that calibration.py and the test suite can
change a value at runtime and have every module see the update.

Every numeric value below is either:
  (a) a measured/verified constant, or
  (b) a PLACEHOLDER carried over from the old ball_5.py / project notes,
      explicitly marked "VERIFY" or "PLACEHOLDER", that MUST be checked
      against the physical robot and the CAD model / photo before you
      trust it under full PID control.

See README.md -> "Calibration" for the step-by-step procedure that fills
these in for real, and inverse_kinematics.py's module docstring for a
detailed discussion of the geometry placeholders in section 5 below.
"""

from dataclasses import dataclass, field
from typing import List


# ---------------------------------------------------------------------------
# 1. CAMERA
# ---------------------------------------------------------------------------

CAMERA_RESOLUTION = (640, 480)      # VERIFY: must match the Picamera2 config used
CAMERA_FRAMERATE = 30

# VERIFY EVERY TIME CAMERA_RESOLUTION CHANGES.
# These were recorded for a specific Picamera2 resolution in the old
# ball_5.py. If CAMERA_RESOLUTION above differs from what was used when
# these were measured, recompute them with:
#   python calibration.py find-camera-center
CAMERA_CENTER_X = 338     # VERIFY against current CAMERA_RESOLUTION
CAMERA_CENTER_Y = 264     # VERIFY against current CAMERA_RESOLUTION

# HSV threshold for "orange ball". MUST be tuned for your ball + lighting.
HSV_ORANGE_LOWER = (5, 120, 120)
HSV_ORANGE_UPPER = (18, 255, 255)

# Ball-detection acceptance window (pixels)
MIN_BALL_RADIUS_PX = 8
MAX_BALL_RADIUS_PX = 120
MIN_CONTOUR_AREA_PX2 = 60

# Reject a detection that jumps further than this from the last accepted
# position in a single frame (pixels). Guards against locking onto a
# spurious orange blob (skin tone, reflections) far from the real ball.
MAX_POSITION_JUMP_PX = 120

# Moving-average filter window (frames) applied to accepted ball positions.
POSITION_FILTER_WINDOW = 5

# Consecutive missed frames allowed before the control loop declares
# "ball lost" and switches to the safe-hold behavior.
MAX_MISSED_FRAMES = 6


# ---------------------------------------------------------------------------
# 2. SIGN CONVENTIONS  (Section 11 -- fill in during calibration)
# ---------------------------------------------------------------------------
# These four signs ARE the "which way does it move" problem. Get these
# right FIRST with `python calibration.py test-direction`, before
# touching PID gains. Getting one wrong turns the loop into positive
# feedback: the platform amplifies the error instead of correcting it.

CAMERA_X_SIGN = 1          # flip to -1 if image X is mirrored vs platform
CAMERA_Y_SIGN = 1          # flip to -1 if image Y is mirrored vs platform
ROLL_CONTROL_SIGN = 1      # flip to -1 if positive roll tilts the wrong way
PITCH_CONTROL_SIGN = 1     # flip to -1 if positive pitch tilts the wrong way


# ---------------------------------------------------------------------------
# 3. PID GAINS  (starting points only -- see README "Tuning procedure")
# ---------------------------------------------------------------------------

PID_X = dict(kp=0.15, ki=0.00, kd=0.05)
PID_Y = dict(kp=0.15, ki=0.00, kd=0.05)

INTEGRAL_LIMIT = 100.0        # anti-windup clamp on the I term (output units)
OUTPUT_LIMIT = 100.0          # clamp on total PID output, pre-angle-scaling

# Deadband around the target, in pixels. Errors smaller than this are
# treated as zero so the platform doesn't hunt around the noise floor.
DEADBAND_X_PX = 5
DEADBAND_Y_PX = 5

# Low-pass filter coefficient (0..1) applied to the error before it is
# differentiated for the D term. 1.0 = no filtering (raw derivative).
# Lower = smoother but more lag. Tune this before trusting Kd.
DERIVATIVE_FILTER_ALPHA = 0.3


# ---------------------------------------------------------------------------
# 4. PID OUTPUT -> DESIRED PLATFORM ORIENTATION
# ---------------------------------------------------------------------------

ANGLE_SCALE_X = 0.05    # degrees of tilt per unit of PID output -- VERIFY
ANGLE_SCALE_Y = 0.05

MAX_ROLL_DEG = 5.0
MAX_PITCH_DEG = 5.0


# ---------------------------------------------------------------------------
# 5. MECHANICAL GEOMETRY  --  *** PLACEHOLDER, NOT VERIFIED ***
# ---------------------------------------------------------------------------
# These numbers came from earlier project notes / ball_5.py, not from a
# measured CAD export or the robot photo. That is exactly why IK_MODEL
# below defaults to "linear" instead of "analytical" -- see
# inverse_kinematics.py's module docstring for the full explanation.
#
#   l1                          servo arm (horn) length, mm
#   l2                          connecting rod length, mm
#   base_radius                 distance from the platform's center axis
#                                to each servo horn's pivot, in the base
#                                plane, mm. NOT PROVIDED anywhere in the
#                                original notes -- the value below is a
#                                placeholder guess. MUST measure from CAD
#                                or the photo before using "analytical" IK.
#   platform_radius              distance from platform center to each
#                                rod's upper attachment point, mm. Derived
#                                here as platform/2, assuming "platform =
#                                165mm" was a diameter -- VERIFY.
#   H                            neutral height, base plane to platform
#                                plane, mm.
#   leg_mount_angles_deg         angular position of each of the 3 legs
#                                around the base, degrees (0 = +X axis,
#                                counter-clockwise positive). 120 degrees
#                                apart is assumed -- VERIFY against the
#                                photo/CAD.
#   platform_angle_offset_deg   rotational offset between the base leg
#                                angles and the platform attachment-point
#                                angles. 0 assumes they line up radially;
#                                many real 3-arm platforms offset this so
#                                the rods clear each other -- VERIFY.
#
# *** CONCRETE FINDING FROM tests/test_kinematics.py ***
# With l1=45 and l2=81 exactly as recorded, the maximum possible reach of
# a single-hinge-horn + rigid-rod leg is l1+l2 = 126mm. That is LESS than
# H=145mm, so this simple leg model cannot even reach the neutral pose
# (flat platform, height H) no matter what base_radius/platform_radius
# are set to. Either:
#   - H=145mm was measured to a different reference than "servo horn
#     pivot to platform attachment point" (e.g. it includes a fixed
#     standoff/frame height that isn't part of the l1/l2 linkage), or
#   - l1/l2 were measured for a different mechanism than a simple
#     horn-plus-rod leg (e.g. there's an intermediate link, or the horn
#     doesn't rotate in the plane this model assumes), or
#   - one or more of these three numbers is simply wrong.
# This is why IK_MODEL defaults to "linear" below, and why "analytical"
# mode will raise IKUnreachable on every pose, including neutral, until
# this is resolved with the real CAD model or robot photo.

@dataclass
class Geometry:
    l1: float = 45.0                        # VERIFY
    l2: float = 81.0                        # VERIFY
    base_radius: float = 60.0               # PLACEHOLDER -- not given anywhere, guessed
    platform_radius: float = 165.0 / 2.0    # PLACEHOLDER -- assumes 165mm was a diameter
    H: float = 145.0                        # VERIFY
    leg_mount_angles_deg: List[float] = field(
        default_factory=lambda: [90.0, 210.0, 330.0])   # VERIFY layout
    platform_angle_offset_deg: float = 0.0  # PLACEHOLDER -- VERIFY from CAD/photo


GEOMETRY = Geometry()

# Which IK strategy to use -- see inverse_kinematics.py for the full
# discussion of both. Keep this at "linear" until Geometry above has been
# verified against the real mechanism, then switch to "analytical" and
# re-run tests/test_kinematics.py plus calibration.py test-direction.
IK_MODEL = "linear"     # "linear" | "analytical"

# Gain for the "linear" (calibrated local mapping) IK model: microseconds
# of servo correction per degree of commanded tilt, per leg. Start small;
# raise it during README "Tuning procedure" Step 2/3.
LINEAR_IK_US_PER_DEG = 8.0


# ---------------------------------------------------------------------------
# 6. SERVOS / PCA9685
# ---------------------------------------------------------------------------

PCA9685_I2C_ADDRESS = 0x40
PCA9685_PWM_FREQUENCY_HZ = 50

SERVO_CONFIG = [
    dict(channel=0, center_us=1500, direction=1, min_us=1300, max_us=1700),
    dict(channel=1, center_us=1550, direction=1, min_us=1350, max_us=1750),
    dict(channel=2, center_us=1500, direction=1, min_us=1300, max_us=1700),
]
# min_us/max_us above are intentionally conservative (+-200us) compared
# to the servo's nominal 1000-2000us range, per the "test with reduced
# movement limits first" safety requirement. Widen them only after each
# servo's safe mechanical range has been confirmed by hand (calibration.py
# test-servo).

# Maximum change allowed in a single control-loop update, in microseconds.
# Stops one bad frame / IK jump from slamming a servo.
MAX_SERVO_STEP_US = 20

# Hard secondary ceiling on any single correction, regardless of what
# PID/IK computed (carried over from ball_5.py's SAFETY_LIMIT, now
# configurable and documented in real units).
SAFETY_LIMIT_US = 60


# ---------------------------------------------------------------------------
# 7. CONTROL LOOP
# ---------------------------------------------------------------------------

CONTROL_LOOP_HZ = 30.0
STARTUP_SETTLE_S = 1.0     # hold neutral pose this long before enabling PID


# ---------------------------------------------------------------------------
# 8. LOGGING
# ---------------------------------------------------------------------------

LOG_DIR = "logs"
LOG_TO_CSV = True
LOG_TO_CONSOLE = True
