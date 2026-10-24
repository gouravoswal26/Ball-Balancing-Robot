#!/usr/bin/env python3
"""
ball_balance_v2.py
===================
SINGLE-FILE ball-balancing control system for:
  Raspberry Pi 4 + Pi Camera (IMX708, via Picamera2) + PCA9685 + 3x MG996R

This is the same rewrite that was delivered earlier as a multi-file
project (config.py / camera.py / ball_detection.py / pid_controller.py /
inverse_kinematics.py / servo_controller.py / safety.py / logger.py /
calibration.py / main.py), flattened into one file so it can be copied
and pasted directly onto the Pi. Nothing was simplified or left out to
make it fit in one file -- it's the exact same logic, just without
module boundaries. If you'd rather have the modular version (e.g. to
run its pytest suite), that's the separate ball_balancing.zip.

=====================================================================
STATUS -- READ BEFORE RUNNING ON THE REAL ROBOT
=====================================================================
1. GEOMETRY IS UNVERIFIED. The mechanical dimensions below (l1, l2,
   base_radius, platform_radius, H) were never confirmed against a CAD
   model or photo. In fact, testing found them INTERNALLY INCONSISTENT:
   l1+l2=126mm is less than H=145mm, so a simple horn+rod leg can't even
   reach the neutral pose. See the GEOMETRY section below for the full
   explanation. Because of this, IK_MODEL defaults to "linear" (a
   calibrated per-servo mixing table that doesn't depend on those
   lengths at all) rather than "analytical" (the full geometric model,
   implemented but not usable until the geometry is fixed).
2. SIGN CONVENTIONS (CAMERA_X_SIGN, CAMERA_Y_SIGN, ROLL_CONTROL_SIGN,
   PITCH_CONTROL_SIGN) are placeholders. Run `--mode test-direction`
   before ever running the full closed loop -- a wrong sign here turns
   the system into positive feedback (amplifies error instead of
   correcting it), which looks exactly like "the platform responds but
   the ball still falls off."
3. Your own ball_balance.py's comments suggest the ORIGINAL ball_5.py
   used a servo correction window only ~100us wide, clamped to ~8us
   total -- essentially no authority to fight gravity. Don't recreate
   that: if the platform visibly isn't moving enough, raise
   LINEAR_IK_US_PER_DEG / SAFETY_LIMIT_US, don't just crank the PID gains.

=====================================================================
USAGE
=====================================================================
    python3 ball_balance_v2.py                       full closed-loop run
    python3 ball_balance_v2.py --debug-no-servos      camera+PID+IK only, no servo motion
    python3 ball_balance_v2.py --simulate-hardware    run against a simulated PCA9685
    python3 ball_balance_v2.py --display              live OpenCV debug window (needs a monitor/X11)
    python3 ball_balance_v2.py --iterations 500        stop after N loop iterations

Calibration modes (do these BEFORE the full closed-loop run above --
see the "CALIBRATION PROCEDURE" comment block further down):
    python3 ball_balance_v2.py --mode neutral
    python3 ball_balance_v2.py --mode test-servo --channel 0
    python3 ball_balance_v2.py --mode find-camera-center --seconds 5
    python3 ball_balance_v2.py --mode test-direction --step 3

Add --simulate to any calibration mode to dry-run against the simulated
servo driver instead of real hardware.
=====================================================================
"""

import argparse
import csv
import math
import os
import time
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

try:
    import cv2
except ImportError as _exc:  # pragma: no cover
    cv2 = None
    _CV2_IMPORT_ERROR = _exc
else:
    _CV2_IMPORT_ERROR = None


# =====================================================================
# 1. CONFIG -- every tunable number lives here. VERIFY / PLACEHOLDER
#    markers show what hasn't been confirmed against your real hardware.
# =====================================================================

# ---- Camera ----------------------------------------------------------
CAMERA_RESOLUTION = (640, 480)      # VERIFY: must match the Picamera2 config used
CAMERA_FRAMERATE = 30

CAMERA_CENTER_X = 338     # VERIFY against current CAMERA_RESOLUTION
CAMERA_CENTER_Y = 264     # VERIFY against current CAMERA_RESOLUTION -- re-derive
                          # with `--mode find-camera-center` if resolution changes

# HSV threshold for "orange ball". Matches your uploaded ball_balance.py
# (a prior tuning attempt), not a fresh guess -- still re-verify against
# your actual ball/lighting if detection looks flaky.
HSV_ORANGE_LOWER = (5, 120, 100)
HSV_ORANGE_UPPER = (25, 255, 255)

MIN_BALL_RADIUS_PX = 8
MAX_BALL_RADIUS_PX = 120
MIN_CONTOUR_AREA_PX2 = 500   # matches ball_balance.py -- the original 60 let noise through

MAX_POSITION_JUMP_PX = 120   # reject a detection that jumps this far in one frame
POSITION_FILTER_WINDOW = 5   # moving-average window (frames) on accepted positions
MAX_MISSED_FRAMES = 6        # consecutive misses before "ball lost" / safe-hold

# ---- Sign conventions (Section 11 -- fill in during calibration) -----
# Get these right FIRST with `--mode test-direction`, before touching PID
# gains. A wrong sign here turns the loop into positive feedback.
CAMERA_X_SIGN = 1          # flip to -1 if image X is mirrored vs platform
CAMERA_Y_SIGN = 1          # flip to -1 if image Y is mirrored vs platform
ROLL_CONTROL_SIGN = 1      # flip to -1 if positive roll tilts the wrong way
PITCH_CONTROL_SIGN = 1     # flip to -1 if positive pitch tilts the wrong way

# ---- PID gains (starting points only) --------------------------------
PID_X = dict(kp=0.15, ki=0.00, kd=0.05)
PID_Y = dict(kp=0.15, ki=0.00, kd=0.05)

INTEGRAL_LIMIT = 100.0        # anti-windup clamp on the I term
OUTPUT_LIMIT = 100.0          # clamp on total PID output, pre-angle-scaling

DEADBAND_X_PX = 5
DEADBAND_Y_PX = 5

DERIVATIVE_FILTER_ALPHA = 0.3  # 0..1, low-pass on error before differentiating

# ---- PID output -> desired platform orientation -----------------------
ANGLE_SCALE_X = 0.05    # degrees of tilt per unit of PID output -- VERIFY
ANGLE_SCALE_Y = 0.05

MAX_ROLL_DEG = 5.0
MAX_PITCH_DEG = 5.0

# ---- Mechanical geometry -- *** PLACEHOLDER, NOT VERIFIED *** ---------
# l1 = servo arm (horn) length, mm. l2 = connecting rod length, mm.
# base_radius = servo-horn-pivot distance from platform center axis, mm
#   (NOT PROVIDED anywhere in the original notes -- guessed).
# platform_radius = rod-attachment distance from platform center, mm
#   (derived here as platform/2, assuming "platform=165mm" was a
#   diameter -- VERIFY).
# H = neutral height, base plane to platform plane, mm.
# leg_mount_angles_deg = angular position of each leg around the base
#   (0 = +X axis, CCW positive); 120 degrees apart assumed -- VERIFY.
# platform_angle_offset_deg = rotational offset between base and
#   platform attachment angles; 0 assumes radial alignment -- VERIFY.
#
# *** CONCRETE FINDING ***: with l1=45 and l2=81 exactly as recorded,
# max leg reach is l1+l2=126mm, LESS than H=145mm -- so this simple leg
# model can't even reach the neutral (flat) pose, regardless of
# base_radius/platform_radius. At least one of l1, l2, H, or the assumed
# mechanism itself isn't what the old notes assumed (e.g. H may include
# a standoff not part of the l1/l2 linkage). This is why IK_MODEL
# defaults to "linear" below, and why "analytical" mode will raise
# IKUnreachable on every pose (including neutral) until a CAD model or
# clear photo resolves this.

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

IK_MODEL = "linear"     # "linear" | "analytical" -- see module docstring above

# ---- Linear IK mix matrix ----------------------------------------------
# Each row is (pitch_coeff, roll_coeff) for one servo: that servo's delta
# is (pitch_deg*pitch_coeff + roll_deg*roll_coeff) * LINEAR_IK_US_PER_DEG.
# Default assumes an ideal 120-degree-symmetric mount (matches
# GEOMETRY.leg_mount_angles_deg = [90, 210, 330]) and does NOT depend on
# l1/l2/H. Your uploaded ball_balance.py had its own hand-picked,
# non-symmetric matrix (CH0=[1,0], CH1=[0,0.7], CH2=[0.5,-0.5], operating
# on raw pixel error rather than degrees -- structure worth reusing if
# `--mode test-direction` shows your platform isn't symmetric, but its
# exact numbers weren't copied since the units don't match and that file
# flags them as unverified too).
MIX_MATRIX = [
    (0.0, 1.0),          # CH0 -- leg at 90 deg
    (-0.8660254, -0.5),  # CH1 -- leg at 210 deg
    (0.8660254, -0.5),   # CH2 -- leg at 330 deg
]

LINEAR_IK_US_PER_DEG = 8.0   # start small; raise during tuning (see usage docstring)

# ---- Servos / PCA9685 ---------------------------------------------------
PCA9685_I2C_ADDRESS = 0x40
PCA9685_PWM_FREQUENCY_HZ = 50

SERVO_CONFIG = [
    dict(channel=0, center_us=1500, direction=1, min_us=1300, max_us=1700),
    dict(channel=1, center_us=1550, direction=1, min_us=1350, max_us=1750),
    dict(channel=2, center_us=1500, direction=1, min_us=1300, max_us=1700),
]
# min_us/max_us are intentionally conservative (+-200us) vs. the servo's
# nominal 1000-2000us range. Widen only after confirming each servo's
# safe mechanical range by hand (`--mode test-servo`).

MAX_SERVO_STEP_US = 20   # max change per control-loop update (rate limit)
SAFETY_LIMIT_US = 60     # hard ceiling on any single correction from center

# ---- Control loop ---------------------------------------------------
CONTROL_LOOP_HZ = 30.0
STARTUP_SETTLE_S = 1.0     # hold neutral this long before enabling PID

# ---- Logging ----------------------------------------------------------
LOG_DIR = "logs"
LOG_TO_CSV = True
LOG_TO_CONSOLE = True


# =====================================================================
# 2. PID CONTROLLER
# =====================================================================

@dataclass
class PIDResult:
    output: float
    p: float
    i: float
    d: float
    filtered_error: float


class PIDController:
    """Anti-windup, filtered derivative, deadband, NaN/dt guards, and a
    reset() to call whenever the ball is lost so stale state doesn't
    fire a large correction the instant it reappears."""

    def __init__(self, kp, ki, kd, *, integral_limit=100.0,
                 output_limit=100.0, deadband=0.0,
                 derivative_filter_alpha=0.3, name="pid"):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.deadband = deadband
        self.alpha = derivative_filter_alpha
        self.name = name

        self._integral = 0.0
        self._filtered_error = 0.0
        self._prev_filtered_error = None
        self._have_prev = False

    def reset(self):
        self._integral = 0.0
        self._filtered_error = 0.0
        self._prev_filtered_error = None
        self._have_prev = False

    def update(self, error, dt):
        if dt is None or dt <= 0 or not math.isfinite(dt):
            dt = 1e-3

        if error is None or not math.isfinite(error):
            error = 0.0

        active_error = 0.0 if abs(error) < self.deadband else error

        self._filtered_error = (
            self.alpha * active_error + (1 - self.alpha) * self._filtered_error
        )

        p = self.kp * active_error

        prospective_integral = self._integral + self.ki * active_error * dt
        prospective_integral = _clamp(
            prospective_integral, -self.integral_limit, self.integral_limit)

        if self._have_prev:
            d = self.kd * (self._filtered_error - self._prev_filtered_error) / dt
        else:
            d = 0.0

        self._prev_filtered_error = self._filtered_error
        self._have_prev = True

        raw_output = p + prospective_integral + d
        output = _clamp(raw_output, -self.output_limit, self.output_limit)

        if raw_output == output:
            self._integral = prospective_integral

        return PIDResult(output=output, p=p, i=self._integral, d=d,
                          filtered_error=self._filtered_error)


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


# =====================================================================
# 3. INVERSE KINEMATICS -- "linear" (default) and "analytical" models.
#    See the module docstring at the top of this file for why "linear"
#    is the default. inverse_kinematics() is the single entry point.
# =====================================================================

class IKUnreachable(ValueError):
    """Raised when a requested (roll, pitch, yaw) has no valid solution
    for the current geometry / model, or when an input is invalid."""


def _rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    """R = Rz(yaw) * Ry(pitch) * Rx(roll). Convention choice, not a
    measured fact -- flip ROLL_CONTROL_SIGN/PITCH_CONTROL_SIGN rather
    than editing this if the sense turns out backwards."""
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


def _inverse_kinematics_linear(roll_deg, pitch_deg, yaw_deg=0.0):
    if yaw_deg:
        raise IKUnreachable("the 'linear' IK model does not support yaw; "
                             "pass yaw_deg=0.0 or switch IK_MODEL to 'analytical'")

    deltas_us = []
    for pitch_coeff, roll_coeff in MIX_MATRIX:
        effective_tilt_deg = pitch_deg * pitch_coeff + roll_deg * roll_coeff
        deltas_us.append(effective_tilt_deg * LINEAR_IK_US_PER_DEG)

    return deltas_us   # 3 microsecond deltas, relative to each servo's center_us


def _leg_positions():
    g = GEOMETRY
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
    """Closed-form solve for a servo horn (radius l1, sweeping the
    vertical plane through this leg's radial direction) connected by a
    rigid rod (length l2) to the given platform point. Same leg equation
    as delta-robot IK -- geometrically correct FOR THIS ASSUMED
    MECHANISM (see the GEOMETRY discussion at the top of the file for
    why that assumption itself is unverified)."""
    g = GEOMETRY
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

    theta1 = phase + acos_term
    theta2 = phase - acos_term
    theta = theta1 if abs(theta1) <= abs(theta2) else theta2

    return math.degrees(theta)


def _inverse_kinematics_analytical(roll_deg, pitch_deg, yaw_deg=0.0):
    g = GEOMETRY
    if abs(roll_deg) > 45 or abs(pitch_deg) > 45:
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


def inverse_kinematics(roll_deg, pitch_deg, yaw_deg=0.0):
    """Returns 3 values, one per servo:
      - IK_MODEL == "linear":     microsecond deltas from center_us
      - IK_MODEL == "analytical": angles in degrees from neutral horn
    Raises IKUnreachable on an unsolvable pose or non-finite input."""
    for value, name in ((roll_deg, "roll_deg"), (pitch_deg, "pitch_deg"),
                        (yaw_deg, "yaw_deg")):
        if not math.isfinite(value):
            raise IKUnreachable(f"{name}={value!r} is not a finite number")

    if IK_MODEL == "linear":
        return _inverse_kinematics_linear(roll_deg, pitch_deg, yaw_deg)
    elif IK_MODEL == "analytical":
        return _inverse_kinematics_analytical(roll_deg, pitch_deg, yaw_deg)
    else:
        raise ValueError(f"Unknown IK_MODEL: {IK_MODEL!r}")


# =====================================================================
# 4. SERVO CONTROLLER (PCA9685) -- falls back to a simulated driver if
#    the I2C bus / adafruit libraries aren't available, so this file can
#    still be smoke-tested off the Pi.
# =====================================================================

class SimulatedPCA9685:
    def __init__(self, *_, **__):
        self.channels = {}
        self.frequency = PCA9685_PWM_FREQUENCY_HZ

    def set_pulse_us(self, channel, pulse_us):
        self.channels[channel] = pulse_us


class _HardwarePCA9685Adapter:  # pragma: no cover - hardware-dependent
    def __init__(self, pca):
        self._pca = pca

    def set_pulse_us(self, channel, pulse_us):
        period_us = 1_000_000.0 / self._pca.frequency
        duty_cycle = int((pulse_us / period_us) * 0xFFFF)
        duty_cycle = max(0, min(0xFFFF, duty_cycle))
        self._pca.channels[channel].duty_cycle = duty_cycle


def _make_hardware_driver():  # pragma: no cover - hardware-dependent
    try:
        import board
        import busio
        from adafruit_pca9685 import PCA9685 as _RealPCA9685

        i2c = busio.I2C(board.SCL, board.SDA)
        pca = _RealPCA9685(i2c, address=PCA9685_I2C_ADDRESS)
        pca.frequency = PCA9685_PWM_FREQUENCY_HZ
        return _HardwarePCA9685Adapter(pca)
    except Exception as exc:
        print(f"[servo] PCA9685 hardware not available ({exc!r}); "
              f"falling back to the simulated driver.")
        return SimulatedPCA9685()


@dataclass
class ServoState:
    channel: int
    center_us: float
    direction: int
    min_us: float
    max_us: float
    current_us: float


class ServoController:
    def __init__(self, servo_config=None, use_hardware=True):
        servo_config = servo_config or SERVO_CONFIG
        self._driver = _make_hardware_driver() if use_hardware else SimulatedPCA9685()

        self.servos = []
        for cfg in servo_config:
            self.servos.append(ServoState(
                channel=cfg["channel"],
                center_us=cfg["center_us"],
                direction=cfg["direction"],
                min_us=cfg["min_us"],
                max_us=cfg["max_us"],
                current_us=cfg["center_us"],
            ))

    def move_to_neutral(self):
        for servo in self.servos:
            self._commit(servo, servo.center_us)

    def emergency_stop(self):
        """Hold every servo exactly where it is -- doesn't move to
        neutral (itself a motion, possibly unsafe mid-fault)."""
        for servo in self.servos:
            self._commit(servo, servo.current_us)

    def apply_ik_output(self, ik_values):
        if len(ik_values) != len(self.servos):
            raise ValueError(
                f"Expected {len(self.servos)} IK values, got {len(ik_values)}")

        commands = []
        for servo, value in zip(self.servos, ik_values):
            if value is None or not math.isfinite(value):
                value = 0.0

            if IK_MODEL == "analytical":
                # Coarse degrees->microseconds slope. VERIFY against the
                # real servo before trusting beyond small-angle bring-up.
                delta_us = value * 10.0 * servo.direction
            else:
                delta_us = value * servo.direction

            delta_us = _clamp(delta_us, -SAFETY_LIMIT_US, SAFETY_LIMIT_US)
            target_us = servo.center_us + delta_us
            target_us = _clamp(target_us, servo.min_us, servo.max_us)

            step = _clamp(target_us - servo.current_us,
                          -MAX_SERVO_STEP_US, MAX_SERVO_STEP_US)
            new_us = servo.current_us + step

            self._commit(servo, new_us)
            commands.append(new_us)

        return commands

    def _commit(self, servo, pulse_us):
        pulse_us = _clamp(pulse_us, servo.min_us, servo.max_us)
        servo.current_us = pulse_us
        self._driver.set_pulse_us(servo.channel, pulse_us)


# =====================================================================
# 5. BALL DETECTION -- HSV threshold -> morphology -> contour -> filter
# =====================================================================

@dataclass
class BallPosition:
    x: float
    y: float
    radius: float


class BallDetector:
    def __init__(self,
                 hsv_lower=None, hsv_upper=None,
                 min_radius=None, max_radius=None,
                 min_area=None, max_jump=None, filter_window=None):
        if cv2 is None:
            raise ImportError(
                "opencv-python is required for BallDetector") from _CV2_IMPORT_ERROR

        self.hsv_lower = np.array(hsv_lower or HSV_ORANGE_LOWER, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper or HSV_ORANGE_UPPER, dtype=np.uint8)
        self.min_radius = min_radius or MIN_BALL_RADIUS_PX
        self.max_radius = max_radius or MAX_BALL_RADIUS_PX
        self.min_area = min_area or MIN_CONTOUR_AREA_PX2
        self.max_jump = max_jump or MAX_POSITION_JUMP_PX

        self._kernel = np.ones((5, 5), np.uint8)
        window = filter_window or POSITION_FILTER_WINDOW
        self._history_x = deque(maxlen=window)
        self._history_y = deque(maxlen=window)
        self._last_accepted: Optional[BallPosition] = None

    def reset(self):
        self._history_x.clear()
        self._history_y.clear()
        self._last_accepted = None

    def detect(self, frame_rgb) -> Optional[BallPosition]:
        if frame_rgb is None:
            return None

        hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        candidate = max(contours, key=cv2.contourArea)
        if cv2.contourArea(candidate) < self.min_area:
            return None

        (cx, cy), radius = cv2.minEnclosingCircle(candidate)
        if not (self.min_radius <= radius <= self.max_radius):
            return None

        if self._last_accepted is not None:
            jump = ((cx - self._last_accepted.x) ** 2 +
                    (cy - self._last_accepted.y) ** 2) ** 0.5
            if jump > self.max_jump:
                return None   # spurious blob far from the real ball

        self._history_x.append(cx)
        self._history_y.append(cy)
        smoothed = BallPosition(
            x=sum(self._history_x) / len(self._history_x),
            y=sum(self._history_y) / len(self._history_y),
            radius=radius,
        )
        self._last_accepted = smoothed
        return smoothed


# =====================================================================
# 6. CAMERA -- Picamera2 wrapper
# =====================================================================

class Camera:
    def __init__(self, resolution=None, framerate=None):
        self.resolution = resolution or CAMERA_RESOLUTION
        self.framerate = framerate or CAMERA_FRAMERATE
        self._picam2 = None

    def start(self):
        from picamera2 import Picamera2  # Raspberry Pi only

        self._picam2 = Picamera2()
        # create_preview_configuration to match your uploaded
        # ball_balance.py, which used this mode successfully.
        cfg = self._picam2.create_preview_configuration(
            main={"size": self.resolution, "format": "RGB888"},
        )
        self._picam2.configure(cfg)
        self._picam2.set_controls({"FrameRate": self.framerate})
        self._picam2.start()

    def read_frame(self):
        if self._picam2 is None:
            raise RuntimeError("Camera.start() must be called before read_frame()")
        return self._picam2.capture_array()

    def stop(self):
        if self._picam2 is not None:
            self._picam2.stop()
            self._picam2 = None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False


# =====================================================================
# 7. SAFETY
# =====================================================================

class SafetyMonitor:
    def __init__(self, max_missed_frames=None):
        self.max_missed_frames = max_missed_frames or MAX_MISSED_FRAMES
        self._missed_frames = 0
        self.tripped = False
        self.trip_reason = None

    def report_detection(self, detected: bool):
        if detected:
            self._missed_frames = 0
        else:
            self._missed_frames += 1

    @property
    def ball_lost(self):
        return self._missed_frames >= self.max_missed_frames

    def check_numeric(self, *values):
        for v in values:
            if v is None:
                continue
            if not math.isfinite(v):
                self.tripped = True
                self.trip_reason = f"non-finite control value encountered: {v!r}"
                return True
        return False

    def reset(self):
        self._missed_frames = 0
        self.tripped = False
        self.trip_reason = None


def startup_sequence(servo_controller, logger=None):
    if logger:
        logger.log_event("startup: moving to neutral position")
    servo_controller.move_to_neutral()
    time.sleep(STARTUP_SETTLE_S)


def safe_hold_action(servo_controller, logger=None, reason=""):
    if logger:
        logger.log_event(f"safety hold: {reason}")
    servo_controller.emergency_stop()


# =====================================================================
# 8. LOGGER
# =====================================================================

_LOG_FIELDNAMES = [
    "t", "ball_x", "ball_y", "target_x", "target_y",
    "error_x", "error_y", "filtered_error_x", "filtered_error_y",
    "pid_x", "pid_y", "roll_deg", "pitch_deg",
    "servo_0_us", "servo_1_us", "servo_2_us", "loop_time_s",
]


class Logger:
    def __init__(self, log_dir=None, to_csv=None, to_console=None):
        self.to_csv = LOG_TO_CSV if to_csv is None else to_csv
        self.to_console = LOG_TO_CONSOLE if to_console is None else to_console
        self._csv_file = None
        self._csv_writer = None
        self.csv_path = None

        if self.to_csv:
            log_dir = log_dir or LOG_DIR
            os.makedirs(log_dir, exist_ok=True)
            path = os.path.join(log_dir, f"run_{int(time.time())}.csv")
            self._csv_file = open(path, "w", newline="")
            self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=_LOG_FIELDNAMES)
            self._csv_writer.writeheader()
            self.csv_path = path

    def log_control_data(self, **kwargs):
        row = {k: kwargs.get(k, "") for k in _LOG_FIELDNAMES}
        row["t"] = time.time()
        if self.to_csv and self._csv_writer:
            self._csv_writer.writerow(row)
            self._csv_file.flush()
        if self.to_console:
            print(
                f"Ball=({_fmt(row['ball_x'])},{_fmt(row['ball_y'])}) "
                f"Error=({_fmt(row['error_x'])},{_fmt(row['error_y'])}) "
                f"PID=({_fmt(row['pid_x'])},{_fmt(row['pid_y'])}) "
                f"RollPitch=({_fmt(row['roll_deg'])},{_fmt(row['pitch_deg'])}) "
                f"Servo=({_fmt(row['servo_0_us'])},{_fmt(row['servo_1_us'])},"
                f"{_fmt(row['servo_2_us'])}) "
                f"LoopTime={_fmt(row['loop_time_s'])}"
            )

    def log_event(self, message):
        print(f"[{time.strftime('%H:%M:%S')}] {message}")

    def close(self):
        if self._csv_file:
            self._csv_file.close()


def _fmt(value):
    if isinstance(value, float):
        return f"{value:.2f}"
    return str(value)


# =====================================================================
# 9. MAIN CONTROL LOOP
#
#    Camera -> Ball Detection -> Position Error -> PID Controller ->
#    Desired Platform Orientation -> Inverse Kinematics ->
#    Individual Servo Commands -> Platform Movement -> Camera Feedback
# =====================================================================

def build_pid_controllers():
    pid_x = PIDController(
        **PID_X, integral_limit=INTEGRAL_LIMIT, output_limit=OUTPUT_LIMIT,
        deadband=DEADBAND_X_PX, derivative_filter_alpha=DERIVATIVE_FILTER_ALPHA, name="x")
    pid_y = PIDController(
        **PID_Y, integral_limit=INTEGRAL_LIMIT, output_limit=OUTPUT_LIMIT,
        deadband=DEADBAND_Y_PX, derivative_filter_alpha=DERIVATIVE_FILTER_ALPHA, name="y")
    return pid_x, pid_y


def calculate_error(ball_x, ball_y):
    """error = target - measured, sign-corrected. Do not assume a
    positive camera error always means the same physical direction
    until `--mode test-direction` has confirmed it."""
    error_x = (CAMERA_CENTER_X - ball_x) * CAMERA_X_SIGN
    error_y = (CAMERA_CENTER_Y - ball_y) * CAMERA_Y_SIGN
    return error_x, error_y


def controller_to_orientation(pid_output_x, pid_output_y):
    roll = _clamp(pid_output_x * ANGLE_SCALE_X * ROLL_CONTROL_SIGN,
                  -MAX_ROLL_DEG, MAX_ROLL_DEG)
    pitch = _clamp(pid_output_y * ANGLE_SCALE_Y * PITCH_CONTROL_SIGN,
                   -MAX_PITCH_DEG, MAX_PITCH_DEG)
    return roll, pitch


def _draw_overlay(frame, ball, error_x, error_y, result_x, result_y,
                   roll, pitch, servo_us):
    """Ball circle, target crosshair, status/error/PID/servo text --
    adapted from your uploaded ball_balance.py's display code. Built
    unconditionally from a fresh copy of `frame` (matching that file's
    own fix #5: never leave `overlay` unbound on a 'ball lost' frame)."""
    overlay = frame.copy()
    w, h = CAMERA_RESOLUTION

    cv2.line(overlay, (CAMERA_CENTER_X, 0), (CAMERA_CENTER_X, h), (255, 255, 255), 1)
    cv2.line(overlay, (0, CAMERA_CENTER_Y), (w, CAMERA_CENTER_Y), (255, 255, 255), 1)
    cv2.circle(overlay, (CAMERA_CENTER_X, CAMERA_CENTER_Y), 8, (0, 255, 255), -1)

    if ball is not None:
        cv2.circle(overlay, (int(ball.x), int(ball.y)), int(ball.radius), (0, 255, 0), 3)
        cv2.circle(overlay, (int(ball.x), int(ball.y)), 5, (0, 0, 255), -1)
        cv2.line(overlay, (int(ball.x), int(ball.y)),
                 (CAMERA_CENTER_X, CAMERA_CENTER_Y), (255, 0, 255), 2)
        cv2.putText(overlay, "BALL DETECTED", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(overlay, f"Error=({error_x:.0f},{error_y:.0f})", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        if result_x is not None:
            cv2.putText(overlay, f"PID=({result_x.output:.1f},{result_y.output:.1f}) "
                                  f"RollPitch=({roll:.1f},{pitch:.1f})", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        if servo_us is not None:
            cv2.putText(overlay,
                        f"Servo=({servo_us[0]:.0f},{servo_us[1]:.0f},{servo_us[2]:.0f})",
                        (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    else:
        cv2.putText(overlay, "BALL NOT DETECTED", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    return overlay


def run(debug_no_servos=False, use_hardware=True, max_iterations=None, display=False):
    logger = Logger()
    detector = BallDetector()
    servo_controller = ServoController(use_hardware=use_hardware and not debug_no_servos)
    pid_x, pid_y = build_pid_controllers()
    safety = SafetyMonitor()

    if display and cv2 is None:
        raise ImportError("opencv-python (with GUI support) is required for --display")

    startup_sequence(servo_controller, logger)

    dt_target = 1.0 / CONTROL_LOOP_HZ
    last_time = time.time()
    iteration = 0

    with Camera() as camera:
        try:
            while max_iterations is None or iteration < max_iterations:
                loop_start = time.time()
                dt = loop_start - last_time
                last_time = loop_start

                error_x = error_y = None
                result_x = result_y = None
                roll = pitch = None
                servo_us = None

                frame = camera.read_frame()
                ball = detector.detect(frame) if frame is not None else None
                safety.report_detection(ball is not None)

                if ball is not None and not safety.ball_lost:
                    error_x, error_y = calculate_error(ball.x, ball.y)

                    result_x = pid_x.update(error_x, dt)
                    result_y = pid_y.update(error_y, dt)

                    roll, pitch = controller_to_orientation(result_x.output,
                                                             result_y.output)

                    if safety.check_numeric(roll, pitch):
                        safe_hold_action(servo_controller, logger, safety.trip_reason)
                        safety.reset()
                        servo_us = [s.current_us for s in servo_controller.servos]
                    else:
                        try:
                            ik_values = inverse_kinematics(roll, pitch)
                            servo_us = servo_controller.apply_ik_output(ik_values)
                        except IKUnreachable as exc:
                            safe_hold_action(servo_controller, logger, str(exc))
                            servo_us = [s.current_us for s in servo_controller.servos]

                    logger.log_control_data(
                        ball_x=ball.x, ball_y=ball.y,
                        target_x=CAMERA_CENTER_X, target_y=CAMERA_CENTER_Y,
                        error_x=error_x, error_y=error_y,
                        filtered_error_x=result_x.filtered_error,
                        filtered_error_y=result_y.filtered_error,
                        pid_x=result_x.output, pid_y=result_y.output,
                        roll_deg=roll, pitch_deg=pitch,
                        servo_0_us=servo_us[0], servo_1_us=servo_us[1],
                        servo_2_us=servo_us[2],
                        loop_time_s=time.time() - loop_start,
                    )
                else:
                    if safety.ball_lost:
                        pid_x.reset()
                        pid_y.reset()
                        detector.reset()
                        safe_hold_action(servo_controller, logger, "ball not detected")

                if display and frame is not None:
                    overlay = _draw_overlay(frame, ball, error_x, error_y,
                                             result_x, result_y, roll, pitch, servo_us)
                    cv2.imshow("Ball Balancing", overlay)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        logger.log_event("'q' pressed, shutting down.")
                        break
                    elif key == ord("n"):
                        servo_controller.move_to_neutral()
                        logger.log_event("'n' pressed: forced neutral.")

                iteration += 1
                elapsed = time.time() - loop_start
                sleep_time = dt_target - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            logger.log_event("Ctrl+C received, shutting down safely.")
        finally:
            servo_controller.move_to_neutral()
            logger.close()
            if display and cv2 is not None:
                cv2.destroyAllWindows()


# =====================================================================
# 10. CALIBRATION MODES
#
# CALIBRATION PROCEDURE -- do these in order before the full closed loop:
#   1. --mode test-servo --channel 0   (repeat for 1, 2)
#      Watch each servo sweep; confirm no binding; note true level pose.
#   2. --mode neutral
#      Confirms all three move to configured centers with PID off.
#   3. --mode find-camera-center --seconds 5
#      Hold the ball at the platform's true center; copy the printed
#      CAMERA_CENTER_X/Y into the CONFIG section above.
#   4. --mode test-direction --step 3        <- the most important step
#      Nudges roll/pitch a known amount one at a time and asks what you
#      observed; use your notes to set CAMERA_X_SIGN, CAMERA_Y_SIGN,
#      ROLL_CONTROL_SIGN, PITCH_CONTROL_SIGN above. Do not skip this.
# =====================================================================

def cmd_neutral(args):
    sc = ServoController(use_hardware=not args.simulate)
    sc.move_to_neutral()
    print(f"Servos moved to configured neutral (center_us). Holding for {args.hold}s.")
    time.sleep(args.hold)


def cmd_test_servo(args):
    sc = ServoController(use_hardware=not args.simulate)
    matches = [s for s in sc.servos if s.channel == args.channel]
    if not matches:
        print(f"No servo configured on channel {args.channel}. "
              f"Configured channels: {[s.channel for s in sc.servos]}")
        return
    servo = matches[0]

    print(f"Sweeping channel {args.channel} between {servo.min_us} and "
          f"{servo.max_us} us. Watch the linkage for binding. Ctrl+C to stop "
          f"(returns to center_us).")
    try:
        while True:
            for us in range(int(servo.min_us), int(servo.max_us), 5):
                sc._commit(servo, us)
                time.sleep(0.02)
            for us in range(int(servo.max_us), int(servo.min_us), -5):
                sc._commit(servo, us)
                time.sleep(0.02)
    except KeyboardInterrupt:
        sc._commit(servo, servo.center_us)
        print("\nStopped, returned to center_us.")


def cmd_find_camera_center(args):
    detector = BallDetector()
    samples = []
    with Camera() as cam:
        print(f"Hold the ball at the platform's true physical center for "
              f"{args.seconds}s...")
        t0 = time.time()
        while time.time() - t0 < args.seconds:
            frame = cam.read_frame()
            pos = detector.detect(frame)
            if pos:
                samples.append((pos.x, pos.y))
            time.sleep(0.03)

    if not samples:
        print("No ball detected during the calibration window -- check "
              "HSV_ORANGE_LOWER/UPPER and lighting above, then retry.")
        return

    avg_x = sum(s[0] for s in samples) / len(samples)
    avg_y = sum(s[1] for s in samples) / len(samples)
    print(f"\nMeasured center from {len(samples)} samples:")
    print(f"  CAMERA_CENTER_X = {avg_x:.1f}")
    print(f"  CAMERA_CENTER_Y = {avg_y:.1f}")
    print("Copy these into the CONFIG section of this file.")


def cmd_test_direction(args):
    sc = ServoController(use_hardware=not args.simulate)
    sc.move_to_neutral()
    time.sleep(0.5)

    tests = [
        ("positive roll", args.step, 0.0),
        ("negative roll", -args.step, 0.0),
        ("positive pitch", 0.0, args.step),
        ("negative pitch", 0.0, -args.step),
    ]

    log = []
    for label, roll, pitch in tests:
        input(f"\nPress Enter to command {label} "
              f"({roll:+.1f} deg roll, {pitch:+.1f} deg pitch)...")
        try:
            ik_values = inverse_kinematics(roll, pitch)
        except Exception as exc:
            print(f"  IK failed: {exc}")
            continue
        sc.apply_ik_output(ik_values)
        print(f"  Applied. Servo pulses (us): "
              f"{[round(s.current_us) for s in sc.servos]}")
        observed = input("  Which way did the platform/ball move? "
                          "(describe it, e.g. 'ball rolled toward camera') > ")
        log.append((label, observed))
        sc.move_to_neutral()
        time.sleep(0.3)

    print("\n--- Summary ---")
    for label, observed in log:
        print(f"  {label:15s} -> {observed}")
    print("\nUse this to set CAMERA_X_SIGN, CAMERA_Y_SIGN, ROLL_CONTROL_SIGN, "
          "PITCH_CONTROL_SIGN in the CONFIG section above so every correction "
          "reduces the error instead of amplifying it.")


# =====================================================================
# 11. ENTRY POINT
# =====================================================================

def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=["run", "neutral", "test-servo",
                                            "find-camera-center", "test-direction"],
                         default="run", help="what to do (default: run the full control loop)")
    parser.add_argument("--simulate", action="store_true",
                         help="calibration modes: use the simulated servo driver")
    parser.add_argument("--channel", type=int, default=0,
                         help="test-servo: which channel to sweep")
    parser.add_argument("--seconds", type=float, default=5.0,
                         help="find-camera-center: how long to sample")
    parser.add_argument("--step", type=float, default=3.0,
                         help="test-direction: degrees of roll/pitch per test")
    parser.add_argument("--hold", type=float, default=3.0,
                         help="neutral: how long to hold before exiting")

    parser.add_argument("--debug-no-servos", action="store_true",
                         help="run mode: camera+PID+IK only, never drive servos")
    parser.add_argument("--simulate-hardware", action="store_true",
                         help="run mode: force the simulated PCA9685 driver")
    parser.add_argument("--display", action="store_true",
                         help="run mode: live OpenCV debug window (needs a monitor/X11)")
    parser.add_argument("--iterations", type=int, default=None,
                         help="run mode: stop after N control-loop iterations")

    args = parser.parse_args()

    if args.mode == "run":
        run(debug_no_servos=args.debug_no_servos,
            use_hardware=not args.simulate_hardware,
            max_iterations=args.iterations,
            display=args.display)
    elif args.mode == "neutral":
        cmd_neutral(args)
    elif args.mode == "test-servo":
        cmd_test_servo(args)
    elif args.mode == "find-camera-center":
        cmd_find_camera_center(args)
    elif args.mode == "test-direction":
        cmd_test_direction(args)


if __name__ == "__main__":
    main()
