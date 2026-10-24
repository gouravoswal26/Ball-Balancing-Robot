"""
safety.py
=========
Cross-cutting safety helpers used by main.py's control loop (Section 14):

  * a startup sequence that eases into neutral rather than snapping to it
  * a SafetyMonitor that tracks consecutive detection failures and
    invalid numeric values, and decides when to fall back to a safe hold
  * a safe-hold action for both "ball lost" and "unsafe value" cases
"""

import math
import time

import config


class SafetyMonitor:
    def __init__(self, max_missed_frames=None):
        self.max_missed_frames = max_missed_frames or config.MAX_MISSED_FRAMES
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
        """Returns True (and records trip_reason) if any value is
        NaN/inf. Does not raise -- the caller decides what to do."""
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
    """Move to the configured neutral position and hold briefly before
    the control loop starts issuing PID corrections. Prevents a violent
    first movement if the servos happened to power up somewhere far from
    center (Section 14: 'do not allow the robot to suddenly move to
    extreme servo positions during startup')."""
    if logger:
        logger.log_event("startup: moving to neutral position")
    servo_controller.move_to_neutral()
    time.sleep(config.STARTUP_SETTLE_S)


def safe_hold_action(servo_controller, logger=None, reason=""):
    """Called when the ball is lost, an IK solution is invalid, or a
    non-finite value is detected. Holds the platform at its last
    commanded (already-safe) position rather than applying a fresh,
    unvalidated correction."""
    if logger:
        logger.log_event(f"safety hold: {reason}")
    servo_controller.emergency_stop()
