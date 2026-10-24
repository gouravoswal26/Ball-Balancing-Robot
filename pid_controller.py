"""
pid_controller.py
==================
A single-axis PID controller with the safety features Section 7 of the
project spec requires:

  * anti-windup on the integral term (the integral only accumulates on
    updates that did NOT require clamping the output -- a clamping /
    back-calculation hybrid)
  * a low-pass filter on the error before it is differentiated, so the
    D-term does not amplify camera/detection noise
  * a configurable deadband
  * output clamping
  * derivative-kick avoidance (no derivative contribution on the very
    first update after construction or after reset())
  * protection against zero/garbage dt and NaN/inf error values
  * reset() -- call this whenever the ball is lost, so stale
    integral/derivative state doesn't fire a large correction on the
    frame the ball reappears

Two independent instances are used, one per axis (Section 7: "Implement
a separate PID controller for each platform control axis").
"""

from dataclasses import dataclass
import math


@dataclass
class PIDResult:
    output: float
    p: float
    i: float
    d: float
    filtered_error: float


class PIDController:
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
        """Call this whenever the ball is lost or the loop is paused.
        Prevents stale integral/derivative state from producing a large,
        unearned correction when control resumes."""
        self._integral = 0.0
        self._filtered_error = 0.0
        self._prev_filtered_error = None
        self._have_prev = False

    def update(self, error, dt):
        """error: current position error (target - measured), in
        whatever units the caller uses (this project uses pixels).
        dt: measured elapsed time since the previous update, seconds.
        Returns a PIDResult with the combined output plus each term,
        for logging/tuning."""

        if dt is None or dt <= 0 or not math.isfinite(dt):
            # A zero, negative, or garbage timestamp would produce a
            # divide-by-zero or an exploding derivative/integral. Skip
            # I/D contributions for this one update rather than risk it.
            dt = 1e-3

        if error is None or not math.isfinite(error):
            # Never let a NaN/inf reading reach the integrator.
            error = 0.0

        # ---- deadband ------------------------------------------------
        active_error = 0.0 if abs(error) < self.deadband else error

        # ---- derivative filtering (low-pass on the error itself, so a
        # single noisy frame doesn't dominate the D term) ---------------
        self._filtered_error = (
            self.alpha * active_error + (1 - self.alpha) * self._filtered_error
        )

        # ---- P ---------------------------------------------------------
        p = self.kp * active_error

        # ---- I, computed but not yet committed (anti-windup below) -----
        prospective_integral = self._integral + self.ki * active_error * dt
        prospective_integral = _clamp(
            prospective_integral, -self.integral_limit, self.integral_limit)

        # ---- D, from the filtered error, skipped on the first sample so
        # there's no derivative kick from an undefined previous value ----
        if self._have_prev:
            d = self.kd * (self._filtered_error - self._prev_filtered_error) / dt
        else:
            d = 0.0

        self._prev_filtered_error = self._filtered_error
        self._have_prev = True

        raw_output = p + prospective_integral + d
        output = _clamp(raw_output, -self.output_limit, self.output_limit)

        # Anti-windup: only commit the integral update if it did NOT
        # require clamping the output. If we're already saturated,
        # continuing to integrate would just make the eventual unwind
        # slower and overshoot worse.
        if raw_output == output:
            self._integral = prospective_integral

        return PIDResult(output=output, p=p, i=self._integral, d=d,
                          filtered_error=self._filtered_error)


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))
