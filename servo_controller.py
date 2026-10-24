"""
servo_controller.py
====================
Wraps the PCA9685 PWM driver and converts inverse_kinematics() output
into safe, rate-limited servo pulse widths.

Hardware access uses adafruit-circuitpython-pca9685 / adafruit-blinka if
available. If the library or the I2C bus isn't present (e.g. running the
control loop off the Raspberry Pi), this module transparently falls back
to a SimulatedPCA9685 that just records the last commanded pulse widths
-- so main.py's control loop, the PID controllers, and the IK module can
all be exercised without hardware attached (Section 15's debug mode).

IMPORTANT (Section 2 / hardware safety): the MG996R servos must be
powered from the 5V/5A supply, NOT from the Raspberry Pi's own 5V pins.
This module has no way to enforce that in software -- verify your wiring
before running any of this with use_hardware=True.
"""

from dataclasses import dataclass
import math

import config


class SimulatedPCA9685:
    """Drop-in stand-in for the real PCA9685 driver. Records commands
    instead of producing PWM, so the rest of the stack can be tested
    without a Raspberry Pi / I2C bus."""

    def __init__(self, *_, **__):
        self.channels = {}
        self.frequency = config.PCA9685_PWM_FREQUENCY_HZ

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
        pca = _RealPCA9685(i2c, address=config.PCA9685_I2C_ADDRESS)
        pca.frequency = config.PCA9685_PWM_FREQUENCY_HZ
        return _HardwarePCA9685Adapter(pca)
    except Exception as exc:
        print(f"[servo_controller] PCA9685 hardware not available "
              f"({exc!r}); falling back to the simulated driver.")
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
        servo_config = servo_config or config.SERVO_CONFIG
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

    # -- initialization / shutdown --------------------------------------

    def move_to_neutral(self):
        for servo in self.servos:
            self._commit(servo, servo.center_us)

    def emergency_stop(self):
        """Hold every servo exactly where it currently is -- does not
        move to neutral (that could itself be an unsafe motion mid-fault)
        -- just re-asserts the last known-safe command."""
        for servo in self.servos:
            self._commit(servo, servo.current_us)

    # -- command path ------------------------------------------------------

    def apply_ik_output(self, ik_values):
        """ik_values: the list returned by inverse_kinematics(). Read as
        microsecond deltas if config.IK_MODEL == "linear", or as
        angle-deltas in degrees from neutral if config.IK_MODEL ==
        "analytical". Either way, the result is clamped to each servo's
        configured min/max, rate-limited by config.MAX_SERVO_STEP_US per
        call, and hard-capped by config.SAFETY_LIMIT_US relative to
        center. Returns the list of resulting pulse widths (us), one per
        servo, in the same order as self.servos."""
        if len(ik_values) != len(self.servos):
            raise ValueError(
                f"Expected {len(self.servos)} IK values, got {len(ik_values)}")

        commands = []
        for servo, value in zip(self.servos, ik_values):
            if value is None or not math.isfinite(value):
                # Never forward NaN/inf to a servo -- treat as "no
                # correction requested" rather than guessing.
                value = 0.0

            if config.IK_MODEL == "analytical":
                # Coarse degrees -> microseconds conversion. VERIFY this
                # slope against the real servo (measure us at two known
                # horn angles and compute us-per-degree) before trusting
                # it for anything beyond small-angle bring-up.
                delta_us = value * 10.0 * servo.direction
            else:
                delta_us = value * servo.direction

            delta_us = _clamp(delta_us, -config.SAFETY_LIMIT_US, config.SAFETY_LIMIT_US)
            target_us = servo.center_us + delta_us
            target_us = _clamp(target_us, servo.min_us, servo.max_us)

            # Rate limit: never move more than MAX_SERVO_STEP_US in one
            # control-loop update, regardless of how big the requested
            # jump was.
            step = _clamp(target_us - servo.current_us,
                          -config.MAX_SERVO_STEP_US, config.MAX_SERVO_STEP_US)
            new_us = servo.current_us + step

            self._commit(servo, new_us)
            commands.append(new_us)

        return commands

    def _commit(self, servo, pulse_us):
        pulse_us = _clamp(pulse_us, servo.min_us, servo.max_us)
        servo.current_us = pulse_us
        self._driver.set_pulse_us(servo.channel, pulse_us)


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))
