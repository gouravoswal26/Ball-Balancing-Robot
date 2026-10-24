# Ball Balancing Robot — Control Software

A closed-loop ball-balancing control system for a 3-servo tilting
platform: Raspberry Pi 4 + Picamera2 (IMX708) + PCA9685 + 3x MG996R.

```
Camera -> Ball Detection -> Position Error -> PID Controller ->
Desired Platform Orientation -> Inverse Kinematics ->
Individual Servo Commands -> Platform Movement -> Camera Feedback
```

This is a clean rewrite of `ball_5.py`, split into modular, individually
testable pieces, with a real PID controller (anti-windup, filtered
derivative, deadband), a documented inverse-kinematics module, hardware
safety limits, and calibration tooling — not just a servo-nudging script.


## Repository layout

| Path | What it is |
|---|---|
| `*.py` (root) | Modular control system: detection, PID, IK, servo control, safety, calibration |
| `tests/` | Unit tests, run without hardware: `python3 tests/_run_all.py` |
| `standalone/ball_balance_v3.py` | Latest single-file script for the Pi (per-axis auto-tune, KI enabled, sign watchdog off by default) |
| `standalone/ball_balance_v2.py` | Single-file version of the modular project |
| `legacy/ball_balance.py` | Earlier version kept for reference |
| `cad/ball_balancing_plate.dxf` | Platform plate drawing |

## Status: geometry needs verification before full closed-loop use

**Read this before running anything on the real robot.** The mechanical
geometry (arm/rod lengths, base radius, platform radius, mount height)
was never fully confirmed against your CAD model or a photo of the
actual mechanism, and one concrete inconsistency was found while writing
this: with `l1=45mm` and `l2=81mm` exactly as recorded, a simple
single-hinge-horn-plus-rod leg can only reach `l1+l2=126mm`, which is
*less* than the recorded neutral height `H=145mm`. That means at least
one of `l1`, `l2`, `H`, or the assumed mechanism itself isn't what these
notes assumed — see `config.py` section 5 and `inverse_kinematics.py`'s
module docstring for the full explanation.

Because of this, the software defaults to a **calibrated linear IK
model** (`config.IK_MODEL = "linear"`) that only assumes the three
servos are 120° apart — it does not depend on the disputed lengths at
all — so you have a working, testable control loop today. A full
geometry-based **analytical** model is also implemented and ready to use
once you send over CAD dimensions or a clear photo of the linkage, at
which point `config.GEOMETRY` gets corrected and `IK_MODEL` switched to
`"analytical"`.

## Project layout

```
ball_balancing/
├── main.py                 control loop entry point
├── camera.py                Picamera2 wrapper (+ DummyCamera for tests)
├── ball_detection.py        orange-ball HSV detection + filtering
├── pid_controller.py        PID class (anti-windup, derivative filter)
├── inverse_kinematics.py    "linear" and "analytical" IK strategies
├── servo_controller.py      PCA9685 wrapper, per-servo calibration, safety
├── calibration.py           CLI: servo test, camera center, direction test
├── safety.py                startup sequence, safe-hold, NaN guards
├── logger.py                CSV + console logging
├── config.py                ALL tunable numbers, with VERIFY/PLACEHOLDER
│                            markers on anything not yet confirmed
├── requirements.txt
└── tests/
    ├── test_pid.py           PID unit tests (no hardware needed)
    ├── test_kinematics.py    IK unit tests (no hardware needed)
    ├── test_servo_mapping.py servo-command unit tests (no hardware needed)
    └── _run_all.py           plain-Python runner, for environments
                               without PyPI access to install pytest
```

## Install

On the Raspberry Pi:

```bash
sudo apt update && sudo apt install -y python3-picamera2
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install -r requirements.txt
```

`picamera2` is a system package on Raspberry Pi OS, not something to pip
install into an isolated venv — see the note at the bottom of
`requirements.txt`.

## Run the unit tests first (no hardware required)

```bash
pytest tests/
# or, if pytest isn't installable in your environment:
python3 tests/_run_all.py
```

All 30 tests should pass. They cover PID behavior (P/I/D terms,
anti-windup, deadband, bad `dt`, NaN input, reset), both IK models
(neutral pose, small tilts, sign symmetry, unreachable poses), and servo
command mapping (direction, clamping, rate limiting, NaN safety).

## Hardware safety first

* **Do not power the MG996R servos from the Raspberry Pi's 5V pins.**
  Use the dedicated 5V/5A supply, with a common ground to the Pi.
* Confirm the PCA9685 is on the I2C bus at `0x40`:
  `i2cdetect -y 1` should show it (a ToF sensor at `0x29` and something
  at `0x70` are expected and unrelated).
* `config.SERVO_CONFIG` ships with a conservative `±200us` range around
  each center. Do not widen it until you've confirmed each servo's real
  safe mechanical range by hand.
* `config.MAX_SERVO_STEP_US` and `config.SAFETY_LIMIT_US` cap how far
  and how fast any single control update can move a servo, regardless
  of what the PID/IK computed.

## Calibration procedure

Do these in order — each one is a `calibration.py` subcommand. Add
`--simulate` to any of them to dry-run against the software-only
simulated servo driver instead of real hardware.

**1. Servo centers and direction** (Section 8, Step 1)

```bash
python3 calibration.py test-servo 0
python3 calibration.py test-servo 1
python3 calibration.py test-servo 2
```

Watch each servo sweep slowly between its configured `min_us`/`max_us`.
Confirm the linkage doesn't bind, and note down where the platform is
actually level — update `center_us` per channel in `config.SERVO_CONFIG`
if it isn't 1500/1550/1500.

```bash
python3 calibration.py neutral
```

Confirms all three servos move to their configured centers and the
platform sits level (or as intended) with no PID running.

**2. Camera center**

```bash
python3 calibration.py find-camera-center --seconds 5
```

Hold the ball still at the platform's true physical center while this
samples detections, then copy the printed `CAMERA_CENTER_X/Y` into
`config.py`. Re-run this any time `CAMERA_RESOLUTION` changes.

**3. Ball color (HSV)**

There's no dedicated subcommand for this yet — if `find-camera-center`
reports "no ball detected," tune `HSV_ORANGE_LOWER`/`HSV_ORANGE_UPPER`
in `config.py` against your actual ball and lighting (a quick way: save
a frame, sample the ball's HSV values with an image tool, and widen the
range a little around what you see).

**4. Control-direction signs** (Section 11 — the most important step)

```bash
python3 calibration.py test-direction --step 3
```

This commands a small, known roll/pitch one at a time and asks you what
actually happened. Use your notes to set `CAMERA_X_SIGN`,
`CAMERA_Y_SIGN`, `ROLL_CONTROL_SIGN`, `PITCH_CONTROL_SIGN` in
`config.py` so that every correction reduces the error instead of
amplifying it. **Do not skip this and do not guess these signs** — a
wrong sign here turns the whole system into positive feedback.

## Tuning the PID

With directions confirmed:

1. Set `PID_X`/`PID_Y` to `ki=0, kd=0` and only `kp` nonzero.
2. Run `python3 main.py --debug-no-servos` and watch the logged PID
   output respond to the ball moving — increase `kp` until the platform
   visibly (once servos are re-enabled) responds and starts to correct,
   backing off if it oscillates or overshoots.
3. Add `kd` gradually to damp oscillation. Camera noise means too much
   `kd` will amplify jitter — `DERIVATIVE_FILTER_ALPHA` in `config.py`
   controls how much the error is smoothed before differentiating.
4. Only add a small `ki` if there's a persistent steady-state offset
   that calibration and `kp`/`kd` don't resolve. `INTEGRAL_LIMIT` caps
   windup.
5. Enable both axes together and watch `logs/run_*.csv` for stability,
   overshoot, and recovery behavior when the ball is momentarily lost.

## Running

```bash
python3 main.py --debug-no-servos      # camera+PID+IK, no servo motion
python3 main.py --simulate-hardware    # full loop, simulated PCA9685
python3 main.py                        # full closed-loop run
```

`main.py` always starts by moving to neutral and holding for
`STARTUP_SETTLE_S` before enabling any PID correction, and always
returns to neutral on exit (including Ctrl+C). Every iteration is logged
to `logs/run_<timestamp>.csv` and mirrored to the console (Section 15's
`Ball=/Error=/PID=/ServoAngles=` style log line).

## What still needs the real mechanism

Everything above works off `IK_MODEL = "linear"`, which only assumes a
120°, three-fold-symmetric layout. To get the more precise
`"analytical"` model working, we need one of:

* A CAD export or clear photo of the linkage showing the servo horn
  pivot locations, the connecting-rod attachment points on the
  platform, and how `H` was actually measured, or
* Direct measurements of: `base_radius` (servo-horn-pivot distance from
  the platform's center axis), `platform_radius` (rod-attachment
  distance from platform center), and confirmation of whether the servo
  horn truly sweeps in the simple vertical plane this model assumes.

Once that's in hand, update `config.GEOMETRY`, set `config.IK_MODEL =
"analytical"`, and re-run `tests/test_kinematics.py` — the
neutral-pose/small-tilt tests are written to tell you clearly whether
the corrected geometry is now self-consistent (reachable) or still
isn't.
