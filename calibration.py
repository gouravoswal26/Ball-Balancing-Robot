"""
calibration.py
================
Command-line calibration utilities. Run these BEFORE enabling the full
PID control loop -- see README.md "Calibration" for the full walk-through
(mirrors Sections 8 and 11 of the project spec).

Subcommands:

  neutral                  Move every servo to its configured center_us
                            and hold. Sanity check after editing config.py.

  test-servo CHANNEL       Sweep a single servo slowly between its
                            configured min_us and max_us so you can watch
                            for binding and confirm the safe range.

  find-camera-center       Hold the ball at the platform's true physical
                            center while this samples detected positions,
                            then prints CAMERA_CENTER_X/Y for config.py.

  test-direction           Nudge roll/pitch by a small, fixed amount one
                            at a time and asks you to record which way
                            the platform/ball actually moved, so you can
                            fill in CAMERA_X_SIGN / CAMERA_Y_SIGN /
                            ROLL_CONTROL_SIGN / PITCH_CONTROL_SIGN
                            (Section 11) instead of guessing them.

Run with --simulate to exercise any of these against the SimulatedPCA9685
driver (no hardware attached) -- useful for reviewing the calibration
flow itself before doing it on the real robot.
"""

import argparse
import time

from servo_controller import ServoController
import config


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
    # Imported lazily: these pull in picamera2/opencv, which aren't
    # needed for the servo-only calibration commands above.
    from ball_detection import BallDetector
    from camera import Camera

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
              "HSV_ORANGE_LOWER/UPPER and lighting in config.py, then retry.")
        return

    avg_x = sum(s[0] for s in samples) / len(samples)
    avg_y = sum(s[1] for s in samples) / len(samples)
    print(f"\nMeasured center from {len(samples)} samples:")
    print(f"  CAMERA_CENTER_X = {avg_x:.1f}")
    print(f"  CAMERA_CENTER_Y = {avg_y:.1f}")
    print("Copy these into config.py (Section 1).")


def cmd_test_direction(args):
    """Section 11: nudge one axis a small, known amount and have the
    operator report which way the platform/ball actually moved, rather
    than assuming the sign is already correct."""
    from inverse_kinematics import inverse_kinematics

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
    print("\nUse this to set CAMERA_X_SIGN, CAMERA_Y_SIGN, "
          "ROLL_CONTROL_SIGN, PITCH_CONTROL_SIGN in config.py so that "
          "the loop always drives the error toward zero, never away "
          "from it.")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--simulate", action="store_true",
                         help="Use the simulated servo driver instead of real hardware")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("neutral")
    p.add_argument("--hold", type=float, default=3.0)
    p.set_defaults(func=cmd_neutral)

    p = sub.add_parser("test-servo")
    p.add_argument("channel", type=int)
    p.set_defaults(func=cmd_test_servo)

    p = sub.add_parser("find-camera-center")
    p.add_argument("--seconds", type=float, default=5.0)
    p.set_defaults(func=cmd_find_camera_center)

    p = sub.add_parser("test-direction")
    p.add_argument("--step", type=float, default=3.0,
                   help="degrees of roll/pitch to command for each test")
    p.set_defaults(func=cmd_test_direction)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
