"""
main.py
========
Entry point: wires camera -> ball_detection -> PID -> inverse_kinematics
-> servo_controller into the control loop described in Section 12 of the
project spec, with the safety (Section 14) and logging (Section 15)
requirements layered in.

    Camera -> Ball Detection -> Position Error -> PID Controller ->
    Desired Platform Orientation -> Inverse Kinematics ->
    Individual Servo Commands -> Platform Movement -> Camera Feedback

Run modes:
    python main.py                     full closed-loop run on hardware
    python main.py --debug-no-servos   run camera+PID+IK, print/log
                                        everything, never actually drive
                                        the servos
    python main.py --simulate-hardware run the whole loop against the
                                        simulated PCA9685 (no I2C bus
                                        required) -- for dry-running on
                                        a dev machine
    python main.py --iterations N      stop after N loop iterations
                                        (used by integration testing)
"""

import argparse
import time

import config
from camera import Camera
from ball_detection import BallDetector
from pid_controller import PIDController
from inverse_kinematics import inverse_kinematics, IKUnreachable
from servo_controller import ServoController
from safety import SafetyMonitor, startup_sequence, safe_hold_action
from logger import Logger


def build_pid_controllers():
    pid_x = PIDController(
        **config.PID_X,
        integral_limit=config.INTEGRAL_LIMIT,
        output_limit=config.OUTPUT_LIMIT,
        deadband=config.DEADBAND_X_PX,
        derivative_filter_alpha=config.DERIVATIVE_FILTER_ALPHA,
        name="x",
    )
    pid_y = PIDController(
        **config.PID_Y,
        integral_limit=config.INTEGRAL_LIMIT,
        output_limit=config.OUTPUT_LIMIT,
        deadband=config.DEADBAND_Y_PX,
        derivative_filter_alpha=config.DERIVATIVE_FILTER_ALPHA,
        name="y",
    )
    return pid_x, pid_y


def calculate_error(ball_x, ball_y):
    """error = target - measured, sign-corrected per the calibrated
    CAMERA_X_SIGN/CAMERA_Y_SIGN (Section 6 / Section 11) -- do not assume
    a positive camera error always means the same physical direction
    until calibration.py test-direction has confirmed it."""
    error_x = (config.CAMERA_CENTER_X - ball_x) * config.CAMERA_X_SIGN
    error_y = (config.CAMERA_CENTER_Y - ball_y) * config.CAMERA_Y_SIGN
    return error_x, error_y


def controller_to_orientation(pid_output_x, pid_output_y):
    """Maps PID output -> desired platform roll/pitch, clamped to the
    configured safe range (Section 9.1). Sign-corrected per
    ROLL_CONTROL_SIGN/PITCH_CONTROL_SIGN."""
    roll = _clamp(pid_output_x * config.ANGLE_SCALE_X * config.ROLL_CONTROL_SIGN,
                  -config.MAX_ROLL_DEG, config.MAX_ROLL_DEG)
    pitch = _clamp(pid_output_y * config.ANGLE_SCALE_Y * config.PITCH_CONTROL_SIGN,
                   -config.MAX_PITCH_DEG, config.MAX_PITCH_DEG)
    return roll, pitch


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def run(debug_no_servos=False, use_hardware=True, max_iterations=None):
    logger = Logger()
    detector = BallDetector()
    servo_controller = ServoController(use_hardware=use_hardware and not debug_no_servos)
    pid_x, pid_y = build_pid_controllers()
    safety = SafetyMonitor()

    startup_sequence(servo_controller, logger)

    dt_target = 1.0 / config.CONTROL_LOOP_HZ
    last_time = time.time()
    iteration = 0

    with Camera() as camera:
        try:
            while max_iterations is None or iteration < max_iterations:
                loop_start = time.time()
                dt = loop_start - last_time
                last_time = loop_start

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
                        target_x=config.CAMERA_CENTER_X,
                        target_y=config.CAMERA_CENTER_Y,
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
                        # Detection-loss handling (Section 5 / Section 14):
                        # reset controller + detector state so stale
                        # integral/derivative/history doesn't produce a
                        # large correction the instant the ball reappears,
                        # and hold position rather than guessing.
                        pid_x.reset()
                        pid_y.reset()
                        detector.reset()
                        safe_hold_action(servo_controller, logger, "ball not detected")

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


def main():
    parser = argparse.ArgumentParser(description="Ball balancing control loop")
    parser.add_argument("--debug-no-servos", action="store_true",
                         help="Run camera+PID+IK but never actually drive servos")
    parser.add_argument("--simulate-hardware", action="store_true",
                         help="Force the simulated PCA9685 driver even on a Pi")
    parser.add_argument("--iterations", type=int, default=None,
                         help="Stop after N control-loop iterations (for testing)")
    args = parser.parse_args()

    run(debug_no_servos=args.debug_no_servos,
        use_hardware=not args.simulate_hardware,
        max_iterations=args.iterations)


if __name__ == "__main__":
    main()
