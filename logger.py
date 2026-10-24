"""
logger.py
=========
Structured logging for tuning/debugging, per Section 15 of the project
spec: one CSV row per control-loop iteration (for later plotting/tuning)
plus a human-readable console mirror, and a separate free-text event
stream for startup/safety messages.
"""

import csv
import os
import time

import config

FIELDNAMES = [
    "t", "ball_x", "ball_y", "target_x", "target_y",
    "error_x", "error_y", "filtered_error_x", "filtered_error_y",
    "pid_x", "pid_y", "roll_deg", "pitch_deg",
    "servo_0_us", "servo_1_us", "servo_2_us", "loop_time_s",
]


class Logger:
    def __init__(self, log_dir=None, to_csv=None, to_console=None):
        self.to_csv = config.LOG_TO_CSV if to_csv is None else to_csv
        self.to_console = config.LOG_TO_CONSOLE if to_console is None else to_console
        self._csv_file = None
        self._csv_writer = None
        self.csv_path = None

        if self.to_csv:
            log_dir = log_dir or config.LOG_DIR
            os.makedirs(log_dir, exist_ok=True)
            path = os.path.join(log_dir, f"run_{int(time.time())}.csv")
            self._csv_file = open(path, "w", newline="")
            self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=FIELDNAMES)
            self._csv_writer.writeheader()
            self.csv_path = path

    def log_control_data(self, **kwargs):
        row = {k: kwargs.get(k, "") for k in FIELDNAMES}
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
