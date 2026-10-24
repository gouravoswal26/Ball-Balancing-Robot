import time
import cv2
import numpy as np
import board
import busio

from picamera2 import Picamera2
from adafruit_pca9685 import PCA9685


# ============================================================
# BALL BALANCING PLATFORM
# Raspberry Pi 4 + Pi Camera + PCA9685 + 3x MG996R
# ============================================================
#
# WHAT WAS FIXED vs your original code:
#
# 1. MIN_US/MAX_US widened. Your original window was only 100us
#    wide (e.g. 1450-1550), which is ~1-2 degrees of real servo
#    travel -> nowhere near enough tilt to hold a ball against
#    gravity. Widened to +-250us from neutral as a starting point.
#    YOU MUST CALIBRATE THIS on your own rig (see CALIBRATION
#    MODE below) before trusting these numbers.
#
# 2. MAX_CORRECTION raised to match (was 8, now 200) so the
#    controller output isn't clipped down to near-zero before it
#    ever reaches the servos.
#
# 3. Added a CALIBRATE_MIX_SIGNS mode: run this once, ball OFF
#    the platform, to visually confirm each MIX row tilts the
#    platform the direction you expect. Wrong sign = platform
#    actively kicks the ball off instead of correcting it, which
#    looks exactly like "servo responds but ball still drops."
#
# 4. Gains reset to conservative defaults appropriate for the
#    new wider range. Re-tune from here, don't reuse old KP/KD.
#
# 5. Minor fix: display var could be unbound before first frame
#    in the "ball lost" branch on the very first loop iteration -
#    display is now always created before the branch.
# ============================================================


# ---------------- CAMERA ----------------

WIDTH = 640
HEIGHT = 480

TARGET_X = 338
TARGET_Y = 264

# Orange ball HSV range
LOWER_ORANGE = np.array([5, 120, 100])
UPPER_ORANGE = np.array([25, 255, 255])


# ---------------- PCA9685 ----------------

PCA_ADDRESS = 0x40

CHANNELS = [0, 1, 2]

# Known mechanical neutral (flat platform). Verify with ball off
# and platform physically level before trusting this.
NEUTRAL = np.array([
    1500.0,
    1550.0,
    1500.0
])

# ------------------------------------------------------------
# SAFE LIMITS - CALIBRATE THESE FOR YOUR RIG.
#
# Starting point below is +-250us from neutral. Before running
# closed loop, run CALIBRATE_MIX_SIGNS (set the flag below) and
# slowly increase this range while watching for:
#   - servo horn / linkage hitting a mechanical stop
#   - platform edge hitting the frame
#   - servo stalling/buzzing (current limit)
# Back off ~20% from whatever the physical max is.
# ------------------------------------------------------------

RANGE_US = 250.0  # half-width of travel from neutral, per channel

MIN_US = NEUTRAL - RANGE_US
MAX_US = NEUTRAL + RANGE_US


# ============================================================
# CONTROL SETTINGS
# ============================================================

# Conservative starting gains for the WIDENED range above.
# Do not reuse gains tuned against the old 8us-clipped system -
# those were compensating for a saturated, nearly-open-loop
# system and will likely be far too aggressive here.
KP_X = 0.6
KP_Y = 0.6

KD_X = 0.15
KD_Y = 0.15

# Camera noise dead zone (pixels)
DEAD_ZONE = 6

# Maximum servo *correction* magnitude from neutral, in us.
# Must be <= RANGE_US, and should usually be somewhat less than
# RANGE_US so smoothing/overshoot doesn't slam into MIN/MAX_US.
MAX_CORRECTION = 200

# Position filtering (higher = trust new measurement more)
FILTER_ALPHA = 0.35

# Servo smoothing (higher = servo moves to target faster)
SERVO_ALPHA = 0.40

LOOP_DELAY = 0.02

# Set True to run a slow, manual, open-loop sign-check instead
# of closed-loop balancing. Ball OFF platform, platform empty.
CALIBRATE_MIX_SIGNS = False


# ============================================================
# SERVO MIXING
#
# IMPORTANT: signs here depend entirely on your mechanism and
# camera orientation. Verify with CALIBRATE_MIX_SIGNS before
# trusting closed loop - a wrong sign tilts the platform AWAY
# from center and looks identical to "not enough force."
# ============================================================

MIX = np.array([
    [ 1.0,  0.0],      # CH0
    [ 0.0,  0.7],      # CH1
    [ 0.5, -0.5]       # CH2
])


# ============================================================
# PCA9685 INITIALIZATION
# ============================================================

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c, address=PCA_ADDRESS)
pca.frequency = 50


def us_to_duty(us):
    period_us = 1000000 / 50
    duty = int((us / period_us) * 65535)
    return max(0, min(65535, duty))


def set_servo(channel, pulse):
    pca.channels[channel].duty_cycle = us_to_duty(pulse)


def write_servos(values):
    values = np.asarray(values, dtype=float)
    values = np.clip(values, MIN_US, MAX_US)
    for i in range(3):
        set_servo(CHANNELS[i], int(values[i]))


def neutral():
    write_servos(NEUTRAL)


# ============================================================
# CAMERA INITIALIZATION
# ============================================================

picam2 = Picamera2()

camera_config = picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (WIDTH, HEIGHT)}
)

picam2.configure(camera_config)
picam2.start()
time.sleep(2)


# ============================================================
# BALL DETECTION
# ============================================================

def detect_ball(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_ORANGE, UPPER_ORANGE)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)

    if area < 500:
        return None

    (x, y), radius = cv2.minEnclosingCircle(contour)
    return (int(x), int(y), int(radius), area)


# ============================================================
# CALIBRATION MODE (open loop, manual sign check)
# ============================================================

def run_mix_sign_calibration():
    """
    Steps each control axis to a fixed test value one at a time
    so you can visually confirm tilt direction matches the
    intended correction direction. Ball should be OFF the
    platform. Press any key to advance, 'q' to quit.
    """
    print()
    print("=== MIX SIGN CALIBRATION ===")
    print("Ball OFF platform. Watch tilt direction for each step.")
    print("Press any key to move to next test, 'q' to quit.")
    print()

    test_value = 100.0  # us equivalent test correction

    tests = [
        ("control_x = +", np.array([test_value, 0.0])),
        ("control_x = -", np.array([-test_value, 0.0])),
        ("control_y = +", np.array([0.0, test_value])),
        ("control_y = -", np.array([0.0, -test_value])),
    ]

    neutral()
    time.sleep(1)

    for label, control in tests:
        servo_delta = MIX @ control
        servo_delta = np.clip(servo_delta, -MAX_CORRECTION, MAX_CORRECTION)
        desired = np.clip(NEUTRAL + servo_delta, MIN_US, MAX_US)

        print(f"{label}  ->  servo_delta={servo_delta.astype(int)}  "
              f"pulses={desired.astype(int)}")

        write_servos(desired)

        key = cv2.waitKey(0) & 0xFF
        if key == ord("q"):
            break

        neutral()
        time.sleep(0.5)

    neutral()
    print("Calibration sequence done.")


# ============================================================
# INITIAL STATE
# ============================================================

neutral()

current_servo = NEUTRAL.copy()

filtered_x = None
filtered_y = None

previous_error_x = 0.0
previous_error_y = 0.0

previous_time = time.monotonic()


print()
print("==============================================")
print("        BALL BALANCING SYSTEM")
print("==============================================")
print()
print("PCA9685 address :", hex(PCA_ADDRESS))
print("Camera          :", WIDTH, "x", HEIGHT)
print("Target          :", TARGET_X, TARGET_Y)
print("Neutral         :", NEUTRAL.astype(int))
print("Pulse range     :", MIN_US.astype(int), "to", MAX_US.astype(int))
print()
print("Q = STOP")
print("N = NEUTRAL")
print()
print("Place the ball on the platform.")
print("==============================================")

if CALIBRATE_MIX_SIGNS:
    run_mix_sign_calibration()
    picam2.stop()
    picam2.close()
    cv2.destroyAllWindows()
    pca.deinit()
    raise SystemExit("Calibration complete. Set CALIBRATE_MIX_SIGNS=False to run closed loop.")


# ============================================================
# MAIN LOOP
# ============================================================

try:
    while True:

        frame = picam2.capture_array()
        result = detect_ball(frame)
        display = frame.copy()

        # ====================================================
        # BALL FOUND
        # ====================================================
        if result is not None:

            x, y, radius, area = result

            # ------------------------------------------------
            # FILTER BALL POSITION
            # ------------------------------------------------
            if filtered_x is None:
                filtered_x = float(x)
                filtered_y = float(y)
            else:
                filtered_x = FILTER_ALPHA * x + (1 - FILTER_ALPHA) * filtered_x
                filtered_y = FILTER_ALPHA * y + (1 - FILTER_ALPHA) * filtered_y

            # ------------------------------------------------
            # POSITION ERROR
            # ------------------------------------------------
            error_x = TARGET_X - filtered_x
            error_y = TARGET_Y - filtered_y

            # ------------------------------------------------
            # DEAD ZONE (applied to the P term only)
            # ------------------------------------------------
            control_x = 0.0 if abs(error_x) < DEAD_ZONE else error_x
            control_y = 0.0 if abs(error_y) < DEAD_ZONE else error_y

            # ------------------------------------------------
            # TIME
            # ------------------------------------------------
            now = time.monotonic()
            dt = now - previous_time
            if dt < 0.001:
                dt = 0.001

            # ------------------------------------------------
            # BALL VELOCITY / DERIVATIVE (on raw error, not
            # dead-zoned error, so D term stays smooth)
            # ------------------------------------------------
            velocity_x = (error_x - previous_error_x) / dt
            velocity_y = (error_y - previous_error_y) / dt

            # ------------------------------------------------
            # PD CONTROLLER
            # ------------------------------------------------
            control_x = KP_X * control_x + KD_X * velocity_x
            control_y = KP_Y * control_y + KD_Y * velocity_y

            control = np.array([control_x, control_y])

            # ------------------------------------------------
            # LIMIT CONTROL (pre-mix)
            # ------------------------------------------------
            control = np.clip(control, -MAX_CORRECTION, MAX_CORRECTION)

            # ------------------------------------------------
            # 3-SERVO MIXING
            # ------------------------------------------------
            servo_delta = MIX @ control
            servo_delta = np.clip(servo_delta, -MAX_CORRECTION, MAX_CORRECTION)

            # ------------------------------------------------
            # SERVO TARGET
            # ------------------------------------------------
            desired_servo = NEUTRAL + servo_delta
            desired_servo = np.clip(desired_servo, MIN_US, MAX_US)

            # ------------------------------------------------
            # SMOOTH SERVO MOVEMENT
            # ------------------------------------------------
            current_servo = (
                SERVO_ALPHA * desired_servo
                + (1 - SERVO_ALPHA) * current_servo
            )

            # ------------------------------------------------
            # SEND TO PCA9685
            # ------------------------------------------------
            write_servos(current_servo)

            # ------------------------------------------------
            # UPDATE HISTORY
            # ------------------------------------------------
            previous_error_x = error_x
            previous_error_y = error_y
            previous_time = now

            # ------------------------------------------------
            # DISPLAY
            # ------------------------------------------------
            cv2.circle(display, (x, y), radius, (0, 255, 0), 3)
            cv2.circle(display, (x, y), 5, (0, 0, 255), -1)
            cv2.circle(display, (TARGET_X, TARGET_Y), 8, (0, 255, 255), -1)
            cv2.line(display, (x, y), (TARGET_X, TARGET_Y), (255, 0, 255), 2)

            cv2.putText(display, f"BALL X={x} Y={y}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)

            cv2.putText(display, f"ERROR X={int(error_x)} Y={int(error_y)}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.60,
                        (255, 255, 255), 2)

            cv2.putText(display,
                        f"SERVO {int(current_servo[0])} "
                        f"{int(current_servo[1])} {int(current_servo[2])}",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (255, 255, 255), 2)

            cv2.putText(display, "BALL DETECTED", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 0), 2)

            print(
                f"Ball=({x},{y}) "
                f"Error=({int(error_x)},{int(error_y)}) "
                f"Servo=({int(current_servo[0])},"
                f"{int(current_servo[1])},{int(current_servo[2])})"
            )

        # ====================================================
        # BALL LOST
        # ====================================================
        else:
            neutral()
            current_servo = NEUTRAL.copy()

            filtered_x = None
            filtered_y = None

            previous_error_x = 0.0
            previous_error_y = 0.0

            previous_time = time.monotonic()

            cv2.putText(display, "BALL NOT DETECTED", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

        # ====================================================
        # CENTER CROSSHAIR
        # ====================================================
        cv2.line(display, (TARGET_X, 0), (TARGET_X, HEIGHT), (255, 255, 255), 1)
        cv2.line(display, (0, TARGET_Y), (WIDTH, TARGET_Y), (255, 255, 255), 1)

        # ====================================================
        # CAMERA WINDOW
        # ====================================================
        cv2.imshow("BALL BALANCING", display)

        # ====================================================
        # KEYBOARD
        # ====================================================
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("n"):
            neutral()
            current_servo = NEUTRAL.copy()
            print("NEUTRAL")

        time.sleep(LOOP_DELAY)


# ============================================================
# SAFE SHUTDOWN
# ============================================================

finally:
    print()
    print("Stopping ball balancing...")

    neutral()
    time.sleep(0.5)

    for channel in CHANNELS:
        pca.channels[channel].duty_cycle = 0

    pca.deinit()

    picam2.stop()
    picam2.close()

    cv2.destroyAllWindows()

    print("System stopped safely.")
