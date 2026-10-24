"""
ball_detection.py
==================
Orange-ball detection: HSV threshold -> morphology -> contour selection
-> smoothing/rejection, per Section 5 of the project spec.

Requires opencv-python and numpy. These are only imported here (not in
pid_controller.py, inverse_kinematics.py, or servo_controller.py) so the
rest of the control stack stays testable on a machine without OpenCV.
"""

from collections import deque
from dataclasses import dataclass
from typing import Optional

import numpy as np

try:
    import cv2
except ImportError as exc:  # pragma: no cover
    cv2 = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None

import config


@dataclass
class BallPosition:
    x: float
    y: float
    radius: float


class BallDetector:
    def __init__(self,
                 hsv_lower=None,
                 hsv_upper=None,
                 min_radius=None,
                 max_radius=None,
                 min_area=None,
                 max_jump=None,
                 filter_window=None):
        if cv2 is None:
            raise ImportError(
                "opencv-python is required for BallDetector") from _IMPORT_ERROR

        self.hsv_lower = np.array(hsv_lower or config.HSV_ORANGE_LOWER, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper or config.HSV_ORANGE_UPPER, dtype=np.uint8)
        self.min_radius = min_radius or config.MIN_BALL_RADIUS_PX
        self.max_radius = max_radius or config.MAX_BALL_RADIUS_PX
        self.min_area = min_area or config.MIN_CONTOUR_AREA_PX2
        self.max_jump = max_jump or config.MAX_POSITION_JUMP_PX

        self._kernel = np.ones((5, 5), np.uint8)
        window = filter_window or config.POSITION_FILTER_WINDOW
        self._history_x = deque(maxlen=window)
        self._history_y = deque(maxlen=window)
        self._last_accepted: Optional[BallPosition] = None

    def reset(self):
        """Call after detection has been lost for a while, so the next
        accepted position isn't rejected as a spurious 'jump' from stale
        history, and the moving-average filter doesn't drag in old data."""
        self._history_x.clear()
        self._history_y.clear()
        self._last_accepted = None

    def detect(self, frame_rgb) -> Optional[BallPosition]:
        """Returns a smoothed BallPosition, or None if no ball was
        confidently detected in this frame."""
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
                # Likely a spurious detection far from the real ball
                # (reflection, skin tone, etc.) -- reject this frame
                # rather than chase it with a large servo correction.
                return None

        self._history_x.append(cx)
        self._history_y.append(cy)
        smoothed = BallPosition(
            x=sum(self._history_x) / len(self._history_x),
            y=sum(self._history_y) / len(self._history_y),
            radius=radius,
        )
        self._last_accepted = smoothed
        return smoothed
