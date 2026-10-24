"""
camera.py
=========
Thin wrapper around Picamera2 so the rest of the codebase doesn't need to
know about camera configuration details, and so the camera can be swapped
for a canned frame sequence during testing/debugging without a Pi camera
attached (Section 15's "debug mode ... without necessarily moving the
servos" applies equally to not needing a live camera).
"""

import config


class Camera:
    """Live Picamera2 source. Import of picamera2 is deferred to start()
    so this module can be imported (and Camera itself referenced) on a
    dev machine that doesn't have picamera2 installed."""

    def __init__(self, resolution=None, framerate=None):
        self.resolution = resolution or config.CAMERA_RESOLUTION
        self.framerate = framerate or config.CAMERA_FRAMERATE
        self._picam2 = None

    def start(self):
        from picamera2 import Picamera2  # Raspberry Pi only

        self._picam2 = Picamera2()
        cfg = self._picam2.create_video_configuration(
            main={"size": self.resolution, "format": "RGB888"},
            controls={"FrameRate": self.framerate},
        )
        self._picam2.configure(cfg)
        self._picam2.start()

    def read_frame(self):
        """Returns an HxWx3 array in RGB order, or raises if start()
        hasn't been called."""
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


class DummyCamera:
    """Camera stand-in for unit tests / offline debugging: cycles through
    a supplied sequence of frames (e.g. loaded from disk with cv2.imread)
    instead of doing live capture. Matches the Camera interface so
    main.py doesn't need to know which one it has."""

    def __init__(self, frames):
        self._frames = list(frames)
        self._i = 0

    def start(self):
        pass

    def read_frame(self):
        if not self._frames:
            return None
        frame = self._frames[self._i % len(self._frames)]
        self._i += 1
        return frame

    def stop(self):
        pass

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False
