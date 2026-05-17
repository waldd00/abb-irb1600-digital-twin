"""
vision_hand_tracker.py  --  Real-time hand tracking via MediaPipe Tasks API.

mediapipe >= 0.10 removed mp.solutions.hands; this module uses the new
Tasks-based HandLandmarker instead.  The model file (~8 MB) is downloaded
automatically to the same directory on first use.

Dependencies:
    pip install mediapipe opencv-python
"""

import numpy as np
import urllib.request
from pathlib import Path
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui  import QImage

try:
    import cv2
    import mediapipe as mp
    from mediapipe.tasks          import python as _mp_python
    from mediapipe.tasks.python   import vision as _mp_vision
    _AVAILABLE = True
except ImportError:
    _AVAILABLE = False

# Hand landmarker model -- downloaded once to the script directory
_MODEL_URL  = ("https://storage.googleapis.com/mediapipe-models/"
               "hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task")
_MODEL_PATH = Path(__file__).parent / "hand_landmarker.task"

# Hand skeleton connection pairs for manual OpenCV drawing
_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),          # thumb
    (0, 5), (5, 6), (6, 7), (7, 8),           # index
    (0, 9), (9, 10), (10, 11), (11, 12),       # middle
    (0, 13), (13, 14), (14, 15), (15, 16),     # ring
    (0, 17), (17, 18), (18, 19), (19, 20),     # pinky
    (5, 9), (9, 13), (13, 17),                 # palm cross-links
]

# Landmark indices
_WRIST      = 0
_THUMB_TIP  = 4
_INDEX_TIP  = 8
_MIDDLE_MCP = 9   # palm-centre proxy and hand-size reference



def _ensure_model() -> bool:
    """Return True when the model file is available (download if absent)."""
    if _MODEL_PATH.exists():
        return True
    try:
        print(f"  Downloading hand landmarker model -> {_MODEL_PATH.name} ...")
        urllib.request.urlretrieve(_MODEL_URL, _MODEL_PATH)
        print("  Model download complete.")
        return True
    except Exception as exc:
        print(f"  Model download failed: {exc}")
        return False


def _pinch_ratio(landmarks) -> float:
    """
    Normalised pinch openness: 0.0 = closed, 1.0 = fully open.

    Scale-invariant: raw thumb-index distance divided by wrist-to-MCP9 length.
    Empirical bounds: 0.08 (closed) to 0.40 (open).
    """
    thumb = np.array([landmarks[_THUMB_TIP].x,  landmarks[_THUMB_TIP].y])
    index = np.array([landmarks[_INDEX_TIP].x,  landmarks[_INDEX_TIP].y])
    wrist = np.array([landmarks[_WRIST].x,       landmarks[_WRIST].y])
    mid   = np.array([landmarks[_MIDDLE_MCP].x,  landmarks[_MIDDLE_MCP].y])
    hand_size = float(np.linalg.norm(mid - wrist)) + 1e-6
    pinch_d   = float(np.linalg.norm(thumb - index))
    lo, hi    = 0.08, 0.40
    return float(np.clip((pinch_d / hand_size - lo) / (hi - lo), 0.0, 1.0))


def _palm_centre(landmarks):
    """
    Normalised palm centre (x, y) in image space.
    Origin top-left; x increases rightward, y increases downward.
    """
    mid = landmarks[_MIDDLE_MCP]
    return float(mid.x), float(mid.y)


def _draw_hand(frame, landmarks, w: int, h: int):
    """Overlay skeleton and landmark dots on frame (BGR, in-place)."""
    pts = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]
    for a, b in _CONNECTIONS:
        cv2.line(frame, pts[a], pts[b], (60, 200, 60), 2, cv2.LINE_AA)
    for i, (x, y) in enumerate(pts):
        colour = (80, 80, 255) if i in (_THUMB_TIP, _INDEX_TIP) else (255, 255, 255)
        cv2.circle(frame, (x, y), 5, colour, -1, cv2.LINE_AA)



class HandTrackerThread(QThread):
    """
    Background thread: captures camera frames, runs MediaPipe HandLandmarker.

    Signals
    -------
    frame_ready : QImage  annotated frame for display in the Vision tab
    hand_data   : dict    keys: 'detected' (bool), 'pinch_ratio' (float 0-1),
                                'palm_x' (float 0-1), 'palm_y' (float 0-1)
    error       : str     human-readable error message; thread exits after emit
    """
    frame_ready = pyqtSignal(QImage)
    hand_data   = pyqtSignal(dict)
    error       = pyqtSignal(str)

    def __init__(self, camera_index: int = 0, parent=None):
        super().__init__(parent)
        self._camera_index = camera_index
        self._running      = False

    def run(self):
        if not _AVAILABLE:
            self.error.emit(
                "mediapipe or opencv-python is not installed.\n"
                "Run:  pip install mediapipe opencv-python")
            return

        if not _ensure_model():
            self.error.emit(
                "Could not obtain the hand_landmarker.task model file.\n"
                "Check your internet connection, or place the file manually at:\n"
                f"  {_MODEL_PATH}")
            return

        cap = cv2.VideoCapture(self._camera_index)
        if not cap.isOpened():
            self.error.emit(f"Cannot open camera index {self._camera_index}.")
            return

        base_options = _mp_python.BaseOptions(model_asset_path=str(_MODEL_PATH))
        options = _mp_vision.HandLandmarkerOptions(
            base_options=base_options,
            num_hands=1,
            min_hand_detection_confidence=0.60,
            min_hand_presence_confidence=0.50,
            min_tracking_confidence=0.50,
        )
        detector = _mp_vision.HandLandmarker.create_from_options(options)

        self._running = True
        while self._running:
            ok, frame = cap.read()
            if not ok:
                continue

            # Mirror so the display behaves like a mirror
            frame     = cv2.flip(frame, 1)
            h, w      = frame.shape[:2]

            # MediaPipe Tasks API requires SRGB numpy array
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image  = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
            result    = detector.detect(mp_image)

            data = {"detected": False, "pinch_ratio": 1.0,
                    "palm_x": 0.5, "palm_y": 0.5}

            if result.hand_landmarks:
                lm = result.hand_landmarks[0]
                _draw_hand(frame, lm, w, h)
                data["detected"]    = True
                data["pinch_ratio"] = _pinch_ratio(lm)
                px, py              = _palm_centre(lm)
                data["palm_x"]      = px
                data["palm_y"]      = py

            # Convert annotated frame (BGR) to QImage (RGB)
            out_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            qh, qw, ch = out_rgb.shape
            qimg = QImage(out_rgb.data, qw, qh, ch * qw, QImage.Format_RGB888)
            self.frame_ready.emit(qimg.copy())
            self.hand_data.emit(data)

        detector.close()
        cap.release()

    def stop(self):
        """Signal the capture loop to exit and block until the thread finishes."""
        self._running = False
        self.wait()
