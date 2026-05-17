"""
vision_tab.py  --  Hand-gesture gripper control for ABB IRB 1600 Digital Twin.

Open hand  ->  open gripper (100 %)
Pinch thumb + index finger  ->  close gripper (0 %)

Signal emitted (connect in MainWindow):
  sig_gripper(float)  gripper target 0-100 %
"""

import numpy as np
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QSlider, QGroupBox, QSizePolicy,
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui  import QPixmap, QFont

from vision_hand_tracker import HandTrackerThread


class VisionTab(QWidget):
    """
    Signal
    ------
    sig_gripper(float)  gripper opening percentage, 0-100
    """
    sig_gripper = pyqtSignal(float)

    def __init__(self, get_current_q, parent=None):
        super().__init__(parent)
        self._get_current_q = get_current_q   # kept for API compatibility
        self._tracker       = None
        self._active        = False
        self._alpha         = 0.50
        self._sm_pinch      = 1.0
        self._build_ui()

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setSpacing(6)

        self._preview = QLabel("Camera inactive")
        self._preview.setAlignment(Qt.AlignCenter)
        self._preview.setMinimumHeight(200)
        self._preview.setStyleSheet(
            "background:#0d1117; border:1px solid #30363d; border-radius:4px;")
        self._preview.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        root.addWidget(self._preview)

        st_row = QHBoxLayout()
        self._status_lbl = QLabel("Hand: not detected")
        self._status_lbl.setFont(QFont("Courier New", 9))
        st_row.addWidget(self._status_lbl)
        st_row.addStretch()
        self._pinch_lbl = QLabel("Gripper: ---")
        self._pinch_lbl.setFont(QFont("Courier New", 9))
        st_row.addWidget(self._pinch_lbl)
        root.addLayout(st_row)

        ctrl_box = QGroupBox("Settings")
        cl = QVBoxLayout(ctrl_box)
        cl.setSpacing(6)

        cam_row = QHBoxLayout()
        cam_row.addWidget(QLabel("Camera:"))
        self._cam_cb = QComboBox()
        self._cam_cb.addItems(["0", "1", "2", "3"])
        self._cam_cb.setMaximumWidth(55)
        cam_row.addWidget(self._cam_cb)
        cam_row.addStretch()
        cl.addLayout(cam_row)

        sm_row = QHBoxLayout()
        sm_row.addWidget(QLabel("Smoothing:"))
        self._smooth_sl = QSlider(Qt.Horizontal)
        self._smooth_sl.setRange(0, 30)
        self._smooth_sl.setValue(15)
        self._smooth_sl.valueChanged.connect(
            lambda v: setattr(self, '_alpha', v / 31.0))
        sm_row.addWidget(self._smooth_sl, 1)
        cl.addLayout(sm_row)

        btn_row = QHBoxLayout()
        self._start_btn = QPushButton("Start Camera")
        self._stop_btn  = QPushButton("Stop Camera")
        self._stop_btn.setEnabled(False)
        self._start_btn.clicked.connect(self._start)
        self._stop_btn.clicked.connect(self._stop)
        btn_row.addWidget(self._start_btn)
        btn_row.addWidget(self._stop_btn)
        cl.addLayout(btn_row)
        root.addWidget(ctrl_box)

        inst_box = QGroupBox("How to use")
        il = QVBoxLayout(inst_box)
        inst = QLabel(
            "Open hand   ->  open gripper\n"
            "Pinch       ->  close gripper\n\n"
            "Smoothing slider: reduce jitter\n"
            "if gripper flickers."
        )
        inst.setFont(QFont("Courier New", 9))
        inst.setStyleSheet("color:#8b949e;")
        il.addWidget(inst)
        root.addWidget(inst_box)
        root.addStretch()

    def _start(self):
        cam_idx = int(self._cam_cb.currentText())
        self._tracker = HandTrackerThread(camera_index=cam_idx, parent=self)
        self._tracker.frame_ready.connect(self._on_frame)
        self._tracker.hand_data.connect(self._on_hand_data)
        self._tracker.error.connect(self._on_error)
        self._tracker.start()
        self._active = True
        self._start_btn.setEnabled(False)
        self._stop_btn.setEnabled(True)
        self._status_lbl.setText("Camera active -- show hand")

    def _stop(self):
        if self._tracker is not None:
            self._tracker.stop()
            self._tracker = None
        self._active = False
        self._start_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        self._preview.setText("Camera inactive")
        self._status_lbl.setText("Hand: not detected")
        self._pinch_lbl.setText("Gripper: ---")

    def _on_frame(self, qimage):
        pix = QPixmap.fromImage(qimage).scaled(
            self._preview.width(), self._preview.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self._preview.setPixmap(pix)

    def _on_hand_data(self, data: dict):
        if not data["detected"]:
            self._status_lbl.setText("Hand: not detected")
            self._pinch_lbl.setText("Gripper: ---")
            return

        a = self._alpha
        self._sm_pinch = a * self._sm_pinch + (1 - a) * data["pinch_ratio"]
        gripper_pct = float(np.clip(self._sm_pinch * 100.0, 0.0, 100.0))

        self._status_lbl.setText("Hand: detected")
        self._pinch_lbl.setText(f"Gripper: {gripper_pct:.0f}%")
        self.sig_gripper.emit(gripper_pct)

    def _on_error(self, msg: str):
        self._status_lbl.setText(f"Error: {msg}")
        self._stop()

    def stop_tracking(self):
        """Cleanly shut down the camera thread on application exit."""
        if self._active:
            self._stop()
