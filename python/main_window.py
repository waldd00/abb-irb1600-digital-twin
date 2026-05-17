"""
main_window.py  --  ABB IRB 1600 Digital Twin  --  Main UI controller

== GRIPPER INITIALISATION ORDER (critical) ==
  1. _build_ui()          -- create checkbox (unchecked)
  2. gripper auto-enable  -- set checkbox True   <-- FIRST
  3. _run_fk_snap()       -- run FK + update_gripper  <-- THEN

== DEAD-BAND BYPASS ==
  _tick() updates the gripper even when the robot is stationary,
  using the cached _last_tfs to avoid an extra MATLAB call.

== GRIPPER SLIDER ==
  On slider change, update_gripper is called immediately
  using the cached T_ee transform.
"""

import json
import re
import time
import csv
import io
import numpy as np
from pathlib import Path
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QSlider, QLabel, QPushButton, QGroupBox,
    QDoubleSpinBox, QFrame, QDialog, QFormLayout,
    QDialogButtonBox, QTabWidget, QListWidget,
    QListWidgetItem, QCheckBox, QFileDialog, QComboBox,
    QMessageBox, QAction, QPlainTextEdit, QScrollArea,
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui  import QFont, QColor, QSyntaxHighlighter, QTextCharFormat

from vision_tab  import VisionTab
from ui_widgets  import GripperBar, WorkspaceMap, LineNumberedEdit


# ── Program editor syntax highlighter ─────────────────────────────────────────
class _ProgramHighlighter(QSyntaxHighlighter):
    def __init__(self, document):
        super().__init__(document)

        def _fmt(color, bold=False):
            f = QTextCharFormat()
            f.setForeground(QColor(color))
            if bold:
                f.setFontWeight(QFont.Bold)
            return f

        self._rules = [
            (re.compile(r'#.*'),                            _fmt("#6a9955")),
            (re.compile(r'\b(HOME|MOVEJ|GRIPPER|WAIT)\b'), _fmt("#569cd6", bold=True)),
            (re.compile(r'\bspeed='),                       _fmt("#ce9178")),
            (re.compile(r'[-+]?\d+\.?\d*'),                 _fmt("#b5cea8")),
        ]

    def highlightBlock(self, text: str):
        for pattern, fmt in self._rules:
            for m in pattern.finditer(text):
                self.setFormat(m.start(), m.end() - m.start(), fmt)


def _rpy_from_matrix(R: np.ndarray):
    """Extract roll-pitch-yaw (ZYX convention) in degrees from a 3x3 rotation matrix."""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll  = np.degrees(np.arctan2( R[2, 1], R[2, 2]))
        pitch = np.degrees(np.arctan2(-R[2, 0], sy))
        yaw   = np.degrees(np.arctan2( R[1, 0], R[0, 0]))
    else:
        roll  = np.degrees(np.arctan2(-R[1, 2], R[1, 1]))
        pitch = np.degrees(np.arctan2(-R[2, 0], sy))
        yaw   = 0.0
    return roll, pitch, yaw

JOINT_LIMITS_DEG = [
    (-180, 180), (-63, 110), (-236, 60),
    (-200, 200), (-115, 115), (-400, 400),
]
HOME_Q_DEG           = np.array([0., 90., 0., 0., 30., 0.])
ANIM_FPS             = 30
ANIM_MS              = int(1000 / ANIM_FPS)
DEAD_BAND            = 0.05    # deg -- minimum joint delta to trigger motion
TRAJ_ARRIVE          = 0.3     # deg -- arrival threshold for waypoint stepping
BASE_JOINT_SPEED_DPS = 75.0    # deg/s at speed multiplier = 1.0


class MainWindow(QMainWindow):

    def __init__(self, matlab_bridge, visualizer):
        super().__init__()
        self.bridge = matlab_bridge
        self.viz    = visualizer

        self.setWindowTitle("ABB IRB 1600 - Digital Twin")
        self.setMinimumSize(1500, 850)

        self.current_q_deg = HOME_Q_DEG.copy()
        self.target_q_deg  = HOME_Q_DEG.copy()

        # Trajectory state
        self.waypoints        = []
        self._traj_play       = False
        self._traj_idx        = 0
        self._traj_loop       = False
        self._traj_speed      = 0.3
        self._motion_active   = False
        self._motion_elapsed  = 0.0
        self._motion_duration = 0.0
        self._motion_start_q  = self.current_q_deg.copy()
        self._motion_target_q = self.target_q_deg.copy()

        # Motion profile
        self._motion_profile_type = 'poly5'  # 'poly5' | 'trap' | 'scurve'

        # TCP velocity tracking
        self._prev_tcp_pos = None

        # Gripper state
        self._grip_pct = 100.0
        self._last_tfs = None   # FK transform cache -- required for gripper dead-band bypass

        # Cycle time
        self._cycle_start: float | None = None

        # Motion log
        self._motion_log: list[dict] = []

        # IK drag pick mode
        self._ik_drag_enabled = False

        # Active joint for keyboard nudge (0-5)
        self._active_joint = 0

        # Program executor state
        self._prog_cmds:    list[dict] = []
        self._prog_idx:     int = 0
        self._prog_running: bool = False

        # 1. Build UI
        self._build_ui()
        self._start_loop()
        self._sync_sliders()

        # 2. Auto-enable gripper before FK snap
        if self.viz.gripper_loaded():
            self.chk_gripper.blockSignals(True)
            self.chk_gripper.setChecked(True)
            self.chk_gripper.blockSignals(False)
            self.grip_slider.setEnabled(True)
            self.viz.set_gripper_visible(True)
            self._update_io()
            self._update_gripper_model_label()

        # 3. FK snap -- gripper is now active
        self._run_fk_snap()

    def _build_ui(self):
        # File menu
        menu = self.menuBar()
        file_menu = menu.addMenu("File")
        act_save = QAction("Save Configuration...", self)
        act_save.setShortcut("Ctrl+S")
        act_save.triggered.connect(self._save_config)
        act_load = QAction("Load Configuration...", self)
        act_load.setShortcut("Ctrl+O")
        act_load.triggered.connect(self._load_config)
        act_export = QAction("Export Motion Log (CSV)...", self)
        act_export.triggered.connect(self._export_log_csv)
        file_menu.addAction(act_save)
        file_menu.addAction(act_load)
        file_menu.addSeparator()
        file_menu.addAction(act_export)

        central = QWidget()
        root    = QHBoxLayout(central)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)
        self.setCentralWidget(central)
        root.addWidget(self.viz.get_qt_widget(), stretch=4)
        root.addWidget(self._build_panel(),       stretch=1)

        sb = self.statusBar()
        sb.setFont(QFont("Courier New", 8))
        sb.setStyleSheet(
            "QStatusBar { background:#0d1117; border-top:1px solid #30363d; }"
            "QStatusBar::item { border:none; }"
        )
        self._sb_tcp = QLabel("TCP: --")
        self._sb_tcp.setFont(QFont("Courier New", 8))
        self._sb_tcp.setStyleSheet("color:#8b949e; padding:0 8px;")
        self._sb_mode = QLabel("Idle")
        self._sb_mode.setFont(QFont("Courier New", 8))
        self._sb_mode.setStyleSheet("color:#8b949e; padding:0 8px;")
        sb.addWidget(self._sb_tcp)
        sb.addPermanentWidget(self._sb_mode)

    def _build_panel(self):
        panel = QWidget(); panel.setMaximumWidth(400)
        vbox  = QVBoxLayout(panel); vbox.setSpacing(8)
        self.tabs = QTabWidget()
        self.tabs.setFont(QFont("Segoe UI", 9))
        self.tabs.addTab(self._build_joints_tab(),     "Joints")
        self.tabs.addTab(self._build_trajectory_tab(), "Trajectory")
        self.tabs.addTab(self._build_gripper_tab(),    "Gripper")
        self._vision_tab = VisionTab(get_current_q=lambda: self.current_q_deg)
        self._vision_tab.sig_gripper.connect(self._on_vision_gripper)
        self.tabs.addTab(self._vision_tab,             "Vision")
        self.tabs.addTab(self._build_log_tab(),        "Log")
        self.tabs.addTab(self._build_program_tab(),    "Program")
        vbox.addWidget(self.tabs)
        return panel

    def _build_joints_tab(self):
        w = QWidget(); vbox = QVBoxLayout(w); vbox.setSpacing(8)

        jbox = QGroupBox("Joint Angles")
        jbox.setFont(QFont("Segoe UI", 9, QFont.Bold))
        jl   = QVBoxLayout(jbox); jl.setSpacing(4)

        self.sliders = []; self.val_labels = []; self._joint_name_labels = []
        for i, (lo, hi) in enumerate(JOINT_LIMITS_DEG):
            row = QHBoxLayout()
            lbl = QLabel(f"J{i+1}"); lbl.setMinimumWidth(22)
            lbl.setFont(QFont("Segoe UI", 9, QFont.Bold))
            lbl.setToolTip(f"Joint {i+1}  |  Press key {i+1} to select, then ↑↓ to nudge")
            sl  = QSlider(Qt.Horizontal)
            sl.setRange(int(lo*10), int(hi*10)); sl.setValue(0)
            sl.setToolTip(f"J{i+1} range: {lo}° to {hi}°")
            sl.valueChanged.connect(lambda v, idx=i: self._on_slider(v, idx))
            vl  = QLabel("  0.0 deg"); vl.setMinimumWidth(56)
            vl.setFont(QFont("Courier New", 9))
            row.addWidget(lbl); row.addWidget(sl, stretch=1); row.addWidget(vl)
            jl.addLayout(row)
            self.sliders.append(sl); self.val_labels.append(vl)
            self._joint_name_labels.append(lbl)
        vbox.addWidget(jbox)

        self._joint_name_labels[0].setStyleSheet("color:#ff7700;")

        for text, slot, tip in [
            ("Run FK (snap)",      self._run_fk_snap,
             "Snap robot instantly to the current slider positions (no interpolation)"),
            ("Run IK — enter XYZ", self._open_ik_dialog,
             "Compute joint angles from a target TCP position (X Y Z in mm)"),
            ("Reset All Joints",   self._reset_joints,
             "Return all joints to the home configuration  [0 90 0 0 30 0]"),
            ("Clear Trail",        self.viz.clear_trail,
             "Erase the TCP path trail from the 3D viewport"),
        ]:
            btn = QPushButton(text); btn.setMinimumHeight(30)
            btn.setToolTip(tip)
            btn.clicked.connect(slot); vbox.addWidget(btn)

        self.btn_ik_drag = QPushButton("IK Drag  —  click point in 3D")
        self.btn_ik_drag.setMinimumHeight(30)
        self.btn_ik_drag.setCheckable(True)
        self.btn_ik_drag.setToolTip(
            "Toggle click-to-move mode: click any surface in the 3D view\n"
            "and the robot will move its TCP to that point via IK")
        self.btn_ik_drag.toggled.connect(self._toggle_ik_drag)
        vbox.addWidget(self.btn_ik_drag)

        nudge_hint = QLabel("Keys 1–6 select joint   ↑ ↓ nudge ±1°   Shift+↑↓ nudge ±5°")
        nudge_hint.setFont(QFont("Segoe UI", 7))
        nudge_hint.setStyleSheet("color:#555e6b; padding:2px 0;")
        vbox.addWidget(nudge_hint)

        self._workspace_map = WorkspaceMap()
        vbox.addWidget(self._workspace_map, alignment=Qt.AlignHCenter)

        self.chk_frames = QCheckBox("Show joint frames (RGB axes)")
        self.chk_frames.setFont(QFont("Segoe UI", 9))
        self.chk_frames.stateChanged.connect(
            lambda s: self.viz.toggle_frames(s == Qt.Checked))
        vbox.addWidget(self.chk_frames)

        self.chk_workspace = QCheckBox("Show workspace envelope")
        self.chk_workspace.setFont(QFont("Segoe UI", 9))
        self.chk_workspace.stateChanged.connect(
            lambda s: self.viz.set_workspace_visible(s == Qt.Checked))
        vbox.addWidget(self.chk_workspace)

        prof_row = QHBoxLayout()
        prof_row.addWidget(QLabel("Motion:"))
        self.prof_combo = QComboBox()
        self.prof_combo.setFont(QFont("Segoe UI", 9))
        self.prof_combo.addItems(["Smooth (Poly-5)", "Trapezoidal", "S-curve"])
        self.prof_combo.currentIndexChanged.connect(self._on_profile_change)
        prof_row.addWidget(self.prof_combo, stretch=1)
        vbox.addLayout(prof_row)

        cam_row = QHBoxLayout()
        cam_row.addWidget(QLabel("View:"))
        for label, preset in [
            ("ISO",   [(3000, -2500, 2000), (400, 0, 800), (0, 0, 1)]),
            ("Front", [(400,  -3500,  800), (400, 0, 800), (0, 0, 1)]),
            ("Side",  [(4000,     0,  800), (400, 0, 800), (0, 0, 1)]),
            ("Top",   [(400,      0, 5000), (400, 0, 800), (1, 0, 0)]),
        ]:
            b = QPushButton(label); b.setMinimumHeight(24)
            b.setFont(QFont("Segoe UI", 8))
            b.clicked.connect(lambda _, p=preset: self._set_camera(p))
            cam_row.addWidget(b)
        vbox.addLayout(cam_row)

        sep = QFrame(); sep.setFrameShape(QFrame.HLine); vbox.addWidget(sep)

        self.out_lbl = QLabel(
            "<pre>Position\n  X: -\n  Y: -\n  Z: -\n\ndet(J): -\ncond(J): -</pre>")
        self.out_lbl.setFont(QFont("Courier New", 9))
        self.out_lbl.setWordWrap(True)
        self.out_lbl.setTextFormat(Qt.RichText)
        vbox.addWidget(self.out_lbl)

        self.vel_lbl = QLabel("TCP speed:   0.0 mm/s")
        self.vel_lbl.setFont(QFont("Courier New", 9))
        self.vel_lbl.setStyleSheet("color:#58a6ff;")
        vbox.addWidget(self.vel_lbl)

        self.sing_lbl = QLabel("")
        self.sing_lbl.setFont(QFont("Segoe UI", 10, QFont.Bold))
        self.sing_lbl.setStyleSheet("color:#FF4444;")
        vbox.addWidget(self.sing_lbl)

        self.safety_lbl = QLabel("Safety: OK")
        self.safety_lbl.setFont(QFont("Courier New", 9))
        self.safety_lbl.setWordWrap(True)
        self.safety_lbl.setStyleSheet("color:#00aa66;")
        vbox.addWidget(self.safety_lbl)
        vbox.addStretch()
        return w

    def _build_trajectory_tab(self):
        w = QWidget(); vbox = QVBoxLayout(w); vbox.setSpacing(8)

        wpbox = QGroupBox("Waypoints")
        wpbox.setFont(QFont("Segoe UI", 9, QFont.Bold))
        wl = QVBoxLayout(wpbox)
        self.wp_list = QListWidget()
        self.wp_list.setFont(QFont("Courier New", 8))
        self.wp_list.setMaximumHeight(180)
        wl.addWidget(self.wp_list)
        row1 = QHBoxLayout()
        for txt, fn, tip in [
                ("Add",    self._wp_add,    "Save current joint angles as a new waypoint"),
                ("Remove", self._wp_remove, "Delete the selected waypoint"),
                ("Clear",  self._wp_clear,  "Delete all waypoints and stop playback")]:
            b = QPushButton(txt); b.setMinimumHeight(28)
            b.setToolTip(tip)
            b.clicked.connect(fn); row1.addWidget(b)
        wl.addLayout(row1)
        vbox.addWidget(wpbox)

        ctrl = QGroupBox("Playback")
        ctrl.setFont(QFont("Segoe UI", 9, QFont.Bold))
        cl   = QVBoxLayout(ctrl)

        spd_row = QHBoxLayout()
        spd_row.addWidget(QLabel("Speed:"))
        self.spd_sl = QSlider(Qt.Horizontal)
        self.spd_sl.setRange(1, 10); self.spd_sl.setValue(3)
        self.spd_sl.valueChanged.connect(lambda v: self._set_traj_speed(v))
        self.spd_lbl = QLabel("0.3x"); self.spd_lbl.setMinimumWidth(36)
        spd_row.addWidget(self.spd_sl, stretch=1); spd_row.addWidget(self.spd_lbl)
        cl.addLayout(spd_row)

        self.chk_loop = QCheckBox("Loop trajectory")
        self.chk_loop.setFont(QFont("Segoe UI", 9))
        self.chk_loop.stateChanged.connect(
            lambda s: setattr(self, '_traj_loop', s == Qt.Checked))
        cl.addWidget(self.chk_loop)

        play_row = QHBoxLayout()
        self.btn_play_traj = QPushButton("▶  Play Trajectory")
        self.btn_play_traj.setToolTip("Start playback through all waypoints in order")
        self.btn_stop_traj = QPushButton("■  Stop")
        for b in (self.btn_play_traj, self.btn_stop_traj):
            b.setMinimumHeight(32)
        self.btn_play_traj.setStyleSheet("background:#1D9E75;color:white;font-weight:bold;")
        self.btn_stop_traj.setStyleSheet("background:#9E1D1D;color:white;font-weight:bold;")
        self.btn_play_traj.clicked.connect(self._traj_start)
        self.btn_stop_traj.clicked.connect(self._traj_stop)
        play_row.addWidget(self.btn_play_traj)
        play_row.addWidget(self.btn_stop_traj)
        cl.addLayout(play_row)

        btn_goto = QPushButton("Go to selected")
        btn_goto.setMinimumHeight(28)
        btn_goto.clicked.connect(self._wp_goto_selected)
        cl.addWidget(btn_goto)

        prev_row = QHBoxLayout()
        btn_prev = QPushButton("Preview Path")
        btn_prev.setMinimumHeight(28)
        btn_prev.setToolTip("Draw the planned TCP path through all waypoints in the 3D view")
        btn_prev.clicked.connect(self._preview_trajectory)
        btn_clr_prev = QPushButton("Clear Preview")
        btn_clr_prev.setMinimumHeight(28)
        btn_clr_prev.setToolTip("Remove the preview path from the 3D view")
        btn_clr_prev.clicked.connect(self.viz.clear_trajectory_preview)
        prev_row.addWidget(btn_prev)
        prev_row.addWidget(btn_clr_prev)
        cl.addLayout(prev_row)
        vbox.addWidget(ctrl)

        self.cycle_lbl = QLabel("Cycle time: --")
        self.cycle_lbl.setFont(QFont("Courier New", 9))
        self.cycle_lbl.setStyleSheet("color:#8b949e;")
        vbox.addWidget(self.cycle_lbl)

        self.traj_lbl = QLabel("Ready")
        self.traj_lbl.setFont(QFont("Courier New", 9))
        self.traj_lbl.setAlignment(Qt.AlignCenter)
        self.traj_lbl.setStyleSheet(
            "background:#0d1117;color:#58a6ff;padding:4px;border-radius:4px;")
        vbox.addWidget(self.traj_lbl)
        vbox.addStretch()
        return w

    def _build_gripper_tab(self):
        w = QWidget(); vbox = QVBoxLayout(w); vbox.setSpacing(8)

        mbox = QGroupBox("Gripper Model")
        mbox.setFont(QFont("Segoe UI", 9, QFont.Bold))
        ml   = QVBoxLayout(mbox)

        self.chk_gripper = QCheckBox("Enable gripper")
        self.chk_gripper.setFont(QFont("Segoe UI", 9))
        self.chk_gripper.stateChanged.connect(self._on_grip_enable)
        ml.addWidget(self.chk_gripper)

        self.grip_model_lbl = QLabel("Model: -")
        self.grip_model_lbl.setFont(QFont("Courier New", 8))
        self.grip_model_lbl.setStyleSheet("color:#aaa;")
        ml.addWidget(self.grip_model_lbl)

        btn_ov = QPushButton("Load different STL...")
        btn_ov.setMinimumHeight(26)
        btn_ov.clicked.connect(self._load_gripper_override)
        ml.addWidget(btn_ov)
        vbox.addWidget(mbox)

        cbox = QGroupBox("Opening Control")
        cbox.setFont(QFont("Segoe UI", 9, QFont.Bold))
        cb   = QVBoxLayout(cbox)

        open_row = QHBoxLayout()
        open_row.addWidget(QLabel("Open %"))
        self.grip_slider = QSlider(Qt.Horizontal)
        self.grip_slider.setRange(0, 100); self.grip_slider.setValue(100)
        self.grip_slider.setEnabled(False)
        self.grip_slider.valueChanged.connect(self._on_grip_slider)
        self.grip_val_lbl = QLabel("100 %"); self.grip_val_lbl.setMinimumWidth(42)
        self.grip_val_lbl.setFont(QFont("Courier New", 9))
        open_row.addWidget(self.grip_slider, stretch=1)
        open_row.addWidget(self.grip_val_lbl)
        cb.addLayout(open_row)

        self._gripper_bar = GripperBar()
        cb.addWidget(self._gripper_bar)

        btn_row = QHBoxLayout()
        for txt, pct in [("Open", 100), ("Mid", 50), ("Close", 0)]:
            b = QPushButton(txt); b.setMinimumHeight(28)
            b.clicked.connect(lambda _, p=pct: self._set_grip(p))
            btn_row.addWidget(b)
        cb.addLayout(btn_row)
        vbox.addWidget(cbox)

        iobox = QGroupBox("Digital I/O (simulation)")
        iobox.setFont(QFont("Segoe UI", 9, QFont.Bold))
        il    = QVBoxLayout(iobox)
        self.io_leds = {}
        for sig, desc in [
            ("DO1", "Gripper CLOSE cmd"),
            ("DO2", "Gripper OPEN cmd"),
            ("DI1", "Gripper CLOSED (sensor)"),
            ("DI2", "Gripper OPENED (sensor)"),
        ]:
            row = QHBoxLayout()
            n   = QLabel(sig); n.setFont(QFont("Courier New", 9, QFont.Bold))
            n.setMinimumWidth(36)
            led = QLabel("●"); led.setFont(QFont("Segoe UI", 14))
            led.setStyleSheet("color:#333;"); led.setMinimumWidth(20)
            d   = QLabel(desc); d.setFont(QFont("Segoe UI", 8))
            d.setStyleSheet("color:#888;")
            row.addWidget(n); row.addWidget(led); row.addWidget(d); row.addStretch()
            il.addLayout(row)
            self.io_leds[sig] = led
        vbox.addWidget(iobox)

        self.grip_status_lbl = QLabel("Gripper: disabled")
        self.grip_status_lbl.setFont(QFont("Courier New", 9))
        self.grip_status_lbl.setAlignment(Qt.AlignCenter)
        self.grip_status_lbl.setStyleSheet(
            "background:#0d1117;color:#58a6ff;padding:4px;border-radius:4px;")
        vbox.addWidget(self.grip_status_lbl)

        self.object_status_lbl = QLabel("Object: ready")
        self.object_status_lbl.setFont(QFont("Courier New", 9))
        self.object_status_lbl.setAlignment(Qt.AlignCenter)
        self.object_status_lbl.setStyleSheet(
            "background:#0d1117;color:#c9d1d9;padding:4px;border-radius:4px;")
        vbox.addWidget(self.object_status_lbl)

        pp_box = QGroupBox("Pick-and-Place Setup")
        pp_box.setFont(QFont("Segoe UI", 9, QFont.Bold))
        pp_l = QVBoxLayout(pp_box)

        pick_lbl = QLabel("Pick Object Position")
        pick_lbl.setFont(QFont("Segoe UI", 8, QFont.Bold))
        pick_lbl.setStyleSheet("color:#d8b15a;")
        pp_l.addWidget(pick_lbl)

        for axis, attr, default in [
            ("X", "_obj_x_spin", 1200.0),
            ("Y", "_obj_y_spin",    0.0),
            ("Z", "_obj_z_spin", 1265.0),
        ]:
            row = QHBoxLayout()
            row.addWidget(QLabel(f"  {axis} (mm)"))
            sp = QDoubleSpinBox()
            sp.setRange(-2000, 2000); sp.setValue(default)
            sp.setSuffix(" mm"); sp.setDecimals(0)
            setattr(self, attr, sp)
            row.addWidget(sp, stretch=1)
            pp_l.addLayout(row)

        show_hide_row = QHBoxLayout()
        btn_show_obj = QPushButton("Show Object Here")
        btn_show_obj.setMinimumHeight(28)
        btn_show_obj.setToolTip("Place the yellow cube at these coordinates and show it in the scene")
        btn_show_obj.clicked.connect(self._show_object)
        btn_hide_obj = QPushButton("Hide Object")
        btn_hide_obj.setMinimumHeight(28)
        btn_hide_obj.setToolTip("Remove the pick object from the 3D view")
        btn_hide_obj.clicked.connect(self._hide_object)
        show_hide_row.addWidget(btn_show_obj)
        show_hide_row.addWidget(btn_hide_obj)
        pp_l.addLayout(show_hide_row)

        sep_pp = QFrame(); sep_pp.setFrameShape(QFrame.HLine)
        pp_l.addWidget(sep_pp)

        place_lbl = QLabel("Place / Drop Zone")
        place_lbl.setFont(QFont("Segoe UI", 8, QFont.Bold))
        place_lbl.setStyleSheet("color:#58a6ff;")
        pp_l.addWidget(place_lbl)

        for axis, attr, default in [
            ("X", "_place_x_spin",  800.0),
            ("Y", "_place_y_spin", -500.0),
            ("Z", "_place_z_spin",  900.0),
        ]:
            row = QHBoxLayout()
            row.addWidget(QLabel(f"  {axis} (mm)"))
            sp = QDoubleSpinBox()
            sp.setRange(-2000, 2000); sp.setValue(default)
            sp.setSuffix(" mm"); sp.setDecimals(0)
            setattr(self, attr, sp)
            row.addWidget(sp, stretch=1)
            pp_l.addLayout(row)

        sep_pp2 = QFrame(); sep_pp2.setFrameShape(QFrame.HLine)
        pp_l.addWidget(sep_pp2)

        btn_demo = QPushButton("⚙  Generate Pick-and-Place Program")
        btn_demo.setMinimumHeight(32)
        btn_demo.setStyleSheet("background:#3a1f6e;color:white;font-weight:bold;")
        btn_demo.setToolTip(
            "Compute IK for pick and place positions\n"
            "and insert a ready-to-run program in the Program tab")
        btn_demo.clicked.connect(self._generate_pick_place)
        pp_l.addWidget(btn_demo)

        vbox.addWidget(pp_box)
        vbox.addStretch()
        return w

    def _start_loop(self):
        self.timer = QTimer(self)
        self.timer.setInterval(ANIM_MS)
        self.timer.timeout.connect(self._tick)
        self.timer.start()

    def _tick(self):
        moved = self._advance_motion()

        # Status bar mode indicator
        if self._prog_running:
            self._sb_mode.setText("Program")
            self._sb_mode.setStyleSheet("color:#ce9178; padding:0 8px;")
        elif self._traj_play:
            self._sb_mode.setText("Trajectory")
            self._sb_mode.setStyleSheet("color:#1D9E75; padding:0 8px;")
        elif moved:
            self._sb_mode.setText("Moving")
            self._sb_mode.setStyleSheet("color:#58a6ff; padding:0 8px;")
        else:
            self._sb_mode.setText("Idle")
            self._sb_mode.setStyleSheet("color:#8b949e; padding:0 8px;")

        if moved:
            # Robot moving: recompute FK and update all mesh actors
            q_rad = np.deg2rad(self.current_q_deg)
            tfs   = self.bridge.get_all_transforms(q_rad)
            self._last_tfs = tfs   # update FK cache

            self.viz.update_transforms(tfs, render=False)
            self._update_out(tfs[-1], q_rad)

            # TCP velocity
            tcp_pos = self.viz.tcp_position(tfs[-1])
            if self._prev_tcp_pos is not None:
                vel = float(np.linalg.norm(tcp_pos - self._prev_tcp_pos)) / (ANIM_MS / 1000.0)
                self.vel_lbl.setText(f"TCP speed: {vel:6.1f} mm/s")
            self._prev_tcp_pos = tcp_pos.copy()

            if self.chk_gripper.isChecked() and self.viz.gripper_loaded():
                self.viz.update_gripper(self._grip_pct, tfs[-1])
            self._update_object_sim(tfs[-1])
            self.viz.add_ee_point(tcp_pos)
            self.viz.plotter.render()

        elif self.chk_gripper.isChecked() and self.viz.gripper_loaded() \
                and self._last_tfs is not None:
            # Robot stationary: update gripper from cached transforms
            self._prev_tcp_pos = None
            self.vel_lbl.setText("TCP speed:   0.0 mm/s")
            self.viz.update_gripper(self._grip_pct, self._last_tfs[-1])
            self._update_object_sim(self._last_tfs[-1])
            self.viz.plotter.render()

        else:
            if self._prev_tcp_pos is not None:
                self._prev_tcp_pos = None
                self.vel_lbl.setText("TCP speed:   0.0 mm/s")

    def _start_joint_motion(self):
        self._motion_start_q  = self.current_q_deg.copy()
        self._motion_target_q = self.target_q_deg.copy()
        max_delta = float(np.max(np.abs(self._motion_target_q - self._motion_start_q)))
        speed = BASE_JOINT_SPEED_DPS * (0.5 + self._traj_speed)
        self._motion_duration = max(0.25, max_delta / speed)
        self._motion_elapsed  = 0.0
        self._motion_active   = max_delta >= DEAD_BAND

    def _advance_motion(self) -> bool:
        if not self._motion_active:
            return False
        self._motion_elapsed += ANIM_MS / 1000.0
        u = min(1.0, self._motion_elapsed / self._motion_duration)
        s = self._apply_profile(u)
        self.current_q_deg = (
            self._motion_start_q
            + s * (self._motion_target_q - self._motion_start_q)
        )
        # Live progress label during trajectory playback
        if self._traj_play and u < 1.0:
            remaining = float(np.max(np.abs(self._motion_target_q - self.current_q_deg)))
            self.traj_lbl.setText(
                f"→ WP{self._traj_idx+1:02d}/{len(self.waypoints):02d}"
                f"   Δ {remaining:.0f}°   {int(u * 100)}%")
        if u >= 1.0:
            self.current_q_deg = self._motion_target_q.copy()
            self._motion_active = False
            self._log_motion(self._motion_start_q, self._motion_target_q,
                             self._motion_elapsed)
            if self._traj_play:
                self._traj_step()
            elif self._prog_running:
                self._prog_step()
        return True

    def _apply_profile(self, u: float) -> float:
        if self._motion_profile_type == 'trap':
            beta  = 0.25
            v_max = 1.0 / (1.0 - beta)   # normalise so the area under the velocity curve equals 1
            k     = v_max / (2.0 * beta)
            if u <= beta:
                return k * u ** 2
            elif u <= 1.0 - beta:
                return v_max * (u - beta / 2.0)
            else:
                t = 1.0 - u
                return 1.0 - k * t ** 2
        elif self._motion_profile_type == 'scurve':
            return u ** 4 * (35 - 84 * u + 70 * u ** 2 - 20 * u ** 3)
        else:  # poly5 -- 5th-order smoothstep
            return u ** 3 * (10.0 + u * (-15.0 + 6.0 * u))

    def _on_profile_change(self, idx):
        self._motion_profile_type = ('poly5', 'trap', 'scurve')[idx]

    def _set_camera(self, preset):
        self.viz.plotter.camera_position = preset
        self.viz.plotter.render()

    def _sync_sliders(self):
        for i, (sl, lb) in enumerate(zip(self.sliders, self.val_labels)):
            sl.blockSignals(True)
            sl.setValue(int(np.clip(
                self.target_q_deg[i] * 10, sl.minimum(), sl.maximum())))
            sl.blockSignals(False)
            lb.setText(f"{self.target_q_deg[i]:+6.1f} deg")

    def _on_slider(self, raw, idx):
        deg = raw / 10.
        self.target_q_deg[idx] = deg
        self.val_labels[idx].setText(f"{deg:+6.1f} deg")
        if self._traj_play:
            self._traj_stop()
        self._start_joint_motion()

    def _run_fk_snap(self):
        """Snap the robot and gripper to the current target without interpolation."""
        self._motion_active = False
        self.current_q_deg  = self.target_q_deg.copy()
        q_rad = np.deg2rad(self.current_q_deg)
        tfs   = self.bridge.get_all_transforms(q_rad)
        self._last_tfs = tfs   # update FK cache

        self.viz.update_transforms(tfs, render=False)
        self._update_out(tfs[-1], q_rad)

        # Reposition gripper regardless of checkbox state -- only if loaded
        if self.viz.gripper_loaded():
            self.viz.update_gripper(self._grip_pct, tfs[-1])
            self._update_object_sim(tfs[-1])
        self.viz.plotter.render()

    def _reset_joints(self):
        self._traj_stop()
        self.target_q_deg  = HOME_Q_DEG.copy()
        self.current_q_deg = HOME_Q_DEG.copy()
        self._sync_sliders()
        self._run_fk_snap()

    def _open_ik_dialog(self):
        dlg  = QDialog(self); dlg.setWindowTitle("IK - Enter Target Position")
        form = QFormLayout(dlg); inputs = {}
        for label, default in [("X (mm)", 700.), ("Y (mm)", 0.), ("Z (mm)", 800.)]:
            sp = QDoubleSpinBox(); sp.setRange(-2000, 2000)
            sp.setValue(default); sp.setSuffix(" mm"); sp.setDecimals(1)
            form.addRow(label, sp); inputs[label] = sp
        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(dlg.accept); btns.rejected.connect(dlg.reject)
        form.addRow(btns)
        if dlg.exec_() == QDialog.Accepted:
            x = inputs["X (mm)"].value()
            y = inputs["Y (mm)"].value()
            z = inputs["Z (mm)"].value()
            T     = self.bridge.forward_kinematics(np.deg2rad(self.current_q_deg))
            T_tcp = self.viz.tcp_transform(T)
            T_tcp[:3, 3] = [x, y, z]
            T_ee_target  = self.viz.ee_from_tcp_target(T_tcp)
            try:
                q_sol = self.bridge.inverse_kinematics(
                    T_ee_target, np.deg2rad(self.current_q_deg))
            except Exception as e:
                QMessageBox.critical(self, "IK Error", f"MATLAB error:\n{e}")
                return
            if not np.all(np.isfinite(q_sol)):
                QMessageBox.warning(self, "IK Failed",
                                    "No solution found (numerical failure).")
                return
            T_check = self.bridge.forward_kinematics(q_sol)
            err = float(np.linalg.norm(T_check[:3, 3] - np.array([x, y, z])))
            if err > 15.0:
                QMessageBox.warning(
                    self, "IK Warning",
                    f"Target unreachable or outside joint limits.\n"
                    f"Position error: {err:.1f} mm")
                return
            self.target_q_deg = np.rad2deg(q_sol)
            self._sync_sliders()
            self._start_joint_motion()

    def _update_statusbar(self, pos: np.ndarray):
        self._sb_tcp.setText(
            f"TCP   X {pos[0]:8.1f}   Y {pos[1]:8.1f}   Z {pos[2]:8.1f} mm")

    def _update_out(self, T_ee, q_rad):
        pos  = self.viz.tcp_position(T_ee)
        self._update_statusbar(pos)
        self._workspace_map.update_q(self.current_q_deg)
        roll, pitch, yaw = _rpy_from_matrix(T_ee[:3, :3])
        J    = self.bridge.jacobian(q_rad)
        det  = float(np.linalg.det(J))
        cond = float(np.linalg.cond(J))
        cond_color = ("#FF4444" if cond > 1e4
                      else "#FFA500" if cond > 1e3
                      else "#c9d1d9")
        self.out_lbl.setText(
            f"<pre>Position\n  X:{pos[0]:8.1f} mm\n  Y:{pos[1]:8.1f} mm\n"
            f"  Z:{pos[2]:8.1f} mm\n\n"
            f"Orientation\n  R:{roll:7.1f} deg\n  P:{pitch:7.1f} deg\n  Y:{yaw:7.1f} deg\n\n"
            f"det(J): {det:10.2f}\n"
            f'<span style="color:{cond_color}">cond(J):{cond:10.1f}</span></pre>')
        if abs(det) < 1e-3:
            q_cur = self.current_q_deg
            if abs(q_cur[4]) < 5:
                sing_text = "SINGULARITY: Wrist (J5 ~ 0 deg)"
            elif float(np.linalg.norm(pos)) > 1400:
                sing_text = "SINGULARITY: Arm extended"
            else:
                sing_text = "SINGULARITY DETECTED"
            self.sing_lbl.setText(sing_text)
        else:
            self.sing_lbl.setText("")
        self._update_safety_status(pos, q_rad, det, cond)

    def _update_safety_status(self, tcp_pos, q_rad, det, cond):
        q_deg     = np.rad2deg(q_rad)
        warnings  = []
        collision = False

        # Joint limit check with value label colouring
        for i, (q, (lo, hi)) in enumerate(zip(q_deg, JOINT_LIMITS_DEG)):
            margin = min(q - lo, hi - q)
            if margin < 3:
                color = "#FF4444"
                warnings.append(f"J{i+1} at limit")
                self.sliders[i].setStyleSheet(
                    "QSlider::sub-page:horizontal{background:#FF4444;border-radius:3px;}")
            elif margin < 10:
                color = "#FFA500"
                warnings.append(f"J{i+1} near limit")
                self.sliders[i].setStyleSheet(
                    "QSlider::sub-page:horizontal{background:#FFA500;border-radius:3px;}")
            else:
                color = "#c9d1d9"
                self.sliders[i].setStyleSheet("")
            self.val_labels[i].setStyleSheet(f"color:{color};")

        # TCP ground clearance
        if tcp_pos[2] < 80:
            warnings.append("TCP near ground")

        # Nominal workspace boundary
        if np.linalg.norm(tcp_pos - np.array([150.0, 0.0, 486.0])) > 1450:
            warnings.append("TCP outside nominal reach")

        # Singularity
        if abs(det) < 1e-3 or cond > 1e6:
            warnings.append("singularity risk")

        # Collision check using link frame origins
        if self._last_tfs is not None:
            origins = [T[:3, 3] for T in self._last_tfs]

            # Ground collision: frame origin Z < 80 mm
            for i, o in enumerate(origins):
                if o[2] < 80:
                    warnings.append(f"L{i+1} near ground ({o[2]:.0f} mm)")
                    collision = True

            # Self-collision: non-adjacent link pairs
            COL_PAIRS  = [(0, 2), (0, 3), (1, 4)]
            COL_THRESH = 200.0   # mm
            for a, b in COL_PAIRS:
                d = float(np.linalg.norm(origins[a] - origins[b]))
                if d < COL_THRESH:
                    warnings.append(f"L{a+1}-L{b+1} collision risk ({d:.0f} mm)")
                    collision = True

        if collision:
            self.safety_lbl.setText("Safety: " + "; ".join(warnings[:3]))
            self.safety_lbl.setStyleSheet("color:#FF4444;font-weight:bold;")
        elif warnings:
            self.safety_lbl.setText("Safety: " + "; ".join(warnings[:3]))
            self.safety_lbl.setStyleSheet("color:#cc7a00;font-weight:normal;")
        else:
            self.safety_lbl.setText("Safety: OK")
            self.safety_lbl.setStyleSheet("color:#00aa66;font-weight:normal;")

    def _update_object_sim(self, T_ee):
        status = self.viz.update_grasp_simulation(T_ee, self._grip_pct)
        self.object_status_lbl.setText("Object: " + status)

    def _wp_add(self):
        q = self.target_q_deg.copy(); self.waypoints.append(q)
        n = len(self.waypoints)
        self.wp_list.addItem(QListWidgetItem(
            f"WP{n:02d} | " +
            "  ".join(f"J{i+1}:{q[i]:+5.1f}" for i in range(6))))
        self.traj_lbl.setText(f"{n} waypoint(s) stored")

    def _wp_remove(self):
        row = self.wp_list.currentRow()
        if row < 0: return
        self.wp_list.takeItem(row); self.waypoints.pop(row)
        for i in range(self.wp_list.count()):
            q = self.waypoints[i]
            self.wp_list.item(i).setText(
                f"WP{i+1:02d} | " +
                "  ".join(f"J{j+1}:{q[j]:+5.1f}" for j in range(6)))
        self.traj_lbl.setText(f"{len(self.waypoints)} waypoint(s) stored")

    def _wp_clear(self):
        self.waypoints = []; self.wp_list.clear()
        self._traj_stop(); self.traj_lbl.setText("Cleared")

    def _set_traj_speed(self, v):
        self._traj_speed = v / 10.
        self.spd_lbl.setText(f"{self._traj_speed:.1f}x")

    def _highlight_wp(self, active_idx: int):
        for i in range(self.wp_list.count()):
            item = self.wp_list.item(i)
            if i == active_idx:
                item.setForeground(QColor("#f0c060"))
                item.setBackground(QColor("#2a1f00"))
                self.wp_list.scrollToItem(item)
            else:
                item.setForeground(QColor("#c9d1d9"))
                item.setBackground(QColor("#161b22"))

    def _clear_wp_highlight(self):
        for i in range(self.wp_list.count()):
            item = self.wp_list.item(i)
            item.setForeground(QColor("#c9d1d9"))
            item.setBackground(QColor("#161b22"))

    def _traj_start(self):
        if not self.waypoints:
            self.traj_lbl.setText("Add a waypoint first"); return
        self._traj_play   = True
        self._traj_idx    = -1
        self._cycle_start = time.time()
        self._traj_step()

    def _traj_stop(self, completed: bool = False):
        self._traj_play     = False
        self._motion_active = False
        self._clear_wp_highlight()
        if completed and self._cycle_start is not None:
            elapsed = time.time() - self._cycle_start
            self.cycle_lbl.setText(f"Cycle time: {elapsed:.2f} s")
            self._cycle_start = None
            self.traj_lbl.setText("Completed")
        else:
            self._cycle_start = None
            self.traj_lbl.setText("Stopped")

    def _wp_goto_selected(self):
        row = self.wp_list.currentRow()
        if row < 0: return
        self.target_q_deg = self.waypoints[row].copy()
        self._sync_sliders()
        self._start_joint_motion()
        self.traj_lbl.setText(f"WP{row+1:02d} selected")

    def _traj_step(self):
        """Advance to the next waypoint, skipping any that coincide with the current pose."""
        for _ in range(len(self.waypoints)):
            self._traj_idx += 1
            if self._traj_idx >= len(self.waypoints):
                if self._traj_loop:
                    self._traj_idx = 0
                else:
                    self._traj_stop(completed=True)
                    return
            self.target_q_deg = self.waypoints[self._traj_idx].copy()
            self._sync_sliders()
            self._start_joint_motion()
            self._highlight_wp(self._traj_idx)
            self.traj_lbl.setText(
                f"WP{self._traj_idx+1:02d} / {len(self.waypoints):02d}")
            if self._motion_active:
                return   # motion initiated
        self._traj_stop(completed=True)

    def _on_grip_enable(self, state):
        enabled = (state == Qt.Checked)
        self.grip_slider.setEnabled(enabled)
        if enabled and not self.viz.gripper_loaded():
            self.viz.load_gripper()   # primitive placeholder fallback
            self.grip_model_lbl.setText("Model: primitive")
        self.viz.set_gripper_visible(enabled)
        if enabled and self._last_tfs is not None:
            self.viz.update_gripper(self._grip_pct, self._last_tfs[-1])
        self._update_io()

    def _on_grip_slider(self, v):
        self._grip_pct = float(v)
        self.grip_val_lbl.setText(f"{v:3d} %")
        self._gripper_bar.set_pct(float(v))
        self._update_io()
        # Update immediately even when the robot is stationary
        if self.viz.gripper_loaded() and self._last_tfs is not None:
            self.viz.update_gripper(self._grip_pct, self._last_tfs[-1])
            self._update_object_sim(self._last_tfs[-1])

    def _set_grip(self, pct):
        self.grip_slider.setValue(pct)

    def _load_gripper_override(self):
        path_l, _ = QFileDialog.getOpenFileName(
            self, "Left Finger STL", "", "STL Files (*.stl)")
        if not path_l: return
        path_r, _ = QFileDialog.getOpenFileName(
            self, "Right Finger STL (cancel = mirror)", "", "STL Files (*.stl)")
        self.viz.load_gripper(
            stl_left  = path_l,
            stl_right = path_r if path_r else None)
        self.grip_model_lbl.setText(
            f"L: {Path(path_l).name}\nR: {Path(path_r).name}"
            if path_r else f"Mirror: {Path(path_l).name}")
        self.chk_gripper.setChecked(True)
        if self._last_tfs is not None:
            self.viz.update_gripper(self._grip_pct, self._last_tfs[-1])

    def _update_gripper_model_label(self):
        model_name = getattr(self.viz, '_gripper_model_name', '')
        if model_name:
            self.grip_model_lbl.setText(model_name)
            return
        t = getattr(self.viz, '_gripper_type', '') or ''
        if 'pair' in t:
            self.grip_model_lbl.setText("Finray_8_V6 (L + R)")
        elif 'mirror' in t:
            self.grip_model_lbl.setText("Mirrored STL")
        elif 'primitive' in t:
            self.grip_model_lbl.setText("Primitive placeholder")

    def _update_io(self):
        pct = self._grip_pct
        on  = "color:#00ff88;"
        off = "color:#333;"
        self.io_leds["DO1"].setStyleSheet(on if pct < 5  else off)
        self.io_leds["DO2"].setStyleSheet(on if pct > 95 else off)
        self.io_leds["DI1"].setStyleSheet(on if pct < 5  else off)
        self.io_leds["DI2"].setStyleSheet(on if pct > 95 else off)
        self.grip_status_lbl.setText(
            "Gripper: closed" if pct < 5  else
            "Gripper: open"   if pct > 95 else
            f"Gripper: {pct:.0f}% open")

    def _on_vision_gripper(self, pct: float):
        """Receive gripper target from the Vision tab and apply it."""
        if not self.viz.gripper_loaded():
            return
        self._set_grip(int(pct))

    def _toggle_ik_drag(self, enabled: bool):
        self._ik_drag_enabled = enabled
        if enabled:
            self.btn_ik_drag.setStyleSheet(
                "background:#1f6feb;color:white;font-weight:bold;")
            self.viz.plotter.enable_point_picking(
                callback=self._on_ik_drag_pick,
                show_message=False,
                use_mesh=True,
                show_point=True,
                point_size=14,
                color="#ff6600",
            )
        else:
            self.btn_ik_drag.setStyleSheet("")
            try:
                self.viz.plotter.disable_picking()
            except Exception:
                pass

    def _on_ik_drag_pick(self, *args):
        if not self._ik_drag_enabled or not args:
            return
        point = np.asarray(args[0], dtype=float).flatten()[:3]
        T = self.bridge.forward_kinematics(np.deg2rad(self.current_q_deg))
        T_tcp = self.viz.tcp_transform(T).copy()
        T_tcp[:3, 3] = point
        T_ee_target  = self.viz.ee_from_tcp_target(T_tcp)
        try:
            q_sol = self.bridge.inverse_kinematics(
                T_ee_target, np.deg2rad(self.current_q_deg))
        except Exception:
            return
        if not np.all(np.isfinite(q_sol)):
            return
        T_check = self.bridge.forward_kinematics(q_sol)
        if float(np.linalg.norm(T_check[:3, 3] - point)) > 60.0:
            return
        self.target_q_deg = np.rad2deg(q_sol)
        self._sync_sliders()
        self._start_joint_motion()

    def _preview_trajectory(self):
        if not self.waypoints:
            self.traj_lbl.setText("No waypoints to preview")
            return
        positions = []
        for q in self.waypoints:
            T   = self.bridge.forward_kinematics(np.deg2rad(q))
            pos = self.viz.tcp_position(T)
            positions.append(pos)
        self.viz.draw_trajectory_preview(positions)
        self.traj_lbl.setText(f"Preview: {len(positions)} waypoints")

    def _log_motion(self, q_start: np.ndarray, q_end: np.ndarray,
                    duration: float):
        if float(np.max(np.abs(q_end - q_start))) < DEAD_BAND:
            return
        entry = {
            "time":     time.strftime("%H:%M:%S"),
            "duration": round(duration, 3),
            "q_start":  q_start.tolist(),
            "q_end":    q_end.tolist(),
        }
        self._motion_log.append(entry)
        q_str = "  ".join(f"J{i+1}:{q_end[i]:+5.1f}" for i in range(6))
        item  = QListWidgetItem(
            f"{entry['time']}  {q_str}  [{duration:.2f}s]")
        item.setFont(QFont("Courier New", 7))
        self.log_list.insertItem(0, item)
        if self.log_list.count() > 200:
            self.log_list.takeItem(self.log_list.count() - 1)

    def _export_log_csv(self):
        if not self._motion_log:
            QMessageBox.information(self, "Export Log", "Motion log is empty.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export Motion Log", "motion_log.csv", "CSV (*.csv)")
        if not path:
            return
        fieldnames = ["time", "duration",
                      "J1_start","J2_start","J3_start","J4_start","J5_start","J6_start",
                      "J1_end",  "J2_end",  "J3_end",  "J4_end",  "J5_end",  "J6_end"]
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            for e in self._motion_log:
                row = {"time": e["time"], "duration": e["duration"]}
                for i in range(6):
                    row[f"J{i+1}_start"] = round(e["q_start"][i], 3)
                    row[f"J{i+1}_end"]   = round(e["q_end"][i], 3)
                w.writerow(row)
        QMessageBox.information(self, "Export Log",
                                f"Saved {len(self._motion_log)} entries to:\n{path}")

    def _build_log_tab(self) -> QWidget:
        w = QWidget(); vbox = QVBoxLayout(w); vbox.setSpacing(6)
        lbox = QGroupBox("Motion History")
        lbox.setFont(QFont("Segoe UI", 9, QFont.Bold))
        ll = QVBoxLayout(lbox)
        self.log_list = QListWidget()
        self.log_list.setFont(QFont("Courier New", 7))
        ll.addWidget(self.log_list)
        btn_row = QHBoxLayout()
        btn_exp = QPushButton("Export CSV")
        btn_exp.setMinimumHeight(28)
        btn_exp.clicked.connect(self._export_log_csv)
        btn_clr = QPushButton("Clear")
        btn_clr.setMinimumHeight(28)
        btn_clr.clicked.connect(lambda: (self.log_list.clear(),
                                         self._motion_log.clear()))
        btn_row.addWidget(btn_exp); btn_row.addWidget(btn_clr)
        ll.addLayout(btn_row)
        vbox.addWidget(lbox)
        vbox.addStretch()
        return w

    def _save_config(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Configuration", "config.json", "JSON (*.json)")
        if not path:
            return
        data = {
            "joints":   self.target_q_deg.tolist(),
            "waypoints": [q.tolist() for q in self.waypoints],
            "gripper":  self._grip_pct,
            "profile":  self._motion_profile_type,
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        QMessageBox.information(self, "Save Config",
                                f"Configuration saved to:\n{path}")

    def _load_config(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Configuration", "", "JSON (*.json)")
        if not path:
            return
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            QMessageBox.critical(self, "Load Config", f"Failed to load:\n{e}")
            return
        self.target_q_deg = np.array(data.get("joints", HOME_Q_DEG.tolist()))
        self.waypoints = [np.array(q) for q in data.get("waypoints", [])]
        self._grip_pct = float(data.get("gripper", 100.0))
        profile = data.get("profile", "poly5")
        self.prof_combo.setCurrentIndex(
            {"poly5": 0, "trap": 1, "scurve": 2}.get(profile, 0))
        self.grip_slider.setValue(int(self._grip_pct))
        self._sync_sliders()
        self._rebuild_wp_list()
        self._run_fk_snap()

    def _rebuild_wp_list(self):
        self.wp_list.clear()
        for i, q in enumerate(self.waypoints):
            self.wp_list.addItem(QListWidgetItem(
                f"WP{i+1:02d} | " +
                "  ".join(f"J{j+1}:{q[j]:+5.1f}" for j in range(6))))

    def _build_program_tab(self) -> QWidget:
        w = QWidget(); vbox = QVBoxLayout(w); vbox.setSpacing(6)

        ebox = QGroupBox("Program Editor")
        ebox.setFont(QFont("Segoe UI", 9, QFont.Bold))
        el = QVBoxLayout(ebox)
        self.prog_editor = LineNumberedEdit()
        self.prog_editor.setFont(QFont("Cascadia Code", 9) if QFont("Cascadia Code").exactMatch()
                                 else QFont("Courier New", 9))
        self._prog_highlighter = _ProgramHighlighter(self.prog_editor.document())
        self.prog_editor.setPlaceholderText(
            "# Commands:\n"
            "# HOME\n"
            "# MOVEJ q1 q2 q3 q4 q5 q6\n"
            "# MOVEJ q1 q2 q3 q4 q5 q6 speed=0.5\n"
            "# GRIPPER pct\n"
            "# WAIT ms\n"
        )
        self.prog_editor.setMinimumHeight(200)
        el.addWidget(self.prog_editor)

        btn_row = QHBoxLayout()
        btn_run = QPushButton("Run")
        btn_run.setMinimumHeight(32)
        btn_run.setStyleSheet("background:#1D9E75;color:white;font-weight:bold;")
        btn_run.clicked.connect(self._run_program)
        btn_stp = QPushButton("Stop")
        btn_stp.setMinimumHeight(32)
        btn_stp.setStyleSheet("background:#9E1D1D;color:white;font-weight:bold;")
        btn_stp.clicked.connect(self._stop_program)
        btn_row.addWidget(btn_run); btn_row.addWidget(btn_stp)
        el.addLayout(btn_row)

        self.prog_status = QLabel("Ready")
        self.prog_status.setFont(QFont("Courier New", 9))
        self.prog_status.setStyleSheet(
            "background:#0d1117;color:#58a6ff;padding:4px;border-radius:4px;")
        self.prog_status.setAlignment(Qt.AlignCenter)
        el.addWidget(self.prog_status)
        vbox.addWidget(ebox)

        hbox = QGroupBox("Quick insert")
        hbox.setFont(QFont("Segoe UI", 9, QFont.Bold))
        hl = QVBoxLayout(hbox)
        hl.addWidget(QLabel("Add current joint angles as MOVEJ:"))
        btn_ins = QPushButton("Insert MOVEJ (current position)")
        btn_ins.setMinimumHeight(26)
        btn_ins.clicked.connect(self._prog_insert_current)
        hl.addWidget(btn_ins)
        vbox.addWidget(hbox)
        vbox.addStretch()
        return w

    def _prog_insert_current(self):
        q = self.current_q_deg
        line = "MOVEJ " + " ".join(f"{q[i]:+.2f}" for i in range(6))
        cursor = self.prog_editor.textCursor()
        cursor.insertText(line + "\n")

    def _parse_program(self, code: str) -> list:
        cmds = []
        for lineno, raw in enumerate(code.splitlines(), 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            # Strip inline comments before parsing
            if "#" in line:
                line = line[:line.index("#")].strip()
            if not line:
                continue
            parts = line.split()
            cmd   = parts[0].upper()
            if cmd == "HOME":
                cmds.append({"type": "HOME"})
            elif cmd == "MOVEJ":
                vals, speed = [], 1.0
                for p in parts[1:]:
                    if p.lower().startswith("speed="):
                        speed = float(p.split("=")[1])
                    else:
                        vals.append(float(p))
                if len(vals) != 6:
                    raise ValueError(
                        f"Line {lineno}: MOVEJ needs 6 joint values, got {len(vals)}")
                cmds.append({"type": "MOVEJ", "q": np.array(vals), "speed": speed})
            elif cmd == "GRIPPER":
                if len(parts) < 2:
                    raise ValueError(f"Line {lineno}: GRIPPER needs a value (0-100)")
                cmds.append({"type": "GRIPPER", "pct": float(parts[1])})
            elif cmd == "WAIT":
                if len(parts) < 2:
                    raise ValueError(f"Line {lineno}: WAIT needs milliseconds")
                cmds.append({"type": "WAIT", "ms": int(parts[1])})
            else:
                raise ValueError(f"Line {lineno}: Unknown command '{cmd}'")
        return cmds

    def _run_program(self):
        try:
            cmds = self._parse_program(self.prog_editor.toPlainText())
        except ValueError as e:
            QMessageBox.critical(self, "Program Error", str(e))
            return
        if not cmds:
            self.prog_status.setText("No commands found")
            return
        if self._traj_play:
            self._traj_stop()
        self._prog_cmds    = cmds
        self._prog_idx     = 0
        self._prog_running = True
        self.prog_status.setText(f"Running  0 / {len(cmds)}")
        self._prog_step()

    def _stop_program(self):
        self._prog_running  = False
        self._motion_active = False
        self.prog_status.setText("Stopped")

    def _prog_step(self):
        if not self._prog_running:
            return
        if self._prog_idx >= len(self._prog_cmds):
            self._prog_running = False
            self.prog_status.setText("Completed")
            return
        cmd = self._prog_cmds[self._prog_idx]
        self._prog_idx += 1
        self.prog_status.setText(
            f"[{self._prog_idx}/{len(self._prog_cmds)}]  {cmd['type']}")

        if cmd["type"] == "HOME":
            self.target_q_deg = HOME_Q_DEG.copy()
            self._sync_sliders()
            self._start_joint_motion()
            if not self._motion_active:
                self._prog_step()

        elif cmd["type"] == "MOVEJ":
            self.target_q_deg = cmd["q"].copy()
            self._sync_sliders()
            self._start_joint_motion()
            if not self._motion_active:
                self._prog_step()

        elif cmd["type"] == "GRIPPER":
            self._set_grip(int(np.clip(cmd["pct"], 0, 100)))
            self._prog_step()

        elif cmd["type"] == "WAIT":
            QTimer.singleShot(cmd["ms"], self._prog_step)

    def _set_active_joint(self, idx: int):
        self._joint_name_labels[self._active_joint].setStyleSheet("")
        self._active_joint = idx
        self._joint_name_labels[idx].setStyleSheet("color:#ff7700;")

    def _nudge_joint(self, delta: float):
        lo, hi = JOINT_LIMITS_DEG[self._active_joint]
        new_deg = float(np.clip(
            self.target_q_deg[self._active_joint] + delta, lo, hi))
        self.target_q_deg[self._active_joint] = new_deg
        sl = self.sliders[self._active_joint]
        sl.blockSignals(True)
        sl.setValue(int(new_deg * 10))
        sl.blockSignals(False)
        self.val_labels[self._active_joint].setText(f"{new_deg:+6.1f} deg")
        if self._traj_play:
            self._traj_stop()
        self._start_joint_motion()

    def keyPressEvent(self, event):
        from PyQt5.QtWidgets import QPlainTextEdit, QLineEdit, QAbstractSpinBox
        if isinstance(self.focusWidget(), (QPlainTextEdit, QLineEdit, QAbstractSpinBox)):
            super().keyPressEvent(event)
            return
        key = event.key()
        if Qt.Key_1 <= key <= Qt.Key_6:
            self._set_active_joint(key - Qt.Key_1)
            return
        shift = bool(event.modifiers() & Qt.ShiftModifier)
        step  = 5.0 if shift else 1.0
        if key in (Qt.Key_Up, Qt.Key_Right):
            self._nudge_joint(step)
        elif key in (Qt.Key_Down, Qt.Key_Left):
            self._nudge_joint(-step)
        else:
            super().keyPressEvent(event)

    def _reset_object_and_spinboxes(self):
        self.viz.set_object_visible(False)
        self._obj_x_spin.setValue(1200.0)
        self._obj_y_spin.setValue(0.0)
        self._obj_z_spin.setValue(1265.0)
        self.object_status_lbl.setText("Object: hidden")

    def _show_object(self):
        pos = np.array([
            self._obj_x_spin.value(),
            self._obj_y_spin.value(),
            self._obj_z_spin.value(),
        ])
        self.viz.set_object_position(pos)
        self._workspace_map.set_object_xy(pos[0], pos[1], visible=True)
        self.object_status_lbl.setText(
            f"Object: X{pos[0]:.0f}  Y{pos[1]:.0f}  Z{pos[2]:.0f}")

    def _hide_object(self):
        self.viz.set_object_visible(False)
        self._workspace_map.set_object_xy(0, 0, visible=False)
        self.object_status_lbl.setText("Object: hidden")

    def _set_object_position(self):
        self._show_object()

    def _generate_pick_place(self):
        obj_pos = np.array([
            self._obj_x_spin.value(),
            self._obj_y_spin.value(),
            self._obj_z_spin.value(),
        ])
        place_pos = np.array([
            self._place_x_spin.value(),
            self._place_y_spin.value(),
            self._place_z_spin.value(),
        ])
        self._show_object()

        pick_approach  = obj_pos   + np.array([0.0, 0.0, 160.0])
        pick_grasp     = obj_pos   + np.array([0.0, 0.0,  10.0])
        place_approach = place_pos + np.array([0.0, 0.0, 160.0])
        place_drop     = place_pos + np.array([0.0, 0.0,  10.0])

        T_cur = self.bridge.forward_kinematics(np.deg2rad(self.current_q_deg))

        def _ik_for_tcp(tcp_pos, q_init_rad):
            T_tcp = self.viz.tcp_transform(T_cur).copy()
            T_tcp[:3, 3] = tcp_pos
            T_ee = self.viz.ee_from_tcp_target(T_tcp)
            return self.bridge.inverse_kinematics(T_ee, q_init_rad)

        try:
            q0            = np.deg2rad(HOME_Q_DEG)
            q_pick_app    = _ik_for_tcp(pick_approach,  q0)
            if not np.all(np.isfinite(q_pick_app)):
                raise ValueError("No IK solution for pick approach position")
            q_pick_grasp  = _ik_for_tcp(pick_grasp,     q_pick_app)
            if not np.all(np.isfinite(q_pick_grasp)):
                raise ValueError("No IK solution for pick grasp position")
            q_place_app   = _ik_for_tcp(place_approach, q_pick_app)
            if not np.all(np.isfinite(q_place_app)):
                raise ValueError("No IK solution for place approach position")
            q_place_drop  = _ik_for_tcp(place_drop,     q_place_app)
            if not np.all(np.isfinite(q_place_drop)):
                raise ValueError("No IK solution for place drop position")
        except Exception as e:
            QMessageBox.critical(self, "Pick-and-Place Demo",
                                 f"IK computation failed:\n{e}\n\n"
                                 "Adjust pick / place coordinates and try again.")
            return

        def _q_line(q_rad, comment):
            q_d = np.rad2deg(q_rad)
            return ("MOVEJ " + " ".join(f"{q:+.2f}" for q in q_d)
                    + f"  # {comment}")

        lines = [
            "# === Pick-and-Place Demo (auto-generated) ===",
            f"# Pick  at  X:{obj_pos[0]:.0f}  Y:{obj_pos[1]:.0f}  Z:{obj_pos[2]:.0f} mm",
            f"# Place at  X:{place_pos[0]:.0f}  Y:{place_pos[1]:.0f}  Z:{place_pos[2]:.0f} mm",
            "",
            "GRIPPER 100",
            "HOME",
            "",
            "# --- PICK ---",
            _q_line(q_pick_app,   "Approach: 160 mm above pick object"),
            _q_line(q_pick_grasp, "Descend:  grasp height"),
            "WAIT 200",
            "GRIPPER 0",
            "WAIT 500",
            _q_line(q_pick_app,   "Lift: back to pick approach"),
            "",
            "# --- PLACE ---",
            _q_line(q_place_app,  "Move to: 160 mm above drop zone"),
            _q_line(q_place_drop, "Descend: drop height"),
            "WAIT 200",
            "GRIPPER 100",
            "WAIT 400",
            _q_line(q_place_app,  "Lift: back to place approach"),
            "",
            "HOME",
        ]
        self.prog_editor.setPlainText("\n".join(lines))
        self.tabs.setCurrentIndex(5)
        QMessageBox.information(
            self, "Pick-and-Place Program Ready",
            "Program inserted in the Program tab.\n\n"
            "Review the joint angles, then press Run.")

    def closeEvent(self, event):
        """Clean up background threads before the window closes."""
        self._vision_tab.stop_tracking()
        super().closeEvent(event)
