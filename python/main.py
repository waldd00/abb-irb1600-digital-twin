"""
main.py  --  ABB IRB 1600 Digital Twin  --  entry point

Startup sequence:
  1. Splash screen
  2. MATLAB engine (~10 s)
  3. Robot link meshes   (cad/links/)
  4. Gripper STL files   (cad/gripper/)  -- loaded automatically if present
  5. PyVista 3D scene
  6. PyQt5 main window
  7. Qt event loop
  8. MATLAB engine shutdown
"""

import sys
from pathlib import Path

from PyQt5.QtWidgets import QApplication, QSplashScreen
from PyQt5.QtGui     import QPixmap, QColor, QPainter, QFont
from PyQt5.QtCore    import Qt

from matlab_bridge    import MatlabBridge
from robot_visualizer import RobotVisualizer, load_link_meshes
from main_window      import MainWindow

BASE_DIR       = Path(__file__).resolve().parent.parent
MATLAB_FOLDER  = str(BASE_DIR / "matlab")
CAD_LINKS      = str(BASE_DIR / "cad" / "links")
CAD_GRIPPER    = BASE_DIR / "cad" / "gripper"

GRIPPER_BASE   = str(CAD_GRIPPER / "robotiq_2f85_base.stl")
GRIPPER_LEFT   = str(CAD_GRIPPER / "robotiq_2f85_left_finger.stl")
GRIPPER_RIGHT  = str(CAD_GRIPPER / "robotiq_2f85_right_finger.stl")

_QSS = """
/* ── Base ───────────────────────────────────────────────────── */
QMainWindow, QWidget       { background: #0d1117; color: #c9d1d9; }
QDialog                    { background: #0d1117; }
QLabel                     { color: #c9d1d9; }

/* ── Menu ───────────────────────────────────────────────────── */
QMenuBar {
    background: #010409;
    color: #8b949e;
    border-bottom: 1px solid #21262d;
    padding: 2px 0;
}
QMenuBar::item:selected {
    background: #21262d;
    color: #c9d1d9;
    border-radius: 4px;
}
QMenu {
    background: #161b22;
    border: 1px solid #30363d;
    border-radius: 6px;
    padding: 4px;
}
QMenu::item { padding: 6px 20px 6px 12px; border-radius: 4px; }
QMenu::item:selected { background: #1f6feb; color: #fff; }
QMenu::separator { height: 1px; background: #30363d; margin: 4px 8px; }

/* ── GroupBox  (card style) ─────────────────────────────────── */
QGroupBox {
    background: #0f1318;
    border: 1px solid #1c2128;
    border-top: 2px solid #21262d;
    border-radius: 8px;
    margin-top: 18px;
    padding: 10px 8px 8px 8px;
}
QGroupBox::title {
    subcontrol-origin:   margin;
    subcontrol-position: top left;
    left: 10px;
    top: -1px;
    padding: 2px 8px;
    background: #0f1318;
    border-radius: 4px;
    color: #58a6ff;
    font-weight: bold;
    font-size: 9pt;
}

/* ── Tabs ───────────────────────────────────────────────────── */
QTabWidget::pane {
    border: 1px solid #1c2128;
    border-radius: 0 6px 6px 6px;
    background: #0d1117;
    top: -1px;
}
QTabBar::tab {
    background: #0d1117;
    color: #8b949e;
    border: 1px solid #1c2128;
    border-bottom: none;
    padding: 5px 9px;
    border-radius: 6px 6px 0 0;
    min-width: 44px;
    font-size: 9pt;
}
QTabBar::tab:selected {
    background: #0d1117;
    color: #c9d1d9;
    border-color: #1c2128;
    border-bottom: 2px solid #58a6ff;
    font-weight: bold;
}
QTabBar::tab:hover:!selected {
    background: #161b22;
    color: #c9d1d9;
}

/* ── Buttons ────────────────────────────────────────────────── */
QPushButton {
    background: #21262d;
    color: #c9d1d9;
    border: 1px solid #30363d;
    border-radius: 6px;
    padding: 5px 12px;
    font-size: 9pt;
}
QPushButton:hover   { background: #30363d; border-color: #8b949e; color: #e6edf3; }
QPushButton:pressed { background: #161b22; border-color: #58a6ff; }
QPushButton:checked { background: #1f3a5f; border-color: #58a6ff; color: #58a6ff; }
QPushButton:disabled{ color: #3d444d; border-color: #1c2128; background: #161b22; }

/* ── Sliders ────────────────────────────────────────────────── */
QSlider::groove:horizontal {
    height: 6px;
    background: #1c2128;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #58a6ff;
    width: 16px;
    height: 16px;
    margin: -5px 0;
    border-radius: 8px;
    border: 2px solid #1f6feb;
}
QSlider::handle:horizontal:hover { background: #79c0ff; border-color: #58a6ff; }
QSlider::sub-page:horizontal {
    background: qlineargradient(x1:0,y1:0,x2:1,y2:0,
                stop:0 #1f6feb, stop:1 #58a6ff);
    border-radius: 3px;
}

/* ── Spin boxes ─────────────────────────────────────────────── */
QDoubleSpinBox, QSpinBox {
    background: #0d1117;
    border: 1px solid #30363d;
    border-radius: 6px;
    color: #c9d1d9;
    padding: 4px 8px;
    selection-background-color: #1f6feb;
}
QDoubleSpinBox:focus, QSpinBox:focus { border-color: #58a6ff; }
QDoubleSpinBox::up-button, QSpinBox::up-button,
QDoubleSpinBox::down-button, QSpinBox::down-button {
    border: none;
    background: transparent;
    width: 14px;
}

/* ── ComboBox ───────────────────────────────────────────────── */
QComboBox {
    background: #21262d;
    border: 1px solid #30363d;
    border-radius: 6px;
    padding: 4px 10px;
    color: #c9d1d9;
    min-height: 24px;
    font-size: 9pt;
}
QComboBox:hover { border-color: #58a6ff; }
QComboBox::drop-down { border: none; width: 20px; }
QComboBox QAbstractItemView {
    background: #21262d;
    border: 1px solid #30363d;
    border-radius: 6px;
    selection-background-color: #1f6feb;
    color: #c9d1d9;
    outline: none;
    padding: 4px;
}

/* ── CheckBox ───────────────────────────────────────────────── */
QCheckBox { spacing: 8px; color: #c9d1d9; font-size: 9pt; }
QCheckBox::indicator {
    width: 16px; height: 16px;
    border: 1.5px solid #30363d;
    border-radius: 4px;
    background: #0d1117;
}
QCheckBox::indicator:hover   { border-color: #58a6ff; }
QCheckBox::indicator:checked {
    background: #1f6feb;
    border-color: #58a6ff;
}

/* ── List ───────────────────────────────────────────────────── */
QListWidget {
    background: #0d1117;
    border: 1px solid #1c2128;
    border-radius: 6px;
    outline: none;
    padding: 4px;
}
QListWidget::item { padding: 3px 6px; border-radius: 4px; }
QListWidget::item:selected { background: #1f6feb; color: #fff; }
QListWidget::item:hover:!selected { background: #161b22; }

/* ── Text / Code editors ────────────────────────────────────── */
QPlainTextEdit {
    background: #0d1117;
    border: 1px solid #1c2128;
    border-radius: 6px;
    color: #c9d1d9;
    selection-background-color: #1f3a5f;
    font-family: "Cascadia Code", "Fira Code", "Consolas", "Courier New";
    font-size: 9pt;
    line-height: 1.4;
}
QPlainTextEdit:focus { border-color: #30363d; }

/* ── Progress bar ───────────────────────────────────────────── */
QProgressBar {
    background: #1c2128;
    border: none;
    border-radius: 5px;
    text-align: center;
    color: #8b949e;
    font-size: 8pt;
    min-height: 14px;
}
QProgressBar::chunk {
    background: qlineargradient(x1:0,y1:0,x2:1,y2:0,
                stop:0 #1f6feb, stop:1 #58a6ff);
    border-radius: 5px;
}

/* ── Scrollbars ─────────────────────────────────────────────── */
QScrollBar:vertical {
    background: #0d1117; width: 8px; border-radius: 4px; margin: 0;
}
QScrollBar::handle:vertical {
    background: #21262d; border-radius: 4px; min-height: 30px;
}
QScrollBar::handle:vertical:hover { background: #30363d; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar:horizontal {
    background: #0d1117; height: 8px; border-radius: 4px; margin: 0;
}
QScrollBar::handle:horizontal {
    background: #21262d; border-radius: 4px; min-width: 30px;
}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }

/* ── Separators ─────────────────────────────────────────────── */
QFrame[frameShape="4"], QFrame[frameShape="5"] { color: #1c2128; }

/* ── Status bar ─────────────────────────────────────────────── */
QStatusBar { background: #010409; border-top: 1px solid #1c2128; font-size: 8pt; }
QStatusBar::item { border: none; }

/* ── Tooltip ────────────────────────────────────────────────── */
QToolTip {
    background: #161b22;
    color: #c9d1d9;
    border: 1px solid #30363d;
    border-radius: 6px;
    padding: 5px 8px;
    font-size: 8pt;
}
"""


def _make_splash() -> QSplashScreen:
    W, H = 560, 200
    pix = QPixmap(W, H)
    pix.fill(QColor("#0d1117"))

    p = QPainter(pix)
    p.setRenderHint(QPainter.Antialiasing)

    p.fillRect(0, 0, W, 4, QColor("#E85D24"))

    p.setPen(QColor("#E85D24"))
    p.setFont(QFont("Segoe UI", 22, QFont.Bold))
    p.drawText(0, 0, W, 80, Qt.AlignCenter, "ABB IRB 1600")

    p.setPen(QColor("#8b949e"))
    p.setFont(QFont("Segoe UI", 11))
    p.drawText(0, 60, W, 40, Qt.AlignCenter, "Digital Twin")

    p.fillRect(0, H - 4, W, 4, QColor("#1f6feb"))

    p.end()

    splash = QSplashScreen(pix, Qt.WindowStaysOnTopHint)
    splash.setFont(QFont("Segoe UI", 9))
    return splash


def _splash_msg(splash, app, msg: str):
    splash.showMessage(f"  {msg}", Qt.AlignBottom | Qt.AlignLeft, QColor("#58a6ff"))
    app.processEvents()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(_QSS)

    splash = _make_splash()
    splash.show()
    app.processEvents()

    print("=" * 50)
    print("  ABB IRB 1600 — Digital Twin")
    print("=" * 50)

    # 1. MATLAB
    _splash_msg(splash, app, "Starting MATLAB engine...")
    bridge = MatlabBridge(MATLAB_FOLDER)

    # 2. Robot link meshes
    _splash_msg(splash, app, "Loading robot link meshes...")
    meshes = load_link_meshes(CAD_LINKS)
    print(f"  {len(meshes)} link meshes loaded.\n")

    # 3. Visualizer + robot mesh
    _splash_msg(splash, app, "Setting up 3D scene...")
    visualizer = RobotVisualizer()
    visualizer.load_meshes(meshes)

    # 4. Gripper
    _splash_msg(splash, app, "Loading gripper meshes...")
    visualizer.load_gripper(
        stl_left  = GRIPPER_LEFT,
        stl_right = GRIPPER_RIGHT,
        stl_base  = GRIPPER_BASE,
    )

    # 5. Main window
    _splash_msg(splash, app, "Initialising main window...")
    window = MainWindow(bridge, visualizer)
    window.show()
    splash.finish(window)

    # 6. Run
    exit_code = app.exec_()

    # 7. Cleanup
    bridge.close()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
