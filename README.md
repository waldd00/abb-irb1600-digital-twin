# ABB IRB 1600 Digital Twin

<p align="center">
  <img src="docs/demo_pick_place.gif" alt="ABB IRB 1600 Digital Twin — Pick and Place Demo" width="100%"/>
</p>

> An interactive 3D digital twin of the **ABB IRB 1600-6/1.45** industrial robot, combining real-time MATLAB kinematics, PyVista 3D visualization, hand-gesture gripper control via MediaPipe, and a built-in robot programming environment.

---

## Features

- **3D Visualization** — Real-time animated robot with STL link meshes and Robotiq 2F-85 gripper
- **Forward & Inverse Kinematics** — MATLAB-powered FK/IK engine with multi-start numerical solver
- **Joint Control** — 6-axis sliders with keyboard shortcuts (keys 1–6 + ↑↓ nudge)
- **IK Drag Mode** — Click any point in the 3D viewport; the robot moves its TCP there
- **Motion Profiles** — Poly-5, Trapezoidal, and S-curve interpolation
- **Trajectory Playback** — Record waypoints, preview the path, and play back at adjustable speed
- **Gripper Control** — Opening slider, animated visual indicator, and digital I/O simulation
- **Hand Gesture Control** — MediaPipe hand tracking: pinch = close gripper, open hand = open gripper
- **Pick-and-Place Demo** — Auto-generate IK-solved programs from pick/place coordinates
- **Program Editor** — Simple scripting language (`HOME`, `MOVEJ`, `GRIPPER`, `WAIT`) with syntax highlighting
- **Safety Monitoring** — Joint-limit checking, singularity detection, ground clearance, self-collision warnings
- **Motion Log** — Time-stamped motion history with CSV export
- **TCP Trail** — Speed-colored spline trail in the 3D viewport

---

## Tech Stack

| Component | Technology |
|-----------|------------|
| Robot kinematics | MATLAB Engine API + custom DH-parameter `.m` files |
| 3D visualization | PyVista + VTK (`BackgroundPlotter`) |
| GUI framework | PyQt5 |
| Hand tracking | MediaPipe Tasks (`HandLandmarker`) |
| Camera capture | OpenCV |
| Numerics | NumPy |
| IK solver | MATLAB `lsqnonlin` (12-element residual, multi-start) |

---

## Screenshots

<table>
  <tr>
    <td align="center"><img src="docs/screenshot_joints.png" width="300"/><br/><sub>Joint Control</sub></td>
    <td align="center"><img src="docs/demo_trajectory.gif" width="300"/><br/><sub>Trajectory Playback</sub></td>
    <td align="center"><img src="docs/screenshot_gripper.png" width="300"/><br/><sub>Gripper Control</sub></td>
  </tr>
  <tr>
    <td align="center"><img src="docs/screenshot_vision.png" width="300"/><br/><sub>Vision — Open Hand</sub></td>
    <td align="center"><img src="docs/demo_vision.gif" width="300"/><br/><sub>Vision — Hand Gesture</sub></td>
    <td align="center"><img src="docs/screenshot_vision2.png" width="300"/><br/><sub>Vision — Pinch Closed</sub></td>
  </tr>
</table>

---

## Architecture

```
abb-irb1600-digital-twin/
├── cad/
│   ├── links/          # STL meshes for robot links 0–6
│   └── gripper/        # Robotiq 2F-85 base + finger STLs
├── matlab/
│   ├── dh_params.m     # DH parameter table (ABB IRB 1600-6/1.45)
│   ├── dh_matrix.m     # Single-joint DH transformation matrix
│   ├── forward_kinematics.m
│   ├── partial_fk.m    # Transform to intermediate frame n (used for animation)
│   ├── inverse_kinematics.m  # Numerical IK via lsqnonlin, multi-start
│   └── jacobian.m      # 6×6 geometric Jacobian
└── python/
    ├── main.py                 # Entry point, splash screen, startup sequence
    ├── main_window.py          # Main UI controller (6 tabs)
    ├── robot_visualizer.py     # PyVista 3D scene + mesh animation
    ├── matlab_bridge.py        # MATLAB Engine API wrapper
    ├── ui_widgets.py           # GripperBar, WorkspaceMap, LineNumberedEdit
    ├── vision_hand_tracker.py  # MediaPipe hand tracking thread
    └── vision_tab.py           # Hand gesture → gripper control UI
```

**Startup sequence:**

1. Splash screen shown
2. MATLAB engine started (~10 s on first launch)
3. Robot link STL meshes loaded from `cad/links/`
4. Gripper STL meshes loaded from `cad/gripper/`
5. PyVista 3D scene initialized
6. PyQt5 main window displayed

---

## Getting Started

### Prerequisites

| Requirement | Version |
|-------------|---------|
| Python | 3.10+ |
| MATLAB | R2020b+ (with Engine API for Python configured) |
| OS | Windows 10/11 |
| RAM | 4 GB+ recommended |
| Camera | Any USB webcam (Vision tab only) |

### Installation

**1. Configure MATLAB Engine API for Python**

Follow the [official MathWorks guide](https://www.mathworks.com/help/matlab/matlab_external/install-the-matlab-engine-for-python.html). From your MATLAB installation folder:

```powershell
cd "C:\Program Files\MATLAB\R20XXx\extern\engines\python"
python setup.py install
```

**2. Install Python dependencies**

```bash
pip install PyQt5 pyvista pyvistaqt numpy mediapipe opencv-python
```

**3. Clone the repository**

```bash
git clone https://github.com/your-username/abb-irb1600-digital-twin.git
cd abb-irb1600-digital-twin
```

### Running

```bash
cd python
python main.py
```

The first launch takes ~10 seconds while MATLAB starts. Subsequent launches reuse the engine from the same process.

---

## Usage

### Joint Control (Joints Tab)

| Action | How |
|--------|-----|
| Move a joint | Drag slider or press key `1`–`6` to select, then `↑` / `↓` to nudge ±1° |
| Large nudge | `Shift` + `↑` / `↓` = ±5° |
| Snap to sliders | **Run FK (snap)** button |
| Move to XYZ target | **Run IK — enter XYZ** button |
| Click-to-move | Enable **IK Drag**, then click any surface in the 3D viewport |
| Reset | **Reset All Joints** returns to home pose [0, 90, 0, 0, 30, 0]° |

### Trajectory

1. Move the robot to a pose
2. Click **Add** to record a waypoint
3. Repeat for all poses
4. Click **▶ Play Trajectory** — use the Speed slider to adjust playback rate
5. Optional: **Preview Path** draws the planned TCP path in the 3D viewport

### Gripper

- Toggle gripper visibility with the **Enable gripper** checkbox
- Drag the **Open %** slider or use the **Open / Mid / Close** quick buttons
- The animated `GripperBar` and Digital I/O LEDs update live

### Hand Gesture Control (Vision Tab)

1. Select your camera index (default: `0`)
2. Click **Start Camera**
3. Show your hand to the camera:
   - **Open hand** → gripper opens (100 %)
   - **Pinch thumb + index** → gripper closes (0 %)
4. Adjust the **Smoothing** slider to reduce jitter

### Program Editor (Program Tab)

```
# Supported commands
HOME
MOVEJ  q1 q2 q3 q4 q5 q6
MOVEJ  q1 q2 q3 q4 q5 q6  speed=0.5
GRIPPER  pct          # 0 = closed, 100 = open
WAIT  ms
```

Click **Insert MOVEJ (current position)** to capture the current pose, then **Run** to execute.

### Pick-and-Place Demo

1. Go to the **Gripper** tab
2. Enter pick object coordinates (X, Y, Z in mm)
3. Enter drop zone coordinates
4. Click **⚙ Generate Pick-and-Place Program**
5. Switch to the Program tab, review, and click **Run**

---

## Joint Limits

| Joint | Min | Max |
|-------|-----|-----|
| J1 | −180° | +180° |
| J2 | −63° | +110° |
| J3 | −236° | +60° |
| J4 | −200° | +200° |
| J5 | −115° | +115° |
| J6 | −400° | +400° |

Reference: ABB IRB 1600 Product Specification (3HAC027340-001)

---

## Configuration

The application state (joint angles, waypoints, gripper %, motion profile) can be saved and loaded as JSON via **File → Save / Load Configuration**.

---

## License

This project is released for educational and research purposes. CAD models of the ABB IRB 1600 and Robotiq 2F-85 are used for visualization only. All trademarks belong to their respective owners.
