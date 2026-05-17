"""
robot_visualizer.py  —  ABB IRB 1600 Digital Twin

== GRIPPER COORDINATE MATHEMATICS ==
Finray STL local coordinates (measured):
  X: 4.2 to 52.8 mm  finger long axis
  Y: -15.6 to 0 mm   width; Y=0 is the gripping surface
  Z: 0 to 12 mm      thickness

EE mounting rotation:
  STL +X -> EE +Z  (finger points forward)
  STL +Y -> EE -/+Y  (left: surface faces -Y, right: +Y)
  STL +Z -> EE +/-X

  R_left  det=+1    R_right det=+1
Opening: each finger is offset by +/-half_gap along the EE Y axis.
"""

import numpy as np
import pyvista as pv
from pyvistaqt import BackgroundPlotter
from pathlib import Path

LINK_COLORS = [
    "#2b2b2b", "#E85D24", "#E85D24", "#E85D24",
    "#E85D24", "#E85D24", "#c0c0c0",
]
FINGER_COLOR   = "#7ecbff"
AXIS_LENGTH_MM = 140
AXIS_RADIUS    = 4

_R_LEFT = np.array([
    [0,  0,  1,  0],
    [0, -1,  0,  0],
    [1,  0,  0,  0],
    [0,  0,  0,  1],
], dtype=float)

_R_RIGHT = np.array([
    [0,  0, -1,  0],
    [0,  1,  0,  0],
    [1,  0,  0,  0],
    [0,  0,  0,  1],
], dtype=float)

_R_LEFT_VISUAL = np.array([
    [1,  0,  0,  0],
    [0, -1,  0,  0],
    [0,  0, -1,  0],
    [0,  0,  0,  1],
], dtype=float)

_R_RIGHT_VISUAL = np.eye(4)

_LINK6_FLANGE_MOUNT = np.array([
    [1, 0, 0, 990.0],
    [0, 1, 0,   0.0],
    [0, 0, 1, 1301.0],
    [0, 0, 0,   1.0],
], dtype=float)

_MOUNT_Z = 0.0     # Frame 6 already includes the wrist/tool length in DH.
_FINGER_X_OFFSET = 92.0
_TCP_X_OFFSET = _FINGER_X_OFFSET + 96.0
_GAP_MIN = 11.0    # half opening at 0%  -> fingertips nearly closed
_GAP_MAX = 42.5    # half opening at 100% -> 85 mm Robotiq-style stroke

_DH = [
    [150,  np.pi / 2,  486],
    [700,  0.0,          0],
    [115,  np.pi / 2,    0],
    [  0, -np.pi / 2,  625],
    [  0,  np.pi / 2,    0],
    [  0,  0.0,        100],
]
Q_HOME = np.deg2rad([0.0, 90.0, 0.0, 0.0, 0.0, 0.0])


def _dh_mat(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [ 0,     sa,     ca,    d],
        [ 0,      0,      0,    1],
    ])


def _partial_fk(q, n):
    T = np.eye(4)
    for i in range(n):
        a, alpha, d = _DH[i]
        T = T @ _dh_mat(q[i], d, a, alpha)
    return T


_T0_INV = [np.eye(4)]
for _i in range(1, 7):
    _T0_INV.append(np.linalg.inv(_partial_fk(Q_HOME, _i)))


def _axis_arrow(direction, length, radius):
    tip_l = 0.22
    shaft = pv.Cylinder(center=(0,0,length*(1-tip_l)/2), direction=(0,0,1),
                        radius=radius, height=length*(1-tip_l), resolution=12)
    tip   = pv.Cone(center=(0,0,length*(1-tip_l/2)), direction=(0,0,1),
                    height=length*tip_l, radius=radius*2.2, resolution=12)
    arrow = shaft.merge(tip)
    z = np.array([0.,0.,1.]); d = np.array(direction,float); d /= np.linalg.norm(d)
    cross = np.cross(z, d); dot = np.dot(z, d)
    if np.linalg.norm(cross) < 1e-9:
        if dot < 0: arrow.rotate_x(180., inplace=True)
    else:
        arrow.rotate_vector(cross/np.linalg.norm(cross),
                            np.degrees(np.arccos(np.clip(dot,-1,1))), inplace=True)
    return arrow


_AX_X = _axis_arrow([1,0,0], AXIS_LENGTH_MM, AXIS_RADIUS)
_AX_Y = _axis_arrow([0,1,0], AXIS_LENGTH_MM, AXIS_RADIUS)
_AX_Z = _axis_arrow([0,0,1], AXIS_LENGTH_MM, AXIS_RADIUS)
_AX_O = pv.Sphere(radius=AXIS_RADIUS*2.)



class RobotVisualizer:

    def __init__(self):
        self.plotter = BackgroundPlotter(
            title="ABB IRB 1600 — Digital Twin",
            window_size=(900, 700),
        )
        # Hide PyVista's own menu bar and toolbars — they duplicate our main window
        self.plotter.app_window.menuBar().hide()
        from PyQt5.QtWidgets import QToolBar
        for tb in self.plotter.app_window.findChildren(QToolBar):
            tb.hide()
        self.link_actors      = []
        self._trail_actor     = None
        self.ee_trail_pts     = []
        self._show_frames     = False
        self._frame_actors    = []
        self._gripper_type    = None   # 'stl_pair' | 'stl_mirrored' | 'primitive'
        self._gripper_model_name = ""
        self._gripper_actors  = []
        self._gripper_base_actor = None
        self._gripper_open_pct = 100.0
        self._tcp_actor = None
        self._workspace_actor = None
        self._object_actor = None
        self._object_pose = np.eye(4)
        self._object_grasped = False
        self._object_grasp_offset = np.eye(4)
        self._traj_preview_actor  = None
        self._traj_preview_markers = []
        self._setup_scene()

    def _setup_scene(self):
        self.plotter.set_background("#1a1a2e")
        self.plotter.add_axes(line_width=2)
        ground = pv.Plane(center=(0,0,0), i_size=4000, j_size=4000)
        self.plotter.add_mesh(ground, color="#2a2a3e", opacity=0.5, show_edges=False)
        self.plotter.camera_position = [
            (3000,-2500,2000), (400,0,800), (0,0,1)
        ]
        self._setup_workspace()
        self._setup_pick_object()

    def _setup_workspace(self):
        sphere = pv.Sphere(radius=1450, center=(150, 0, 486),
                           theta_resolution=64, phi_resolution=32)
        self._workspace_actor = self.plotter.add_mesh(
            sphere, color="#58a6ff", opacity=0.08, style="wireframe",
            render=False, reset_camera=False)
        self._workspace_actor.SetVisibility(False)

    def _setup_pick_object(self):
        obj = pv.Box(bounds=(-35, 35, -35, 35, -35, 35))
        self._object_pose = np.eye(4)
        self._object_pose[:3, 3] = [1200.0, 0.0, 1265.0]
        self._object_actor = self.plotter.add_mesh(
            obj, color="#d8b15a", smooth_shading=True,
            specular=0.25, specular_power=10,
            render=False, reset_camera=False)
        self._object_actor.user_matrix = self._object_pose
        self._object_actor.SetVisibility(False)   # hidden until user places it

    def load_meshes(self, meshes):
        self.link_actors = []
        for i, mesh in enumerate(meshes):
            a = self.plotter.add_mesh(
                mesh, name=f"link_{i}",
                color=LINK_COLORS[i % len(LINK_COLORS)],
                smooth_shading=True, specular=0.5, specular_power=20,
            )
            self.link_actors.append(a)

    def update_transforms(self, transforms, render: bool = True):
        for i, actor in enumerate(self.link_actors):
            actor.user_matrix = (np.eye(4) if i == 0
                                 else transforms[i-1] @ _T0_INV[i]
                                 if (i-1) < len(transforms) else np.eye(4))
        if self._show_frames and self._frame_actors:
            self._reposition_frames(transforms)
        if render:
            self.plotter.render()

    def _build_frame_actors(self):
        for _ in range(7):
            self._frame_actors.append(
                self.plotter.add_mesh(_AX_O.copy(), color="white",
                                      render=False, reset_camera=False))
            for mesh, col in zip((_AX_X,_AX_Y,_AX_Z),
                                  ("#ff4444","#44ff44","#4499ff")):
                self._frame_actors.append(
                    self.plotter.add_mesh(mesh.copy(), color=col,
                                          render=False, reset_camera=False))
        for a in self._frame_actors:
            a.SetVisibility(False)

    def _reposition_frames(self, transforms):
        for f, T in enumerate([np.eye(4)] + list(transforms)):
            for j in range(4):
                self._frame_actors[f*4+j].user_matrix = T

    def toggle_frames(self, visible: bool):
        if visible and not self._frame_actors:
            self._build_frame_actors()
        self._show_frames = visible
        for a in self._frame_actors:
            a.SetVisibility(visible)
        self.plotter.render()

    def set_workspace_visible(self, visible: bool):
        if self._workspace_actor is not None:
            self._workspace_actor.SetVisibility(visible)
        self.plotter.render()

    def load_gripper(self,
                     stl_left:  str | None = None,
                     stl_right: str | None = None,
                     stl_base:  str | None = None):
        """
        Load gripper STL files and prepare them for the EE frame.

        Mode A: stl_left + stl_right  -- separate left/right Finray STLs
        Mode B: stl_left only         -- right finger derived by Y-mirror
        Mode C: neither               -- box primitive fallback
        """
        for a in self._gripper_actors:
            self.plotter.remove_actor(a, render=False)
        self._gripper_actors = []
        if self._gripper_base_actor is not None:
            self.plotter.remove_actor(self._gripper_base_actor, render=False)
            self._gripper_base_actor = None

        left_ok  = stl_left  and Path(stl_left).exists()
        right_ok = stl_right and Path(stl_right).exists()
        base_ok  = stl_base  and Path(stl_base).exists()

        def _load_stl(path):
            mesh = pv.read(str(path))
            # Automatic unit detection
            ext = max(mesh.bounds[2*i+1] - mesh.bounds[2*i] for i in range(3))
            if ext > 1000:                              # metres to mm
                mesh.scale([1000.,1000.,1000.], inplace=True)
            # Finray STL anchor:
            #   local +X is the finger length, so Xmin is the mounting root;
            #   local Y=0 is the gripping face;
            #   local Z is thickness, centered to keep the pair on the TCP axis.
            xmin, _, _, _, zmin, zmax = mesh.bounds
            anchor = np.array([xmin, 0.0, 0.5 * (zmin + zmax)])
            mesh.points = mesh.points - anchor
            return mesh

        def _load_base_stl(path):
            mesh = pv.read(str(path))
            ext = max(mesh.bounds[2*i+1] - mesh.bounds[2*i] for i in range(3))
            if ext > 1000:
                mesh.scale([1000.,1000.,1000.], inplace=True)
            return mesh

        if base_ok:
            self._gripper_base_actor = self.plotter.add_mesh(
                _load_base_stl(stl_base), color="#8a929a",
                smooth_shading=True, specular=0.35,
                specular_power=15, render=False)

        if left_ok and right_ok:
            for path in (stl_left, stl_right):
                a = self.plotter.add_mesh(
                    _load_stl(path), color=FINGER_COLOR,
                    smooth_shading=True, render=False)
                self._gripper_actors.append(a)
            self._gripper_type = "stl_pair"
            self._gripper_model_name = (
                "Robotiq 2F-85 proxy" if "robotiq_2f85" in Path(stl_left).name
                else "STL pair")
            print(f"  Gripper: {Path(stl_left).name} + {Path(stl_right).name}")

        elif left_ok:
            mesh_l = _load_stl(stl_left)
            mesh_r = mesh_l.copy()
            pts = mesh_r.points.copy(); pts[:,1] *= -1
            mesh_r.points = pts; mesh_r.flip_normals()
            for m in (mesh_l, mesh_r):
                a = self.plotter.add_mesh(m, color=FINGER_COLOR,
                                          smooth_shading=True, render=False)
                self._gripper_actors.append(a)
            self._gripper_type = "stl_mirrored"
            self._gripper_model_name = "Mirrored STL"
            print(f"  Gripper (mirrored): {Path(stl_left).name}")

        else:
            for _ in range(2):
                a = self.plotter.add_mesh(
                    pv.Box(bounds=(-6,6,0,15,0,50)), color=FINGER_COLOR,
                    smooth_shading=True, opacity=0.9, render=False)
                self._gripper_actors.append(a)
            self._gripper_type = "primitive"
            self._gripper_model_name = "Primitive placeholder"
            print("  Gripper: primitive placeholder")

        self.plotter.render()

    def update_gripper(self, opening_pct: float, T_ee: np.ndarray):
        """
        Lock the gripper to the EE frame and apply the opening animation.

        Actor[0] = left finger  -> +Y direction, rotated by R_LEFT
        Actor[1] = right finger -> -Y direction, rotated by R_RIGHT
        Z offset aligns the mounting root to the EE flange.
        """
        if not self._gripper_actors and self._gripper_base_actor is None:
            return

        self._gripper_open_pct = float(np.clip(opening_pct, 0, 100))
        half_gap = _GAP_MIN + (self._gripper_open_pct / 100.) * (_GAP_MAX - _GAP_MIN)

        if self._gripper_type in ("stl_pair", "stl_mirrored"):
            if self._gripper_base_actor is not None:
                T_mount = T_ee @ _T0_INV[6] @ _LINK6_FLANGE_MOUNT
                self._gripper_base_actor.user_matrix = T_mount
                rotations = [_R_LEFT_VISUAL, _R_RIGHT_VISUAL]
                base_matrix = T_mount
            else:
                rotations = [_R_LEFT, _R_RIGHT]
                base_matrix = T_ee
            for actor, R_base, side in zip(
                    self._gripper_actors, rotations, [1, -1]):
                T = R_base.copy()
                T[0, 3] = _FINGER_X_OFFSET
                T[1, 3] = side * half_gap   # Y: finger separation
                T[2, 3] = _MOUNT_Z          # Z: mounting alignment
                actor.user_matrix = base_matrix @ T

        elif self._gripper_type == "primitive":
            if self._gripper_base_actor is not None:
                self._gripper_base_actor.user_matrix = T_ee
            for idx, actor in enumerate(self._gripper_actors):
                T = np.eye(4)
                T[1, 3] = (1 if idx == 0 else -1) * half_gap
                actor.user_matrix = T_ee @ T

        self.update_tcp_marker(T_ee)
        if self._object_grasped:
            self._object_pose = self.tcp_transform(T_ee) @ self._object_grasp_offset
            self._object_actor.user_matrix = self._object_pose

    def set_gripper_visible(self, visible: bool):
        for a in self._gripper_actors:
            a.SetVisibility(visible)
        if self._gripper_base_actor is not None:
            self._gripper_base_actor.SetVisibility(visible)
        self.plotter.render()

    def gripper_loaded(self) -> bool:
        return bool(self._gripper_actors) or self._gripper_base_actor is not None

    def tool_mount_transform(self, T_ee: np.ndarray) -> np.ndarray:
        if self._gripper_base_actor is not None:
            return T_ee @ _T0_INV[6] @ _LINK6_FLANGE_MOUNT
        return T_ee

    def tcp_transform(self, T_ee: np.ndarray) -> np.ndarray:
        T = np.eye(4)
        T[0, 3] = _TCP_X_OFFSET if self._gripper_base_actor is not None else 0.0
        return self.tool_mount_transform(T_ee) @ T

    def tcp_position(self, T_ee: np.ndarray) -> np.ndarray:
        return self.tcp_transform(T_ee)[:3, 3]

    def ee_from_tcp_target(self, T_tcp_desired: np.ndarray) -> np.ndarray:
        T = np.eye(4)
        T[0, 3] = _TCP_X_OFFSET if self._gripper_base_actor is not None else 0.0
        if self._gripper_base_actor is not None:
            return T_tcp_desired @ np.linalg.inv(T) @ np.linalg.inv(_LINK6_FLANGE_MOUNT) @ _partial_fk(Q_HOME, 6)
        return T_tcp_desired

    def update_tcp_marker(self, T_ee: np.ndarray):
        if self._tcp_actor is None:
            marker = pv.Sphere(radius=14)
            self._tcp_actor = self.plotter.add_mesh(
                marker, color="#ffffff", render=False, reset_camera=False)
        self._tcp_actor.user_matrix = self.tcp_transform(T_ee)

    def object_position(self) -> np.ndarray:
        return self._object_pose[:3, 3].copy()

    def reset_pick_object(self):
        self._object_grasped = False
        self._object_pose = np.eye(4)
        self._object_pose[:3, 3] = [1200.0, 0.0, 1265.0]
        if self._object_actor is not None:
            self._object_actor.user_matrix = self._object_pose
        self.plotter.render()

    def set_object_position(self, pos: np.ndarray):
        self._object_grasped = False
        self._object_pose = np.eye(4)
        self._object_pose[:3, 3] = pos.copy()
        if self._object_actor is not None:
            self._object_actor.user_matrix = self._object_pose
            self._object_actor.SetVisibility(True)
        self.plotter.render()

    def set_object_visible(self, visible: bool):
        if self._object_actor is not None:
            self._object_actor.SetVisibility(visible)
        if not visible:
            self._object_grasped = False
        self.plotter.render()

    def update_grasp_simulation(self, T_ee: np.ndarray, opening_pct: float) -> str:
        if self._object_actor is None:
            return "No object"
        if not self._object_actor.GetVisibility():
            return "Object hidden"
        T_tcp = self.tcp_transform(T_ee)
        tcp_pos = T_tcp[:3, 3]
        obj_pos = self.object_position()
        distance = float(np.linalg.norm(tcp_pos - obj_pos))
        if self._object_grasped:
            if opening_pct > 70:
                self._object_grasped = False
                return "Released"
            self._object_pose = T_tcp @ self._object_grasp_offset
            self._object_actor.user_matrix = self._object_pose
            return "Holding object"
        if opening_pct < 20 and distance < 90:
            self._object_grasped = True
            self._object_grasp_offset = np.linalg.inv(T_tcp) @ self._object_pose
            self._object_pose = T_tcp @ self._object_grasp_offset
            self._object_actor.user_matrix = self._object_pose
            return "Object grasped"
        return f"Object distance: {distance:.0f} mm"

    def add_ee_point(self, position):
        self.ee_trail_pts.append(position.copy())
        n = len(self.ee_trail_pts)
        if n < 2:
            return
        pts    = np.array(self.ee_trail_pts)
        spline = pv.Spline(pts, n_points=max(n * 3, 10))

        # Per-point speed (mm/frame) → colour: cyan = slow, magenta = fast
        v_orig    = np.zeros(n)
        for i in range(1, n):
            v_orig[i] = np.linalg.norm(pts[i] - pts[i - 1])
        v_orig[0] = v_orig[1]

        t_orig   = np.linspace(0.0, 1.0, n)
        t_spline = np.linspace(0.0, 1.0, spline.n_points)
        spline['speed'] = np.interp(t_spline, t_orig, v_orig)

        if self._trail_actor is not None:
            self.plotter.remove_actor(self._trail_actor, render=False)
        self._trail_actor = self.plotter.add_mesh(
            spline, scalars='speed', cmap='cool', line_width=2,
            show_scalar_bar=False, render=False, reset_camera=False)

    def clear_trail(self):
        self.ee_trail_pts = []
        if self._trail_actor is not None:
            self.plotter.remove_actor(self._trail_actor)
            self._trail_actor = None

    def draw_trajectory_preview(self, tcp_positions: list):
        """Draw a dashed path and waypoint markers for the planned trajectory."""
        self.clear_trajectory_preview()
        if len(tcp_positions) < 2:
            return
        pts = np.array(tcp_positions)
        spline = pv.Spline(pts, n_points=max(len(pts) * 8, 20))
        self._traj_preview_actor = self.plotter.add_mesh(
            spline, color="#ffaa00", line_width=3,
            render=False, reset_camera=False)
        for pos in tcp_positions:
            m = self.plotter.add_mesh(
                pv.Sphere(radius=14, center=pos),
                color="#ffaa00", opacity=0.85,
                render=False, reset_camera=False)
            self._traj_preview_markers.append(m)
        self.plotter.render()

    def clear_trajectory_preview(self):
        if self._traj_preview_actor is not None:
            self.plotter.remove_actor(self._traj_preview_actor, render=False)
            self._traj_preview_actor = None
        for a in self._traj_preview_markers:
            self.plotter.remove_actor(a, render=False)
        self._traj_preview_markers = []
        self.plotter.render()

    def get_qt_widget(self):
        return self.plotter.app_window



def load_link_meshes(cad_folder):
    meshes   = []
    cad_path = Path(cad_folder)
    for i in range(7):
        path = cad_path / f"link{i}.stl"
        if path.exists():
            mesh = pv.read(str(path))
            mesh.scale([1000.,1000.,1000.], inplace=True)
            meshes.append(mesh)
            c = mesh.center
            print(f"  link{i}.stl  centre ({c[0]:.0f}, {c[1]:.0f}, {c[2]:.0f}) mm")
        else:
            print(f"  WARNING: link{i}.stl not found")
            meshes.append(pv.PolyData())
    return meshes
