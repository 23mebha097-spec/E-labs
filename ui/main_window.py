from PyQt5 import QtWidgets, QtCore, QtGui
from graphics.canvas import RobotCanvas
from core.robot import Robot
from ui.panels.align_panel import AlignPanel
from ui.panels.joint_panel import JointPanel
from ui.panels.experiment_panel import ExperimentPanel
from ui.panels.program_panel import ProgramPanel
from ui.panels.gripper_panel import GripperPanel
from ui.panels.ik_fk_panel import IKFKPanel
import os
import numpy as np
import random
from ui.widgets.code_drawer import CodeDrawer
from core.firmware_gen import generate_esp32_firmware

from ui.mixins.links_mixin import LinksMixin
from ui.mixins.navigation_mixin import NavigationMixin
from ui.mixins.project_mixin import ProjectMixin

class TypeOnlyDoubleSpinBox(QtWidgets.QDoubleSpinBox):
    def stepBy(self, steps): pass
    def wheelEvent(self, event): event.ignore()

class TypeOnlySpinBox(QtWidgets.QSpinBox):
    def stepBy(self, steps): pass
    def wheelEvent(self, event): event.ignore()

from core.path_planner import WorkspacePlan
from core.import_units import get_engine_units_per_cm

class WorkspaceCalculatorThread(QtCore.QThread):
    finished = QtCore.pyqtSignal(dict)
    
    def __init__(self, robot, tcp_link_name, plane_mode="inclined", workspace_points=None, parent=None):
        super().__init__(parent)
        self.robot = robot
        self.tcp_link_name = tcp_link_name
        self.plane_mode = str(plane_mode or "inclined")
        self.workspace_points = np.array(workspace_points, dtype=float) if workspace_points is not None else None
        self.units_per_cm = get_engine_units_per_cm() or 1.0

    def _get_home_plane_point_cm(self, tcp_link_obj, plane_rotation):
        old_values = {name: joint.current_value for name, joint in self.robot.joints.items()}
        try:
            for joint in self.robot.joints.values():
                zero_home = float(np.clip(0.0, joint.min_limit, joint.max_limit))
                joint.current_value = zero_home
            self.robot.update_kinematics()

            base_world = np.array(self.robot.base_link.t_world, dtype=float)
            tcp_world = self.robot.get_tcp_world_pose(tcp_link_obj)[:3, 3] / self.units_per_cm
            inv_base = np.linalg.inv(base_world)
            tcp_world_h = np.append(tcp_world * self.units_per_cm, 1.0)
            tcp_base = (inv_base @ tcp_world_h)[:3] / self.units_per_cm
            return tcp_base @ plane_rotation
        finally:
            for name, value in old_values.items():
                self.robot.joints[name].current_value = value
            self.robot.update_kinematics()

    def _board_sample_points(self, width_cm, height_cm, spacing_cm=8.0):
        nx = max(3, min(7, int(np.ceil(width_cm / spacing_cm)) + 1))
        ny = max(3, min(7, int(np.ceil(height_cm / spacing_cm)) + 1))
        xs = np.linspace(0.0, float(width_cm), nx)
        ys = np.linspace(0.0, float(height_cm), ny)
        return [np.array([x, y, 0.0], dtype=float) for y in ys for x in xs]

    def _board_point_to_world(self, origin_base_cm, local_pt_cm, plane_rotation, base_world_cm):
        point_base_cm = np.array(origin_base_cm, dtype=float) + plane_rotation @ np.array(local_pt_cm, dtype=float)
        point_world_cm = (base_world_cm @ np.append(point_base_cm, 1.0))[:3]
        return point_base_cm, point_world_cm * self.units_per_cm

    def _validate_board_reachability(
        self,
        tcp_link_obj,
        origin_base_cm,
        width_cm,
        height_cm,
        plane_rotation,
        base_world_cm,
    ):
        old_values = {name: joint.current_value for name, joint in self.robot.joints.items()}
        target_pose = self.robot.get_tcp_world_pose(tcp_link_obj).copy()
        samples = self._board_sample_points(width_cm, height_cm)
        checked = 0
        try:
            for local_pt in samples:
                if self.isInterruptionRequested():
                    return False, checked, "cancelled"

                point_base_cm, point_world = self._board_point_to_world(
                    origin_base_cm,
                    local_pt,
                    plane_rotation,
                    base_world_cm,
                )
                if np.any(point_base_cm < -1e-6):
                    return False, checked, "outside_positive_octant"

                target_pose[:3, 3] = point_world
                success, _ = self.robot.inverse_kinematics_pose(
                    target_pose,
                    tcp_link_obj,
                    max_iters=90,
                    position_tolerance=max(0.8 * self.units_per_cm, 0.4),
                    orientation_tolerance=1e6,
                    orientation_weight=0.0,
                    joint_change_weight=0.04,
                )
                checked += 1
                if not success:
                    return False, checked, "ik_failed"
            return True, checked, "ok"
        finally:
            for name, value in old_values.items():
                self.robot.joints[name].current_value = value
            self.robot.update_kinematics()

    def _candidate_origins(self, plane_rotation, base_world, base_world_cm, tcp_link_obj, width_cm, workspace_points_base_cm=None):
        origins = [
            np.array([2.0, 0.0, 0.0], dtype=float),
            np.array([max(2.0, width_cm * 0.1), 0.0, 0.0], dtype=float),
        ]

        try:
            tcp_world = self.robot.get_tcp_world_pose(tcp_link_obj)[:3, 3] / self.units_per_cm
            inv_base = np.linalg.inv(base_world)
            tcp_base = (inv_base @ np.append(tcp_world * self.units_per_cm, 1.0))[:3] / self.units_per_cm
            origins.append(np.maximum(tcp_base - plane_rotation @ np.array([width_cm * 0.5, 2.0, 0.0]), 0.0))
        except Exception:
            pass

        if workspace_points_base_cm is not None and len(workspace_points_base_cm):
            positive_pts = workspace_points_base_cm[np.all(workspace_points_base_cm >= -1e-6, axis=1)]
            if len(positive_pts):
                norms = np.linalg.norm(positive_pts, axis=1)
                for idx in np.argsort(norms)[:8]:
                    pt = positive_pts[idx]
                    origins.append(np.maximum(pt - plane_rotation @ np.array([width_cm * 0.25, 0.0, 0.0]), 0.0))

        unique = []
        for origin in origins:
            origin = np.maximum(np.array(origin, dtype=float), 0.0)
            if not any(np.linalg.norm(origin - existing) < 1.0 for existing in unique):
                unique.append(origin)
        return sorted(unique, key=lambda p: float(np.linalg.norm(p)))

    def _score_board_candidate(self, origin_base_cm, width_cm, height_cm, plane_rotation, reach_cm):
        center_base = np.array(origin_base_cm, dtype=float) + plane_rotation @ np.array(
            [width_cm / 2.0, height_cm / 2.0, 0.0],
            dtype=float,
        )
        center_distance = float(np.linalg.norm(center_base))
        ideal_distance = max(8.0, 0.45 * reach_cm)
        distance_score = abs(center_distance - ideal_distance) / max(ideal_distance, 1.0)
        too_close_penalty = max(0.0, (0.18 * reach_cm - center_distance) / max(0.18 * reach_cm, 1.0))
        too_far_penalty = max(0.0, (center_distance - 0.82 * reach_cm) / max(0.82 * reach_cm, 1.0))
        area_score = (width_cm * height_cm) / max(reach_cm * reach_cm, 1.0)
        aspect_penalty = abs((width_cm / max(height_cm, 1e-6)) - 1.35) * 0.18
        positive_penalty = float(np.count_nonzero(np.array(origin_base_cm) < -1e-6)) * 10.0
        return (
            area_score * 2.0
            - distance_score
            - (1.5 * too_close_penalty)
            - (1.2 * too_far_penalty)
            - aspect_penalty
            - positive_penalty
        )

    def _candidate_boards(
        self,
        plane_rotation,
        base_world,
        base_world_cm,
        tcp_link_obj,
        reach_cm,
        base_width,
        base_height,
        min_board_cm,
        workspace_points_base_cm=None,
    ):
        boards = []
        width_cap = max(min_board_cm, min(50.0, 0.55 * reach_cm))
        height_cap = max(min_board_cm, min(38.0, 0.42 * reach_cm))
        width_seed = max(min_board_cm, min(float(base_width), width_cap))
        height_seed = max(min_board_cm, min(float(base_height), height_cap))

        sizes = []
        for scale in (1.0, 0.88, 0.75, 0.62, 0.5, 0.38, 0.28, 0.2, 0.14):
            width_cm = max(min_board_cm, width_seed * scale)
            height_cm = max(min_board_cm, height_seed * scale)
            if width_cm < min_board_cm + 0.2 or height_cm < min_board_cm + 0.2:
                continue
            if not any(abs(width_cm - w) < 0.5 and abs(height_cm - h) < 0.5 for w, h in sizes):
                sizes.append((width_cm, height_cm))

        try:
            tcp_world = self.robot.get_tcp_world_pose(tcp_link_obj)[:3, 3] / self.units_per_cm
            inv_base = np.linalg.inv(base_world)
            tcp_base = (inv_base @ np.append(tcp_world * self.units_per_cm, 1.0))[:3] / self.units_per_cm
            tcp_plane = tcp_base @ plane_rotation
        except Exception:
            tcp_plane = np.array([0.35 * reach_cm, 0.0, 0.0], dtype=float)

        center_hints = [
            np.array([max(0.18 * reach_cm, tcp_plane[0]), max(0.12 * reach_cm, tcp_plane[1] + 0.15 * reach_cm), tcp_plane[2]], dtype=float),
            np.array([0.38 * reach_cm, 0.18 * reach_cm, tcp_plane[2]], dtype=float),
            np.array([0.48 * reach_cm, 0.16 * reach_cm, tcp_plane[2]], dtype=float),
            np.array([max(4.0, tcp_plane[0]), max(2.0, tcp_plane[1] + 4.0), tcp_plane[2]], dtype=float),
        ]

        if workspace_points_base_cm is not None and len(workspace_points_base_cm):
            pts_plane = workspace_points_base_cm @ plane_rotation
            mask = np.all(workspace_points_base_cm >= -1e-6, axis=1)
            if np.any(mask):
                reachable_xy = pts_plane[mask][:, :2]
                for pct_x, pct_y in ((50, 50), (55, 45), (45, 55), (60, 50), (50, 60)):
                    center_hints.append(
                        np.array([
                            float(np.percentile(reachable_xy[:, 0], pct_x)),
                            float(np.percentile(reachable_xy[:, 1], pct_y)),
                            tcp_plane[2],
                        ], dtype=float)
                    )

        for width_cm, height_cm in sizes:
            for center_plane in center_hints:
                origin_plane = np.array([
                    center_plane[0] - (width_cm / 2.0),
                    center_plane[1] - (height_cm / 2.0),
                    center_plane[2],
                ], dtype=float)
                origin_base = plane_rotation @ origin_plane
                if np.any(origin_base < -1e-6):
                    origin_base = np.maximum(origin_base, 0.0)
                score = self._score_board_candidate(origin_base, width_cm, height_cm, plane_rotation, reach_cm)
                boards.append((score, origin_base, width_cm, height_cm))

            for origin in self._candidate_origins(
                plane_rotation,
                base_world,
                base_world_cm,
                tcp_link_obj,
                width_cm,
                workspace_points_base_cm=workspace_points_base_cm,
            ):
                score = self._score_board_candidate(origin, width_cm, height_cm, plane_rotation, reach_cm)
                boards.append((score, origin, width_cm, height_cm))

        unique = []
        for score, origin, width_cm, height_cm in sorted(boards, key=lambda item: item[0], reverse=True):
            if any(
                abs(width_cm - w) < 0.5
                and abs(height_cm - h) < 0.5
                and np.linalg.norm(origin - existing_origin) < 1.0
                for _, existing_origin, w, h in unique
            ):
                continue
            unique.append((score, origin, width_cm, height_cm))
            if len(unique) >= 36:
                break
        return unique
        
    def run(self):
        try:
            if not self.robot or not self.robot.base_link:
                self.finished.emit({"error": "No robot or base link"})
                return
                
            tcp_link_obj = None
            if self.tcp_link_name and self.tcp_link_name in self.robot.links:
                tcp_link_obj = self.robot.links[self.tcp_link_name]
            else:
                self.finished.emit({"error": "Invalid TCP link"})
                return
                
            reach = sum(np.linalg.norm(j.origin) for j in self.robot.joints.values())
            if reach < 10.0:
                reach = 30.0 * self.units_per_cm
            reach_cm = reach / self.units_per_cm

            base_world = np.array(self.robot.base_link.t_world, dtype=float)
            base_world_cm = base_world.copy()
            base_world_cm[:3, 3] = base_world_cm[:3, 3] / self.units_per_cm
            seed_plan = WorkspacePlan(origin=np.zeros(3), plane_mode=self.plane_mode)
            plane_rotation = seed_plan.t_workspace_base[:3, :3]
            home_plane_pt = self._get_home_plane_point_cm(tcp_link_obj, plane_rotation)

            best_center = None
            best_origin = None
            best_size = (
                max(18.0, min(42.0, 0.35 * reach_cm)),
                max(14.0, min(30.0, 0.25 * reach_cm)),
            )
            best_validation_count = 0
            source = "kinematic_estimate"
            workspace_points_base_cm = None

            if self.workspace_points is not None and len(self.workspace_points) >= 8:
                inv_base = np.linalg.inv(base_world)
                pts_h = np.hstack([self.workspace_points, np.ones((len(self.workspace_points), 1))])
                workspace_points_base_cm = (pts_h @ inv_base.T)[:, :3] / self.units_per_cm
                pts_plane = workspace_points_base_cm @ plane_rotation

                slice_pts = None
                for normal_tol_cm in (2.0, 3.5, 5.0, 7.5, 10.0):
                    candidate = pts_plane[
                        np.abs(pts_plane[:, 2] - home_plane_pt[2]) <= normal_tol_cm
                    ]
                    if len(candidate) >= 12:
                        slice_pts = candidate
                        break
                if slice_pts is None:
                    nearest = np.argsort(np.abs(pts_plane[:, 2] - home_plane_pt[2]))
                    take_n = min(len(nearest), 32)
                    slice_pts = pts_plane[nearest[:take_n]]

                q_low = np.percentile(slice_pts[:, :2], 5, axis=0)
                q_high = np.percentile(slice_pts[:, :2], 95, axis=0)
                x_offsets = slice_pts[:, 0] - home_plane_pt[0]
                y_offsets = slice_pts[:, 1] - home_plane_pt[1]

                left_reach = max(6.0, float(np.percentile(np.clip(-x_offsets, 0.0, None), 85)))
                right_reach = max(6.0, float(np.percentile(np.clip(x_offsets, 0.0, None), 85)))
                upward_reach = max(10.0, float(np.percentile(np.clip(y_offsets, 0.0, None), 88)))
                bottom_margin_cm = 2.5

                board_w = float(max(14.0, left_reach + right_reach))
                board_h = float(max(14.0, upward_reach + bottom_margin_cm))
                center_plane = np.array([
                    home_plane_pt[0] + (right_reach - left_reach) / 2.0,
                    home_plane_pt[1] + (board_h / 2.0) - bottom_margin_cm,
                    home_plane_pt[2],
                ], dtype=float)

                half_w = board_w / 2.0
                half_h = board_h / 2.0
                best_center = plane_rotation @ center_plane
                best_size = (board_w, board_h)
                origin_plane = np.array([
                    center_plane[0] - half_w,
                    center_plane[1] - half_h,
                    center_plane[2],
                ], dtype=float)
                best_origin = np.maximum(plane_rotation @ origin_plane, 0.0)
                inside = (
                    (np.abs(slice_pts[:, 0] - center_plane[0]) <= half_w)
                    & (np.abs(slice_pts[:, 1] - center_plane[1]) <= half_h)
                )
                best_validation_count = int(np.count_nonzero(inside))
                source = "home_anchored_plane"

            if best_center is None:
                best_center = plane_rotation @ np.array([0.55 * reach_cm, 0.0, home_plane_pt[2]], dtype=float)
                best_origin = np.array([max(2.0, 0.05 * reach_cm), 0.0, 0.0], dtype=float)
            elif best_origin is None:
                best_origin = np.maximum(
                    best_center - np.array([best_size[0] / 2.0, 0.0, 0.0], dtype=float),
                    0.0,
                )

            min_board_cm = max(2.0, min(6.0, 0.08 * reach_cm))
            board_candidates = self._candidate_boards(
                plane_rotation,
                base_world,
                base_world_cm,
                tcp_link_obj,
                reach_cm,
                best_size[0],
                best_size[1],
                min_board_cm,
                workspace_points_base_cm=workspace_points_base_cm,
            )

            reachable_board = None
            failed_reason = "not_checked"
            checked_total = 0
            attempted_boards = 0
            best_candidate_score = None
            for score, origin, width_cm, height_cm in board_candidates:
                if self.isInterruptionRequested():
                    self.finished.emit({"cancelled": True})
                    return

                ok, checked, reason = self._validate_board_reachability(
                    tcp_link_obj,
                    origin,
                    width_cm,
                    height_cm,
                    plane_rotation,
                    base_world_cm,
                )
                checked_total += checked
                attempted_boards += 1
                failed_reason = reason
                if reason == "cancelled" or self.isInterruptionRequested():
                    self.finished.emit({"cancelled": True})
                    return
                if ok:
                    reachable_board = (origin, width_cm, height_cm, checked)
                    best_candidate_score = score
                    break

            if reachable_board is None:
                self.finished.emit({
                    "error": (
                        "No chalkboard plane found where every sampled point is reachable "
                        f"(last check: {failed_reason}, boards tried: {attempted_boards}, "
                        f"samples checked: {checked_total}, candidates: {len(board_candidates)}, "
                        f"smallest board: {min_board_cm:.1f} cm)."
                    )
                })
                return

            best_origin, board_w, board_h, verified_samples = reachable_board
            best_size = (board_w, board_h)
            best_center = best_origin + plane_rotation @ np.array([board_w / 2.0, board_h / 2.0, 0.0], dtype=float)
            best_validation_count = int(verified_samples)
            source = f"{source}_ik_verified"
                    
            if best_center is not None:
                grid_size_cm = 10.0
                cells_x = max(1, int(np.floor(best_size[0] / grid_size_cm)))
                cells_y = max(1, int(np.floor(best_size[1] / grid_size_cm)))
                self.finished.emit({
                    "best_center": best_center.tolist(),
                    "best_origin": best_origin.tolist(),
                    "best_size": best_size,
                    "validation_count": best_validation_count,
                    "plane_mode": self.plane_mode,
                    "reach_cm": reach_cm,
                    "source": source,
                    "grid_size_cm": grid_size_cm,
                    "cells_x": cells_x,
                    "cells_y": cells_y,
                    "cell_count": int(cells_x * cells_y),
                    "home_plane_point": home_plane_pt.tolist(),
                    "attempted_boards": attempted_boards,
                    "min_board_cm": min_board_cm,
                    "board_score": best_candidate_score,
                })
            else:
                self.finished.emit({"error": "No reachable workspace found"})
        except Exception as e:
            self.finished.emit({"error": str(e)})


class MainWindow(QtWidgets.QMainWindow, LinksMixin, NavigationMixin, ProjectMixin):
    log_signal = QtCore.pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("E-lab - Programmable 3-D Robotic Assembly")
        self.resize(1200, 800)
        
        self.robot = Robot()
        self.alignment_cache = {} # Cache for storing alignment points: {(parent, child): point}
        self.current_speed = 50   # Global speed setting (0-100%)
        self.import_preferences = {
            "last_stl_unit": "mm",
            "last_up_axis": "preserve",
        }
        self.workspace_calc_thread = None
        self._workspace_calc_pending = False
        self._closing = False
        self.workspace_plane_mode = "inclined"
        self.last_workspace_report = None
        self.last_project_dir = os.getcwd()
        self.robot_sessions = [{"title": "ToRoTrOn", "project_file_path": None}]
        self.current_session_index = 0
        self._restoring_robot_session = False
        self._init_navigation_mixin()
        self.init_ui()
        self.apply_styles()
        self._setup_live_point_refresh()
        
        # Connect signals
        self.log_signal.connect(self.log)
        
        # Center the window and fix geometry warnings
        self.center_on_screen()

    def _current_program_code(self):
        try:
            return self.experiment_tab.program_tab.code_edit.toPlainText()
        except Exception:
            return ""

    def _set_program_code(self, code):
        try:
            self.experiment_tab.program_tab.code_edit.setPlainText(code or "")
        except Exception:
            pass

    def _capture_current_robot_session(self):
        """Store the currently visible robot assembly into its active session tab."""
        if getattr(self, "_restoring_robot_session", False):
            return
        if not hasattr(self, "robot_sessions") or self.current_session_index < 0:
            return
        if self.current_session_index >= len(self.robot_sessions):
            return

        session = self.robot_sessions[self.current_session_index]
        session["robot"] = self.robot
        session["alignment_cache"] = dict(getattr(self, "alignment_cache", {}))
        session["current_speed"] = getattr(self, "current_speed", 50)
        session["import_preferences"] = dict(getattr(self, "import_preferences", {}))
        session["program_code"] = self._current_program_code()
        session["home_tcp_coords"] = getattr(
            self,
            "home_tcp_coords",
            (
                float(self.home_x.value()) if hasattr(self, "home_x") else 0.0,
                float(self.home_y.value()) if hasattr(self, "home_y") else 0.0,
                float(self.home_z.value()) if hasattr(self, "home_z") else 0.0,
            ),
        )
        if hasattr(self, "joint_tab"):
            import copy
            session["joint_panel_joints"] = copy.deepcopy(getattr(self.joint_tab, "joints", {}))
        if hasattr(self, "align_tab"):
            session["alignment_point"] = None if getattr(self.align_tab, "alignment_point", None) is None else np.array(self.align_tab.alignment_point).copy()
            session["alignment_normal"] = None if getattr(self.align_tab, "alignment_normal", None) is None else np.array(self.align_tab.alignment_normal).copy()

    def _clear_visible_robot_scene(self):
        """Remove visible robot actors and transient assembly overlays before restoring a session."""
        if hasattr(self.canvas, "clear_highlights"):
            self.canvas.clear_highlights()
        for name in list(getattr(self.canvas, "actors", {}).keys()):
            self.canvas.remove_actor(name)
        self.canvas.fixed_actors.clear()
        if hasattr(self.canvas, "clear_joint_ghosts"):
            self.canvas.clear_joint_ghosts()
        if hasattr(self.canvas, "clear_cad_drawings"):
            self.canvas.clear_cad_drawings()
        if hasattr(self.canvas, "show_workspace_cloud"):
            self.canvas.show_workspace_cloud(None)
        if hasattr(self.canvas, "clear_workspace_plane"):
            self.canvas.clear_workspace_plane()

    def _restore_robot_session(self, index):
        """Load one robot session into the shared editor, panels, and 3D canvas."""
        if index < 0 or index >= len(self.robot_sessions):
            return

        self._restoring_robot_session = True
        try:
            session = self.robot_sessions[index]
            self.robot = session.get("robot") or Robot()
            session["robot"] = self.robot
            self.alignment_cache = dict(session.get("alignment_cache", {}))
            self.current_speed = int(session.get("current_speed", 50))
            self.import_preferences = dict(session.get("import_preferences", {
                "last_stl_unit": "mm",
                "last_up_axis": "preserve",
            }))

            self._clear_visible_robot_scene()
            if hasattr(self, "links_list"):
                self.links_list.clear()

            self.robot.update_kinematics()
            for name, link in self.robot.links.items():
                if hasattr(self, "add_link_item"):
                    self.add_link_item(name)
                if getattr(link, "mesh", None) is not None:
                    self.canvas.update_link_mesh(name, link.mesh, link.t_world, color=getattr(link, "color", "lightgray"))
                if getattr(link, "is_base", False):
                    self.canvas.fixed_actors.add(name)

            if hasattr(self, "joint_tab"):
                import copy
                self.joint_tab.joints = copy.deepcopy(session.get("joint_panel_joints", {}))
                self.joint_tab.active_joint_control = None
                self.joint_tab.refresh_joints_history()
                self.joint_tab.refresh_links()
            if hasattr(self, "align_tab"):
                self.align_tab.reset_panel()
                if session.get("alignment_point") is not None:
                    self.align_tab.alignment_point = np.array(session["alignment_point"]).copy()
                if session.get("alignment_normal") is not None:
                    self.align_tab.alignment_normal = np.array(session["alignment_normal"]).copy()
            if hasattr(self, "gripper_tab"):
                self.gripper_tab.refresh_joints()
            if hasattr(self, "experiment_tab"):
                self.experiment_tab.refresh_sliders()
                self.experiment_tab.update_display()
            self._set_program_code(session.get("program_code", ""))

            home = session.get("home_tcp_coords", (0.0, 0.0, 0.0))
            self.home_tcp_coords = tuple(float(v) for v in home)
            if hasattr(self, "home_x"):
                self.home_x.setValue(self.home_tcp_coords[0])
                self.home_y.setValue(self.home_tcp_coords[1])
                self.home_z.setValue(self.home_tcp_coords[2])

            if hasattr(self, "speed_slider"):
                self.speed_slider.setValue(self.current_speed)
            if hasattr(self, "speed_spin"):
                self.speed_spin.setValue(self.current_speed)

            self.update_link_colors()
            self.canvas.update_transforms(self.robot)
            self.update_live_ui(render=False)
            if self.canvas.plotter:
                self.canvas.plotter.render()
        finally:
            self._restoring_robot_session = False

    def add_robot_session(self):
        """Create a new independent robot assembly tab without clearing existing sessions."""
        self._capture_current_robot_session()
        next_number = len(self.robot_sessions) + 1
        title = f"Robo {next_number}"
        self.robot_sessions.append({
            "title": title,
            "robot": Robot(),
            "project_file_path": None,
            "alignment_cache": {},
            "joint_panel_joints": {},
            "program_code": "",
            "current_speed": getattr(self, "current_speed", 50),
            "home_tcp_coords": (0.0, 0.0, 0.0),
            "import_preferences": {
                "last_stl_unit": "mm",
                "last_up_axis": "preserve",
            },
        })
        if hasattr(self, "session_tab_bar"):
            self.session_tab_bar.addTab(title)
            self.session_tab_bar.setCurrentIndex(len(self.robot_sessions) - 1)
        else:
            self.current_session_index = len(self.robot_sessions) - 1
            self._restore_robot_session(self.current_session_index)

        if hasattr(self, "assembly_btn"):
            self.assembly_btn.setChecked(True)
            self.toggle_assembly_panel()
        if hasattr(self, "switch_panel"):
            self.switch_panel(0)
        self.log(f"Add Robo: created new assembly tab '{title}'.")

    def _robot_session_tab_at(self, pos):
        if not hasattr(self, "session_tab_bar"):
            return -1
        for idx in range(self.session_tab_bar.count()):
            if self.session_tab_bar.tabRect(idx).contains(pos):
                return idx
        return -1

    def show_robot_session_menu(self, pos):
        """Show Rename/Delete actions when a robot session tab is right-clicked."""
        index = self._robot_session_tab_at(pos)
        if index < 0:
            return

        menu = QtWidgets.QMenu(self)
        rename_action = menu.addAction("Rename")
        delete_action = menu.addAction("Delete")
        delete_action.setEnabled(len(self.robot_sessions) > 1)

        action = menu.exec_(self.session_tab_bar.mapToGlobal(pos))
        if action == rename_action:
            self.rename_robot_session(index)
        elif action == delete_action:
            self.delete_robot_session(index)

    def rename_robot_session(self, index):
        """Rename a robot assembly tab."""
        if index < 0 or index >= len(self.robot_sessions):
            return
        current_name = self.robot_sessions[index].get("title", f"Robo {index + 1}")
        new_name, ok = QtWidgets.QInputDialog.getText(
            self,
            "Rename Robo",
            "Robot tab name:",
            QtWidgets.QLineEdit.Normal,
            current_name,
        )
        new_name = str(new_name).strip()
        if not ok or not new_name:
            return

        self.robot_sessions[index]["title"] = new_name
        if hasattr(self, "session_tab_bar"):
            self.session_tab_bar.setTabText(index, new_name)
        self.log(f"Renamed robot assembly tab to: {new_name}")

    def delete_robot_session(self, index):
        """Delete a robot assembly tab and switch to the nearest remaining tab."""
        if index < 0 or index >= len(self.robot_sessions):
            return
        if len(self.robot_sessions) <= 1:
            self.show_toast("At least one robot tab is required", "warning")
            return

        title = self.robot_sessions[index].get("title", f"Robo {index + 1}")
        reply = QtWidgets.QMessageBox.question(
            self,
            "Delete Robo",
            f"Delete robot assembly tab '{title}'? This will not delete any saved .trn file.",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return

        if index == self.current_session_index:
            next_index = index - 1 if index == len(self.robot_sessions) - 1 else index
        else:
            next_index = self.current_session_index
            if index < self.current_session_index:
                next_index -= 1

        self.robot_sessions.pop(index)
        if hasattr(self, "session_tab_bar"):
            self._restoring_robot_session = True
            try:
                self.session_tab_bar.removeTab(index)
            finally:
                self._restoring_robot_session = False

        self.current_session_index = max(0, min(next_index, len(self.robot_sessions) - 1))
        if hasattr(self, "session_tab_bar"):
            self.session_tab_bar.setCurrentIndex(self.current_session_index)
        self._restore_robot_session(self.current_session_index)
        self.log(f"Deleted robot assembly tab: {title}")
        if hasattr(self, "show_toast"):
            self.show_toast("Robot tab deleted", "success")
    def on_robot_session_changed(self, index):
        """Persist the previous tab and restore the selected robot assembly tab."""
        if getattr(self, "_restoring_robot_session", False):
            return
        if index < 0 or index == getattr(self, "current_session_index", -1):
            return
        self._capture_current_robot_session()
        self.current_session_index = index
        self._restore_robot_session(index)
        title = self.robot_sessions[index].get("title", f"Robo {index + 1}")
        self.log(f"Switched to robot assembly tab: {title}")
    def _clone_robot_for_workspace_calculation(self):
        """Create a mesh-free robot copy so background IK cannot move the live UI robot."""
        clone = Robot()

        for name, link in self.robot.links.items():
            copied = clone.add_link(name, mesh=None)
            copied.color = getattr(link, "color", "lightgray")
            copied.is_base = bool(getattr(link, "is_base", False))
            copied.pick_pos = list(getattr(link, "pick_pos", [0.0, 0.0, 0.0]))
            copied.place_pos = list(getattr(link, "place_pos", [0.0, 0.0, 0.0]))
            copied.t_offset = np.array(getattr(link, "t_offset", np.eye(4)), dtype=float).copy()
            copied.t_world = np.array(getattr(link, "t_world", np.eye(4)), dtype=float).copy()
            if getattr(link, "custom_tcp_offset", None) is not None:
                copied.custom_tcp_offset = np.array(link.custom_tcp_offset, dtype=float).copy()
            if getattr(link, "auto_tcp_offset", None) is not None:
                copied.auto_tcp_offset = np.array(link.auto_tcp_offset, dtype=float).copy()
            copied.custom_tcp_rpy_deg = list(getattr(link, "custom_tcp_rpy_deg", [0.0, 0.0, 0.0]))
            copied.mass = float(getattr(link, "mass", 1.0))
            copied.inertia = dict(getattr(link, "inertia", copied.inertia))
            copied.com = list(getattr(link, "com", [0.0, 0.0, 0.0]))

        if self.robot.base_link is not None and self.robot.base_link.name in clone.links:
            clone.base_link = clone.links[self.robot.base_link.name]

        for name, joint in self.robot.joints.items():
            if joint.parent_link.name not in clone.links or joint.child_link.name not in clone.links:
                continue
            copied_joint = clone.add_joint(name, joint.parent_link.name, joint.child_link.name)
            copied_joint.joint_type = getattr(joint, "joint_type", "revolute")
            copied_joint.is_gripper = bool(getattr(joint, "is_gripper", False))
            copied_joint.origin = np.array(getattr(joint, "origin", np.zeros(3)), dtype=float).copy()
            copied_joint.axis = np.array(getattr(joint, "axis", [0.0, 0.0, 1.0]), dtype=float).copy()
            copied_joint.axis_name = getattr(joint, "axis_name", "Z")
            copied_joint.min_limit = float(getattr(joint, "min_limit", -180.0))
            copied_joint.max_limit = float(getattr(joint, "max_limit", 180.0))
            copied_joint.current_value = float(getattr(joint, "current_value", 0.0))

        clone.joint_relations = {
            master: list(slaves)
            for master, slaves in getattr(self.robot, "joint_relations", {}).items()
        }
        clone.home_joint_values = dict(getattr(self.robot, "home_joint_values", {}))
        clone.update_kinematics()
        return clone

    def _setup_live_point_refresh(self):
        """Continuously refresh the live point display while the UI is running."""
        self.live_point_timer = QtCore.QTimer(self)
        self.live_point_timer.setInterval(50)
        self.live_point_timer.timeout.connect(self.update_live_ui)
        self.live_point_timer.start()

    def init_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        self.main_layout = QtWidgets.QVBoxLayout(central)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(0)
        
        # --- TOP BAR ---
        top_bar = QtWidgets.QWidget()
        top_bar.setStyleSheet("background-color: white; border-bottom: 1px solid #e0e0e0;")
        top_bar.setFixedHeight(55)
        top_layout = QtWidgets.QHBoxLayout(top_bar)
        top_layout.setContentsMargins(15, 5, 15, 5)
        top_layout.setSpacing(10)
        
        # --- Logo / Title ---
        logo_label = QtWidgets.QLabel("E-lab")
        logo_label.setStyleSheet("""
            color: #1976d2;
            font-size: 22px;
            font-weight: bold;
            font-family: 'Segoe UI', Roboto, sans-serif;
            padding: 5px;
        """)
        top_layout.addWidget(logo_label)

        # --- Assembly Toggle Button ---
        self.assembly_btn = QtWidgets.QPushButton("  Assembly")
        self.assembly_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_TitleBarMaxButton))
        self.assembly_btn.setCheckable(True)
        self.assembly_btn.setChecked(True)
        self.assembly_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.assembly_btn.setToolTip("Toggle Assembly Panel")
        self.assembly_btn.setStyleSheet("""
            QPushButton {
                background-color: white;
                color: #1976d2;
                border: 2px solid #1976d2;
                border-radius: 8px;
                padding: 6px 14px;
                font-weight: bold;
                font-size: 13px;
                margin-left: 12px;
            }
            QPushButton:hover {
                background-color: #e3f2fd;
                color: #1565c0;
                border-color: #1565c0;
            }
            QPushButton:checked {
                background-color: #1976d2;
                color: white;
                border-color: #1976d2;
            }
            QPushButton:checked:hover {
                background-color: #1565c0;
                border-color: #0d47a1;
                color: #ffffff;
            }
        """)
        self.assembly_btn.clicked.connect(self.toggle_assembly_panel)
        top_layout.addWidget(self.assembly_btn)

        # --- Experiment Toggle Button ---
        self.experiment_btn = QtWidgets.QPushButton("  Experiment")
        self.experiment_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_FileDialogInfoView))
        self.experiment_btn.setCheckable(True)
        self.experiment_btn.setChecked(False)
        self.experiment_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.experiment_btn.setToolTip("Toggle Experiment Panel")
        self.experiment_btn.setStyleSheet("""
            QPushButton {
                background-color: white;
                color: #2e7d32;
                border: 2px solid #2e7d32;
                border-radius: 8px;
                padding: 6px 14px;
                font-weight: bold;
                font-size: 13px;
                margin-left: 8px;
            }
            QPushButton:hover {
                background-color: #e8f5e9;
                color: #1b5e20;
                border-color: #1b5e20;
            }
            QPushButton:checked {
                background-color: #2e7d32;
                color: white;
                border-color: #2e7d32;
            }
            QPushButton:checked:hover {
                background-color: #1b5e20;
                border-color: #0d47a1;
                color: #ffffff;
            }
        """)
        self.experiment_btn.clicked.connect(self.toggle_experiment_panel)
        top_layout.addWidget(self.experiment_btn)

        # --- Add Robo Button ---
        self.add_robo_btn = QtWidgets.QPushButton("  Add Robo")
        self.add_robo_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_FileIcon))
        self.add_robo_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.add_robo_btn.setToolTip("Start a new robot assembly")
        self.add_robo_btn.setStyleSheet("""
            QPushButton {
                background-color: white;
                color: #6a1b9a;
                border: 2px solid #6a1b9a;
                border-radius: 8px;
                padding: 6px 14px;
                font-weight: bold;
                font-size: 13px;
                margin-left: 8px;
            }
            QPushButton:hover {
                background-color: #f3e5f5;
                color: #4a148c;
                border-color: #4a148c;
            }
            QPushButton:pressed {
                background-color: #e1bee7;
                color: #311b92;
                border-color: #311b92;
            }
        """)
        self.add_robo_btn.clicked.connect(self.add_robot_session)
        top_layout.addWidget(self.add_robo_btn)
        
        # --- Home TCP Coordinate Panel ---
        home_widget = QtWidgets.QFrame()
        home_widget.setStyleSheet(
            "QFrame { background-color: white; border: 1px solid #cbd5df; border-radius: 10px; }"
            "QLabel { color: #1f2933; font-size: 12px; }"
            "QDoubleSpinBox { background: #fbfbff; border: 1px solid #d1d5db; border-radius: 5px; padding: 3px 6px; }"
            "QPushButton { background: #ffffff; border: 1px solid #cbd5df; border-radius: 6px; padding: 4px 10px; font-size: 12px; }"
            "QPushButton:hover { border-color: #1976d2; color: #1976d2; }"
        )
        home_layout = QtWidgets.QHBoxLayout(home_widget)
        home_layout.setContentsMargins(10, 6, 10, 6)
        home_layout.setSpacing(8)

        home_label = QtWidgets.QLabel("Home TCP")
        home_label.setStyleSheet("font-weight:700; color:#1976d2; margin-right: 10px;")
        home_layout.addWidget(home_label)

        home_layout.addWidget(QtWidgets.QLabel("X"))
        self.home_x = QtWidgets.QDoubleSpinBox()
        self.home_x.setRange(-10000.0, 10000.0)
        self.home_x.setDecimals(2)
        self.home_x.setSingleStep(0.1)
        self.home_x.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.home_x.setFixedWidth(90)
        self.home_x.setToolTip("Home X coordinate in cm")
        home_layout.addWidget(self.home_x)

        home_layout.addWidget(QtWidgets.QLabel("Y"))
        self.home_y = QtWidgets.QDoubleSpinBox()
        self.home_y.setRange(-10000.0, 10000.0)
        self.home_y.setDecimals(2)
        self.home_y.setSingleStep(0.1)
        self.home_y.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.home_y.setFixedWidth(90)
        self.home_y.setToolTip("Home Y coordinate in cm")
        home_layout.addWidget(self.home_y)

        home_layout.addWidget(QtWidgets.QLabel("Z"))
        self.home_z = QtWidgets.QDoubleSpinBox()
        self.home_z.setRange(-10000.0, 10000.0)
        self.home_z.setDecimals(2)
        self.home_z.setSingleStep(0.1)
        self.home_z.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.home_z.setFixedWidth(90)
        self.home_z.setToolTip("Home Z coordinate in cm")
        home_layout.addWidget(self.home_z)

        save_btn = QtWidgets.QPushButton("Save")
        save_btn.setCursor(QtCore.Qt.PointingHandCursor)
        save_btn.setFixedHeight(30)
        save_btn.clicked.connect(self.set_home_coords)
        home_layout.addWidget(save_btn)

        go_btn = QtWidgets.QPushButton("Go")
        go_btn.setCursor(QtCore.Qt.PointingHandCursor)
        go_btn.setFixedHeight(30)
        go_btn.clicked.connect(self.go_home_tcp)
        home_layout.addWidget(go_btn)

        top_layout.addStretch()
        top_layout.addWidget(home_widget)
        top_layout.addStretch()

        # --- Save/Open Buttons ---
        btn_file_style = """
            QPushButton {
                background-color: white;
                color: #212121;
                border: 2px solid #e0e0e0;
                border-radius: 8px;
                padding: 6px 16px;
                font-weight: bold;
                font-size: 13px;
                margin-left: 8px;
            }
            QPushButton:hover {
                background-color: #f5f5f5;
                border-color: #bdbdbd;
            }
            QPushButton:pressed {
                background-color: #eeeeee;
            }
        """

        self.open_btn = QtWidgets.QPushButton("")
        self.open_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_DialogOpenButton))
        self.open_btn.setToolTip("Open")
        self.open_btn.setStyleSheet(btn_file_style)
        self.open_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.open_btn.setFixedWidth(42)
        self.open_btn.clicked.connect(self.load_project)
        top_layout.addWidget(self.open_btn)

        self.save_btn = QtWidgets.QPushButton("Save")
        self.save_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_DialogSaveButton))
        self.save_btn.setStyleSheet(btn_file_style)
        self.save_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.save_btn.clicked.connect(self.save_project)
        top_layout.addWidget(self.save_btn)
        
        self.main_layout.addWidget(top_bar)

        # --- Robot Session Tabs ---
        self.session_tab_bar = QtWidgets.QTabBar()
        self.session_tab_bar.setExpanding(False)
        self.session_tab_bar.setMovable(True)
        self.session_tab_bar.setDocumentMode(True)
        self.session_tab_bar.setElideMode(QtCore.Qt.ElideNone)
        self.session_tab_bar.setUsesScrollButtons(True)
        self.session_tab_bar.setMinimumHeight(36)
        self.session_tab_bar.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.session_tab_bar.customContextMenuRequested.connect(self.show_robot_session_menu)
        self.session_tab_bar.setStyleSheet("""
            QTabBar {
                background: #ffffff;
                border-bottom: 1px solid #d7dde5;
            }
            QTabBar::tab {
                background: #f5f7fb;
                color: #334155;
                border: 1px solid #d7dde5;
                border-bottom: none;
                border-top-left-radius: 6px;
                border-top-right-radius: 6px;
                min-width: 104px;
                min-height: 30px;
                padding: 0 18px;
                margin-left: 6px;
                font-weight: 600;
            }
            QTabBar::tab:selected {
                background: #ffffff;
                color: #1976d2;
                border-color: #1976d2;
            }
        """)
        for session in self.robot_sessions:
            self.session_tab_bar.addTab(session.get("title", "Robo"))
        self.session_tab_bar.setCurrentIndex(self.current_session_index)
        self.session_tab_bar.currentChanged.connect(self.on_robot_session_changed)
        self.main_layout.addWidget(self.session_tab_bar)
        
        # --- MAIN CONTENT AREA ---
        self.main_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        
        # Left Side - Navigation + Panel Stack
        self.left_container = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(self.left_container)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(0)

        # Experiment Panel
        self.experiment_tab = ExperimentPanel(self)
        self.experiment_container = QtWidgets.QWidget()
        self.experiment_container.setMinimumWidth(430)
        self.experiment_container.setStyleSheet("background-color: #f0f4f7; border-right: 1px solid #cfd8dc;")
        exp_layout = QtWidgets.QVBoxLayout(self.experiment_container)
        exp_layout.setContentsMargins(0,0,0,0)
        exp_layout.addWidget(self.experiment_tab)
        self.experiment_container.setVisible(False)
        
        # --- ICON NAVIGATION BAR ---
        nav_bar = QtWidgets.QWidget()
        nav_bar.setObjectName("nav_bar_widget")
        nav_bar.setStyleSheet("background-color: white; border-bottom: 2px solid #e0e0e0;")
        nav_bar.setFixedHeight(50)
        nav_layout = QtWidgets.QHBoxLayout(nav_bar)
        nav_layout.setContentsMargins(8, 5, 8, 5)
        nav_layout.setSpacing(6)
        
        # Create navigation buttons with text (no icons/emojis)
        self.nav_buttons = []
        nav_items = [
            ("Links", "Manage robot links and components"),
            ("Align", "Align components together"),
            ("Joint", "Create and control joints"),
            ("Gripper", "Control and calibrate robotic grippers")
        ]
        
        # Ensure panel_stack is initialized before buttons are connected
        self.panel_stack = QtWidgets.QStackedWidget()
        self.panel_stack.setMinimumWidth(280)
        
        # Create panels
        self.links_tab = QtWidgets.QWidget()
        self.setup_links_tab()
        
        self.align_tab = AlignPanel(self)
        self.joint_tab = JointPanel(self)
        self.gripper_tab = GripperPanel(self)
        
        self.panel_stack.addWidget(self.links_tab)
        self.panel_stack.addWidget(self.align_tab)
        self.panel_stack.addWidget(self.joint_tab)
        self.panel_stack.addWidget(self.gripper_tab)
        
        for name, tooltip in nav_items:
            btn = QtWidgets.QPushButton(name)
            btn.setObjectName(name)
            btn.setToolTip(tooltip)
            btn.setFixedHeight(40)
            btn.setCursor(QtCore.Qt.PointingHandCursor)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #f5f5f5;
                    color: #424242;
                    border: none;
                    border-radius: 6px;
                    font-size: 13px;
                    font-weight: bold;
                    padding: 6px 18px;
                }
                QPushButton:hover {
                    background-color: #e3f2fd;
                    color: #1976d2;
                }
                QPushButton:pressed {
                    background-color: #bbdefb;
                }
            """)
            btn.clicked.connect(lambda checked, idx=len(self.nav_buttons): self.switch_panel(idx))
            nav_layout.addWidget(btn)
            self.nav_buttons.append(btn)
        
        nav_layout.addStretch()
        left_layout.addWidget(nav_bar)
        
        # left_container added later
        
        # --- STACKED WIDGET FOR PANELS ---
        # Wrap panel_stack in a Scroll Area for responsiveness on small screens
        panel_scroll = QtWidgets.QScrollArea()
        panel_scroll.setWidgetResizable(True)
        panel_scroll.setWidget(self.panel_stack)
        panel_scroll.setStyleSheet("QScrollArea { border: none; }")
        
        left_layout.addWidget(panel_scroll, 1)
        
        # Connect tab change handler for feature switching (like disabling drag)
        self.panel_stack.currentChanged.connect(self.on_tab_changed)
        
        # Right Side - Vertical Splitter (Canvas on top, Console on bottom)
        self.right_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        
        # --- CANVAS AREA ---
        self.canvas = RobotCanvas()
        
        # Add a floating Isometric View button directly to the canvas
        # We use a white circular button with a 'Home' icon
        self.iso_btn = QtWidgets.QPushButton(self.canvas)
        self.iso_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_ComputerIcon))
        self.iso_btn.setToolTip("Reset to Isometric View")
        self.iso_btn.setFixedSize(38, 38)
        self.iso_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.iso_btn.setStyleSheet("""
            QPushButton {
                background-color: white;
                border: 2px solid #e0e0e0;
                border-radius: 19px;
                padding: 6px;
            }
            QPushButton:hover {
                background-color: #f5f5f5;
                border-color: #1976d2;
            }
            QPushButton:pressed {
                background-color: #e3f2fd;
            }
        """)
        self.iso_btn.clicked.connect(lambda: self.canvas.view_isometric())
        
        # --- Focus Point Button (next to isometric) ---
        self.focus_btn = QtWidgets.QPushButton(self.canvas)
        self.focus_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_DialogApplyButton))
        self.focus_btn.setToolTip("Set Focus Point - click a surface to zoom in")
        self.focus_btn.setFixedSize(38, 38)
        self.focus_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.focus_btn.setStyleSheet("""
            QPushButton {
                background-color: white;
                border: 2px solid #e0e0e0;
                border-radius: 19px;
                padding: 6px;
            }
            QPushButton:hover {
                background-color: #f5f5f5;
                border-color: #1976d2;
            }
            QPushButton:pressed {
                background-color: #e3f2fd;
            }
        """)
        self.focus_btn.clicked.connect(lambda: self.canvas.start_focus_point_picking())

        # --- Live Point Visibility Toggle Button ---
        self.live_point_btn = QtWidgets.QPushButton(self.canvas)
        self.live_point_btn.setCheckable(True)
        self.live_point_btn.setChecked(True)
        self.live_point_btn.setText("●")
        self.live_point_btn.setToolTip("Toggle Live Point (red dot) visibility")
        self.live_point_btn.setFixedSize(38, 38)
        self.live_point_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.live_point_btn.setStyleSheet("""
            QPushButton {
                background-color: white;
                color: #d32f2f;
                border: 2px solid #e0e0e0;
                border-radius: 19px;
                font-size: 18px;
                font-weight: bold;
                padding: 0px;
            }
            QPushButton:hover {
                background-color: #f5f5f5;
                border-color: #d32f2f;
            }
            QPushButton:checked {
                background-color: #ffebee;
                border-color: #d32f2f;
                color: #d32f2f;
            }
            QPushButton:!checked {
                color: #bdbdbd;
                border-color: #e0e0e0;
            }
            QPushButton:pressed {
                background-color: #ffcdd2;
            }
        """)
        self.live_point_btn.clicked.connect(self._toggle_live_point_marker)

        # Workspace plane / visualization buttons removed from the canvas

        # --- Floating Import Object Button (upper-left of canvas) ---
        # REMOVED: Moved to Simulation Panel sidebar
        
        # --- Simulation Objects Toggle Button (bottom-right of canvas) ---
        # REMOVED: Moved to Simulation Panel sidebar
        
        # --- Simulation Objects Popup Panel ---
        # REMOVED: Moved to Simulation Panel sidebar
        
        # REMOVED: Simulation Panel moved to sidebar
        
        # --- Gripper Surface Button (bottom-right of canvas) ---
        self.gripper_surface_btn = QtWidgets.QPushButton("Select Gripper Surface", self.canvas)
        self.gripper_surface_btn.setToolTip("Click to select the inner surface of the gripper for contact")
        self.gripper_surface_btn.setFixedSize(160, 40)
        self.gripper_surface_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.gripper_surface_btn.setStyleSheet("""
            QPushButton {
                background-color: white;
                color: #2e7d32;
                border: 2px solid #4caf50;
                border-radius: 8px;
                font-weight: bold;
                font-size: 13px;
                padding: 5px;
            }
            QPushButton:hover {
                background-color: #e8f5e9;
            }
            QPushButton:pressed {
                background-color: #c8e6c9;
            }
        """)
        self.gripper_surface_btn.clicked.connect(self.joint_tab.on_select_gripper_surface)
        self.gripper_surface_btn.setVisible(False)  # Only visible in Joint Mode

        # Initial positions
        # Sidebar handles everything now
        original_resize = self.canvas.resizeEvent
        def patched_resize(event):
            original_resize(event)
            self.iso_btn.move(self.canvas.width() - 160, 24)
            self.focus_btn.move(self.canvas.width() - 160, 68)
            self.live_point_btn.move(self.canvas.width() - 204, 68)
            self.gripper_surface_btn.move(self.canvas.width() - 180, self.canvas.height() - 60)
        
        self.canvas.resizeEvent = patched_resize
        
        self.right_splitter.addWidget(self.canvas)
        
        self.console = QtWidgets.QTextEdit()
        self.console.setReadOnly(True)
        self.console.setPlaceholderText("System Log...")
        self.console.setVisible(False)  # Hidden by default
        self.console.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #d4d4d4;
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 13px;
                border: none;
                padding: 10px;
                selection-background-color: #264f78;
            }
        """)
        self.right_splitter.addWidget(self.console)
        
        # Hide console initially — canvas takes full space
        self.right_splitter.setSizes([800, 0])
        
        # --- TERMINAL TOGGLE BUTTON (bottom-right) ---
        self.terminal_btn = QtWidgets.QPushButton("⌘ Terminal")
        self.terminal_btn.setCheckable(True)
        self.terminal_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.terminal_btn.setToolTip("Toggle system terminal")
        self.terminal_btn.setAccessibleName("Toggle Terminal")
        self.terminal_btn.setFixedHeight(30)
        self.terminal_btn.setStyleSheet("""
            QPushButton {
                background-color: #1e1e1e;
                color: #d4d4d4;
                border: none;
                border-radius: 0px;
                font-family: 'Consolas', monospace;
                font-size: 12px;
                font-weight: bold;
                padding: 4px 16px;
            }
            QPushButton:checked {
                background-color: #1976d2;
                color: white;
            }
            QPushButton:hover {
                background-color: #333;
            }
        """)
        self.terminal_btn.clicked.connect(self.toggle_terminal)
        
        # Add components to main horizontal splitter

        # --- UNIVERSAL SPEED CONTROL ---
        speed_container = QtWidgets.QWidget()
        speed_container.setStyleSheet("""
            QWidget {
                background-color: white;
                border-top: 2px solid #1976d2;
            }
        """)
        speed_layout = QtWidgets.QHBoxLayout(speed_container)
        speed_layout.setContentsMargins(12, 10, 12, 10)
        speed_layout.setSpacing(12)
        
        speed_header = QtWidgets.QLabel("Speed")
        speed_header.setStyleSheet("font-weight: bold; font-size: 15px; color: #1976d2; background: transparent; border: none;")
        speed_layout.addWidget(speed_header)
        
        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setRange(0, 100)
        self.speed_slider.setValue(self.current_speed)
        self.speed_slider.setCursor(QtCore.Qt.PointingHandCursor)
        self.speed_slider.setStyleSheet("""
            QSlider {
                background: transparent;
                border: none;
                min-height: 28px;
            }
            QSlider::groove:horizontal {
                height: 10px;
                background: #f0f0f0;
                border-radius: 5px;
                border: 1px solid #ddd;
            }
            QSlider::sub-page:horizontal {
                background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 0,
                    stop: 0 #bbdefb, stop: 1 #1976d2);
                border-radius: 5px;
            }
            QSlider::handle:horizontal {
                background: white;
                border: 2px solid #1976d2;
                width: 22px;
                height: 22px;
                margin-top: -7px;
                margin-bottom: -7px;
                border-radius: 11px;
            }
            QSlider::handle:horizontal:hover {
                background: #e3f2fd;
                border-color: #1565c0;
            }
        """)
        speed_layout.addWidget(self.speed_slider, 1)
        
        self.speed_spin = TypeOnlySpinBox()
        self.speed_spin.setRange(0, 100)
        self.speed_spin.setValue(self.current_speed)
        self.speed_spin.setSuffix("%")
        self.speed_spin.setFixedWidth(80)
        self.speed_spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.speed_spin.setStyleSheet("""
            QSpinBox {
                background: white;
                color: #1976d2;
                border: 2px solid #1976d2;
                border-radius: 4px;
                padding: 4px;
                font-weight: bold;
                font-size: 14px;
            }
        """)
        speed_layout.addWidget(self.speed_spin)
        
        self.speed_slider.valueChanged.connect(self.on_speed_change)
        self.speed_spin.valueChanged.connect(self.on_speed_change)
        
        left_layout.addWidget(speed_container)

        self.main_splitter.addWidget(self.left_container)
        self.main_splitter.addWidget(self.experiment_container)
        
        # Wrap right splitter + terminal button in a container
        right_container = QtWidgets.QWidget()
        right_container.setMinimumWidth(420)
        right_vbox = QtWidgets.QVBoxLayout(right_container)
        right_vbox.setContentsMargins(0, 0, 0, 0)
        right_vbox.setSpacing(0)
        right_vbox.addWidget(self.right_splitter, 1)
        right_vbox.addWidget(self.terminal_btn)
        
        self.main_splitter.addWidget(right_container)
        
        # --- CODE DRAWER (Right sidebar) ---
        self.code_drawer = CodeDrawer(self)
        self.main_splitter.addWidget(self.code_drawer)
        
        self.canvas.on_deselect_callback = self.on_deselect
        
        # --- FINALIZE LAYOUT ---
        self.main_layout.addWidget(self.main_splitter, 1)

        # Fix for geometry warnings: Set splitter sizes after a small delay
        # This ensures the window is fully mapped before we move sub-components
        QtCore.QTimer.singleShot(100, lambda: self.main_splitter.setSizes([350, 0, 850, 0]))

    def center_on_screen(self):
        """Standard helper to center the window on the primary screen."""
        frame_gm = self.frameGeometry()
        screen = QtWidgets.QApplication.desktop().screenNumber(QtWidgets.QApplication.desktop().cursor().pos())
        center_point = QtWidgets.QApplication.desktop().screenGeometry(screen).center()
        frame_gm.moveCenter(center_point)
        self.move(frame_gm.topLeft())

    def _set_main_splitter_layout(self, show_assembly, show_experiment):
        sizes = self.main_splitter.sizes()
        if len(sizes) < 4:
            # [assembly, experiment, right(canvas), code_drawer]
            sizes = [0, 0, 0, 0]

        drawer_size = sizes[3]
        window_w = max(1000, self.width())
        min_right = 420

        assembly_w = 350 if show_assembly else 0
        experiment_w = 430 if show_experiment else 0

        right_w = window_w - assembly_w - experiment_w - drawer_size
        if right_w < min_right:
            right_w = min_right

        self.main_splitter.setSizes([assembly_w, experiment_w, right_w, drawer_size])



    def toggle_assembly_panel(self):
        """Toggles the visibility of the assembly (left) panel."""
        show = self.assembly_btn.isChecked()
        
        if show:
            # If opening assembly, close experiment
            self.experiment_btn.setChecked(False)
            self.experiment_container.setVisible(False)
            
        self.left_container.setVisible(show)

        # Keep the right 3D pane visible even if the user previously collapsed it.
        self._set_main_splitter_layout(show_assembly=show, show_experiment=False)
        
        if show:
            # Identifies if we need to refresh the current visible tab
            widget = self.panel_stack.currentWidget()
            if hasattr(widget, 'refresh_sliders'):
                widget.refresh_sliders()

    def toggle_experiment_panel(self):
        """Toggles the visibility of the experiment panel."""
        show = self.experiment_btn.isChecked()
        
        if show:
            # If opening experiment, close assembly
            self.assembly_btn.setChecked(False)
            self.left_container.setVisible(False)
            # Load joint matrices
            self.experiment_tab.refresh_sliders()
            self.experiment_tab.update_display()
            
        self.experiment_container.setVisible(show)

        # Keep the right 3D pane visible even if the user previously collapsed it.
        self._set_main_splitter_layout(show_assembly=False, show_experiment=show)

    def reset_to_home(self):
        """Resets all robot joint values to the global HOME_POSITION."""
        # Try to get HOME_POSITION from main module
        import __main__
        home_angle = getattr(__main__, 'HOME_POSITION', 0.0)
        
        self.log(f"🏠 Resetting robot to Home Position ({home_angle}°)...")
        self.robot.reset_to_home(home_angle)
        
        # Sync all UI panels
        if hasattr(self, 'joint_tab'):
            # Update internal joint_tab dictionary
            for child_name, data in self.joint_tab.joints.items():
                data['current_angle'] = home_angle
            self.joint_tab.refresh_joints_history()
            
        if hasattr(self, 'experiment_tab'):
            self.experiment_tab.refresh_sliders()
            self.experiment_tab.update_display()
            
        # Update 3D view
        self.canvas.update_transforms(self.robot)
        self.update_live_ui()
        self.log("✅ Home Position Restored.")
        
        # Show a friendly toast if method exists
        if hasattr(self, 'show_toast'):
            self.show_toast("Home Position Reset", "success")

    # --- Home coordinate helpers ---------------------------------
    def set_home_coords(self):
        """Save the current values from the Home X/Y/Z fields into the main window."""
        self.home_tcp_coords = (float(self.home_x.value()), float(self.home_y.value()), float(self.home_z.value()))
        self.log(f"Saved Home TCP coordinates: ({self.home_tcp_coords[0]:.2f}, {self.home_tcp_coords[1]:.2f}, {self.home_tcp_coords[2]:.2f}) cm")
        if hasattr(self, 'show_toast'):
            self.show_toast("Home Coordinates Saved", "success")

    def go_home_tcp(self):
        """Move the robot TCP to the saved home coordinates using IK."""
        coords = getattr(
            self,
            "home_tcp_coords",
            (float(self.home_x.value()), float(self.home_y.value()), float(self.home_z.value())),
        )
        tcp_link = self._get_preferred_tcp_link()
        if tcp_link is None:
            QtWidgets.QMessageBox.warning(self, "Go Home", "No valid TCP (End Effector) found for home movement.")
            return

        self.log(f"Moving TCP to Home coords: ({coords[0]:.2f}, {coords[1]:.2f}, {coords[2]:.2f}) cm")
        success, info = self._move_tcp_to_xyz(coords[0], coords[1], coords[2], tcp_link)
        if not success:
            self.log("Failed to move to Home coordinates.")


    def _stop_workspace_calc_thread(self, timeout_ms=3000):
        """Stop the background workspace calculator before replacing or closing the window."""
        thread = getattr(self, "workspace_calc_thread", None)
        self._workspace_calc_pending = False
        if thread is None:
            return

        try:
            if thread.isRunning():
                thread.requestInterruption()
                if not thread.wait(int(timeout_ms)):
                    thread.terminate()
                    thread.wait(1000)
        except RuntimeError:
            pass
        finally:
            self.workspace_calc_thread = None

    def closeEvent(self, event):
        self._closing = True
        self._stop_workspace_calc_thread(timeout_ms=3000)
        super().closeEvent(event)

    def _toggle_live_point_marker(self):
        """Toggle the red live-point dot on the 3D canvas."""
        visible = self.canvas.toggle_live_point_marker()
        self.live_point_btn.setChecked(visible)
        self.show_toast(
            "Live Point visible" if visible else "Live Point hidden",
            "info",
        )

    def _toggle_workspace_plane(self):
        """Toggle the reachable yellow drawing plane on the 3D canvas."""
        btn = getattr(self, 'workspace_plane_btn', None)
        if btn is None:
            self.show_toast("Drawing plane control moved to sidebar", "info")
            return

        if getattr(self.canvas, "current_workspace_plan", None) is None:
            btn.setChecked(False)
            btn.setEnabled(False)
            self.show_toast("Make Robo first to create a drawing plane", "warning")
            return

        visible = self.canvas.set_workspace_plane_visible(btn.isChecked())
        btn.setChecked(visible)
        self.show_toast(
            "Drawing plane visible" if visible else "Drawing plane hidden",
            "info",
        )

    def _toggle_workspace_visualization(self):
        """Toggle the smooth reachable-workspace sphere on the 3D canvas."""
        btn = getattr(self, 'workspace_visualization_btn', None)
        if btn is None:
            self.show_toast("Workspace visualization moved to sidebar", "info")
            return

        if getattr(self.canvas, "_workspace_cloud_points", None) is None:
            btn.setChecked(False)
            btn.setEnabled(False)
            self.show_toast("Make Robo first to create reachable workspace", "warning")
            return

        visible = self.canvas.set_workspace_cloud_visible(btn.isChecked())
        btn.setChecked(visible)
        self.show_toast(
            "Workspace sphere visible" if visible else "Workspace sphere hidden",
            "info",
        )

    def load_workspace_plane_mode(self, mode_key):
        """Workspace plane loading is disabled."""
        self.log("Workspace plane and board visualization have been removed from the application.")
        self.show_toast("Workspace plane removed", "info")

    def _get_preferred_tcp_link(self):
        links = list(self.robot.links.values())
        if not links:
            return None

        def chain_len(link):
            if link is None:
                return -1
            return len(self.robot.get_kinematic_chain(link))

        custom_tcp = getattr(self, "custom_tcp_name", None)
        if custom_tcp and custom_tcp in self.robot.links:
            return self.robot.links[custom_tcp]

        tcp_candidates = [link for link in links if getattr(link, "custom_tcp_offset", None) is not None]
        if tcp_candidates:
            return max(tcp_candidates, key=chain_len)

        gripper_anchor_candidates = []
        for joint in self.robot.joints.values():
            if getattr(joint, "is_gripper", False) and joint.parent_link is not None:
                gripper_anchor_candidates.append(joint.parent_link)
        if gripper_anchor_candidates:
            unique_anchors = list(dict.fromkeys(gripper_anchor_candidates))
            return max(unique_anchors, key=chain_len)

        gripper_candidates = [
            joint.child_link for joint in self.robot.joints.values()
            if getattr(joint, "is_gripper", False) and joint.child_link is not None
        ]
        if gripper_candidates:
            return max(gripper_candidates, key=chain_len)

        leaf_candidates = [link for link in links if link.parent_joint is not None and not link.child_joints]
        if leaf_candidates:
            return max(leaf_candidates, key=chain_len)

        non_base = [link for link in links if not getattr(link, "is_base", False)]
        if non_base:
            return max(non_base, key=chain_len)

        return max(links, key=chain_len)

    def _auto_detect_topmost_tcp(self):
        """
        Analyses all robot link meshes in their current world positions to find the 
        absolute highest Z-coordinate. It then sets the Live Point (TCP) to the 
        centroid of all vertices sharing that maximum height.
        """
        import numpy as np
        
        max_z = -1e12
        top_verts_data = [] # List of (world_vertex, link_object)
        
        # 1. First pass: Find the global maximum height
        for link in self.robot.links.values():
            if link.mesh is None: continue
            
            # Get world-transformed vertices
            try:
                # link.t_world is updated by update_kinematics
                mat = np.array(link.t_world, dtype=float)
                verts = np.array(link.mesh.vertices, dtype=float)
                
                # Transform to world space: (N, 3) -> (N, 4) -> mat @ verts.T -> (3, N)
                ones = np.ones((verts.shape[0], 1))
                verts_homog = np.hstack([verts, ones])
                world_verts = (mat @ verts_homog.T).T[:, :3]
                
                local_max_z = np.max(world_verts[:, 2])
                if local_max_z > max_z:
                    max_z = local_max_z
            except Exception:
                continue
        
        if max_z < -1e11:
            return False # No geometry found
            
        # 2. Second pass: Collect all vertices at the peak (with small epsilon)
        epsilon = 0.5 # 0.5 internal units (e.g. 0.05mm if units=mm)
        for link in self.robot.links.values():
            if link.mesh is None: continue
            
            try:
                mat = np.array(link.t_world, dtype=float)
                verts = np.array(link.mesh.vertices, dtype=float)
                ones = np.ones((verts.shape[0], 1))
                verts_homog = np.hstack([verts, ones])
                world_verts = (mat @ verts_homog.T).T[:, :3]
                
                # Filter vertices at max height
                mask = world_verts[:, 2] >= (max_z - epsilon)
                top_v = world_verts[mask]
                for v in top_v:
                    top_verts_data.append((v, link))
            except Exception:
                continue
        
        if not top_verts_data:
            return False
            
        # 3. Calculate Centroid of the peak
        top_pts = np.array([item[0] for item in top_verts_data])
        centroid_world = np.mean(top_pts, axis=0)
        
        # 4. Choose the 'best' link to host the TCP 
        # (The leaf-most link that contributed to the peak)
        def chain_len(link):
            return len(self.robot.get_kinematic_chain(link))
            
        unique_links = list(set(item[1] for item in top_verts_data))
        target_link = max(unique_links, key=chain_len)
        
        # 5. Convert world centroid to local link coordinates
        inv_mat = np.linalg.inv(np.array(target_link.t_world, dtype=float))
        local_centroid = (inv_mat @ np.append(centroid_world, 1))[:3]
        
        # 6. Apply TCP transform
        self.robot.set_tcp_transform(target_link.name, position=local_centroid)
        self.custom_tcp_name = target_link.name
        
        self.log(f"📍 Auto-TCP Rewired: Live Point set to topmost center of '{target_link.name}' at Z={max_z/getattr(self.canvas, 'grid_units_per_cm', 1.0):.2f} cm.")
        return True

    def make_robot(self):
        """
        Finalize the current assembly by rebuilding kinematics and syncing UI state.
        Returns True when the robot model is ready enough to use.
        """
        if not self.robot.links:
            self.log("Cannot finalize assembly: no links have been imported yet.")
            self.show_toast("Import links first", "warning")
            return False

        if not self.robot.base_link:
            self.log("Cannot finalize assembly: set a base link before building the robot.")
            self.show_toast("Set a base link first", "warning")
            return False

        try:
            # --- KINEMATIC & STRUCTURAL ANALYSIS ---
            self.robot.update_kinematics()
            tcp_link = self._get_preferred_tcp_link()
            has_gripper_tip = bool(
                tcp_link is not None
                and any(getattr(joint, "is_gripper", False) for joint in tcp_link.child_joints)
            )
            if has_gripper_tip:
                self._refresh_auto_tcp_offset(tcp_link)
                self.custom_tcp_name = tcp_link.name
                self.log(f"Live Point locked to gripper TCP on '{tcp_link.name}'.")
            elif self._auto_detect_topmost_tcp():
                self.log("Live Point locked from the current robot top-most point. It will now move with that TCP.")
            else:
                self.log("Could not auto-detect a top-most TCP from the current robot geometry.")

            joint_count = len(self.robot.joints)
            self.log("🔍 ANALYSING ROBOT STRUCTURE...")
            
            # Diagnostic: Check for disconnected components or invalid axes
            for name, joint in self.robot.joints.items():
                if np.linalg.norm(joint.axis) < 0.1:
                    self.log(f"⚠️ WARNING: Joint '{name}' has a near-zero rotation axis! Accuracy will suffer.")
                if not joint.parent_link or not joint.child_link:
                    self.log(f"❌ CRITICAL: Joint '{name}' is missing a parent or child link connection.")
            
            self.canvas.update_transforms(self.robot)
            self.update_link_colors()
            self.update_live_ui()

            if hasattr(self, "joint_tab"):
                self.joint_tab.refresh_links()
                self.joint_tab.refresh_joints_history()

            if hasattr(self, "gripper_tab"):
                self.gripper_tab.refresh_joints()

            if hasattr(self, "experiment_tab"):
                self.experiment_tab.refresh_sliders()
                self.experiment_tab.update_display()

            tcp_link = self._get_preferred_tcp_link()
            self.canvas.show_workspace_cloud(None)
            if hasattr(self, "workspace_visualization_btn"):
                self.workspace_visualization_btn.setEnabled(False)
                self.workspace_visualization_btn.setChecked(False)
            self.last_workspace_report = None

            if joint_count:
                self.log(f"Robot model ready: {joint_count} joint(s) linked from base '{self.robot.base_link.name}'.")
                self.show_toast("Assembly Finalized", "success")
            else:
                self.log(f"Assembly refreshed with base '{self.robot.base_link.name}', but no joints are defined yet.")
            
            return True
        except Exception as exc:
            self.log(f"MAKE ROBO ERROR: {exc}")
            self.show_toast("Unable to finalize assembly", "error")
            return False

    def auto_calculate_inclined_workspace(self):
        """
        Start background calculation of optimal workspace dimensions.
        """
        if getattr(self, "_closing", False):
            return

        if self.workspace_calc_thread is not None and self.workspace_calc_thread.isRunning():
            self.workspace_calc_thread.requestInterruption()
            self._workspace_calc_pending = True
            self.log("Previous workspace calculation is stopping; keeping the live point stable.")
            return

        tcp_link = self._get_preferred_tcp_link()
        tcp_link_name = tcp_link.name if tcp_link is not None else None
        if hasattr(self, "workspace_plane_btn"):
            self.workspace_plane_btn.setEnabled(False)

        try:
            robot_for_calc = self._clone_robot_for_workspace_calculation()
        except Exception as exc:
            self.log(f"Workspace calculation skipped: could not snapshot robot ({exc}).")
            return

        self.log("📊 Computing optimal workspace dimensions...")
        self._workspace_calc_pending = False
        self.workspace_calc_thread = WorkspaceCalculatorThread(
            robot_for_calc,
            tcp_link_name,
            plane_mode=getattr(self, "workspace_plane_mode", "inclined"),
            workspace_points=(getattr(self, "last_workspace_report", None) or {}).get("points"),
            parent=self,
        )
        self.workspace_calc_thread.finished.connect(self._on_workspace_calculated)
        self.workspace_calc_thread.start()

    def _on_workspace_calculated(self, result):
        """Handle completed workspace calculation from background thread."""
        thread = self.sender()
        self.workspace_calc_thread = None
        pending_restart = bool(getattr(self, "_workspace_calc_pending", False))
        self._workspace_calc_pending = False

        if thread is not None:
            try:
                thread.deleteLater()
            except RuntimeError:
                pass

        if getattr(self, "_closing", False):
            return

        if result.get("cancelled"):
            if pending_restart:
                QtCore.QTimer.singleShot(0, self.auto_calculate_inclined_workspace)
            return

        error = result.get("error")
        if error:
            if hasattr(self, "workspace_plane_btn"):
                self.workspace_plane_btn.setEnabled(False)
            self.log(f"⚠️ Workspace calculation error: {error}")
            return
            
        best_center = result.get("best_center")
        best_origin = result.get("best_origin") or best_center
        best_size = result.get("best_size")

        if best_origin is None or best_size[0] < 10.0:
            if hasattr(self, "workspace_plane_btn"):
                self.workspace_plane_btn.setEnabled(False)
            self.log("⚠️ Could not calculate optimal workspace (robot may not be fully reachable)")
            return
            
        self.log(
            f"Workspace analysis complete: {best_size[0]:.1f}×{best_size[1]:.1f} cm estimated near ({best_origin[0]:.1f}, {best_origin[1]:.1f}, {best_origin[2]:.1f})."
        )

    def move_live_point_to_xyz(self, x_cm, y_cm, z_cm):
        """Solve joint angles so the robot live point/TCP reaches the given XYZ target in centimeters."""
        if not self.robot.joints:
            self.log("MOVE failed: no joints are defined.")
            self.show_toast("Create joints first", "warning")
            return False, {}

        links = list(self.robot.links.values())
        if not links:
            self.log("MOVE failed: no links are loaded.")
            self.show_toast("Import links first", "warning")
            return False, {}

        tcp_link = self._get_preferred_tcp_link()
        if tcp_link is None:
            self.log("MOVE failed: no valid TCP link is available.")
            self.show_toast("Configure TCP first", "warning")
            return False, {}

        ratio = getattr(self.canvas, "grid_units_per_cm", 1.0) or 1.0
        target_world = np.array([x_cm, y_cm, z_cm], dtype=float) * ratio
        requested_world = target_world.copy()
        workspace_hint = self.robot.get_workspace_target_hint(target_world, tcp_link)
        target_snapped = False
        issues = self.robot.diagnose_ik_setup(tcp_link, requested_world)
        if issues:
            self.log("MOVE diagnostic report:")
            for issue in issues:
                self.log(f"  Warning: {issue}")

        target_tcp_pose = self.robot.get_tcp_world_pose(tcp_link)
        target_tcp_pose[:3, 3] = requested_world

        old_angles = {name: joint.current_value for name, joint in self.robot.joints.items()}
        success, info = self.robot.inverse_kinematics_pose(
            target_tcp_pose,
            tcp_link,
            max_iters=350,
            position_tolerance=max(0.2 * ratio, 0.2),
            orientation_tolerance=1e6,
            orientation_weight=0.0,
            joint_change_weight=0.18,
        )
        used_target_world = requested_world.copy()
        if (
            not success
            and workspace_hint.get("ok")
            and not workspace_hint.get("inside_workspace", True)
        ):
            for name, value in old_angles.items():
                self.robot.joints[name].current_value = value
            self.robot.update_kinematics()

            fallback_world = np.array(workspace_hint["nearest_point"], dtype=float)
            fallback_tcp_pose = self.robot.get_tcp_world_pose(tcp_link)
            fallback_tcp_pose[:3, 3] = fallback_world
            fallback_success, fallback_info = self.robot.inverse_kinematics_pose(
                fallback_tcp_pose,
                tcp_link,
                max_iters=350,
                position_tolerance=max(0.2 * ratio, 0.2),
                orientation_tolerance=1e6,
                orientation_weight=0.0,
                joint_change_weight=0.18,
            )
            if fallback_success:
                success = fallback_success
                info = fallback_info
                used_target_world = fallback_world
                target_snapped = True
                snapped_cm = fallback_world / ratio
                self.log(
                    "MOVE fallback used nearest sampled reachable point after direct solve failed: "
                    f"({snapped_cm[0]:.2f}, {snapped_cm[1]:.2f}, {snapped_cm[2]:.2f}) cm."
                )

        if not success:
            for name, value in old_angles.items():
                self.robot.joints[name].current_value = value
            self.robot.update_kinematics()
            self.canvas.update_transforms(self.robot)
            self.update_live_ui()
            self.log(f"MOVE failed: target ({x_cm:.2f}, {y_cm:.2f}, {z_cm:.2f}) cm is unreachable.")
            self.show_toast("Target unreachable", "warning")
            return False, info

        self.robot.update_kinematics()
        self.canvas.update_transforms(self.robot)
        self.update_live_ui()

        joint_tab = getattr(self, "joint_tab", None)
        if joint_tab is not None:
            for child_name, data in joint_tab.joints.items():
                joint_id = data.get("joint_id", child_name)
                if joint_id in self.robot.joints:
                    data["current_angle"] = float(self.robot.joints[joint_id].current_value)
            joint_tab.refresh_joints_history()

        if hasattr(self, "experiment_tab"):
            self.experiment_tab.refresh_sliders()
            self.experiment_tab.update_display()

        solved_angles = {}
        for joint in self.robot.get_kinematic_chain(tcp_link):
            solved_angles[joint.name] = float(joint.current_value)

        self.log(
            f"MOVE success: live point moved to ({x_cm:.2f}, {y_cm:.2f}, {z_cm:.2f}) cm."
        )
        if target_snapped:
            snapped_cm = used_target_world / ratio
            requested_cm = requested_world / ratio
            self.log(
                "Nearest reachable point used for move: "
                f"requested=({requested_cm[0]:.2f}, {requested_cm[1]:.2f}, {requested_cm[2]:.2f}) cm, "
                f"reached=({snapped_cm[0]:.2f}, {snapped_cm[1]:.2f}, {snapped_cm[2]:.2f}) cm."
            )
        self.log(
            "Solved Angles: "
            + ", ".join(f"{joint_name}={angle:.2f}°" for joint_name, angle in solved_angles.items())
        )
        self.show_toast("Nearest reachable point reached" if target_snapped else "Live point target reached", "success")
        return True, {
            "tcp_link": tcp_link.name,
            "joint_angles": solved_angles,
            "target_snapped": target_snapped,
            "requested_target_world": requested_world.copy(),
            "used_target_world": used_target_world.copy(),
            **info,
        }

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())








