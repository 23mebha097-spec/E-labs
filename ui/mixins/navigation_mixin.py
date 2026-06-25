from PyQt5 import QtWidgets, QtCore, QtGui
import numpy as np
import os

from core.firmware_gen import generate_esp32_firmware


class ToastNotification(QtWidgets.QFrame):
    """Animated toast notification that slides in from bottom-right and auto-fades."""
    
    COLORS = {
        'success': ('#4caf50', '✓'),
        'error': ('#d32f2f', '✗'),
        'warning': ('#ff9800', '⚠'),
        'info': ('#1976d2', 'ℹ'),
    }
    
    def __init__(self, parent, message, toast_type='info', duration=3000):
        super().__init__(parent)
        color, icon = self.COLORS.get(toast_type, self.COLORS['info'])
        
        self.setStyleSheet(f"""
            QFrame {{
                background-color: {color};
                border-radius: 8px;
                border: none;
            }}
        """)
        
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(16, 10, 16, 10)
        layout.setSpacing(10)
        
        icon_label = QtWidgets.QLabel(icon)
        icon_label.setStyleSheet("color: white; font-size: 18px; font-weight: bold; background: transparent;")
        layout.addWidget(icon_label)
        
        text_label = QtWidgets.QLabel(message)
        text_label.setStyleSheet("color: white; font-size: 14px; font-weight: bold; background: transparent;")
        text_label.setWordWrap(True)
        layout.addWidget(text_label, 1)
        
        self.setFixedWidth(320)
        self.adjustSize()
        self.setFixedHeight(max(self.sizeHint().height(), 44))
        
        # Position off-screen (bottom-right)
        self.target_y = parent.height() - self.height() - 20
        self.move(parent.width() - self.width() - 20, parent.height())
        self.show()
        self.raise_()
        
        # Slide-in animation
        self.slide_anim = QtCore.QPropertyAnimation(self, b"pos")
        self.slide_anim.setDuration(300)
        self.slide_anim.setStartValue(self.pos())
        self.slide_anim.setEndValue(QtCore.QPoint(parent.width() - self.width() - 20, self.target_y))
        self.slide_anim.setEasingCurve(QtCore.QEasingCurve.OutCubic)
        self.slide_anim.start()
        
        # Fade-out after duration
        self.fade_timer = QtCore.QTimer(self)
        self.fade_timer.setSingleShot(True)
        self.fade_timer.timeout.connect(self._start_fade)
        self.fade_timer.start(duration)
    
    def _start_fade(self):
        self.effect = QtWidgets.QGraphicsOpacityEffect(self)
        self.setGraphicsEffect(self.effect)
        self.fade_anim = QtCore.QPropertyAnimation(self.effect, b"opacity")
        self.fade_anim.setDuration(400)
        self.fade_anim.setStartValue(1.0)
        self.fade_anim.setEndValue(0.0)
        self.fade_anim.setEasingCurve(QtCore.QEasingCurve.InCubic)
        self.fade_anim.finished.connect(self.deleteLater)
        self.fade_anim.start()


class NavigationMixin:
    """Methods for panel switching, simulation, speed, terminal, styling, and robot movement."""

    def _init_navigation_mixin(self):
        """Initialize navigation/movement state. Should be called by MainWindow.__init__."""
        self._anim_timer = QtCore.QTimer(self)
        self._anim_timer.timeout.connect(self._on_anim_tick)
        self._anim_joint_ids = []
        self._anim_child_names = []
        self._target_angles = []
        self._current_angles = []
        self._anim_success = True
        self._anim_blocking = False # Used for scripts to wait for animation

    def on_deselect(self):
        """Clears list selections when 3D selection is cancelled (Esc)."""
        self.links_list.clearSelection()
        self.links_list.setCurrentItem(None)
        self.set_base_btn.setText("Set as Base")
        
        # Reset Align Tool selection state
        self.align_tab.reset_panel()

    def on_focus_base(self):
        if not self.robot.base_link:
            self.log("No Base set to focus on.")
            return
        
        base_name = self.robot.base_link.name
        if base_name in self.canvas.actors:
            actor = self.canvas.actors[base_name]
            bounds = actor.GetBounds()
            self.canvas.focus_on_bounds(bounds)
            self.log(f"Focused camera on Base: {base_name}")

    def sync_link_transform(self, name, matrix):
        """Saves a 3D visual transformation back to the robot link model."""
        if name not in self.robot.links:
            return
            
        link = self.robot.links[name]
        
        # --- BASE PROTECTION RULE: The Base is functionally fixed at (0,0,0) ---
        if link.is_base:
            self.log(f"⚠️ Locked: '{name}' is the Base and its position is frozen.")
            return
        
        if link.parent_joint:
            # Solve for local offset in parent-joint frame
            parent_world = link.parent_joint.parent_link.t_world
            joint_rot = link.parent_joint.get_matrix()
            
            # Child_World = Parent_World @ Joint_Matrix @ Child_Offset
            # => Child_Offset = Inv(Joint_Matrix) @ Inv(Parent_World) @ Matrix
            inv_parent = np.linalg.inv(parent_world)
            inv_joint = np.linalg.inv(joint_rot)
            
            link.t_offset = inv_joint @ inv_parent @ matrix
        else:
            # It's a root/floating link, offset is absolute world position
            link.t_offset = matrix
            
        self.robot.update_kinematics()
        self.update_link_colors()
        self.log(f"Synced coordinates for: {name}")
        # Re-run kinematics to ensure the whole branch moves correctly
        self.robot.update_kinematics()
        self.update_live_ui()

    def switch_panel(self, index):
        self.panel_stack.setCurrentIndex(index)
        
        # Update button styles
        for i, btn in enumerate(self.nav_buttons):
            if i == index:
                btn.setStyleSheet("""
                    QPushButton {
                        background-color: #1976d2;
                        color: white;
                        border: none;
                        border-radius: 6px;
                        font-size: 13px;
                        font-weight: bold;
                        padding: 6px 18px;
                    }
                """)
            else:
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
                    QPushButton:hover { background-color: #e3f2fd; color: #1976d2; }
                """)

    def on_speed_change(self, value):
        self.current_speed = value
        # Sync slider and spinbox without infinite loop
        if self.speed_slider.value() != value:
            self.speed_slider.blockSignals(True)
            self.speed_slider.setValue(value)
            self.speed_slider.blockSignals(False)
        if self.speed_spin.value() != value:
            self.speed_spin.blockSignals(True)
            self.speed_spin.setValue(value)
            self.speed_spin.blockSignals(False)
        self.show_speed_overlay()

    def update_live_ui(self, render=True):
        """Updates the Live Point (LP) coordinates and handles Pick-and-Place simulation logic."""
        tcp_link = None
        if hasattr(self, "_get_preferred_tcp_link"):
            tcp_link = self._get_preferred_tcp_link()

        if not tcp_link:
            # Fallback: Find the physically "top-most" point among all links
            best_z = -float('inf')
            for link in self.robot.links.values():
                if link.is_base: continue
                # Get the tool point (top center of this specific link)
                w_pos, _, _ = self.get_link_tool_point(link)
                if w_pos[2] > best_z:
                    best_z = w_pos[2]
                    tcp_link = link
        if tcp_link:
            self._refresh_auto_tcp_offset(tcp_link)

            # Use the kinematic TCP pose for coordinate display.
            # This is the same transform the IK solver targets, so the
            # displayed position always matches the IK solution exactly.
            tcp_pose = self.robot.get_tcp_world_pose(tcp_link)
            pos = tcp_pose[:3, 3].copy()

            ratio = self.canvas.grid_units_per_cm
            lx, ly, lz = pos[0] / ratio, pos[1] / ratio, pos[2] / ratio

            # Keep the latest live point available globally even if the LP widgets
            # are not present in the current UI layout.
            self.current_live_point_cm = (lx, ly, lz)

            live_x = getattr(self, "live_x", None)
            live_y = getattr(self, "live_y", None)
            live_z = getattr(self, "live_z", None)
            if all(widget is not None for widget in (live_x, live_y, live_z)):
                live_x.blockSignals(True)
                live_y.blockSignals(True)
                live_z.blockSignals(True)

                live_x.setValue(lx)
                live_y.setValue(ly)
                live_z.setValue(lz)

                live_x.blockSignals(False)
                live_y.blockSignals(False)
                live_z.blockSignals(False)

            # Update 3D Engine HUD
            if hasattr(self.canvas, 'update_hud_coords'):
                self.canvas.update_hud_coords(lx, ly, lz, render=render)
            if hasattr(self.canvas, 'update_live_point_marker'):
                # Use the exact grid-unit position for marker alignment
                self.canvas.update_live_point_marker(pos, render=render)

            # Pick-and-Place Simulation Logic (MAGNET MODE)
            # Uses mesh-based tool point for accurate contact geometry.
            sim_tab = getattr(self, 'simulation_tab', None)
            if sim_tab and hasattr(sim_tab, 'is_sim_active') and sim_tab.is_sim_active:
                sim_pos, _, _ = self.get_link_tool_point(tcp_link)
                self._handle_sim_pick_place(tcp_link, sim_pos, ratio)
        else:
            lx, ly, lz = 0.0, 0.0, 0.0
            self.current_live_point_cm = (lx, ly, lz)

            live_x = getattr(self, "live_x", None)
            live_y = getattr(self, "live_y", None)
            live_z = getattr(self, "live_z", None)
            if all(widget is not None for widget in (live_x, live_y, live_z)):
                live_x.blockSignals(True)
                live_y.blockSignals(True)
                live_z.blockSignals(True)
                live_x.setValue(lx)
                live_y.setValue(ly)
                live_z.setValue(lz)
                live_x.blockSignals(False)
                live_y.blockSignals(False)
                live_z.blockSignals(False)

            if hasattr(self.canvas, 'update_hud_coords'):
                self.canvas.update_hud_coords(lx, ly, lz, render=render)
            if hasattr(self.canvas, 'clear_live_point_marker'):
                self.canvas.clear_live_point_marker()

    # ─── Simulation Object Helpers (ported from Torotron) ─────────────

    def on_sim_object_clicked(self, item):
        """Selects and focuses on the sim object in the 3D scene."""
        name = item.text()
        if name in self.canvas.actors:
            self.canvas.select_actor(name)

        if name not in self.robot.links:
            return

        link = self.robot.links[name]

        # Block signals to avoid self-triggering save while loading
        for sb in [self.pick_x, self.pick_y, self.pick_z,
                    self.place_x, self.place_y, self.place_z]:
            sb.blockSignals(True)

        is_aligned = False
        if hasattr(self, 'alignment_cache'):
            for (p, c), pt in self.alignment_cache.items():
                if c == name:
                    is_aligned = True
                    break

        is_locked = link.is_base or link.parent_joint or is_aligned

        ratio = self.canvas.grid_units_per_cm
        pick = getattr(link, 'pick_pos', [0, 0, 0])
        place = getattr(link, 'place_pos', [0, 0, 0])

        self.pick_x.setValue(pick[0] / ratio)
        self.pick_y.setValue(pick[1] / ratio)
        self.pick_z.setValue(pick[2] / ratio)

        self.place_x.setValue(place[0] / ratio)
        self.place_y.setValue(place[1] / ratio)
        self.place_z.setValue(place[2] / ratio)

        for sb in [self.pick_x, self.pick_y, self.pick_z,
                    self.place_x, self.place_y, self.place_z]:
            sb.blockSignals(False)
            sb.setEnabled(not is_locked)
            if is_locked:
                sb.setStyleSheet("background: #f5f5f5; color: #9e9e9e; border: 1px solid #ddd; border-radius: 4px; font-size: 12px; padding: 2px 4px; font-weight: bold;")
            else:
                color = "#1976d2" if sb in (self.pick_x, self.pick_y, self.pick_z) else "#388E3C"
                sb.setStyleSheet(f"background: white; color: {color}; border: 1px solid #ddd; border-radius: 4px; font-size: 12px; padding: 2px 4px; font-weight: bold;")

        # Refresh Property Display
        sim_tab = getattr(self, 'simulation_tab', None)
        if sim_tab:
            sim_tab.refresh_object_info(name)

    def save_sim_object_coords(self):
        """Saves current spinbox values back to the selected simulation object."""
        if not hasattr(self, 'sim_objects_list'):
            return
        current_item = self.sim_objects_list.currentItem()
        if not current_item:
            return

        name = current_item.text()
        if name in self.robot.links:
            link = self.robot.links[name]
            if link.is_base:
                return
            ratio = self.canvas.grid_units_per_cm
            link.pick_pos = [self.pick_x.value() * ratio, self.pick_y.value() * ratio, self.pick_z.value() * ratio]
            link.place_pos = [self.place_x.value() * ratio, self.place_y.value() * ratio, self.place_z.value() * ratio]

    def _handle_sim_pick_place(self, tcp_link, tcp_pos, ratio):
        """Monitors proximity to P1/P2 and manages object attachment using the Tool Point."""
        sim_tab = getattr(self, 'simulation_tab', None)
        if not sim_tab:
            return

        tool_pos, tool_local, gap_dist = self.get_link_tool_point(tcp_link)

        p1 = np.array([sim_tab.pick_x.value(), sim_tab.pick_y.value(), sim_tab.pick_z.value()]) * ratio
        p2 = np.array([sim_tab.place_x.value(), sim_tab.place_y.value(), sim_tab.place_z.value()]) * ratio

        THRESHOLD = ((gap_dist / 2.0) + (1.0 * ratio)) if gap_dist else 2.5 * ratio

        item = sim_tab.objects_list.currentItem()
        if not item:
            return
        obj_name = item.text()
        if obj_name not in self.robot.links:
            return
        obj_link = self.robot.links[obj_name]

        is_aligned = False
        if hasattr(self, 'alignment_cache'):
            for (p, c), pt in self.alignment_cache.items():
                if c == obj_name:
                    is_aligned = True
                    break
        if obj_link.is_base or obj_link.parent_joint or is_aligned:
            return

        # STATE A: Not gripping → look for P1
        if not sim_tab.gripped_object:
            dist_p1 = np.linalg.norm(tool_pos - p1)

            if gap_dist and obj_link.mesh:
                obj_size = np.max(obj_link.mesh.bounds[1] - obj_link.mesh.bounds[0])
                if obj_size > gap_dist and dist_p1 < 5.0 * ratio:
                    self.log(f"⚠ Warning: {obj_name} is too large for the current finger gap!")

            if dist_p1 < 10.0 * ratio:
                self._control_gripper_fingers(close=False)

            if dist_p1 < THRESHOLD:
                self.log(f"🧲 GRIPPED: {obj_name} at P1")
                sim_tab.gripped_object = obj_name
                inv_tcp = np.linalg.inv(tcp_link.t_world)
                sim_tab.grip_offset = inv_tcp @ obj_link.t_world
                self._control_gripper_fingers(close=True)
                self.show_toast(f"Gripped {obj_name}", "success")

        # STATE B: Gripping → follow robot and look for P2
        else:
            if sim_tab.gripped_object == obj_name:
                obj_link.t_offset = tcp_link.t_world @ sim_tab.grip_offset
                self.canvas.update_transforms(self.robot)
                sim_tab.refresh_object_info(obj_name)

                dist_p2 = np.linalg.norm(tool_pos - p2)
                if dist_p2 < THRESHOLD:
                    self.log(f"📦 PLACED: {obj_name} at P2")
                    sim_tab.gripped_object = None
                    sim_tab.grip_offset = None
                    self._control_gripper_fingers(close=False)
                    self.show_toast(f"Placed {obj_name}", "success")
                    sim_tab.start_btn.setChecked(False)
                    sim_tab.toggle_pick_place_sim(False)

    def _compute_finger_gap(self):
        """Measures the current distance between finger tips (world space)."""
        sim_tab = getattr(self, 'simulation_tab', None)
        tcp_link = None
        if sim_tab:
            tcp_link = sim_tab._get_tcp_link()
        if not tcp_link:
            return None
        _, _, gap = self.get_link_tool_point(tcp_link)
        return gap

    def _control_gripper_fingers(self, close=True, target_gap_world=None, apply=True):
        """
        Moves gripper master joints to open/close the fingers.
        """
        master_joints = [
            j for j_name, j in self.robot.joints.items()
            if j_name in self.robot.joint_relations
        ]

        if not master_joints:
            return {} if not apply else None

        targets = {}

        # Case A: Precise gap targeting via bisection
        if target_gap_world is not None:
            saved = {j.name: j.current_value for j in master_joints}

            for joint in master_joints:
                lo, hi = joint.min_limit, joint.max_limit
                best_mid = joint.current_value

                for _ in range(20):
                    mid = (lo + hi) / 2.0
                    joint.current_value = mid
                    for s_id, r in self.robot.joint_relations[joint.name]:
                        if s_id in self.robot.joints:
                            self.robot.joints[s_id].current_value = mid * r
                    self.robot.update_kinematics()

                    gap_now = self._compute_finger_gap()
                    if gap_now is None:
                        break
                    if gap_now > target_gap_world:
                        lo = mid
                    else:
                        hi = mid
                    best_mid = mid

                targets[joint.name] = best_mid

            for j in master_joints:
                j.current_value = saved[j.name]
                for s_id, r in self.robot.joint_relations[j.name]:
                    if s_id in self.robot.joints:
                        self.robot.joints[s_id].current_value = saved[j.name] * r
            self.robot.update_kinematics()

            if apply:
                for j_name, val in targets.items():
                    self.robot.joints[j_name].current_value = val
                    for s_id, r in self.robot.joint_relations[j_name]:
                        if s_id in self.robot.joints:
                            self.robot.joints[s_id].current_value = val * r
                self.robot.update_kinematics()
                self.canvas.update_transforms(self.robot)
                return None
            return targets

        # Case B: Full open / close
        for joint in master_joints:
            target = joint.max_limit if close else joint.min_limit
            targets[joint.name] = target
            if apply:
                joint.current_value = target
                for s_id, r in self.robot.joint_relations[joint.name]:
                    if s_id in self.robot.joints:
                        self.robot.joints[s_id].current_value = target * r

        if apply:
            self.robot.update_kinematics()
            self.canvas.update_transforms(self.robot)
            return None
        return targets

    # ─── End Simulation Helpers ───────────────────────────────────────

    def get_link_tool_point(self, link, return_vec=False):
        """
        Calculates the Tool Center Point (TCP) in World and Local coords.
        """
        if not link:
            if return_vec: return np.zeros(3), np.zeros(3), None
            return np.zeros(3), np.zeros(3), 0.0

        def mesh_vertices_world(mesh_link):
            if mesh_link is None or mesh_link.mesh is None:
                return None
            verts = np.array(getattr(mesh_link.mesh, "vertices", []), dtype=float)
            if verts.size == 0:
                return None
            verts = verts.reshape((-1, 3))
            verts_h = np.hstack([verts, np.ones((len(verts), 1))])
            return (np.array(mesh_link.t_world, dtype=float) @ verts_h.T).T[:, :3]

        def distal_point(mesh_link, origin_world):
            world_pts = mesh_vertices_world(mesh_link)
            if world_pts is None:
                return None
            distances = np.linalg.norm(world_pts - origin_world, axis=1)
            if distances.size == 0:
                return None
            max_dist = float(np.max(distances))
            tolerance = max(max_dist * 0.01, 1e-6)
            tip_pts = world_pts[distances >= (max_dist - tolerance)]
            if tip_pts.size == 0:
                tip_pts = world_pts[[int(np.argmax(distances))]]
            return np.mean(tip_pts, axis=0)

        def upper_contact_point(mesh_link):
            world_pts = mesh_vertices_world(mesh_link)
            if world_pts is None:
                return None
            max_z = float(np.max(world_pts[:, 2]))
            tolerance = max(np.ptp(world_pts[:, 2]) * 0.01, 1e-6)
            upper_pts = world_pts[world_pts[:, 2] >= (max_z - tolerance)]
            if upper_pts.size == 0:
                upper_pts = world_pts[[int(np.argmax(world_pts[:, 2]))]]
            return np.mean(upper_pts, axis=0)

        def remember_auto_tcp(local_tool_point):
            if getattr(link, "custom_tcp_offset", None) is None:
                link.auto_tcp_offset = np.array(local_tool_point, dtype=float)

        # 1. Identify 'Fingers'
        fingers = []
        for joint in link.child_joints:
            if getattr(joint, 'is_gripper', False) and joint.child_link:
                fingers.append(joint.child_link)

        # Priority 1: User-Defined Custom TCP
        if hasattr(self, "robot") and link.name in self.robot.links:
            tcp_local_tf = self.robot.get_tcp_local_transform(link)
        else:
            tcp_local_tf = np.eye(4)

        if hasattr(link, 'custom_tcp_offset') and link.custom_tcp_offset is not None:
            local_tool_point = tcp_local_tf[:3, 3]
            world_tool_point = (link.t_world @ np.append(local_tool_point, 1.0))[:3]
            gap = 0.0
            if fingers:
                pts_world = []
                for f in fingers:
                    if f.mesh:
                        b = f.mesh.bounds
                        c_finger = (b[0] + b[1]) / 2.0
                        pts_world.append((f.t_world @ np.append(c_finger, 1.0))[:3])
                if len(pts_world) >= 2:
                    for i in range(len(pts_world)):
                        for j in range(i + 1, len(pts_world)):
                            gap = max(gap, np.linalg.norm(pts_world[i] - pts_world[j]))
            if return_vec:
                return world_tool_point, local_tool_point, None
            return world_tool_point, local_tool_point, gap

        # 2. Case: Multiple Fingers (upper exposed contact point)
        if len(fingers) >= 2:
            pts_world = []
            pts_local = []
            hand_origin = np.array(link.t_world[:3, 3], dtype=float)
            for f in fingers:
                w_pt = upper_contact_point(f)
                if w_pt is None:
                    w_pt = distal_point(f, hand_origin)
                if w_pt is None:
                    continue
                pts_world.append(w_pt)
                inv_hand = np.linalg.inv(link.t_world)
                pt_in_hand = (inv_hand @ np.append(w_pt, 1.0))[:3]
                pts_local.append(pt_in_hand)
            if pts_local:
                pts_local_np = np.array(pts_local, dtype=float)
                upper_z = np.max(pts_local_np[:, 2])
                upper_local_pts = pts_local_np[pts_local_np[:, 2] >= (upper_z - 1e-6)]
                local_tool_point = np.mean(upper_local_pts, axis=0)
                world_tool_point = (link.t_world @ np.append(local_tool_point, 1.0))[:3]
                remember_auto_tcp(local_tool_point)
                max_span = 0.0
                best_vec = np.array([1.0, 0.0, 0.0])
                for i in range(len(pts_world)):
                    for j in range(i + 1, len(pts_world)):
                        v = pts_world[i] - pts_world[j]
                        d = np.linalg.norm(v)
                        if d > max_span:
                            max_span = d
                            best_vec = v
                real_gap = max_span
                finger_depth = 0.0
                all_local = []
                inv_hand = np.linalg.inv(link.t_world)
                for f in fingers:
                    f_world = mesh_vertices_world(f)
                    if f_world is not None:
                        f_local = (inv_hand @ np.hstack([f_world, np.ones((len(f_world), 1))]).T).T[:, :3]
                        all_local.append(f_local)
                if all_local:
                    all_local = np.vstack(all_local)
                    finger_depth = float(np.ptp(all_local[:, 2]))
                geo = {
                    "real_gap": real_gap,
                    "finger_depth": finger_depth,
                    "fingers_world": pts_world,
                    "primary_axis": best_vec,
                    "contact_mode": "upper",
                }
                if return_vec:
                    return world_tool_point, local_tool_point, geo
                return world_tool_point, local_tool_point, real_gap

        # 3. Fallback: Standard leaf or mesh-top point
        if not link.mesh:
            res = (link.t_world[:3, 3], np.zeros(3), None)
            return res if return_vec else (res[0], res[1], 0.0)

        parent_link = getattr(getattr(link, "parent_joint", None), "parent_link", None)
        origin_world = (
            np.array(parent_link.t_world[:3, 3], dtype=float)
            if parent_link is not None
            else np.zeros(3, dtype=float)
        )
        world_tip = upper_contact_point(link)
        if world_tip is None:
            world_tip = distal_point(link, origin_world)
        if world_tip is not None:
            inv_link = np.linalg.inv(link.t_world)
            local_tool_point = (inv_link @ np.append(world_tip, 1.0))[:3]
        else:
            bounds = link.mesh.bounds
            center_x = (bounds[0] + bounds[1]) / 2.0
            center_y = (bounds[2] + bounds[3]) / 2.0
            top_z = bounds[5]
            local_tool_point = np.array([center_x, center_y, top_z])
        world_tool_point = (link.t_world @ np.append(local_tool_point, 1.0))[:3]
        remember_auto_tcp(local_tool_point)
        if return_vec:
            return world_tool_point, local_tool_point, {"contact_mode": "upper"}
        return world_tool_point, local_tool_point, None

    def _refresh_auto_tcp_offset(self, tcp_link):
        """Update an automatic TCP offset from current world gripper geometry."""
        if tcp_link is None or getattr(tcp_link, "custom_tcp_offset", None) is not None:
            return
        try:
            before = None
            if getattr(tcp_link, "auto_tcp_offset", None) is not None:
                before = np.array(tcp_link.auto_tcp_offset, dtype=float).copy()
            self.get_link_tool_point(tcp_link)
            after = getattr(tcp_link, "auto_tcp_offset", None)
            if before is not None and after is not None and np.allclose(before, after, atol=1e-6):
                return
        except Exception:
            pass

    def show_speed_overlay(self):
        """Displays current speed percentage on the 3D canvas temporarily"""
        text = f"Speed: {self.current_speed}%"
        self.canvas.plotter.add_text(text, position='lower_right', font_size=12, color='#1976d2', name="speed_overlay")
        self.canvas.plotter.render()

    def on_tab_changed(self, index):
        is_links = index == self.panel_stack.indexOf(self.links_tab)
        self.canvas.enable_drag = is_links
        if hasattr(self, 'gripper_surface_btn'):
            self.gripper_surface_btn.setVisible(index == self.panel_stack.indexOf(self.joint_tab))
        widget = self.panel_stack.widget(index)
        if not widget: return
        if hasattr(self, 'gripper_tab') and widget == self.gripper_tab:
            self.gripper_tab.refresh_joints()
        if hasattr(widget, 'refresh_links'): widget.refresh_links()
        if hasattr(widget, 'update_display'): widget.update_display()
        if hasattr(widget, 'refresh_sliders'): widget.refresh_sliders()
        self.update_live_ui()

    def log(self, text):
        """Logs a message to the terminal with color-coded formatting."""
        import html as html_mod
        safe = html_mod.escape(str(text))
        lower = safe.lower()
        if (
            lower.startswith('error')
            or lower.startswith('python error')
            or any(k in lower for k in ['❌', 'fail', 'failed', 'crash'])
        ):
            color = '#f44336'
        elif any(k in lower for k in ['success', 'finished', 'loaded', 'saved', '✅']): color = '#4caf50'
        elif any(k in lower for k in ['warning', '⚠']): color = '#ff9800'
        else: color = '#d4d4d4'
        html = f'<span style="color:{color};">› {safe}</span>'
        self.console.append(html)
        if '#f44336' in color and not self.terminal_btn.isChecked():
            self.terminal_btn.setChecked(True)
            self.toggle_terminal()

    def toggle_terminal(self):
        """Show/hide the terminal console."""
        if self.terminal_btn.isChecked():
            self.console.setVisible(True)
            self.right_splitter.setSizes([500, 250])
        else:
            self.console.setVisible(False)
            self.right_splitter.setSizes([800, 0])

    def on_generate_code(self):
        """Generates cross-platform control code and populates the sidebar panel."""
        if not self.robot.joints:
            self.log("⚠️ No joints defined! Add some joints first.")
            self.show_toast("No joints defined yet", "warning")
            return
            
        from core.script_gen import generate_python_script
        
        arduino_code = generate_esp32_firmware(self.robot, default_speed=self.current_speed)
        python_code = generate_python_script(self.robot)
        
        self.code_drawer.set_codes(arduino_code, python_code)
        self.code_drawer.show()
        self.main_splitter.setSizes([350, 450, 400])
        self.log("⚡ Cross-Platform Code (ESP32/Python) Generated in Sidebar.")
        self.show_toast("Robot code built successfully", "success")

    def show_toast(self, message, toast_type='info', duration=3000):
        """Show an animated toast notification."""
        ToastNotification(self, message, toast_type, duration)

    # ─── Robot Movement & Animation ───────────────────────────────────

    def _start_joint_animation(self, joint_ids, child_names, target_deg_list, success=True, blocking=False):
        """Starts a smooth animation of multiple joints to target angles."""
        if not hasattr(self, '_anim_timer'):
            self._init_navigation_mixin()

        if self._anim_timer.isActive():
            self._anim_timer.stop()

        self._anim_success = success
        self._anim_joint_ids = list(joint_ids)
        self._anim_child_names = list(child_names)
        self._target_angles = list(target_deg_list)
        self._current_angles = []
        self._anim_blocking = blocking

        for joint_id in self._anim_joint_ids:
            joint = self.robot.joints.get(joint_id)
            self._current_angles.append(joint.current_value if joint else 0.0)

        # Disable some UI buttons during animation if needed
        # (Usually handled by panels, but we can emit a signal or set a flag)

        self._anim_timer.start(30) # 33 FPS roughly

        if blocking:
            # Simple spin-wait for scripts (must process events to keep UI alive)
            while self._anim_timer.isActive():
                QtWidgets.QApplication.processEvents()
                import time
                time.sleep(0.01)

    def _on_anim_tick(self):
        """Handles each step of the joint animation."""
        done = True
        # Speed scales with the global current_speed setting
        base_max_step = 3.0 * (self.current_speed / 50.0) 
        base_min_step = 0.15 * (self.current_speed / 50.0)
        ramp_dist = 18.0

        for idx, child_name in enumerate(self._anim_child_names):
            curr = self._current_angles[idx]
            target = self._target_angles[idx]
            diff = target - curr
            
            if abs(diff) < 0.05:
                next_angle = target
            else:
                step_mag = max(base_min_step, min(base_max_step, base_max_step * (abs(diff) / ramp_dist)))
                next_angle = curr + np.sign(diff) * min(abs(diff), step_mag)
                done = False
                
            self._current_angles[idx] = float(next_angle)
            self.robot.set_joint_value(self._anim_joint_ids[idx], self._current_angles[idx], propagate_relations=True)
            
            # Sync UI
            self._sync_joint_ui_globally(child_name, self._current_angles[idx])

        self.robot.update_kinematics()
        self.canvas.update_transforms(self.robot)
        self.update_live_ui(render=True)

        if done:
            self._anim_timer.stop()
            # If there was a pending "success" toast from IK
            if hasattr(self, 'show_toast'):
                self.show_toast("Target Reached", "success" if self._anim_success else "warning")
            
            # Update panels that might need a final refresh
            if hasattr(self, 'experiment_tab'):
                self.experiment_tab.update_display()

    def _sync_joint_ui_globally(self, child_name, angle_deg):
        """Synchronizes all UI components with a new joint value."""
        # 1. Joint Tab
        if hasattr(self, 'joint_tab'):
            if child_name in self.joint_tab.joints:
                self.joint_tab.joints[child_name]['current_angle'] = float(angle_deg)
                if self.joint_tab.active_joint_control == child_name:
                    self.joint_tab.joint_control_slider.blockSignals(True)
                    self.joint_tab.joint_control_slider.setValue(int(round(angle_deg * 10.0)))
                    self.joint_tab.joint_control_slider.blockSignals(False)
                    self.joint_tab.joint_control_spinbox.blockSignals(True)
                    self.joint_tab.joint_control_spinbox.setValue(float(angle_deg))
                    self.joint_tab.joint_control_spinbox.blockSignals(False)

        # 2. Experiment Tab (Matrices, IK/FK)
        if hasattr(self, 'experiment_tab'):
            self.experiment_tab.sync_slider(child_name, angle_deg)

    def _move_tcp_to_xyz(self, x_cm, y_cm, z_cm, tcp_link, blocking=False):
        """Solves IK for a target and starts smooth animation."""
        if not tcp_link:
            return False, "No TCP found"

        # 1. Calculate Target Pose
        self.get_link_tool_point(tcp_link)
        ratio = self.canvas.grid_units_per_cm
        target_world = np.array([x_cm, y_cm, z_cm]) * ratio
        
        target_tcp_pose = self.robot.get_tcp_world_pose(tcp_link)
        target_tcp_pose[:3, 3] = target_world

        # 2. Store old state
        old_angles = {name: joint.current_value for name, joint in self.robot.joints.items()}
        chain = self.robot.get_kinematic_chain(tcp_link)
        
        # 3. Solve IK with precision tolerance (0.01 cm = 0.1 mm)
        success, info = self.robot.inverse_kinematics_pose(
            target_tcp_pose,
            tcp_link,
            max_iters=3000,
            position_tolerance=0.01 * ratio,
            orientation_tolerance=1e6,
            orientation_weight=0.0,
            joint_change_weight=0.35,
        )

        if not success:
            for name, val in old_angles.items():
                self.robot.joints[name].current_value = val
            self.robot.update_kinematics()
            self.log(f"Not possible to reach target ({x_cm:.2f}, {y_cm:.2f}, {z_cm:.2f}) cm.")
            self.show_toast("Not possible", "warning")
            return False, info

        # 4. Extract results
        ordered_child_names = [joint.child_link.name for joint in chain]
        ordered_joint_ids = [joint.name for joint in chain]
        new_angles = [joint.current_value for joint in chain]
        
        # Log actual reached position
        actual_tcp_pose = self.robot.get_tcp_world_pose(tcp_link)
        actual_pos_cm = actual_tcp_pose[:3, 3] / ratio
        actual_error = np.linalg.norm(np.array(actual_pos_cm) - np.array([x_cm, y_cm, z_cm]))
        
        self.log(f"Target: ({x_cm:.3f}, {y_cm:.3f}, {z_cm:.3f}) cm")
        self.log(f"Reached: ({actual_pos_cm[0]:.3f}, {actual_pos_cm[1]:.3f}, {actual_pos_cm[2]:.3f}) cm")
        self.log(f"Position error: {actual_error:.3f} cm")
        
        if isinstance(info, dict) and "motion_total_abs_deg" in info:
            self.log(
                "Minimal-motion IK selected "
                f"{info['motion_total_abs_deg']:.2f} deg total joint travel "
                f"(max single joint {info.get('motion_max_abs_deg', 0.0):.2f} deg)."
            )

        # 5. Revert for animation
        for name, value in old_angles.items():
            self.robot.joints[name].current_value = value
        self.robot.update_kinematics()

        # 6. Start Animation - only animate joints that actually moved
        moved_joint_ids = []
        moved_child_names = []
        moved_targets = []
        
        for jid, cname, n_ang in zip(ordered_joint_ids, ordered_child_names, new_angles):
            o_ang = old_angles[jid]
            if abs(n_ang - o_ang) > 0.001:
                moved_joint_ids.append(jid)
                moved_child_names.append(cname)
                moved_targets.append(n_ang)

        if moved_joint_ids:
            self._start_joint_animation(moved_joint_ids, moved_child_names, moved_targets, success=success, blocking=blocking)
        else:
            self.log("Target already reached.")
            
        return success, info

    def move_joint_animated(self, joint_id, target_angle, blocking=False):
        """Moves a single joint smoothly."""
        joint = self.robot.joints.get(joint_id)
        if not joint:
            return False
        
        child_name = joint.child_link.name if joint.child_link else joint_id
        self._start_joint_animation([joint_id], [child_name], [target_angle], blocking=blocking)
        return True

    def apply_styles(self):
        self.setStyleSheet("""
            QMainWindow, QWidget { background-color: #f5f5f5; color: #212121; font-family: 'Segoe UI', Roboto, sans-serif; font-size: 18px; }
            QPushButton { background-color: white; border: 2px solid #e0e0e0; padding: 10px 15px; border-radius: 8px; color: #212121; font-weight: bold; }
            QPushButton:hover { color: #1976d2; border-color: #1976d2; }
            QListWidget { background-color: white; border: 1px solid #bbb; }
            QTextEdit { background-color: white; color: #1565c0; font-family: 'Consolas', monospace; border: 1px solid #bbb; }
            QSplitter::handle { background-color: #bbb; }
            QSplitter::handle:hover { background-color: #1976d2; }
        """)

