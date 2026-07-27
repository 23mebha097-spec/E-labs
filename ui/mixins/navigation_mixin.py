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
        
        if hasattr(self, "align_tab"):
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
        matrix = np.array(matrix, dtype=float)

        if link.parent_joint:
            # Solve for local offset in parent-joint frame.
            parent_world = np.array(link.parent_joint.parent_link.t_world, dtype=float)
            joint_rot = np.array(link.parent_joint.get_matrix(), dtype=float)

            # Child_World = Parent_World @ Joint_Matrix @ Child_Offset
            # => Child_Offset = Inv(Joint_Matrix) @ Inv(Parent_World) @ Matrix
            inv_parent = np.linalg.inv(parent_world)
            inv_joint = np.linalg.inv(joint_rot)
            link.t_offset = inv_joint @ inv_parent @ matrix
        else:
            # Root/floating link: preserve the assembly root pose.
            link.t_offset = matrix

        self.robot.update_kinematics()
        self.update_link_colors()
        self.log(f"Synced coordinates for: {name}")
        # Re-run kinematics so any children follow the moved base/root link.
        self.robot.update_kinematics()
        self.canvas.update_transforms(self.robot)
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

    def update_live_ui(self, render=True, force_top_face=False):
        """Updates the Live Point (LP) coordinates and handles Pick-and-Place simulation logic."""
        tcp_link = None
        if hasattr(self, "_get_preferred_tcp_link"):
            tcp_link = self._get_preferred_tcp_link()

        # Use global live-point lock if present, otherwise fall back to sim panel lock.
        locked = False
        pos = None
        ratio = self.canvas.grid_units_per_cm
        sim_panel = getattr(self, 'simulation_tab', None)

        if getattr(self, 'live_point_locked', False) and getattr(self, 'locked_live_point', None) is not None:
            lx, ly, lz = self.locked_live_point
            self.current_live_point_cm = (lx, ly, lz)
            pos = np.array([lx, ly, lz]) * ratio
            locked = True
        elif sim_panel and getattr(sim_panel, 'live_point_locked', False) and getattr(sim_panel, 'locked_live_point', None) is not None:
            lx, ly, lz = sim_panel.locked_live_point
            self.current_live_point_cm = (lx, ly, lz)
            pos = np.array([lx, ly, lz]) * ratio
            locked = True

        if not locked:
            # Keep the live-point marker dormant until the user explicitly finalizes
            # the robot assembly with Make Robo. This prevents a default TCP/live point
            # from appearing immediately on startup.
            if not getattr(self, "robot_finalized", False):
                self.current_live_point_cm = None
                lx = ly = lz = 0.0
                if hasattr(self.canvas, "clear_live_point_marker"):
                    self.canvas.clear_live_point_marker()
                if hasattr(self.canvas, "clear_live_tcp_marker"):
                    self.canvas.clear_live_tcp_marker()
                if hasattr(self.canvas, "update_hud_coords"):
                    self.canvas.update_hud_coords(lx, ly, lz, status="PENDING", render=render)
                return

            if hasattr(self, "robot") and getattr(self.robot, "links", None):
                if force_top_face and not self._tcp_link_uses_explicit_live_point(tcp_link):
                    pos = self._compute_robot_top_face_center_point()
                if pos is None:
                    self._refresh_auto_tcp_offset(tcp_link)
                    pos = self._get_live_point_world(tcp_link)

                if pos is None:
                    self.current_live_point_cm = None
                    lx = ly = lz = 0.0
                    if hasattr(self.canvas, "clear_live_point_marker"):
                        self.canvas.clear_live_point_marker()
                    if hasattr(self.canvas, "clear_live_tcp_marker"):
                        self.canvas.clear_live_tcp_marker()
                    if hasattr(self.canvas, "update_hud_coords"):
                        self.canvas.update_hud_coords(lx, ly, lz, status="PENDING", render=render)
                    return

                lx, ly, lz = pos[0] / ratio, pos[1] / ratio, pos[2] / ratio
                self.current_live_point_cm = (lx, ly, lz)
                locked = False
            else:
                self.current_live_point_cm = None
                lx = ly = lz = 0.0
                if hasattr(self.canvas, "clear_live_point_marker"):
                    self.canvas.clear_live_point_marker()
                if hasattr(self.canvas, "clear_live_tcp_marker"):
                    self.canvas.clear_live_tcp_marker()
                if hasattr(self.canvas, "update_hud_coords"):
                    self.canvas.update_hud_coords(lx, ly, lz, status="PENDING", render=render)
                return
        else:
            lx, ly, lz = pos[0] / ratio, pos[1] / ratio, pos[2] / ratio
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
            status = "🔒 LOCKED" if locked else "LIVE"
            self.canvas.update_hud_coords(lx, ly, lz, status=status, render=render)
        if hasattr(self.canvas, 'update_live_point_marker'):
            # Use the exact grid-unit position for marker alignment
            self.canvas.update_live_point_marker(pos, render=render)

        # Drive the visible red TCP marker from the live coordinates every refresh.
        if hasattr(self.canvas, 'update_live_tcp_marker'):
            self.canvas.update_live_tcp_marker(pos)

        # If we are not locked anymore, drop any stale fixed marker so only the live marker remains.
        if not locked and hasattr(self.canvas, 'plotter'):
            try:
                if "fixed_live_point_marker" in self.canvas.plotter.renderer.actors:
                    self.canvas.plotter.remove_actor("fixed_live_point_marker")
                    if render:
                        self.canvas.plotter.render()
            except Exception:
                pass

        # Pick-and-Place Simulation Logic (MAGNET MODE)
        # Uses mesh-based tool point for accurate contact geometry.
        sim_tab = getattr(self, 'simulation_tab', None)
        if (
            sim_tab
            and hasattr(sim_tab, 'is_sim_active')
            and sim_tab.is_sim_active
            and getattr(sim_tab, "active_operation", None) is None
            and tcp_link is not None
        ):
            sim_pos, _, _ = self.get_link_tool_point(tcp_link)
            self._handle_sim_pick_place(tcp_link, sim_pos, ratio)

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

    def _saved_gripper_definition(self):
        """Return the active saved Gripper Tool definition."""
        payload = getattr(self, "gripper_tool_config", None)
        if not isinstance(payload, dict):
            payload = getattr(self, "end_effector_tool_config", None)
        if not isinstance(payload, dict):
            return {}
        definition = payload.get("EndEffector", payload)
        if not isinstance(definition, dict):
            return {}
        if str(definition.get("ToolType", "")).strip().lower() != "gripper tool":
            return {}
        return definition

    def _configured_gripper_joint_names(self):
        """Resolve saved jaw joints, including independent joints without relations."""
        definition = self._saved_gripper_definition()
        names = [
            str(jaw.get("JointID"))
            for jaw in definition.get("Jaws", [])
            if isinstance(jaw, dict) and jaw.get("JointID")
        ]
        names.extend(getattr(self, "active_gripper_joint_names", []) or [])
        names.extend(
            name for name, joint in self.robot.joints.items()
            if getattr(joint, "is_gripper", False)
        )

        unique = []
        seen = set()
        for name in names:
            if name in seen or name not in self.robot.joints:
                continue
            unique.append(name)
            seen.add(name)
        return unique

    def _saved_gripper_jaw_map(self):
        """Return saved jaw settings keyed by joint ID."""
        definition = self._saved_gripper_definition()
        return {
            str(jaw.get("JointID")): jaw
            for jaw in definition.get("Jaws", [])
            if isinstance(jaw, dict) and jaw.get("JointID")
        }

    def ensure_saved_gripper_tcp(self):
        """Restore the saved gripper TCP using the centroid of all saved jaws."""
        definition = self._saved_gripper_definition()
        jaw_map = self._saved_gripper_jaw_map()
        anchors = []
        face_centers = []
        for joint_name in self._configured_gripper_joint_names():
            joint = self.robot.joints.get(joint_name)
            jaw = jaw_map.get(joint_name, {})
            local_center = jaw.get("FaceCenterLocal")
            if joint is None or joint.child_link is None or local_center is None:
                continue
            try:
                local_center = np.asarray(local_center, dtype=float).reshape(3)
            except Exception:
                continue
            face_centers.append(
                (np.asarray(joint.child_link.t_world, dtype=float) @ np.append(local_center, 1.0))[:3]
            )
            anchor = joint.parent_link
            if hasattr(self, "_resolve_rigid_tcp_link"):
                anchor = self._resolve_rigid_tcp_link(joint.child_link) or anchor
            if anchor is not None and anchor not in anchors:
                anchors.append(anchor)

        if anchors and len(face_centers) >= 2:
            tcp_link = max(anchors, key=lambda link: len(self.robot.get_kinematic_chain(link)))
            midpoint_world = np.mean(np.asarray(face_centers, dtype=float), axis=0)
            midpoint_local = (
                np.linalg.inv(np.asarray(tcp_link.t_world, dtype=float))
                @ np.append(midpoint_world, 1.0)
            )[:3]
            self.robot.set_tcp_transform(tcp_link.name, position=midpoint_local)
            self.robot.ensure_tcp_transform(tcp_link)
            self.custom_tcp_name = tcp_link.name
            return tcp_link

        alignment_face = definition.get("BaseAlignmentFace")
        if isinstance(alignment_face, dict):
            tcp_name = alignment_face.get("TCPLink") or definition.get("TCPLink")
            tcp_link = self.robot.links.get(tcp_name)
            link_local_center = alignment_face.get("FaceCenterLinkLocal")
            if tcp_link is not None and link_local_center is not None:
                try:
                    link_local_center = np.asarray(
                        link_local_center, dtype=float
                    ).reshape(3)
                except (TypeError, ValueError):
                    link_local_center = None
                if link_local_center is not None:
                    self.robot.set_tcp_transform(
                        tcp_link.name, position=link_local_center
                    )
                    self.robot.ensure_tcp_transform(tcp_link)
                    self.custom_tcp_name = tcp_link.name
                    return tcp_link

        return None

    def _gripper_control_joint_names(self):
        """Return independent gripper controls while avoiding duplicate relation slaves."""
        configured = self._configured_gripper_joint_names()
        configured_set = set(configured)
        slave_master = {}
        for master_name, slaves in self.robot.joint_relations.items():
            for slave_name, _ in slaves:
                slave_master[slave_name] = master_name
        return [
            name for name in configured
            if slave_master.get(name) not in configured_set
        ]

    def _apply_gripper_control_value(self, joint_name, value):
        """Rotate a selected jaw joint and every relation-linked jaw."""
        joint = self.robot.joints.get(joint_name)
        if joint is None:
            return
        joint.current_value = float(np.clip(value, joint.min_limit, joint.max_limit))
        for slave_name, ratio in self.robot.joint_relations.get(joint_name, []):
            slave = self.robot.joints.get(slave_name)
            if slave is None:
                continue
            slave_value = joint.current_value * float(ratio)
            slave.current_value = float(np.clip(slave_value, slave.min_limit, slave.max_limit))

    def _apply_gripper_target_values(self, targets):
        """Apply a complete jaw target map without relation members overwriting each other."""
        clean_targets = {
            name: float(value)
            for name, value in (targets or {}).items()
            if name in self.robot.joints
        }
        explicit_names = set(clean_targets)

        # Every explicitly configured jaw owns its calculated angle. This is
        # essential for mirrored master/slave jaws with different endpoints.
        for joint_name, value in clean_targets.items():
            joint = self.robot.joints[joint_name]
            joint.current_value = float(np.clip(value, joint.min_limit, joint.max_limit))

        # Preserve legacy relation-only grippers: propagate to slaves only when
        # the slave does not already have an explicit per-jaw target.
        for master_name, value in clean_targets.items():
            for slave_name, ratio in self.robot.joint_relations.get(master_name, []):
                if slave_name in explicit_names:
                    continue
                slave = self.robot.joints.get(slave_name)
                if slave is None:
                    continue
                slave_value = value * float(ratio)
                slave.current_value = float(np.clip(slave_value, slave.min_limit, slave.max_limit))

    def _saved_gripper_angle_bounds(self, joint_name, jaw_map, definition):
        """Resolve closed/open bounds, repairing old zero-span relation slaves."""
        joint = self.robot.joints[joint_name]
        jaw = jaw_map.get(joint_name, {})
        closed = jaw.get("ClosedAngle", definition.get("MinOpening", joint.min_limit))
        opened = jaw.get("OpenAngle", definition.get("MaxOpening", joint.max_limit))
        if closed is None:
            closed = joint.min_limit
        if opened is None:
            opened = joint.max_limit
        closed = float(np.clip(float(closed), joint.min_limit, joint.max_limit))
        opened = float(np.clip(float(opened), joint.min_limit, joint.max_limit))

        if abs(opened - closed) < 1e-9:
            for master_name, slaves in self.robot.joint_relations.items():
                relation = next(
                    ((slave_name, float(ratio)) for slave_name, ratio in slaves if slave_name == joint_name),
                    None,
                )
                if relation is None or master_name not in self.robot.joints:
                    continue
                ratio = relation[1]
                master = self.robot.joints[master_name]
                master_jaw = jaw_map.get(master_name, {})
                master_closed = master_jaw.get("ClosedAngle", definition.get("MinOpening", master.min_limit))
                master_opened = master_jaw.get("OpenAngle", definition.get("MaxOpening", master.max_limit))
                if master_closed is None or master_opened is None:
                    continue
                repaired_closed = float(np.clip(float(master_closed) * ratio, joint.min_limit, joint.max_limit))
                repaired_opened = float(np.clip(float(master_opened) * ratio, joint.min_limit, joint.max_limit))
                if abs(repaired_opened - repaired_closed) > 1e-9:
                    closed, opened = repaired_closed, repaired_opened
                    break
        return closed, opened

    def set_gripper_opening_percent(self, percent, apply=True):
        """Set all gripper jaws with one normalized opening command."""
        definition = self._saved_gripper_definition()
        jaw_map = self._saved_gripper_jaw_map()
        configured_names = self._configured_gripper_joint_names()
        has_saved_endpoints = any(
            "ClosedAngle" in jaw or "OpenAngle" in jaw
            for jaw in jaw_map.values()
        )
        explicit_names = [
            name for name in configured_names
            if has_saved_endpoints and name in jaw_map
        ]
        control_names = explicit_names or self._gripper_control_joint_names()
        if not control_names:
            return {}

        fraction = float(np.clip(percent, 0.0, 100.0)) / 100.0
        targets = {}
        for joint_name in control_names:
            joint = self.robot.joints.get(joint_name)
            if joint is None:
                continue
            closed, opened = self._saved_gripper_angle_bounds(joint_name, jaw_map, definition)
            target = closed + fraction * (opened - closed)
            targets[joint_name] = float(target)

        if apply:
            self._apply_gripper_target_values(targets)
            self.robot.update_kinematics()
            self.canvas.update_transforms(self.robot)
        return targets

    def get_gripper_opening_percent(self):
        """Estimate the current shared slider percentage from all saved jaws."""
        jaw_map = self._saved_gripper_jaw_map()
        definition = self._saved_gripper_definition()
        percentages = []
        for joint_name in self._configured_gripper_joint_names():
            joint = self.robot.joints.get(joint_name)
            if joint is None:
                continue
            closed, opened = self._saved_gripper_angle_bounds(joint_name, jaw_map, definition)
            span = float(opened) - float(closed)
            if abs(span) < 1e-9:
                continue
            percentages.append((float(joint.current_value) - float(closed)) / span * 100.0)
        if not percentages:
            return 0.0
        return float(np.clip(np.mean(percentages), 0.0, 100.0))

    def _control_gripper_fingers(self, close=True, target_gap_world=None, apply=True):
        """
        Move every saved gripper jaw joint so its child object opens or closes.

        MinOpening is the closed position and MaxOpening is the open position.
        Relation-linked jaws are driven by their master; independent selected
        jaws are controlled directly.
        """
        targets = self.set_gripper_opening_percent(0.0 if close else 100.0, apply=apply)
        return None if apply else targets

    # ─── End Simulation Helpers ───────────────────────────────────────

    def get_link_tool_point(self, link, return_vec=False):
        """Return the current TCP world position and local offset for the given link."""
        if not link:
            if return_vec:
                return np.zeros(3), np.zeros(3), None
            return np.zeros(3), np.zeros(3), 0.0

        if hasattr(self, "robot") and getattr(self.robot, "get_tcp_world_pose", None) is not None:
            try:
                link_name = getattr(link, "name", None)
                if link_name in self.robot.links:
                    tcp_pose = self.robot.get_tcp_world_pose(link)
                    local_tcp = self.robot.get_tcp_local_transform(link)[:3, 3]
                    if return_vec:
                        return tcp_pose[:3, 3].copy(), local_tcp.copy(), None
                    return tcp_pose[:3, 3].copy(), local_tcp.copy(), 0.0
            except Exception:
                pass

        return np.zeros(3), np.zeros(3), None if return_vec else 0.0

    def _get_live_point_world(self, tcp_link):
        """Return the current live point in this robot pose in world coordinates."""
        if tcp_link is not None and hasattr(self, 'get_link_tool_point'):
            try:
                candidate, _, _ = self.get_link_tool_point(tcp_link)
                candidate = np.asarray(candidate, dtype=float).reshape(3)
                if np.linalg.norm(candidate) > 1e-9 and self._tcp_link_uses_explicit_live_point(tcp_link):
                    return candidate
            except Exception:
                pass

        if self._tcp_link_uses_explicit_live_point(tcp_link):
            try:
                tcp_pose = self.robot.get_tcp_world_pose(tcp_link)
                return tcp_pose[:3, 3].copy()
            except Exception:
                pass

        top_point = self._compute_robot_top_face_center_point()
        if top_point is not None:
            return top_point

        return np.zeros(3, dtype=float)

    def _tcp_link_uses_explicit_live_point(self, tcp_link):
        """Return True if the selected TCP link is intended to define the robot live point."""
        if tcp_link is None:
            return False
        if getattr(tcp_link, 'custom_tcp_offset', None) is not None:
            return True
        if getattr(self, 'custom_tcp_name', None) == getattr(tcp_link, 'name', None):
            return True
        return False

    def _get_mesh_face_centers(self, mesh):
        """Return an ndarray of face center points for a supported mesh object."""
        try:
            import trimesh
            if isinstance(mesh, trimesh.Trimesh):
                if hasattr(mesh, 'triangles_center'):
                    return np.asarray(mesh.triangles_center, dtype=float)
                if hasattr(mesh, 'faces') and hasattr(mesh, 'vertices'):
                    faces = np.asarray(mesh.faces, dtype=int)
                    verts = np.asarray(mesh.vertices, dtype=float)
                    if faces.ndim == 2 and faces.shape[1] >= 3:
                        return np.mean(verts[faces[:, :3]], axis=1)
        except Exception:
            pass

        try:
            import pyvista as pv
            if hasattr(mesh, 'cell_centers'):
                centers = mesh.cell_centers().points
                if centers is not None and len(centers):
                    return np.asarray(centers, dtype=float)
        except Exception:
            pass

        if hasattr(mesh, 'faces') and hasattr(mesh, 'vertices'):
            faces = np.asarray(mesh.faces)
            verts = np.asarray(mesh.vertices, dtype=float)
            if faces.ndim == 2 and faces.shape[1] >= 3:
                return np.mean(verts[faces[:, :3]], axis=1)
            if faces.ndim == 1 and faces.size > 0:
                # Support flat pyvista face arrays: [n, i0, i1, i2, n, j0, j1, j2, ...]
                face_centers = []
                idx = 0
                while idx < faces.size:
                    count = int(faces[idx])
                    if count < 3 or idx + count >= faces.size:
                        break
                    indices = faces[idx + 1: idx + 1 + count]
                    face_centers.append(np.mean(verts[indices[:3]], axis=0))
                    idx += count + 1
                if face_centers:
                    return np.asarray(face_centers, dtype=float)

        return None

    def _compute_robot_top_face_center_point(self):
        """Compute the topmost face center for the assembled robot in world coordinates."""
        if not hasattr(self, "robot") or not getattr(self.robot, "links", None):
            return None

        top_centers = []
        top_z = -np.inf
        for link in self.robot.links.values():
            mesh = getattr(link, 'mesh', None)
            if mesh is None:
                continue

            transform = np.asarray(getattr(link, 't_world', np.eye(4)), dtype=float)
            if transform.shape != (4, 4):
                continue

            centers = self._get_mesh_face_centers(mesh)
            if centers is None or len(centers) == 0:
                continue

            homogeneous = np.hstack((centers, np.ones((len(centers), 1), dtype=float)))
            world_centers = (transform @ homogeneous.T).T[:, :3]
            if world_centers.size == 0:
                continue

            link_top_z = np.max(world_centers[:, 2])
            if link_top_z > top_z + 1e-9:
                top_z = link_top_z
                top_centers = [world_centers[world_centers[:, 2] >= link_top_z - 1e-6]]
            elif abs(link_top_z - top_z) <= 1e-6:
                top_centers.append(world_centers[world_centers[:, 2] >= top_z - 1e-6])

        if not top_centers:
            return None

        top_centers = np.vstack(top_centers)
        return np.mean(top_centers, axis=0)

    def _compute_robot_top_face_center_point_data(self):
        """Return the topmost face center point, its link name, and local link coordinates."""
        if not hasattr(self, "robot") or not getattr(self.robot, "links", None):
            return None

        top_z = -np.inf
        best_link = None
        best_centers = None

        for link in self.robot.links.values():
            mesh = getattr(link, 'mesh', None)
            if mesh is None:
                continue

            transform = np.asarray(getattr(link, 't_world', np.eye(4)), dtype=float)
            if transform.shape != (4, 4):
                continue

            centers = self._get_mesh_face_centers(mesh)
            if centers is None or len(centers) == 0:
                continue

            homogeneous = np.hstack((centers, np.ones((len(centers), 1), dtype=float)))
            world_centers = (transform @ homogeneous.T).T[:, :3]
            if world_centers.size == 0:
                continue

            link_top_z = np.max(world_centers[:, 2])
            if link_top_z > top_z + 1e-9:
                top_z = link_top_z
                best_link = link
                best_centers = world_centers[world_centers[:, 2] >= link_top_z - 1e-6]
            elif abs(link_top_z - top_z) <= 1e-6 and best_centers is not None:
                extra = world_centers[world_centers[:, 2] >= top_z - 1e-6]
                if extra.size:
                    best_centers = np.vstack((best_centers, extra))

        if best_link is None or best_centers is None or len(best_centers) == 0:
            return None

        top_point = np.mean(best_centers, axis=0)
        inv_transform = np.linalg.inv(np.asarray(getattr(best_link, 't_world', np.eye(4)), dtype=float))
        local = (inv_transform @ np.append(top_point, 1.0))[:3]
        return top_point, best_link.name, local

    def _refresh_auto_tcp_offset(self, tcp_link):
        """Keep the TCP offset as a fixed local point on the selected link."""
        if tcp_link is None:
            return
        if getattr(tcp_link, "custom_tcp_offset", None) is not None:
            if getattr(tcp_link, "auto_tcp_offset", None) is None:
                tcp_link.auto_tcp_offset = np.array(tcp_link.custom_tcp_offset, dtype=float).copy()
            return
        if getattr(tcp_link, "auto_tcp_offset", None) is None:
            tcp_link.auto_tcp_offset = np.zeros(3, dtype=float)

    def show_speed_overlay(self):
        """Displays current speed percentage on the 3D canvas temporarily"""
        text = f"Speed: {self.current_speed}%"
        self.canvas.plotter.add_text(text, position='lower_right', font_size=12, color='#1976d2', name="speed_overlay")
        self.canvas.plotter.render()

    def on_tab_changed(self, index):
        is_links = index == self.panel_stack.indexOf(self.links_tab)
        self.canvas.enable_drag = is_links
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


