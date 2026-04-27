from PyQt5 import QtWidgets, QtCore
import numpy as np


class TypeOnlyDoubleSpinBox(QtWidgets.QDoubleSpinBox):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def stepBy(self, steps):
        pass

    def wheelEvent(self, event):
        event.ignore()


class GripperPanel(QtWidgets.QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.mw = main_window
        self.init_ui()

    def _group_style(self):
        return """
            QGroupBox {
                border: 1px solid #e0e0e0;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 15px;
                font-weight: bold;
                color: #616161;
            }
        """

    def _surface_list_style(self):
        return """
            QListWidget {
                border: 1px solid #ddd;
                border-radius: 4px;
                background: white;
                font-size: 12px;
            }
            QListWidget::item {
                padding: 6px;
                border-bottom: 1px solid #f5f5f5;
            }
            QListWidget::item:selected {
                background: #e8f5e9;
                color: #2e7d32;
            }
        """

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(12)

        header = QtWidgets.QLabel("GRIPPER CONTROL")
        header.setStyleSheet(
            "font-weight: bold; font-size: 16px; color: #2e7d32; margin-bottom: 5px;"
        )
        layout.addWidget(header)

        # --- MAKE ROBO BUTTON ---
        self.make_robo_btn = QtWidgets.QPushButton("🚀 Make Robo")
        self.make_robo_btn.setFixedHeight(45)
        self.make_robo_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.make_robo_btn.setStyleSheet("""
            QPushButton {
                background-color: #1976d2;
                color: white;
                border: none;
                border-radius: 8px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover { background-color: #1565c0; }
            QPushButton:pressed { background-color: #0d47a1; }
        """)
        self.make_robo_btn.clicked.connect(self.on_make_robo)
        layout.addWidget(self.make_robo_btn)

        selection_group = QtWidgets.QGroupBox("1. SELECT GRIPPER JOINT")
        selection_group.setStyleSheet(self._group_style())
        sel_layout = QtWidgets.QVBoxLayout(selection_group)

        self.joints_list = QtWidgets.QListWidget()
        self.joints_list.setFixedHeight(120)
        self.joints_list.setStyleSheet(self._surface_list_style())
        self.joints_list.itemClicked.connect(self.on_joint_selected)
        sel_layout.addWidget(self.joints_list)

        self.mark_gripper_check = QtWidgets.QCheckBox("Mark as Gripper")
        self.mark_gripper_check.setStyleSheet(
            "font-weight: bold; color: #2e7d32; padding: 5px;"
        )
        self.mark_gripper_check.toggled.connect(self.on_mark_toggled)
        sel_layout.addWidget(self.mark_gripper_check)

        layout.addWidget(selection_group)

        control_group = QtWidgets.QGroupBox("2. MANUAL ACTIONS")
        control_group.setStyleSheet(self._group_style())
        ctrl_layout = QtWidgets.QVBoxLayout(control_group)

        ctrl_layout.addWidget(QtWidgets.QLabel("Precision Stroke:"))
        self.stroke_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.stroke_slider.setRange(0, 100)
        self.stroke_slider.setStyleSheet("""
            QSlider::groove:horizontal { height: 6px; background: #eee; border-radius: 3px; }
            QSlider::handle:horizontal {
                background: white;
                border: 2px solid #2e7d32;
                width: 14px;
                height: 14px;
                margin-top: -5px;
                border-radius: 7px;
            }
        """)
        self.stroke_slider.valueChanged.connect(self.on_stroke_changed)
        ctrl_layout.addWidget(self.stroke_slider)

        layout.addWidget(control_group)

        compute_group = QtWidgets.QGroupBox("3. GRIPPER COMPUTE")
        compute_group.setStyleSheet(self._group_style())
        compute_layout = QtWidgets.QVBoxLayout(compute_group)

        compute_hint = QtWidgets.QLabel(
            "Select a gripper pair and press Compute to analyze the jaw shape, opening axis, and pick-and-place fit."
        )
        compute_hint.setWordWrap(True)
        compute_hint.setStyleSheet("color: #616161; font-size: 12px;")
        compute_layout.addWidget(compute_hint)

        self.compute_btn = QtWidgets.QPushButton("Compute")
        self.compute_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.compute_btn.setStyleSheet("""
            QPushButton {
                background-color: #2e7d32;
                color: white;
                border: none;
                border-radius: 6px;
                padding: 8px 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #25692a;
            }
            QPushButton:disabled {
                background-color: #c8e6c9;
                color: #7f8b7f;
            }
        """)
        self.compute_btn.clicked.connect(self.on_compute_gripper_clicked)
        compute_layout.addWidget(self.compute_btn)

        self.compute_table = QtWidgets.QTableWidget(0, 2)
        self.compute_table.setHorizontalHeaderLabels(["Metric", "Value"])
        self.compute_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.compute_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.compute_table.setFocusPolicy(QtCore.Qt.NoFocus)
        self.compute_table.setAlternatingRowColors(True)
        self.compute_table.horizontalHeader().setStretchLastSection(True)
        self.compute_table.verticalHeader().setVisible(False)
        self.compute_table.setMinimumHeight(180)
        self.compute_table.setStyleSheet("""
            QTableWidget {
                background: white;
                border: 1px solid #e0e0e0;
                border-radius: 6px;
                gridline-color: #eeeeee;
                font-size: 12px;
            }
            QHeaderView::section {
                background: #f1f8e9;
                color: #2e7d32;
                border: none;
                padding: 6px;
                font-weight: bold;
            }
        """)
        compute_layout.addWidget(self.compute_table)

        self.compute_note_label = QtWidgets.QLabel(
            "Shape profile only. Force/torque and motor load are not computed here."
        )
        self.compute_note_label.setWordWrap(True)
        self.compute_note_label.setStyleSheet("color: #757575; font-size: 12px;")
        compute_layout.addWidget(self.compute_note_label)

        layout.addWidget(compute_group)

        layout.addStretch()

    def _selected_joint_name(self):
        item = self.joints_list.currentItem()
        if not item:
            return None
        return item.data(QtCore.Qt.UserRole)

    def _selected_group_members(self):
        item = self.joints_list.currentItem()
        if not item:
            return []

        members = item.data(QtCore.Qt.UserRole + 1)
        if not isinstance(members, list) or not members:
            selected = item.data(QtCore.Qt.UserRole)
            members = [selected] if isinstance(selected, str) else []

        return [name for name in members if isinstance(name, str) and name in self.mw.robot.joints]

    def _refresh_compute_ui(self):
        if not hasattr(self, "compute_btn") or not hasattr(self, "compute_table"):
            return

        item = self.joints_list.currentItem()
        members = self._selected_group_members()
        selected_name = item.text() if item else "the selected pair"
        pair_ready = len(members) >= 2 and any(
            getattr(self.mw.robot.joints.get(name), "is_gripper", False) for name in members
        )
        object_ready = self._selected_sim_object_name() is not None

        self.compute_btn.setEnabled(pair_ready)
        if pair_ready and object_ready:
            self.compute_note_label.setText(
                f"Ready to compute for {selected_name}. Object data will be included if selected."
            )
        elif pair_ready:
            self.compute_note_label.setText(
                "Ready to compute gripper opening for the selected pair. Select an imported object to also check fit."
            )
        else:
            self.compute_note_label.setText(
                "Select a marked gripper pair to enable the compute button."
            )

        self.compute_table.setRowCount(0)

    def _selected_sim_object_name(self):
        if hasattr(self.mw, "sim_objects_list"):
            item = self.mw.sim_objects_list.currentItem()
            if item:
                return item.text()

        sim_tab = getattr(self.mw, "simulation_tab", None)
        if sim_tab is not None and hasattr(sim_tab, "objects_list"):
            item = sim_tab.objects_list.currentItem()
            if item:
                return item.text()

        return None

    def _selected_sim_object_summary(self):
        sim_tab = getattr(self.mw, "simulation_tab", None)
        obj_name = self._selected_sim_object_name()
        if sim_tab is None or obj_name is None or obj_name not in self.mw.robot.links:
            return None

        link = self.mw.robot.links[obj_name]
        ratio = getattr(self.mw.canvas, "grid_units_per_cm", 1.0) or 1.0

        if hasattr(sim_tab, "refresh_object_info"):
            sim_tab.refresh_object_info(obj_name)

        def _coords_from_spinboxes(xs, ys, zs):
            return (
                float(xs.value()) if xs is not None else 0.0,
                float(ys.value()) if ys is not None else 0.0,
                float(zs.value()) if zs is not None else 0.0,
            )

        pick = _coords_from_spinboxes(
            getattr(sim_tab, "pick_x", None),
            getattr(sim_tab, "pick_y", None),
            getattr(sim_tab, "pick_z", None),
        )
        place = _coords_from_spinboxes(
            getattr(sim_tab, "place_x", None),
            getattr(sim_tab, "place_y", None),
            getattr(sim_tab, "place_z", None),
        )
        live = _coords_from_spinboxes(
            getattr(sim_tab, "live_x", None),
            getattr(sim_tab, "live_y", None),
            getattr(sim_tab, "live_z", None),
        )
        dims = _coords_from_spinboxes(
            getattr(sim_tab, "obj_width", None),
            getattr(sim_tab, "obj_depth", None),
            getattr(sim_tab, "obj_height", None),
        )

        grip_width_cm = None
        grip_height_cm = None
        if hasattr(sim_tab, "_get_object_grip_width"):
            try:
                grip_width, z_offset, _ = sim_tab._get_object_grip_width()
                if grip_width is not None:
                    grip_width_cm = float(grip_width) / ratio
                if z_offset is not None:
                    grip_height_cm = float(z_offset) / ratio
            except Exception:
                grip_width_cm = None
                grip_height_cm = None

        current_pos = tuple((link.t_world[:3, 3] / ratio).tolist())

        return {
            "name": obj_name,
            "pick": pick,
            "place": place,
            "live": live,
            "dims": dims,
            "current_pos": current_pos,
            "grip_width_cm": grip_width_cm,
            "grip_height_cm": grip_height_cm,
        }

    def _format_cm(self, value):
        return f"{value:.2f} cm"

    def _format_deg(self, value):
        return f"{value:.1f} deg"

    def _axis_label(self, vec):
        if vec is None:
            return "Unknown"

        arr = np.array(vec, dtype=float)
        norm = float(np.linalg.norm(arr))
        if norm < 1e-9:
            return "Unknown"

        arr /= norm
        idx = int(np.argmax(np.abs(arr)))
        axis_name = "XYZ"[idx]
        sign = "+" if arr[idx] >= 0 else "-"
        return f"{axis_name}{sign}"

    def _gripper_shape_profile(self, anchor_link, summary):
        ratio = getattr(self.mw.canvas, "grid_units_per_cm", 1.0) or 1.0
        geo_data = None
        finger_count = 0
        span_axis = "Unknown"
        approach_axis = "Unknown"
        shape_label = "Unknown"
        grip_style = "Generic contact"
        contact_mode = "Fallback"
        strategy = "Center the object on the TCP before closing."
        reach_cm = 0.0
        span_cm = summary["max_gap_cm"]

        if anchor_link is not None and hasattr(self.mw, "get_link_tool_point"):
            try:
                _, _, geo_data = self.mw.get_link_tool_point(anchor_link, return_vec=True)
            except Exception:
                geo_data = None

        if isinstance(geo_data, dict):
            finger_count = len(geo_data.get("fingers_world", []) or [])
            span_axis = self._axis_label(geo_data.get("primary_axis"))
            approach_axis = self._axis_label(geo_data.get("approach_axis"))
            reach_raw = geo_data.get("finger_depth", 0.0) or 0.0
            span_raw = geo_data.get("real_gap", summary["max_gap_cm"] * ratio) or 0.0
            reach_cm = float(reach_raw) / ratio
            span_cm = float(span_raw) / ratio

            if geo_data.get("using_selected_gripping_surfaces"):
                shape_label = "Face clamp"
                grip_style = "Selected face-to-face contact"
                contact_mode = "Selected surfaces"
                strategy = "Keep the object centered between the selected gripping faces."
            elif finger_count >= 3:
                shape_label = f"Multi-finger ({finger_count})"
                grip_style = "Distributed pinch"
                contact_mode = "Finger cluster"
                strategy = "Use the span axis for thickness and keep the approach axis aligned."
            elif finger_count == 2:
                shape_label = "Parallel jaw"
                grip_style = "Two-finger pinch"
                contact_mode = "Two-jaw"
                strategy = "Align the narrow object dimension with the jaw span before closing."
            elif finger_count == 1:
                shape_label = "Single-contact / leaf"
                grip_style = "Single surface support"
                contact_mode = "Leaf / single point"
                strategy = "Approach gently and keep the object centered on the TCP."
            else:
                shape_label = "Fallback mesh gripper"
                grip_style = "Mesh-based estimate"
                contact_mode = "Mesh estimate"
                strategy = "Use the object's measured thickness to choose a safe open gap."

        return {
            "shape_label": shape_label,
            "grip_style": grip_style,
            "contact_mode": contact_mode,
            "strategy": strategy,
            "finger_count": finger_count,
            "span_axis": span_axis,
            "approach_axis": approach_axis,
            "reach_cm": reach_cm,
            "span_cm": span_cm,
            "anchor_link": anchor_link.name if anchor_link is not None else "-",
        }

    def _set_compute_rows(self, rows):
        self.compute_table.setRowCount(len(rows))
        for row, (metric, value) in enumerate(rows):
            metric_item = QtWidgets.QTableWidgetItem(metric)
            metric_item.setFlags(metric_item.flags() & ~QtCore.Qt.ItemIsEditable)
            value_item = QtWidgets.QTableWidgetItem(value)
            value_item.setFlags(value_item.flags() & ~QtCore.Qt.ItemIsEditable)
            self.compute_table.setItem(row, 0, metric_item)
            self.compute_table.setItem(row, 1, value_item)

        self.compute_table.resizeRowsToContents()

    def _compute_selected_pair_summary(self):
        members = self._selected_group_members()
        if len(members) < 2:
            return None

        selected_item = self.joints_list.currentItem()
        pair_label = selected_item.text() if selected_item else ", ".join(members)
        selected_joints = [self.mw.robot.joints[name] for name in members if name in self.mw.robot.joints]
        if len(selected_joints) < 2:
            return None

        saved_active_names = list(getattr(self.mw, "active_gripper_joint_names", []) or [])
        saved_active_name = getattr(self.mw, "active_gripper_joint_name", None)
        summary = None

        try:
            self.mw.active_gripper_joint_names = members
            self.mw.active_gripper_joint_name = members[0]

            if hasattr(self.mw, "_control_gripper_fingers"):
                self.mw._control_gripper_fingers(close=False, apply=False)

            gap_limits = getattr(self.mw, "_last_gripper_gap_limits", {}) or {}
            global_limits = gap_limits.get("_global")
            if global_limits is None:
                all_limits = [
                    value for value in gap_limits.values()
                    if isinstance(value, tuple) and len(value) == 2
                ]
                if all_limits:
                    global_limits = (
                        float(min(limit[0] for limit in all_limits)),
                        float(max(limit[1] for limit in all_limits)),
                    )

            ratio = getattr(self.mw.canvas, "grid_units_per_cm", 1.0) or 1.0

            current_gap_cm = None
            primary_joint = selected_joints[0]
            parent_link = getattr(primary_joint, "parent_link", None)
            if parent_link is not None and hasattr(self.mw, "get_link_tool_point"):
                try:
                    _, _, gap_value = self.mw.get_link_tool_point(parent_link, return_vec=True)
                    if isinstance(gap_value, dict):
                        raw_gap = gap_value.get("real_gap", gap_value.get("centers_span"))
                        if raw_gap is not None:
                            current_gap_cm = float(raw_gap) / ratio
                    elif gap_value is not None:
                        current_gap_cm = float(gap_value) / ratio
                except Exception:
                    current_gap_cm = None

            joint_travel = []
            for name in members:
                joint = self.mw.robot.joints.get(name)
                if joint is None:
                    continue
                joint_travel.append(
                    f"{name}: {self._format_deg(abs(joint.max_limit - joint.min_limit))}"
                )

            if global_limits is not None:
                min_gap_cm = float(global_limits[0]) / ratio
                max_gap_cm = float(global_limits[1]) / ratio
                hold_width_cm = max(0.0, max_gap_cm - 0.5)
            else:
                min_gap_cm = current_gap_cm if current_gap_cm is not None else 0.0
                max_gap_cm = current_gap_cm if current_gap_cm is not None else 0.0
                hold_width_cm = max(0.0, max_gap_cm - 0.5)

            travel_cm = max(0.0, max_gap_cm - min_gap_cm)
            status = "Ready" if max_gap_cm > 0 else "Unable to measure"

            summary = {
                "pair_label": pair_label,
                "members": members,
                "joint_travel": ", ".join(joint_travel) if joint_travel else "-",
                "min_gap_cm": min_gap_cm,
                "max_gap_cm": max_gap_cm,
                "travel_cm": travel_cm,
                "hold_width_cm": hold_width_cm,
                "current_gap_cm": current_gap_cm,
                "status": status,
            }
        finally:
            self.mw.active_gripper_joint_names = saved_active_names
            self.mw.active_gripper_joint_name = saved_active_name

        if summary is not None:
            anchor_link = None
            if selected_joints:
                for joint in selected_joints:
                    if joint.parent_link is not None:
                        anchor_link = joint.parent_link
                        break
                if anchor_link is None and selected_joints[0].child_link is not None:
                    anchor_link = selected_joints[0].child_link
            summary["shape"] = self._gripper_shape_profile(anchor_link, summary)

        return summary

    def on_compute_gripper_clicked(self):
        if not hasattr(self, "compute_table"):
            return

        summary = self._compute_selected_pair_summary()
        if summary is None:
            self.compute_note_label.setText(
                "Select a gripper pair with at least two related joints, then press Compute."
            )
            self.compute_table.setRowCount(0)
            self.mw.show_toast("Select a gripper pair first", "warning")
            return

        obj_summary = self._selected_sim_object_summary()
        rows = [
            ("Selected Pair", summary["pair_label"]),
            ("Gripper Shape", summary["shape"]["shape_label"]),
            ("Grip Style", summary["shape"]["grip_style"]),
            ("Contact Mode", summary["shape"]["contact_mode"]),
            ("Anchor Link", summary["shape"]["anchor_link"]),
            ("Finger Count", str(summary["shape"]["finger_count"])),
            ("Opening Axis", summary["shape"]["span_axis"]),
            ("Approach Axis", summary["shape"]["approach_axis"]),
            ("Grip Span", self._format_cm(summary["shape"]["span_cm"])),
            ("Finger Reach", self._format_cm(summary["shape"]["reach_cm"])),
            ("Maximum Opening", self._format_cm(summary["max_gap_cm"])),
            ("Estimated Hold Width", self._format_cm(summary["hold_width_cm"])),
            ("Current Gap", self._format_cm(summary["current_gap_cm"] or 0.0)),
            ("Joint Travel", summary["joint_travel"]),
            ("Status", summary["status"]),
        ]

        if obj_summary is not None:
            required_width_cm = obj_summary["grip_width_cm"]
            if required_width_cm is None:
                required_width_cm = max(obj_summary["dims"][0], obj_summary["dims"][1])

            object_height_cm = obj_summary["dims"][2]
            clearance_cm = max(0.5, min(2.0, max(0.5, object_height_cm * 0.12)))
            squeeze_cm = max(0.05, min(0.25, required_width_cm * 0.03))
            required_open_cm = required_width_cm + clearance_cm
            recommended_close_cm = max(0.0, required_width_cm - squeeze_cm)
            recommended_release_cm = summary["max_gap_cm"]
            fits = (
                summary["max_gap_cm"] >= required_open_cm
                and summary["min_gap_cm"] <= required_width_cm
            )
            open_margin_cm = summary["max_gap_cm"] - required_open_cm

            rows.extend([
                ("Selected Object", obj_summary["name"]),
                ("P1 (Bottom Face Center)", f"({obj_summary['pick'][0]:.2f}, {obj_summary['pick'][1]:.2f}, {obj_summary['pick'][2]:.2f}) cm"),
                ("P2 (Bottom Face Center)", f"({obj_summary['place'][0]:.2f}, {obj_summary['place'][1]:.2f}, {obj_summary['place'][2]:.2f}) cm"),
                ("LP (TCP)", f"({obj_summary['live'][0]:.2f}, {obj_summary['live'][1]:.2f}, {obj_summary['live'][2]:.2f}) cm"),
                ("DIM", f"{obj_summary['dims'][0]:.2f} x {obj_summary['dims'][1]:.2f} x {obj_summary['dims'][2]:.2f} cm"),
                ("Required Width", self._format_cm(required_width_cm)),
                ("Required Open", self._format_cm(required_open_cm)),
                ("Recommended Close", self._format_cm(recommended_close_cm)),
                ("Recommended Release", self._format_cm(recommended_release_cm)),
                ("Open Margin", self._format_cm(open_margin_cm)),
                ("Pick/Place Tip", summary["shape"]["strategy"]),
                ("Fit Check", "Fits" if fits else "Too wide"),
            ])

            if fits:
                self.compute_note_label.setText(
                    "Gripper shape and fit computed successfully. Use the opening axis and pick/place tip for a safe grasp."
                )
            else:
                self.compute_note_label.setText(
                    "Gripper shape computed, but the selected object needs more opening or a smaller grip width."
                )
        else:
            self.compute_note_label.setText(
                "Gripper shape computed successfully. Select an imported object if you also want fit checking and pick/place execution."
            )

        self._set_compute_rows(rows)
        if obj_summary is None:
            return

        required_width_cm = obj_summary["grip_width_cm"]
        if required_width_cm is None:
            required_width_cm = max(obj_summary["dims"][0], obj_summary["dims"][1])

        required_open_cm = required_width_cm + max(0.5, min(2.0, max(0.5, obj_summary["dims"][2] * 0.12)))
        if summary["max_gap_cm"] < required_open_cm:
            self.mw.show_toast("Object is too wide for this gripper pair", "warning")

    def _has_contact_surface_ui(self):
        return hasattr(self, "surface_target_label")

    def _selected_surface_candidate(self):
        if not hasattr(self, "surface_list"):
            return None
        item = self.surface_list.currentItem()
        if not item:
            return None
        candidate = item.data(QtCore.Qt.UserRole)
        return candidate if isinstance(candidate, dict) else None

    def _selected_second_surface_candidate(self):
        if not hasattr(self, "second_surface_list"):
            return None
        item = self.second_surface_list.currentItem()
        candidate = item.data(QtCore.Qt.UserRole) if item is not None else None
        if candidate is None:
            # Backward-safe fallback if an older UI state still stores candidate in combo.
            candidate = self.second_surface_combo.currentData(QtCore.Qt.UserRole)
        return candidate if isinstance(candidate, dict) else None

    def _selected_second_joint_name(self):
        if not hasattr(self, "second_link_combo"):
            return None
        joint_name = self.second_link_combo.currentData(QtCore.Qt.UserRole)
        return joint_name if isinstance(joint_name, str) else None

    def _candidate_from_paired_surface(self, joint_name):
        joint = self.mw.robot.joints.get(joint_name)
        if joint is None:
            return None

        link_name = getattr(joint, "paired_gripping_surface_link_name", None)
        center_local = getattr(joint, "paired_gripping_surface_center_local", None)
        normal_local = getattr(joint, "paired_gripping_surface_normal_local", None)
        surface_name = getattr(joint, "paired_gripping_surface_name", None)
        if not link_name or center_local is None or link_name not in self.mw.robot.links:
            return None

        link = self.mw.robot.links[link_name]
        local_center = np.array(center_local, dtype=float)
        local_normal = (
            np.array(normal_local, dtype=float)
            if normal_local is not None
            else np.zeros(3)
        )
        world_center = (link.t_world @ np.append(local_center, 1.0))[:3]
        world_normal = (
            link.t_world[:3, :3] @ local_normal
            if normal_local is not None
            else np.zeros(3)
        )
        world_normal_norm = np.linalg.norm(world_normal)
        if world_normal_norm > 1e-9:
            world_normal = world_normal / world_normal_norm

        return {
            "link_name": link_name,
            "surface_name": surface_name or "Surface",
            "display_name": f"{link_name} - {surface_name or 'Surface'}",
            "local_center": local_center,
            "local_normal": local_normal,
            "world_center": world_center,
            "world_normal": world_normal,
        }

    def _update_selected_faces_overlay(self, joint_name=None):
        if not self._has_contact_surface_ui() or not hasattr(self.mw, "canvas"):
            return

        if not self.show_selected_faces_check.isChecked():
            if hasattr(self.mw.canvas, "clear_selected_face_overlays"):
                self.mw.canvas.clear_selected_face_overlays()
            return

        if (
            not joint_name
            or joint_name not in self.mw.robot.joints
            or not hasattr(self.mw.canvas, "show_selected_gripping_faces")
        ):
            if hasattr(self.mw.canvas, "clear_selected_face_overlays"):
                self.mw.canvas.clear_selected_face_overlays()
            return

        primary_candidate = self._build_candidate_from_joint_surface(
            joint_name, "gripping"
        )
        if primary_candidate is None:
            primary_candidate = self._build_candidate_from_joint_surface(
                joint_name, "contact"
            )
        secondary_candidate = self._candidate_from_paired_surface(joint_name)
        self.mw.canvas.show_selected_gripping_faces(
            primary_candidate, secondary_candidate
        )

    def _build_candidate_from_joint_surface(self, joint_name, prefix):
        joint = self.mw.robot.joints.get(joint_name)
        if joint is None:
            return None

        link_name = getattr(joint, f"{prefix}_surface_link_name", None)
        center_local = getattr(joint, f"{prefix}_surface_center_local", None)
        normal_local = getattr(joint, f"{prefix}_surface_normal_local", None)
        surface_name = getattr(joint, f"{prefix}_surface_name", None)
        if not link_name or center_local is None or link_name not in self.mw.robot.links:
            return None

        link = self.mw.robot.links[link_name]
        local_center = np.array(center_local, dtype=float)
        local_normal = (
            np.array(normal_local, dtype=float)
            if normal_local is not None
            else np.zeros(3)
        )
        world_center = (link.t_world @ np.append(local_center, 1.0))[:3]
        world_normal = link.t_world[:3, :3] @ local_normal if normal_local is not None else np.zeros(3)
        world_normal_norm = np.linalg.norm(world_normal)
        if world_normal_norm > 1e-9:
            world_normal = world_normal / world_normal_norm

        return {
            "link_name": link_name,
            "surface_name": surface_name or "Surface",
            "display_name": f"{link_name} - {surface_name or 'Surface'}",
            "local_center": local_center,
            "local_normal": local_normal,
            "world_center": world_center,
            "world_normal": world_normal,
        }

    def _current_surface_candidate_for_action(self, joint_name=None):
        joint_name = joint_name or self._selected_joint_name()
        if not joint_name:
            return None

        candidate = self._selected_surface_candidate()
        if candidate is not None:
            return candidate

        return self._build_candidate_from_joint_surface(joint_name, "contact")

    def _get_joint_surface_links(self, joint):
        if not joint or not joint.child_link:
            return []

        allowed = []
        seen = set()
        stack = [joint.child_link]
        while stack:
            link = stack.pop()
            if link is None or link.name in seen:
                continue

            seen.add(link.name)
            allowed.append(link.name)
            for child_joint in link.child_joints:
                if child_joint.child_link is not None:
                    stack.append(child_joint.child_link)

        return sorted(allowed)

    def _get_related_joint_names(self, joint_name):
        robot = self.mw.robot
        related = {joint_name}
        changed = True

        while changed:
            changed = False
            for master_id, slaves in robot.joint_relations.items():
                chain = {master_id}
                chain.update(slave_id for slave_id, _ in slaves)
                if related.intersection(chain) and not chain.issubset(related):
                    related.update(chain)
                    changed = True

        return related

    def _get_pairable_gripper_joints(self, joint_name):
        robot = self.mw.robot
        joint = robot.joints.get(joint_name)
        if joint is None:
            return []

        siblings = []
        for other_name, other_joint in robot.joints.items():
            if other_name == joint_name:
                continue
            if not getattr(other_joint, 'is_gripper', False) or other_joint.child_link is None:
                continue
            if other_joint.parent_link is joint.parent_link:
                siblings.append(other_name)

        if siblings:
            return sorted(siblings)

        related = []
        for other_name in sorted(self._get_related_joint_names(joint_name)):
            if other_name == joint_name:
                continue
            other_joint = robot.joints.get(other_name)
            if other_joint is None or not getattr(other_joint, 'is_gripper', False):
                continue
            if other_joint.child_link is None or other_joint.child_link is joint.child_link:
                continue
            related.append(other_name)

        return related

    def _get_second_surface_candidates(self, joint_name):
        second_candidates = []
        for other_joint_name in self._get_pairable_gripper_joints(joint_name):
            for candidate in self._get_surface_candidates(other_joint_name):
                pair_candidate = dict(candidate)
                pair_candidate['source_joint_name'] = other_joint_name
                pair_candidate['pair_display_name'] = (
                    f"{other_joint_name} | {candidate['display_name']}"
                )
                second_candidates.append(pair_candidate)

        second_candidates.sort(
            key=lambda candidate: (
                candidate.get('source_joint_name', ''),
                int(candidate.get('table_group', 3)),
                int(candidate.get('table_index', 999)),
                candidate.get('surface_name', ''),
            )
        )
        return second_candidates

    def _candidate_priority(self, candidate):
        base_name = str(
            candidate.get('base_surface_name')
            or candidate.get('surface_name')
            or "Surface"
        )
        if "Inner Surface" in base_name:
            return 0
        if "Teethed Surface" in base_name:
            return 1
        if "Outer Surface" in base_name:
            return 2
        return self._surface_priority(base_name)

    def _choose_auto_primary_candidate(self, candidates):
        if not candidates:
            return None

        return min(
            candidates,
            key=lambda c: (
                self._candidate_priority(c),
                -float(c.get('area', 0.0)),
            ),
        )

    def _choose_auto_pair_candidate(self, primary_candidate, second_candidates):
        if primary_candidate is None or not second_candidates:
            return None

        primary_normal = np.array(
            primary_candidate.get('world_normal', np.zeros(3)),
            dtype=float
        )
        primary_normal_norm = np.linalg.norm(primary_normal)
        if primary_normal_norm > 1e-9:
            primary_normal = primary_normal / primary_normal_norm

        primary_center = np.array(
            primary_candidate.get('world_center', np.zeros(3)),
            dtype=float
        )

        def _pair_rank(candidate):
            cand_normal = np.array(candidate.get('world_normal', np.zeros(3)), dtype=float)
            cand_normal_norm = np.linalg.norm(cand_normal)
            if cand_normal_norm > 1e-9:
                cand_normal = cand_normal / cand_normal_norm
            normal_opposition = float(-np.dot(primary_normal, cand_normal))
            center_distance = float(
                np.linalg.norm(np.array(candidate.get('world_center', np.zeros(3)), dtype=float) - primary_center)
            )
            return (
                self._candidate_priority(candidate),
                -normal_opposition,
                -center_distance,
                -float(candidate.get('area', 0.0)),
            )

        return min(second_candidates, key=_pair_rank)

    def _set_active_gripper_context(self, joint_names):
        clean = []
        seen = set()
        for name in joint_names or []:
            if not isinstance(name, str) or name in seen:
                continue
            joint = self.mw.robot.joints.get(name)
            if joint is None or not getattr(joint, 'is_gripper', False):
                continue
            clean.append(name)
            seen.add(name)

        self.mw.active_gripper_joint_names = clean
        self.mw.active_gripper_joint_name = clean[0] if clean else None

    def _clear_joint_paired_gripping_surface(self, joint_name):
        joint = self.mw.robot.joints.get(joint_name)
        if joint is None:
            return

        joint.paired_gripping_enabled = False
        joint.paired_gripping_surface_joint_name = None
        joint.paired_gripping_surface_name = None
        joint.paired_gripping_surface_link_name = None
        joint.paired_gripping_surface_center_local = None
        joint.paired_gripping_surface_normal_local = None

        joint_cache = self.mw.joint_tab.joints.get(joint.child_link.name)
        if joint_cache is not None:
            joint_cache['paired_gripping_enabled'] = False
            joint_cache['paired_gripping_surface_joint_name'] = None
            joint_cache['paired_gripping_surface_name'] = None
            joint_cache['paired_gripping_surface_link'] = None
            joint_cache['paired_gripping_surface_center_local'] = None
            joint_cache['paired_gripping_surface_normal_local'] = None

    def _candidate_from_saved_surface(self, joint_name, prefix):
        saved = self._build_candidate_from_joint_surface(joint_name, prefix)
        if saved is None:
            return None

        candidates = self._get_surface_candidates(joint_name)
        if not candidates:
            return None

        for candidate in candidates:
            if (
                candidate.get('link_name') == saved.get('link_name')
                and candidate.get('surface_name') == saved.get('surface_name')
            ):
                return candidate
        return None

    def _pick_auto_joint_name(self, preferred_joint_name=None):
        robot = self.mw.robot
        if preferred_joint_name in robot.joints:
            preferred = robot.joints[preferred_joint_name]
            if getattr(preferred, 'is_gripper', False):
                return preferred_joint_name

        selected = self._selected_joint_name()
        if selected in robot.joints and getattr(robot.joints[selected], 'is_gripper', False):
            return selected

        pairable = []
        fallback = []
        for joint_name, joint in robot.joints.items():
            if not getattr(joint, 'is_gripper', False):
                continue
            if self._get_pairable_gripper_joints(joint_name):
                pairable.append(joint_name)
            else:
                fallback.append(joint_name)

        if pairable:
            return sorted(pairable)[0]
        if fallback:
            return sorted(fallback)[0]
        return None

    def ensure_auto_gripping_ready(self, preferred_joint_name=None, quiet=False, force=False):
        """
        Auto-select gripping surfaces so Pick-and-Place can run with minimal manual steps.
        Returns a dict with details about the selected gripper joint.
        """
        joint_name = self._pick_auto_joint_name(preferred_joint_name=preferred_joint_name)
        if not joint_name:
            return {"configured": False, "reason": "no_gripper_joint"}

        joint = self.mw.robot.joints.get(joint_name)
        if joint is None:
            return {"configured": False, "reason": "missing_joint"}

        candidates = self._get_surface_candidates(joint_name)
        if not candidates:
            return {"configured": False, "reason": "no_primary_surface_candidates", "joint_name": joint_name}

        primary_candidate = self._candidate_from_saved_surface(joint_name, "gripping")
        if primary_candidate is None:
            primary_candidate = self._candidate_from_saved_surface(joint_name, "contact")
        if primary_candidate is None or force:
            primary_candidate = self._choose_auto_primary_candidate(candidates)
            if primary_candidate is None:
                return {"configured": False, "reason": "unable_to_pick_primary", "joint_name": joint_name}

            self._apply_surface_candidate(joint_name, primary_candidate, log_selection=False)
            if not self._set_joint_gripping_surface(joint_name, primary_candidate):
                return {"configured": False, "reason": "unable_to_save_primary", "joint_name": joint_name}

        second_candidates = self._get_second_surface_candidates(joint_name)
        pair_candidate = None
        if second_candidates:
            pair_candidate = self._selected_second_surface_candidate()
            if force or not isinstance(pair_candidate, dict):
                pair_candidate = self._choose_auto_pair_candidate(primary_candidate, second_candidates)
            if isinstance(pair_candidate, dict):
                if self._set_joint_paired_gripping_surface(joint_name, pair_candidate):
                    self._set_paired_gripping_enabled(joint_name, True)
                else:
                    pair_candidate = None

        if pair_candidate is None:
            self._clear_joint_paired_gripping_surface(joint_name)

        active_joints = [joint_name]
        if isinstance(pair_candidate, dict):
            pair_joint_name = pair_candidate.get('source_joint_name')
            if isinstance(pair_joint_name, str):
                active_joints.append(pair_joint_name)
        self._set_active_gripper_context(active_joints)

        self.refresh_contact_surface_ui(joint_name)
        if not quiet:
            if pair_candidate is not None:
                self.mw.log(
                    "Auto Gripper Ready: "
                    f"{joint_name} paired with {pair_candidate.get('source_joint_name', '-')}. "
                    f"Using '{primary_candidate.get('surface_name', 'Surface')}' and "
                    f"'{pair_candidate.get('surface_name', 'Surface')}'."
                )
                self.mw.show_toast("Auto gripper pair configured", "success")
            else:
                self.mw.log(
                    "Auto Gripper Ready: "
                    f"{joint_name} configured with '{primary_candidate.get('surface_name', 'Surface')}'."
                )
                self.mw.show_toast("Auto gripper surface configured", "success")

        return {
            "configured": True,
            "joint_name": joint_name,
            "paired": pair_candidate is not None,
            "pair_joint_name": pair_candidate.get('source_joint_name') if isinstance(pair_candidate, dict) else None,
            "primary_surface": primary_candidate.get('surface_name') if isinstance(primary_candidate, dict) else None,
            "pair_surface": pair_candidate.get('surface_name') if isinstance(pair_candidate, dict) else None,
        }

    def _second_joint_display_name(self, joint_name):
        joint = self.mw.robot.joints.get(joint_name)
        if joint is None or joint.child_link is None:
            return joint_name
        return f"{joint_name} ({joint.child_link.name})"

    def _joint_name_sort_key(self, joint_name):
        prefix = "".join(ch for ch in joint_name if not ch.isdigit())
        digits = "".join(ch for ch in joint_name if ch.isdigit())
        return (prefix.lower(), int(digits) if digits else -1, joint_name.lower())

    def _joint_selection_entries(self):
        robot = self.mw.robot
        entries = []
        pair_index = 1

        for master_name in sorted(robot.joint_relations.keys(), key=self._joint_name_sort_key):
            if master_name not in robot.joints:
                continue

            master_joint = robot.joints[master_name]
            for slave_name, ratio in robot.joint_relations.get(master_name, []):
                slave_joint = robot.joints.get(slave_name)
                if slave_joint is None:
                    continue

                members = [master_name, slave_name]
                display_name = f"Pair {pair_index}: {master_name}, {slave_name}"
                tooltip = (
                    f"{master_name}: {master_joint.parent_link.name} -> {master_joint.child_link.name} | "
                    f"{slave_name}: {slave_joint.parent_link.name} -> {slave_joint.child_link.name} | "
                    f"ratio: {ratio}"
                )
                entries.append(
                    {
                        "primary_name": master_name,
                        "members": members,
                        "display_name": display_name,
                        "tooltip": tooltip,
                    }
                )
                pair_index += 1

        single_index = 1
        for joint_name in sorted(robot.joints.keys(), key=self._joint_name_sort_key):
            if any(joint_name in entry["members"] for entry in entries):
                continue

            joint = robot.joints[joint_name]
            if not joint.is_gripper:
                continue

            entries.append(
                {
                    "primary_name": joint_name,
                    "members": [joint_name],
                    "display_name": f"Single {single_index}: {joint_name}",
                    "tooltip": (
                        f"{joint_name}: {joint.parent_link.name} -> {joint.child_link.name}"
                    ),
                }
            )
            single_index += 1

        return entries

    def on_make_robo(self):
        self.mw.log("🚀 FINALIZING ASSEMBLY: Building Robot Kinematic Tree...")
        if self.mw.make_robot():
            self.refresh_joints()
            self.mw.show_toast("Assembly Finalized", "success")

    def refresh_sliders(self):
        self.refresh_joints()

    def refresh_joints(self):
        """Update the list of available gripper-capable joints."""
        selected_joint_name = self._selected_joint_name()
        self.joints_list.clear()
        self.mark_gripper_check.setText("Mark as Gripper")

        selected_item = None
        for entry in self._joint_selection_entries():
            item = QtWidgets.QListWidgetItem(entry["display_name"])
            item.setData(QtCore.Qt.UserRole, entry["primary_name"])
            item.setData(QtCore.Qt.UserRole + 1, entry["members"])
            item.setToolTip(entry["tooltip"])
            self.joints_list.addItem(item)

            if selected_joint_name in entry["members"]:
                selected_item = item

        if selected_item is not None:
            self.joints_list.setCurrentItem(selected_item)
            self.on_joint_selected(selected_item)
        else:
            self.refresh_contact_surface_ui()
            self._refresh_compute_ui()

    def on_joint_selected(self, item):
        name = item.data(QtCore.Qt.UserRole)
        if not name:
            return

        joint = self.mw.robot.joints[name]
        group_members = item.data(QtCore.Qt.UserRole + 1)
        if not isinstance(group_members, list) or not group_members:
            group_members = [name]

        if len(group_members) == 2:
            self.mark_gripper_check.setText("Mark selected Pair as Gripper")
        elif len(group_members) > 2:
            self.mark_gripper_check.setText("Mark selected Group as Gripper")
        else:
            self.mark_gripper_check.setText("Mark as Gripper")

        is_group_gripper = any(
            getattr(self.mw.robot.joints.get(joint_name), 'is_gripper', False)
            for joint_name in group_members
        )
        self.mark_gripper_check.blockSignals(True)
        self.mark_gripper_check.setChecked(is_group_gripper)
        self.mark_gripper_check.blockSignals(False)

        active_names = [
            joint_name
            for joint_name in group_members
            if getattr(self.mw.robot.joints.get(joint_name), 'is_gripper', False)
        ]
        if getattr(joint, 'paired_gripping_enabled', False):
            pair_joint = getattr(joint, 'paired_gripping_surface_joint_name', None)
            if isinstance(pair_joint, str):
                active_names.append(pair_joint)
        if not active_names:
            active_names = [name]
        self._set_active_gripper_context(active_names)

        joint_span = joint.max_limit - joint.min_limit
        val_pct = 0 if abs(joint_span) < 1e-9 else int(
            (joint.current_value - joint.min_limit) / joint_span * 100
        )
        self.stroke_slider.blockSignals(True)
        self.stroke_slider.setValue(val_pct)
        self.stroke_slider.blockSignals(False)

        if joint.is_gripper:
            self.ensure_auto_gripping_ready(preferred_joint_name=name, quiet=True, force=False)
        else:
            self.refresh_contact_surface_ui(name)
        self._refresh_compute_ui()

    def on_mark_toggled(self, checked):
        item = self.joints_list.currentItem()
        if not item:
            return

        name = item.data(QtCore.Qt.UserRole)
        robot = self.mw.robot
        robot.joints[name].is_gripper = checked

        rel_chain = item.data(QtCore.Qt.UserRole + 1)
        if not isinstance(rel_chain, list) or not rel_chain:
            rel_chain = sorted(
                self._get_related_joint_names(name),
                key=self._joint_name_sort_key,
            )
        rel_chain = [joint_name for joint_name in rel_chain if joint_name in robot.joints]
        if not rel_chain:
            rel_chain = [name]

        for joint_id in rel_chain:
            if joint_id in robot.joints:
                robot.joints[joint_id].is_gripper = checked

        self.mw.log(
            f"Gripper Linkage: {', '.join(rel_chain)} marked as "
            f"{'Gripper' if checked else 'Standard'}"
        )

        if hasattr(self.mw, 'joint_tab'):
            active_child = getattr(self.mw.joint_tab, 'active_joint_control', None)
            if active_child and active_child in self.mw.joint_tab.joints:
                active_joint_id = self.mw.joint_tab.joints[active_child].get('joint_id')
                self.mw.joint_tab.set_lp_btn.setVisible(bool(checked and active_joint_id in rel_chain))
            self.mw.joint_tab.refresh_links()

        self.refresh_joints()
        if checked:
            self.ensure_auto_gripping_ready(preferred_joint_name=name, quiet=False, force=False)
        else:
            if getattr(self.mw, 'active_gripper_joint_name', None) in rel_chain:
                self._set_active_gripper_context([])
        self._refresh_compute_ui()

    def _surface_priority(self, base_name):
        priority = {
            "Inner Surface": 0,
            "Teethed Surface": 1,
            "Outer Surface": 2,
            "Top Surface": 3,
            "Bottom Surface": 4,
            "Front Surface": 5,
            "Back Surface": 6,
            "Right Surface": 7,
            "Left Surface": 8,
            "Surface": 9,
        }
        return priority.get(base_name, 99)

    def _surface_base_name(
        self, axis_index, axis_sign, inner_axis_index, inner_axis_sign, normal_alignment=None
    ):
        if normal_alignment is not None:
            if normal_alignment >= 0.35:
                return "Inner Surface"
            if normal_alignment <= -0.35:
                return "Outer Surface"

        if inner_axis_index is not None and axis_index == inner_axis_index:
            return "Inner Surface" if axis_sign == inner_axis_sign else "Outer Surface"

        axis_names = {
            0: ("Right Surface", "Left Surface"),
            1: ("Front Surface", "Back Surface"),
            2: ("Top Surface", "Bottom Surface"),
        }
        positive_name, negative_name = axis_names.get(axis_index, ("Surface", "Surface"))
        return positive_name if axis_sign > 0 else negative_name

    def _is_teethed_group(self, base_name, group, max_area):
        """Best-effort detection for serrated/toothed gripping surfaces."""
        if len(group) < 4 or max_area <= 1e-9:
            return False

        areas = [
            float(candidate.get('area', 0.0))
            for candidate in group
            if float(candidate.get('area', 0.0)) > 0.0
        ]
        if not areas:
            return False

        median_area = float(np.median(areas))
        mean_area = float(np.mean(areas))
        small_relative = median_area <= max_area * 0.45 and mean_area <= max_area * 0.55

        centers = np.array(
            [np.array(candidate['local_center'], dtype=float) for candidate in group],
            dtype=float
        )
        spreads = np.ptp(centers, axis=0) if len(centers) > 1 else np.zeros(3)

        normal_axis = int(np.argmax(np.abs(np.array(group[0]['local_normal'], dtype=float))))
        tangent_spreads = [spreads[idx] for idx in range(3) if idx != normal_axis]
        longest_tangent = max(tangent_spreads) if tangent_spreads else 0.0
        shortest_tangent = min(tangent_spreads) if tangent_spreads else 0.0

        repeated_strip = longest_tangent > 0.0 and (
            shortest_tangent <= longest_tangent * 0.45 or len(group) >= 6
        )

        preferred_base = base_name in {"Inner Surface", "Top Surface", "Bottom Surface"}
        return small_relative and repeated_strip and preferred_base

    def _build_composite_surface_candidate(self, link_name, link, group, surface_name):
        """Create a synthetic candidate that covers a whole grouped surface."""
        if not group:
            return None

        areas = np.array(
            [max(float(candidate.get('area', 0.0)), 1e-6) for candidate in group],
            dtype=float
        )
        weights = areas / max(float(np.sum(areas)), 1e-6)

        local_centers = np.array(
            [np.array(candidate['local_center'], dtype=float) for candidate in group],
            dtype=float
        )
        local_normals = np.array(
            [np.array(candidate['local_normal'], dtype=float) for candidate in group],
            dtype=float
        )

        local_center = np.average(local_centers, axis=0, weights=weights)
        local_normal = np.average(local_normals, axis=0, weights=weights)
        local_normal_norm = np.linalg.norm(local_normal)
        if local_normal_norm <= 1e-9:
            local_normal = np.array(group[0]['local_normal'], dtype=float)
        else:
            local_normal = local_normal / local_normal_norm

        world_center = (link.t_world @ np.append(local_center, 1.0))[:3]
        world_normal = link.t_world[:3, :3] @ local_normal
        world_normal_norm = np.linalg.norm(world_normal)
        if world_normal_norm > 1e-9:
            world_normal = world_normal / world_normal_norm

        combined_edges = []
        for candidate in group:
            combined_edges.extend(candidate.get('mesh_boundary_edges') or [])

        deduped_edges = None
        if combined_edges:
            edge_set = {
                tuple(sorted((int(edge[0]), int(edge[1]))))
                for edge in combined_edges
                if len(edge) == 2
            }
            deduped_edges = [list(edge) for edge in sorted(edge_set)]

        combined_faces = []
        for candidate in group:
            combined_faces.extend(candidate.get('mesh_face_indices') or [])

        deduped_faces = sorted({int(face_id) for face_id in combined_faces})

        return {
            'link_name': link_name,
            'local_center': local_center,
            'local_normal': local_normal,
            'world_center': world_center,
            'world_normal': world_normal,
            'area': float(np.sum(areas)),
            'mesh_boundary_edges': deduped_edges,
            'mesh_face_indices': deduped_faces or None,
            'base_surface_name': surface_name,
            'surface_name': surface_name,
            'display_name': f"{link_name} - {surface_name}",
            'composite_surface': True,
        }

    def _build_bbox_surface_candidates(self, link_name, link, link_center_world, assembly_center_world):
        mesh = link.mesh
        bounds = np.array(mesh.bounds, dtype=float)
        local_center = (bounds[0] + bounds[1]) / 2.0
        extents = bounds[1] - bounds[0]
        rot = link.t_world[:3, :3]

        candidates = []
        for axis_index in range(3):
            for axis_sign in (-1, 1):
                local_normal = np.zeros(3)
                local_normal[axis_index] = float(axis_sign)

                local_point = local_center.copy()
                local_point[axis_index] = bounds[1][axis_index] if axis_sign > 0 else bounds[0][axis_index]

                free_axes = [axis for axis in range(3) if axis != axis_index]
                outline_points = []
                corner_pairs = [(0, 0), (1, 0), (1, 1), (0, 1)]
                for first_idx, second_idx in corner_pairs:
                    corner = local_center.copy()
                    corner[axis_index] = local_point[axis_index]
                    corner[free_axes[0]] = bounds[first_idx][free_axes[0]]
                    corner[free_axes[1]] = bounds[second_idx][free_axes[1]]
                    outline_points.append(corner.tolist())

                world_center = (link.t_world @ np.append(local_point, 1.0))[:3]
                world_normal = rot @ local_normal
                norm = np.linalg.norm(world_normal)
                if norm > 1e-9:
                    world_normal = world_normal / norm

                candidates.append({
                    'link_name': link_name,
                    'local_center': local_point,
                    'local_normal': local_normal,
                    'world_center': world_center,
                    'world_normal': world_normal,
                    'area': float(np.prod(np.delete(extents, axis_index))),
                    'outline_points': outline_points,
                    'outline_edges': [(0, 1), (1, 2), (2, 3), (3, 0)],
                })

        return self._label_link_surface_candidates(
            link_name, link, link_center_world, assembly_center_world, candidates
        )

    def _label_link_surface_candidates(
        self, link_name, link, link_center_world, assembly_center_world, candidates
    ):
        if not candidates:
            return []

        to_assembly_world = np.array(assembly_center_world, dtype=float) - np.array(link_center_world, dtype=float)
        to_assembly_local = link.t_world[:3, :3].T @ to_assembly_world

        inner_axis_index = None
        inner_axis_sign = None
        if np.linalg.norm(to_assembly_local) > 1e-9:
            inner_axis_index = int(np.argmax(np.abs(to_assembly_local)))
            inner_axis_sign = 1 if to_assembly_local[inner_axis_index] >= 0 else -1

        grouped = {}
        for candidate in candidates:
            local_normal = np.array(candidate['local_normal'], dtype=float)
            axis_index = int(np.argmax(np.abs(local_normal)))
            axis_sign = 1 if local_normal[axis_index] >= 0 else -1
            to_assembly_local = link.t_world[:3, :3].T @ (
                np.array(assembly_center_world, dtype=float) - np.array(candidate['world_center'], dtype=float)
            )
            to_assembly_norm = np.linalg.norm(to_assembly_local)
            normal_alignment = None
            if to_assembly_norm > 1e-9:
                normal_alignment = float(
                    np.dot(local_normal, to_assembly_local / to_assembly_norm)
                )
            base_name = self._surface_base_name(
                axis_index, axis_sign, inner_axis_index, inner_axis_sign, normal_alignment
            )

            candidate['original_base_surface_name'] = base_name
            candidate['base_surface_name'] = base_name
            candidate['surface_name'] = base_name
            candidate['display_name'] = f"{link_name} - {base_name}"
            grouped.setdefault(base_name, []).append(candidate)

        max_area = max(float(candidate.get('area', 0.0)) for candidate in candidates) if candidates else 0.0
        for base_name, group in grouped.items():
            if self._is_teethed_group(base_name, group, max_area):
                for candidate in group:
                    candidate['source_base_surface_name'] = base_name
                    candidate['base_surface_name'] = "Teethed Surface"
                    candidate['surface_name'] = "Teethed Surface"
                    candidate['display_name'] = f"{link_name} - Teethed Surface"

        extra_candidates = []
        inner_group = [
            candidate for candidate in candidates
            if candidate.get('original_base_surface_name') == "Inner Surface"
        ]
        if len(inner_group) > 1:
            composite_inner = self._build_composite_surface_candidate(
                link_name, link, inner_group, "Inner Surface"
            )
            if composite_inner is not None:
                extra_candidates.append(composite_inner)

        for base_name, group in grouped.items():
            if len(group) <= 1:
                continue

            effective_base_name = group[0].get('base_surface_name', base_name)

            group.sort(
                key=lambda candidate: (
                    -float(candidate.get('area', 0.0)),
                    round(float(candidate['local_center'][2]), 6),
                    round(float(candidate['local_center'][1]), 6),
                    round(float(candidate['local_center'][0]), 6),
                )
            )
            for index, candidate in enumerate(group, start=1):
                candidate['surface_name'] = f"{effective_base_name} {index}"
                candidate['display_name'] = f"{link_name} - {candidate['surface_name']}"

        candidates.sort(
            key=lambda candidate: (
                candidate['link_name'],
                self._surface_priority(candidate.get('base_surface_name', 'Surface')),
                -float(candidate.get('area', 0.0)),
            )
        )
        candidates = extra_candidates + candidates

        outer_index = 0
        inner_index = 0
        teethed_index = 0
        for candidate in candidates:
            detailed_name = candidate['surface_name']
            candidate['detailed_surface_name'] = detailed_name

            if candidate.get('composite_surface') and candidate.get('base_surface_name') == "Inner Surface":
                candidate['surface_name'] = "Inner Surface"
                candidate['table_group'] = 0
                candidate['table_index'] = 0
            elif candidate.get('base_surface_name') == "Teethed Surface":
                teethed_index += 1
                candidate['surface_name'] = f"Teethed Surface {teethed_index}"
                candidate['table_group'] = 1
                candidate['table_index'] = teethed_index
            elif candidate.get('base_surface_name') == "Inner Surface":
                inner_index += 1
                candidate['table_group'] = 2
                candidate['table_index'] = inner_index
            else:
                outer_index += 1
                outer_name = f"Outer Surface {outer_index}"
                candidate['outer_surface_name'] = outer_name
                candidate['table_group'] = 3
                candidate['table_index'] = outer_index

                if detailed_name.startswith("Outer Surface"):
                    candidate['surface_name'] = outer_name
                else:
                    candidate['surface_name'] = f"{outer_name} ({detailed_name})"

            candidate['display_name'] = f"{link_name} - {candidate['surface_name']}"

        candidates.sort(
            key=lambda candidate: (
                candidate['link_name'],
                int(candidate.get('table_group', 3)),
                int(candidate.get('table_index', 999)),
                self._surface_priority(candidate.get('base_surface_name', 'Surface')),
                -float(candidate.get('area', 0.0)),
            )
        )
        return candidates

    def _build_link_surface_candidates(self, link_name, link, link_center_world, assembly_center_world):
        mesh = getattr(link, 'mesh', None)
        if mesh is None:
            return []

        facets = list(getattr(mesh, 'facets', []) or [])
        facet_boundaries = list(getattr(mesh, 'facets_boundary', []) or [])
        facet_centers = np.asarray(getattr(mesh, 'facets_origin', []), dtype=float)
        facet_normals = np.asarray(getattr(mesh, 'facets_normal', []), dtype=float)
        facet_areas = np.asarray(getattr(mesh, 'facets_area', []), dtype=float)

        facet_count = min(len(facets), len(facet_centers), len(facet_normals), len(facet_areas))
        if facet_count <= 0:
            return self._build_bbox_surface_candidates(
                link_name, link, link_center_world, assembly_center_world
            )

        max_area = float(np.max(facet_areas[:facet_count])) if facet_count > 0 else 0.0
        min_area = max_area * 0.08 if facet_count > 12 and max_area > 0 else 0.0

        rot = link.t_world[:3, :3]
        candidates = []
        for index in range(facet_count):
            area = float(facet_areas[index])
            if not np.isfinite(area) or area < min_area:
                continue

            local_center = np.array(facet_centers[index], dtype=float)
            local_normal = np.array(facet_normals[index], dtype=float)
            local_norm = np.linalg.norm(local_normal)
            if local_norm <= 1e-9:
                continue
            local_normal = local_normal / local_norm

            world_center = (link.t_world @ np.append(local_center, 1.0))[:3]
            world_normal = rot @ local_normal
            world_norm = np.linalg.norm(world_normal)
            if world_norm > 1e-9:
                world_normal = world_normal / world_norm

            candidates.append({
                'link_name': link_name,
                'local_center': local_center,
                'local_normal': local_normal,
                'world_center': world_center,
                'world_normal': world_normal,
                'area': area,
                'mesh_face_indices': np.asarray(facets[index], dtype=int).tolist(),
                'mesh_boundary_edges': (
                    np.asarray(facet_boundaries[index], dtype=int).tolist()
                    if index < len(facet_boundaries)
                    else None
                ),
            })

        if not candidates:
            return self._build_bbox_surface_candidates(
                link_name, link, link_center_world, assembly_center_world
            )

        return self._label_link_surface_candidates(
            link_name, link, link_center_world, assembly_center_world, candidates
        )

    def _get_surface_candidates(self, joint_name):
        joint = self.mw.robot.joints.get(joint_name)
        if joint is None:
            return []

        link_payloads = []
        for link_name in self._get_joint_surface_links(joint):
            link = self.mw.robot.links.get(link_name)
            if link is None or getattr(link, 'mesh', None) is None:
                continue

            bounds = np.array(link.mesh.bounds, dtype=float)
            local_center = (bounds[0] + bounds[1]) / 2.0
            world_center = (link.t_world @ np.append(local_center, 1.0))[:3]
            link_payloads.append((link_name, link, world_center))

        if not link_payloads:
            return []

        assembly_center_world = np.mean(
            [payload[2] for payload in link_payloads], axis=0
        )

        candidates = []
        for link_name, link, link_center_world in link_payloads:
            candidates.extend(
                self._build_link_surface_candidates(
                    link_name, link, link_center_world, assembly_center_world
                )
            )
        return candidates

    def _set_joint_surface_name(self, joint_name, surface_name):
        joint = self.mw.robot.joints.get(joint_name)
        if joint is None:
            return

        joint.contact_surface_name = surface_name
        joint_cache = self.mw.joint_tab.joints.get(joint.child_link.name)
        if joint_cache is not None:
            joint_cache['contact_surface_name'] = surface_name

    def _set_joint_gripping_surface(self, joint_name, candidate):
        joint = self.mw.robot.joints.get(joint_name)
        if joint is None or not isinstance(candidate, dict):
            return False

        joint.gripping_surface_name = candidate.get('surface_name')
        joint.gripping_surface_link_name = candidate.get('link_name')
        joint.gripping_surface_center_local = np.array(candidate.get('local_center'), dtype=float)

        local_normal = candidate.get('local_normal')
        joint.gripping_surface_normal_local = (
            np.array(local_normal, dtype=float)
            if local_normal is not None
            else None
        )

        joint_cache = self.mw.joint_tab.joints.get(joint.child_link.name)
        if joint_cache is not None:
            joint_cache['gripping_surface_name'] = joint.gripping_surface_name
            joint_cache['gripping_surface_link'] = joint.gripping_surface_link_name
            joint_cache['gripping_surface_center_local'] = joint.gripping_surface_center_local.tolist()
            joint_cache['gripping_surface_normal_local'] = (
                joint.gripping_surface_normal_local.tolist()
                if joint.gripping_surface_normal_local is not None
                else None
            )
        return True

    def _set_paired_gripping_enabled(self, joint_name, enabled):
        joint = self.mw.robot.joints.get(joint_name)
        if joint is None:
            return False

        joint.paired_gripping_enabled = bool(enabled)
        joint_cache = self.mw.joint_tab.joints.get(joint.child_link.name)
        if joint_cache is not None:
            joint_cache['paired_gripping_enabled'] = joint.paired_gripping_enabled
        return True

    def _set_joint_paired_gripping_surface(self, joint_name, candidate):
        joint = self.mw.robot.joints.get(joint_name)
        if joint is None or not isinstance(candidate, dict):
            return False

        joint.paired_gripping_surface_joint_name = candidate.get('source_joint_name')
        joint.paired_gripping_surface_name = candidate.get('surface_name')
        joint.paired_gripping_surface_link_name = candidate.get('link_name')
        joint.paired_gripping_surface_center_local = np.array(
            candidate.get('local_center'),
            dtype=float
        )

        local_normal = candidate.get('local_normal')
        joint.paired_gripping_surface_normal_local = (
            np.array(local_normal, dtype=float)
            if local_normal is not None
            else None
        )
        joint.paired_gripping_enabled = True

        joint_cache = self.mw.joint_tab.joints.get(joint.child_link.name)
        if joint_cache is not None:
            joint_cache['paired_gripping_enabled'] = True
            joint_cache['paired_gripping_surface_joint_name'] = (
                joint.paired_gripping_surface_joint_name
            )
            joint_cache['paired_gripping_surface_name'] = (
                joint.paired_gripping_surface_name
            )
            joint_cache['paired_gripping_surface_link'] = (
                joint.paired_gripping_surface_link_name
            )
            joint_cache['paired_gripping_surface_center_local'] = (
                joint.paired_gripping_surface_center_local.tolist()
            )
            joint_cache['paired_gripping_surface_normal_local'] = (
                joint.paired_gripping_surface_normal_local.tolist()
                if joint.paired_gripping_surface_normal_local is not None
                else None
            )
        return True

    def _gripping_surface_summary(self, joint_name):
        joint = self.mw.robot.joints.get(joint_name)
        if joint is None:
            return None

        link_name = getattr(joint, 'gripping_surface_link_name', None)
        center_local = getattr(joint, 'gripping_surface_center_local', None)
        if not link_name or center_local is None or link_name not in self.mw.robot.links:
            return None

        link = self.mw.robot.links[link_name]
        world_center = (link.t_world @ np.append(np.array(center_local, dtype=float), 1.0))[:3]
        ratio = getattr(self.mw.canvas, 'grid_units_per_cm', 1.0) or 1.0
        center_cm = world_center / ratio
        center_str = ", ".join(f"{coord:.2f}" for coord in center_cm)
        surface_name = getattr(joint, 'gripping_surface_name', None) or "Surface"
        return surface_name, link_name, center_str

    def _paired_gripping_surface_summary(self, joint_name):
        joint = self.mw.robot.joints.get(joint_name)
        if joint is None:
            return None

        link_name = getattr(joint, 'paired_gripping_surface_link_name', None)
        center_local = getattr(joint, 'paired_gripping_surface_center_local', None)
        if not link_name or center_local is None or link_name not in self.mw.robot.links:
            return None

        link = self.mw.robot.links[link_name]
        world_center = (link.t_world @ np.append(np.array(center_local, dtype=float), 1.0))[:3]
        ratio = getattr(self.mw.canvas, 'grid_units_per_cm', 1.0) or 1.0
        center_cm = world_center / ratio
        center_str = ", ".join(f"{coord:.2f}" for coord in center_cm)
        surface_name = getattr(joint, 'paired_gripping_surface_name', None) or "Surface"
        pair_joint_name = getattr(joint, 'paired_gripping_surface_joint_name', None) or "Pair"
        return surface_name, link_name, pair_joint_name, center_str

    def _apply_surface_candidate(self, joint_name, candidate, log_selection=True):
        joint = self.mw.robot.joints.get(joint_name)
        if joint is None:
            return

        joint.contact_surface_link_name = candidate['link_name']
        joint.contact_surface_center_local = np.array(candidate['local_center'], dtype=float)
        joint.contact_surface_normal_local = np.array(candidate['local_normal'], dtype=float)
        self._set_joint_surface_name(joint_name, candidate['surface_name'])

        joint_cache = self.mw.joint_tab.joints.get(joint.child_link.name)
        if joint_cache is not None:
            joint_cache['contact_surface_link'] = candidate['link_name']
            joint_cache['contact_surface_center_local'] = joint.contact_surface_center_local.tolist()
            joint_cache['contact_surface_normal_local'] = joint.contact_surface_normal_local.tolist()

        if log_selection:
            self.mw.log(
                f"Named contact surface selected: '{candidate['surface_name']}' on '{candidate['link_name']}'."
            )
            self.mw.show_toast(f"Using {candidate['surface_name']}", "success")

        self.refresh_contact_surface_ui(joint_name)

    def _find_matching_surface_candidate(self, joint_name, link_name, world_center, world_normal):
        candidates = self._get_surface_candidates(joint_name)
        if not candidates:
            return None

        world_center = np.array(world_center, dtype=float)
        world_normal = np.array(world_normal, dtype=float)
        normal_norm = np.linalg.norm(world_normal)
        if normal_norm > 1e-9:
            world_normal = world_normal / normal_norm

        best_candidate = None
        best_score = float('inf')

        link = self.mw.robot.links.get(link_name)
        link_scale = 1.0
        if link is not None and getattr(link, 'mesh', None) is not None:
            extents = np.array(link.mesh.bounds[1] - link.mesh.bounds[0], dtype=float)
            link_scale = max(float(np.linalg.norm(extents)), 1.0)

        for candidate in candidates:
            if candidate['link_name'] != link_name:
                continue

            center_delta = np.linalg.norm(candidate['world_center'] - world_center) / link_scale
            normal_delta = 1.0 - abs(float(np.dot(candidate['world_normal'], world_normal)))
            score = center_delta + 0.35 * normal_delta

            if score < best_score:
                best_score = score
                best_candidate = candidate

        return best_candidate

    def sync_surface_from_pick(self, selection):
        if not isinstance(selection, dict):
            return

        joint_name = selection.get('joint_id')
        link_name = selection.get('link_name')
        world_center = selection.get('world_center')
        world_normal = selection.get('world_normal')

        if not joint_name or link_name is None or world_center is None or world_normal is None:
            return

        candidate = self._find_matching_surface_candidate(
            joint_name, link_name, world_center, world_normal
        )

        surface_name = candidate['surface_name'] if candidate is not None else "Custom Surface"
        self._set_joint_surface_name(joint_name, surface_name)
        selection['surface_name'] = surface_name
        self.refresh_contact_surface_ui(joint_name)

    def _highlight_surface_candidate(self, candidate):
        if not hasattr(self.mw, 'canvas'):
            return

        if not isinstance(candidate, dict):
            self.mw.canvas.clear_highlights()
            return

        link_name = candidate.get('link_name')
        if not link_name:
            self.mw.canvas.clear_highlights()
            return

        if not self.mw.canvas.highlight_surface_candidate(link_name, candidate):
            self.mw.canvas.clear_highlights()

    def _populate_second_surface_combo(self, joint_name, candidates):
        if not self._has_contact_surface_ui():
            return None
        joint = self.mw.robot.joints.get(joint_name)
        saved_joint_name = getattr(joint, 'paired_gripping_surface_joint_name', None) if joint else None
        joint_names = sorted(
            {
                candidate.get('source_joint_name')
                for candidate in candidates
                if isinstance(candidate.get('source_joint_name'), str)
            }
        )

        self.second_link_combo.blockSignals(True)
        self.second_link_combo.clear()
        self.second_surface_combo.blockSignals(True)
        self.second_surface_combo.clear()
        self.second_surface_combo.blockSignals(False)

        if not joint_names:
            self.second_link_combo.addItem("No opposite gripper links found")
            self.second_link_combo.setItemData(0, None, QtCore.Qt.UserRole)
            self.second_link_combo.blockSignals(False)
            self._populate_second_surface_list(joint_name, candidates, None)
            return None

        selected_joint_name = (
            saved_joint_name if saved_joint_name in joint_names else joint_names[0]
        )
        selected_index = 0
        for index, other_joint_name in enumerate(joint_names):
            self.second_link_combo.addItem(
                self._second_joint_display_name(other_joint_name)
            )
            self.second_link_combo.setItemData(index, other_joint_name, QtCore.Qt.UserRole)
            if other_joint_name == selected_joint_name:
                selected_index = index

        self.second_link_combo.setCurrentIndex(selected_index)
        self.second_link_combo.blockSignals(False)
        return self._populate_second_surface_list(
            joint_name, candidates, selected_joint_name
        )

    def _populate_second_surface_list(self, joint_name, candidates, selected_joint_name):
        if not self._has_contact_surface_ui():
            return None
        joint = self.mw.robot.joints.get(joint_name)
        saved_joint_name = getattr(joint, 'paired_gripping_surface_joint_name', None) if joint else None
        saved_name = getattr(joint, 'paired_gripping_surface_name', None) if joint else None
        saved_link = getattr(joint, 'paired_gripping_surface_link_name', None) if joint else None
        saved_center = (
            np.array(joint.paired_gripping_surface_center_local, dtype=float)
            if joint is not None and getattr(joint, 'paired_gripping_surface_center_local', None) is not None
            else None
        )
        preferred_name = None
        if joint is not None:
            preferred_name = getattr(joint, 'gripping_surface_name', None) or getattr(
                joint,
                'contact_surface_name',
                None
            )

        filtered_candidates = [
            candidate
            for candidate in candidates
            if candidate.get('source_joint_name') == selected_joint_name
        ]

        self.second_surface_list.blockSignals(True)
        self.second_surface_list.clear()

        if not filtered_candidates:
            placeholder = QtWidgets.QListWidgetItem("No faces found for selected second link")
            placeholder.setData(QtCore.Qt.UserRole, None)
            self.second_surface_list.addItem(placeholder)
            self.second_surface_list.blockSignals(False)
            return None

        selected_item = None
        selected_candidate = None
        best_item = None
        best_candidate = None
        best_distance = float('inf')

        for candidate in filtered_candidates:
            item = QtWidgets.QListWidgetItem(candidate['display_name'])
            item.setData(QtCore.Qt.UserRole, candidate)
            self.second_surface_list.addItem(item)

            if (
                saved_joint_name == candidate.get('source_joint_name')
                and saved_name == candidate['surface_name']
                and saved_link == candidate['link_name']
            ):
                selected_item = item
                selected_candidate = candidate

            if saved_link is not None and saved_center is not None and candidate['link_name'] == saved_link:
                distance = float(
                    np.linalg.norm(
                        np.array(candidate['local_center'], dtype=float) - saved_center
                    )
                )
                if distance < best_distance:
                    best_distance = distance
                    best_item = item
                    best_candidate = candidate

        if selected_item is None and best_item is not None:
            selected_item = best_item
            selected_candidate = best_candidate

        if selected_item is None and preferred_name:
            for row in range(self.second_surface_list.count()):
                item = self.second_surface_list.item(row)
                candidate = item.data(QtCore.Qt.UserRole)
                if isinstance(candidate, dict) and candidate.get('surface_name') == preferred_name:
                    selected_item = item
                    selected_candidate = candidate
                    break

        if selected_item is None and self.second_surface_list.count() > 0:
            selected_item = self.second_surface_list.item(0)
            row_candidate = selected_item.data(QtCore.Qt.UserRole)
            selected_candidate = row_candidate if isinstance(row_candidate, dict) else None

        if selected_item is not None:
            self.second_surface_list.setCurrentItem(selected_item)

        self.second_surface_list.blockSignals(False)
        return selected_candidate

    def _update_gripping_surface_labels(self, joint_name):
        if not self._has_contact_surface_ui():
            return
        joint = self.mw.robot.joints.get(joint_name) if joint_name else None

        summary = self._gripping_surface_summary(joint_name) if joint is not None else None
        if summary is None:
            self.gripping_surface_status_label.setText("Gripping Surface: not set.")
            self.gripping_surface_status_label.setStyleSheet(
                "color: #616161; font-size: 12px;"
            )
        else:
            surface_name, link_name, center_str = summary
            self.gripping_surface_status_label.setText(
                f"Gripping Surface: {surface_name} on {link_name} @ ({center_str}) cm"
            )
            self.gripping_surface_status_label.setStyleSheet(
                "color: #2e7d32; font-size: 12px; font-weight: bold;"
            )

        pair_enabled = bool(getattr(joint, 'paired_gripping_enabled', False)) if joint is not None else False
        pair_summary = self._paired_gripping_surface_summary(joint_name) if joint is not None else None
        if not pair_enabled:
            if pair_summary is None:
                self.paired_gripping_surface_status_label.setText(
                    "Second Gripping Surface: disabled."
                )
            else:
                surface_name, link_name, pair_joint_name, _ = pair_summary
                self.paired_gripping_surface_status_label.setText(
                    f"Second Gripping Surface: disabled ({surface_name} on {link_name} via {pair_joint_name})."
                )
            self.paired_gripping_surface_status_label.setStyleSheet(
                "color: #616161; font-size: 12px;"
            )
            return

        if pair_summary is None:
            self.paired_gripping_surface_status_label.setText(
                "Second Gripping Surface: enabled. Choose second link, then select its face from the list."
            )
            self.paired_gripping_surface_status_label.setStyleSheet(
                "color: #ef6c00; font-size: 12px; font-weight: bold;"
            )
            return

        surface_name, link_name, pair_joint_name, center_str = pair_summary
        self.paired_gripping_surface_status_label.setText(
            f"Second Gripping Surface: {surface_name} on {link_name} via {pair_joint_name} @ ({center_str}) cm"
        )
        self.paired_gripping_surface_status_label.setStyleSheet(
            "color: #2e7d32; font-size: 12px; font-weight: bold;"
        )

    def _populate_surface_list(self, joint_name, candidates):
        if not self._has_contact_surface_ui():
            return None
        joint = self.mw.robot.joints.get(joint_name)
        selected_name = getattr(joint, 'contact_surface_name', None) if joint else None
        selected_link = getattr(joint, 'contact_surface_link_name', None) if joint else None
        selected_center = (
            np.array(joint.contact_surface_center_local, dtype=float)
            if joint is not None and getattr(joint, 'contact_surface_center_local', None) is not None
            else None
        )

        self.surface_list.blockSignals(True)
        self.surface_list.clear()
        selected_item = None
        selected_candidate = None
        candidate_items = []

        for candidate in candidates:
            item = QtWidgets.QListWidgetItem(candidate['display_name'])
            item.setData(QtCore.Qt.UserRole, candidate)
            self.surface_list.addItem(item)
            candidate_items.append((item, candidate))

            if (
                selected_name == candidate['surface_name']
                and selected_link == candidate['link_name']
            ):
                selected_item = item
                selected_candidate = candidate

        if selected_item is None and selected_link is not None and selected_center is not None:
            best_item = None
            best_candidate = None
            best_distance = float('inf')

            for item, candidate in candidate_items:
                if candidate['link_name'] != selected_link:
                    continue

                candidate_center = np.array(candidate['local_center'], dtype=float)
                distance = float(np.linalg.norm(candidate_center - selected_center))
                if distance < best_distance:
                    best_distance = distance
                    best_item = item
                    best_candidate = candidate

            if best_item is not None:
                selected_item = best_item
                selected_candidate = best_candidate
                self._set_joint_surface_name(joint_name, best_candidate['surface_name'])

        if selected_item is not None:
            self.surface_list.setCurrentItem(selected_item)
        self.surface_list.blockSignals(False)
        return selected_candidate

    def refresh_contact_surface_ui(self, joint_name=None):
        if not self._has_contact_surface_ui():
            return
        joint_name = joint_name or self._selected_joint_name()
        if not joint_name or joint_name not in self.mw.robot.joints:
            self.surface_target_label.setText("Target Link: -")
            self.surface_list.clear()
            self.surface_list.setEnabled(False)
            self._highlight_surface_candidate(None)
            self.surface_status_label.setText(
                "Select a gripper joint to see its face names."
            )
            self.surface_status_label.setStyleSheet(
                "color: #757575; font-size: 12px; padding-top: 4px;"
            )
            self.select_surface_btn.setEnabled(False)
            self.refresh_surface_btn.setEnabled(False)
            self.select_gripping_surface_btn.setEnabled(False)
            self.use_second_surface_check.blockSignals(True)
            self.use_second_surface_check.setChecked(False)
            self.use_second_surface_check.blockSignals(False)
            self.use_second_surface_check.setEnabled(False)
            self.second_link_combo.blockSignals(True)
            self.second_link_combo.clear()
            self.second_link_combo.addItem("Select a gripper joint first")
            self.second_link_combo.setItemData(0, None, QtCore.Qt.UserRole)
            self.second_link_combo.blockSignals(False)
            self.second_link_combo.setEnabled(False)
            self.second_surface_list.clear()
            self.second_surface_list.addItem("Select a gripper joint first")
            self.second_surface_list.setEnabled(False)
            self.second_surface_combo.clear()
            self.second_surface_combo.addItem("Select a gripper joint first")
            self.second_surface_combo.setItemData(0, None, QtCore.Qt.UserRole)
            self.second_surface_combo.setEnabled(False)
            self._update_gripping_surface_labels(None)
            self._update_selected_faces_overlay(None)
            return

        joint = self.mw.robot.joints[joint_name]
        target_link = joint.child_link.name if joint.child_link else "-"
        self.surface_target_label.setText(f"Target Link: {target_link}")
        self.select_surface_btn.setEnabled(bool(joint.is_gripper))
        self.refresh_surface_btn.setEnabled(bool(joint.is_gripper))
        self.select_gripping_surface_btn.setEnabled(bool(joint.is_gripper))

        if not joint.is_gripper:
            self.surface_list.clear()
            self.surface_list.setEnabled(False)
            self._highlight_surface_candidate(None)
            self.surface_status_label.setText(
                "Mark this joint as Gripper to show its named contact faces."
            )
            self.surface_status_label.setStyleSheet(
                "color: #ef6c00; font-size: 12px; padding-top: 4px;"
            )
            self.use_second_surface_check.blockSignals(True)
            self.use_second_surface_check.setChecked(False)
            self.use_second_surface_check.blockSignals(False)
            self.use_second_surface_check.setEnabled(False)
            self.second_link_combo.blockSignals(True)
            self.second_link_combo.clear()
            self.second_link_combo.addItem("Mark this joint as Gripper first")
            self.second_link_combo.setItemData(0, None, QtCore.Qt.UserRole)
            self.second_link_combo.blockSignals(False)
            self.second_link_combo.setEnabled(False)
            self.second_surface_list.clear()
            self.second_surface_list.addItem("Mark this joint as Gripper first")
            self.second_surface_list.setEnabled(False)
            self.second_surface_combo.clear()
            self.second_surface_combo.addItem("Mark this joint as Gripper first")
            self.second_surface_combo.setItemData(0, None, QtCore.Qt.UserRole)
            self.second_surface_combo.setEnabled(False)
            self._update_gripping_surface_labels(joint_name)
            self._update_selected_faces_overlay(None)
            return

        candidates = self._get_surface_candidates(joint_name)
        selected_candidate = self._populate_surface_list(joint_name, candidates)
        second_candidates = self._get_second_surface_candidates(joint_name)
        self._populate_second_surface_combo(joint_name, second_candidates)
        self.surface_list.setEnabled(bool(candidates))
        self.use_second_surface_check.blockSignals(True)
        self.use_second_surface_check.setChecked(
            bool(getattr(joint, 'paired_gripping_enabled', False))
        )
        self.use_second_surface_check.blockSignals(False)
        self.use_second_surface_check.setEnabled(True)
        second_enabled = bool(getattr(joint, 'paired_gripping_enabled', False) and second_candidates)
        self.second_link_combo.setEnabled(second_enabled)
        self.second_surface_list.setEnabled(second_enabled)
        self.second_surface_combo.setEnabled(second_enabled)
        if not second_candidates:
            self.second_surface_list.clear()
            self.second_surface_list.addItem("No opposite gripper surfaces found")
            self.second_surface_list.setEnabled(False)
            self.second_link_combo.setEnabled(False)
            self.second_surface_combo.setEnabled(False)
        elif not getattr(joint, 'paired_gripping_enabled', False):
            self.second_surface_list.setEnabled(False)
            self.second_link_combo.setEnabled(False)
            self.second_surface_combo.setEnabled(False)
            if self.second_surface_list.count() == 0:
                self.second_surface_list.addItem("Tick the second-surface option to select")
        self.second_surface_combo.setEnabled(
            bool(getattr(joint, 'paired_gripping_enabled', False) and second_candidates)
        )

        if not candidates:
            self._highlight_surface_candidate(None)
            self.surface_status_label.setText(
                "No face names could be detected for this gripper yet."
            )
            self.surface_status_label.setStyleSheet(
                "color: #757575; font-size: 12px; padding-top: 4px;"
            )
            self._update_gripping_surface_labels(joint_name)
            self._update_selected_faces_overlay(joint_name)
            return

        surface_name = getattr(joint, 'contact_surface_name', None)
        link_name = getattr(joint, 'contact_surface_link_name', None)
        center_local = getattr(joint, 'contact_surface_center_local', None)

        if link_name and center_local is not None and link_name in self.mw.robot.links:
            self._highlight_surface_candidate(selected_candidate)
            link = self.mw.robot.links[link_name]
            world_center = (
                link.t_world @ np.append(np.array(center_local, dtype=float), 1.0)
            )[:3]
            ratio = getattr(self.mw.canvas, 'grid_units_per_cm', 1.0) or 1.0
            center_cm = world_center / ratio
            center_str = ", ".join(f"{coord:.2f}" for coord in center_cm)
            surface_label = surface_name if surface_name else link_name
            self.surface_status_label.setText(
                f"Selected Surface: {surface_label} on {link_name} @ ({center_str}) cm"
            )
            self.surface_status_label.setStyleSheet(
                "color: #2e7d32; font-size: 12px; padding-top: 4px;"
            )
            self._update_gripping_surface_labels(joint_name)
            self._update_selected_faces_overlay(joint_name)
            return

        self._highlight_surface_candidate(selected_candidate)
        self.surface_status_label.setText(
            f"{len(candidates)} face names detected. Click one below or pick a face in 3D."
        )
        self.surface_status_label.setStyleSheet(
            "color: #757575; font-size: 12px; padding-top: 4px;"
        )
        self._update_gripping_surface_labels(joint_name)
        self._update_selected_faces_overlay(joint_name)

    def on_refresh_surface_names(self):
        if not self._has_contact_surface_ui():
            return
        joint_name = self._selected_joint_name()
        if not joint_name:
            self.mw.log("Select a gripper joint first before refreshing face names.")
            self.mw.show_toast("Select a gripper joint first", "warning")
            return

        self.refresh_contact_surface_ui(joint_name)
        self.mw.log(f"Refreshed named surfaces for gripper joint '{joint_name}'.")

    def on_surface_candidate_clicked(self, item):
        if not self._has_contact_surface_ui():
            return
        candidate = item.data(QtCore.Qt.UserRole)
        joint_name = self._selected_joint_name()
        if not joint_name or not isinstance(candidate, dict):
            return

        self._apply_surface_candidate(joint_name, candidate)

    def on_second_link_changed(self, _index):
        if not self._has_contact_surface_ui():
            return
        joint_name = self._selected_joint_name()
        if not joint_name:
            return

        second_joint_name = self._selected_second_joint_name()
        second_candidates = self._get_second_surface_candidates(joint_name)
        self._populate_second_surface_list(joint_name, second_candidates, second_joint_name)

    def on_second_surface_candidate_clicked(self, _item):
        if not self._has_contact_surface_ui():
            return
        # Selection is read directly when "Select As Gripping Surface" is pressed.
        pass

    def on_show_selected_faces_toggled(self, _checked):
        if not self._has_contact_surface_ui():
            return
        self._update_selected_faces_overlay(self._selected_joint_name())

    def on_use_second_surface_toggled(self, checked):
        if not self._has_contact_surface_ui():
            return
        joint_name = self._selected_joint_name()
        if not joint_name:
            return

        self._set_paired_gripping_enabled(joint_name, checked)
        if checked and not self._get_second_surface_candidates(joint_name):
            self.mw.log(
                "No opposite gripper surfaces were found yet. Create or mark the second jaw first."
            )
            self.mw.show_toast("No second gripper surface found", "warning")
        self.refresh_contact_surface_ui(joint_name)

    def on_select_gripping_surface(self):
        if not self._has_contact_surface_ui():
            return
        joint_name = self._selected_joint_name()
        if not joint_name:
            self.mw.log("Select a gripper joint first before assigning a gripping surface.")
            self.mw.show_toast("Select a gripper joint first", "warning")
            return

        candidate = self._current_surface_candidate_for_action(joint_name)
        if candidate is None:
            self.mw.log("Select or pick a surface first, then assign it as the gripping surface.")
            self.mw.show_toast("Select a surface first", "warning")
            return

        pair_candidate = None
        if self.use_second_surface_check.isChecked():
            pair_candidate = self._selected_second_surface_candidate()
            if pair_candidate is None:
                self.mw.log(
                    "Choose the second link and then select the second gripping face, or untick the second-surface option."
                )
                self.mw.show_toast("Select the second gripping surface", "warning")
                return

        if not self._set_joint_gripping_surface(joint_name, candidate):
            self.mw.log("Unable to save the gripping surface selection.")
            self.mw.show_toast("Unable to save gripping surface", "error")
            return

        if self.use_second_surface_check.isChecked():
            if not self._set_joint_paired_gripping_surface(joint_name, pair_candidate):
                self.mw.log("Unable to save the second gripping surface selection.")
                self.mw.show_toast("Unable to save second gripping surface", "error")
                return
            pair_joint_name = pair_candidate.get('source_joint_name')
            self._set_active_gripper_context([joint_name, pair_joint_name])

            self.mw.log(
                "Gripping pair set: "
                f"'{candidate.get('surface_name', 'Surface')}' and "
                f"'{pair_candidate.get('surface_name', 'Surface')}'."
            )
            self.mw.show_toast("Gripping pair saved", "success")
        else:
            self._set_paired_gripping_enabled(joint_name, False)
            self._set_active_gripper_context([joint_name])
            self.mw.log(
                f"Gripping surface set: '{candidate.get('surface_name', 'Surface')}' on '{candidate.get('link_name', '-')}'."
            )
            self.mw.show_toast("Gripping surface saved", "success")

        self.refresh_contact_surface_ui(joint_name)

    def on_select_contact_surface(self):
        if not self._has_contact_surface_ui():
            return
        joint_name = self._selected_joint_name()
        if not joint_name:
            self.mw.log("Select a gripper joint first before choosing a contact surface.")
            self.mw.show_toast("Select a gripper joint first", "warning")
            return

        self.mw.joint_tab.on_select_gripper_surface(
            joint_id=joint_name,
            on_surface_picked=self._on_contact_surface_picked
        )

    def _on_contact_surface_picked(self, selection):
        joint_name = selection.get('joint_id') if isinstance(selection, dict) else None
        self.refresh_contact_surface_ui(joint_name)

    def _propagate_relation(self, joint_name, value):
        """Propagate movement across related joints (bidirectional)."""
        robot = self.mw.robot

        if joint_name in robot.joint_relations:
            for slave_id, ratio in robot.joint_relations[joint_name]:
                if slave_id in robot.joints:
                    slave_joint = robot.joints[slave_id]
                    slave_val = np.clip(
                        value * ratio, slave_joint.min_limit, slave_joint.max_limit
                    )
                    self._update_joint_silent(slave_id, slave_val)
        else:
            for master_id, slaves in robot.joint_relations.items():
                for slave_id, ratio in slaves:
                    if slave_id == joint_name and abs(ratio) > 1e-6:
                        master_val = value / ratio
                        master_joint = robot.joints.get(master_id)
                        if master_joint:
                            master_val = np.clip(
                                master_val, master_joint.min_limit, master_joint.max_limit
                            )
                            self._update_joint_silent(master_id, master_val)

                            for other_slave_id, other_ratio in robot.joint_relations[master_id]:
                                if other_slave_id != joint_name:
                                    other_val = np.clip(
                                        master_val * other_ratio,
                                        robot.joints[other_slave_id].min_limit,
                                        robot.joints[other_slave_id].max_limit
                                    )
                                    self._update_joint_silent(other_slave_id, other_val)
                        break

    def _update_joint_silent(self, joint_id, value):
        """Update a joint value and sync UI without triggering signals."""
        if joint_id not in self.mw.robot.joints:
            return

        joint = self.mw.robot.joints[joint_id]
        joint.current_value = value

        link_name = None
        if hasattr(self.mw, 'joint_tab'):
            for name, data in self.mw.joint_tab.joints.items():
                if data.get('joint_id') == joint_id:
                    link_name = name
                    break

            if link_name:
                self.mw.joint_tab.joints[link_name]['current_angle'] = value
                if self.mw.joint_tab.active_joint_control == link_name:
                    self.mw.joint_tab.joint_control_slider.blockSignals(True)
                    self.mw.joint_tab.joint_control_slider.setValue(int(value * 10))
                    self.mw.joint_tab.joint_control_slider.blockSignals(False)
                    self.mw.joint_tab.joint_control_spinbox.blockSignals(True)
                    self.mw.joint_tab.joint_control_spinbox.setValue(value)
                    self.mw.joint_tab.joint_control_spinbox.blockSignals(False)

        if hasattr(self.mw, 'matrices_tab'):
            self.mw.matrices_tab.sync_slider(link_name if link_name else joint_id, value)

    def on_stroke_changed(self, value):
        item = self.joints_list.currentItem()
        if not item:
            return

        name = item.data(QtCore.Qt.UserRole)
        joint = self.mw.robot.joints[name]
        joint_span = joint.max_limit - joint.min_limit
        target = joint.min_limit if abs(joint_span) < 1e-9 else (
            joint.min_limit + (value / 100.0) * joint_span
        )

        joint.current_value = target
        self._propagate_relation(name, target)

        # Send to Hardware (Digital Twin Sync)
        if hasattr(self.mw, 'serial_mgr') and self.mw.serial_mgr.is_connected:
            speed = float(getattr(self.mw, 'current_speed', 50))
            self.mw.serial_mgr.send_command(name, target, speed=speed)

        self.mw.robot.update_kinematics()
        self.mw.canvas.update_transforms(self.mw.robot)
        self.refresh_contact_surface_ui(name)

