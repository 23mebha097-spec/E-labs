from PyQt5 import QtWidgets, QtGui, QtCore
import numpy as np
import os
import random

from core.import_units import (
    SUPPORTED_LENGTH_UNITS,
    UP_AXIS_OPTIONS,
    detect_file_unit,
    get_engine_internal_unit,
    get_engine_units_per_cm,
    rotation_matrix_for_up_axis,
    unit_scale_to_internal,
)


class ImportOptionsDialog(QtWidgets.QDialog):
    def __init__(self, parent, file_name, detected_unit=None, detected_source="", default_unit="mm", default_up_axis="preserve"):
        super().__init__(parent)
        self.setWindowTitle("Import Units")
        self.setModal(True)
        self.resize(420, 220)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setSpacing(12)

        header = QtWidgets.QLabel(f"Import options for {file_name}")
        header.setStyleSheet("font-size: 15px; font-weight: bold; color: #1976d2;")
        layout.addWidget(header)

        detected_text = detected_unit or "Not detected"
        source_text = detected_source or "manual selection required"
        info = QtWidgets.QLabel(
            f"Detected unit: {detected_text}\nSource: {source_text}\nEngine internal unit: {get_engine_internal_unit()} (1 cm = {get_engine_units_per_cm():.0f} {get_engine_internal_unit()})"
        )
        info.setWordWrap(True)
        layout.addWidget(info)

        form = QtWidgets.QFormLayout()

        self.unit_combo = QtWidgets.QComboBox()
        self.unit_combo.addItems(list(SUPPORTED_LENGTH_UNITS))
        self.unit_combo.setCurrentText(detected_unit or default_unit)
        form.addRow("CAD unit", self.unit_combo)

        self.axis_combo = QtWidgets.QComboBox()
        self.axis_combo.addItems(list(UP_AXIS_OPTIONS))
        self.axis_combo.setCurrentText(default_up_axis if default_up_axis in UP_AXIS_OPTIONS else "preserve")
        form.addRow("Source up-axis", self.axis_combo)

        layout.addLayout(form)

        button_box = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

    def selected_options(self):
        return {
            "unit": self.unit_combo.currentText(),
            "up_axis": self.axis_combo.currentText(),
        }


class LinksMixin:
    """Methods for managing robot links: import, select, base, remove, color."""

    def setup_links_tab(self):
        layout = QtWidgets.QVBoxLayout(self.links_tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)
        
        # Section header
        header = QtWidgets.QLabel("COMPONENTS")
        header.setStyleSheet("color: #1976d2; font-size: 16px; font-weight: bold; padding: 4px 0;")
        layout.addWidget(header)
        
        import_btn = QtWidgets.QPushButton("Import STEP / STL")
        import_btn.setCursor(QtCore.Qt.PointingHandCursor)
        import_btn.setStyleSheet("""
            QPushButton {
                background-color: #1976d2;
                color: white;
                font-weight: bold;
                font-size: 14px;
                padding: 12px;
                border: none;
                border-radius: 8px;
            }
            QPushButton:hover { background-color: #1565c0; }
        """)
        import_btn.clicked.connect(self.import_mesh)
        layout.addWidget(import_btn)
        
        self.links_list = QtWidgets.QListWidget()
        self.links_list.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.links_list.setStyleSheet("""
            QListWidget {
                background-color: white;
                border: 1px solid #e0e0e0;
                border-radius: 6px;
                font-size: 14px;
            }
            QListWidget::item {
                padding: 6px 4px;
                border-bottom: 1px solid #f0f0f0;
            }
            QListWidget::item:selected {
                background-color: #e3f2fd;
            }
        """)
        self.links_list.itemClicked.connect(self.on_link_selected)
        layout.addWidget(self.links_list)
        
        btn_layout = QtWidgets.QHBoxLayout()
        btn_layout.setSpacing(8)
        self.set_base_btn = QtWidgets.QPushButton("Set as Base")
        self.set_base_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.remove_btn = QtWidgets.QPushButton("Remove")
        self.remove_btn.setCursor(QtCore.Qt.PointingHandCursor)
        
        self.set_base_btn.clicked.connect(self.set_as_base)
        self.remove_btn.clicked.connect(self.remove_link)
        
        btn_layout.addWidget(self.set_base_btn)
        btn_layout.addWidget(self.remove_btn)
        layout.addLayout(btn_layout)

        self.color_btn = QtWidgets.QPushButton("Change Color")
        self.color_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.color_btn.clicked.connect(self.change_color)
        layout.addWidget(self.color_btn)

        rigid_card = QtWidgets.QFrame()
        rigid_card.setStyleSheet("""
            QFrame {
                background-color: #fafafa;
                border: 1px solid #dbe8f6;
                border-radius: 8px;
            }
            QLabel {
                border: none;
                background: transparent;
            }
        """)
        rigid_layout = QtWidgets.QVBoxLayout(rigid_card)
        rigid_layout.setContentsMargins(10, 10, 10, 10)
        rigid_layout.setSpacing(6)

        rigid_title = QtWidgets.QLabel("Rigid Group")
        rigid_title.setStyleSheet("color: #1976d2; font-size: 14px; font-weight: bold;")
        rigid_layout.addWidget(rigid_title)

        rigid_help = QtWidgets.QLabel(
            "Click Make Rigid Group, then click components in the scene. Press OK when done."
        )
        rigid_help.setWordWrap(True)
        rigid_help.setStyleSheet("color: #546e7a; font-size: 12px;")
        rigid_layout.addWidget(rigid_help)

        self.rigid_group_list_title = QtWidgets.QLabel("Saved Rigid Groups")
        self.rigid_group_list_title.setStyleSheet("color: #1976d2; font-size: 13px; font-weight: bold;")
        rigid_layout.addWidget(self.rigid_group_list_title)

        self.rigid_group_list = QtWidgets.QListWidget()
        self.rigid_group_list.setMinimumHeight(160)
        self.rigid_group_list.setStyleSheet("""
            QListWidget {
                background-color: white;
                border: 1px solid #d7e3f3;
                border-radius: 6px;
                font-size: 12px;
            }
            QListWidget::item {
                padding: 5px 6px;
            }
            QListWidget::item:selected {
                background-color: #e3f2fd;
            }
        """)
        self.rigid_group_list.itemSelectionChanged.connect(self.on_rigid_group_selection_changed)
        rigid_layout.addWidget(self.rigid_group_list)

        rigid_btn_row = QtWidgets.QHBoxLayout()

        self.rigid_group_btn = QtWidgets.QPushButton("Make Rigid Group")
        self.rigid_group_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.rigid_group_btn.setStyleSheet("""
            QPushButton {
                background-color: #2e7d32;
                color: white;
                font-weight: bold;
                font-size: 13px;
                padding: 10px;
                border: none;
                border-radius: 6px;
            }
            QPushButton:hover {
                background-color: #256428;
            }
        """)
        self.rigid_group_btn.clicked.connect(self.begin_rigid_group_selection)
        rigid_btn_row.addWidget(self.rigid_group_btn)

        self.rigid_group_ok_btn = QtWidgets.QPushButton("OK")
        self.rigid_group_ok_btn.setEnabled(False)
        self.rigid_group_ok_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.rigid_group_ok_btn.setStyleSheet("""
            QPushButton {
                background-color: #1e88e5;
                color: white;
                font-weight: bold;
                font-size: 13px;
                padding: 10px;
                border: none;
                border-radius: 6px;
            }
            QPushButton:hover { background-color: #1976d2; }
            QPushButton:disabled { background-color: #bbdefb; color: #e3f2fd; }
        """)
        self.rigid_group_ok_btn.clicked.connect(self.confirm_rigid_group_selection)
        rigid_btn_row.addWidget(self.rigid_group_ok_btn)

        self.rigid_group_cancel_btn = QtWidgets.QPushButton("Cancel")
        self.rigid_group_cancel_btn.setEnabled(False)
        self.rigid_group_cancel_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.rigid_group_cancel_btn.setStyleSheet("""
            QPushButton {
                background-color: #ef6c00;
                color: white;
                font-weight: bold;
                font-size: 13px;
                padding: 10px;
                border: none;
                border-radius: 6px;
            }
            QPushButton:hover { background-color: #e65100; }
            QPushButton:disabled { background-color: #ffe0b2; color: #fff3e0; }
        """)
        self.rigid_group_cancel_btn.clicked.connect(self.cancel_rigid_group_selection)
        rigid_btn_row.addWidget(self.rigid_group_cancel_btn)

        rigid_layout.addLayout(rigid_btn_row)

        rigid_delete_row = QtWidgets.QHBoxLayout()
        self.rigid_group_delete_btn = QtWidgets.QPushButton("Delete Relation")
        self.rigid_group_delete_btn.setEnabled(False)
        self.rigid_group_delete_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.rigid_group_delete_btn.setStyleSheet("""
            QPushButton {
                background-color: #d32f2f;
                color: white;
                font-weight: bold;
                font-size: 13px;
                padding: 10px;
                border: none;
                border-radius: 6px;
            }
            QPushButton:hover { background-color: #b71c1c; }
            QPushButton:disabled { background-color: #ffcdd2; color: #fff5f5; }
        """)
        self.rigid_group_delete_btn.clicked.connect(self.delete_selected_rigid_group_relation)
        rigid_delete_row.addWidget(self.rigid_group_delete_btn)
        rigid_layout.addLayout(rigid_delete_row)

        layout.addWidget(rigid_card)
        self._refresh_rigid_group_list()

        layout.addStretch()

    def on_link_selected(self, item):
        name = item.text()
        
        # Allow all objects to be selected, including jointed ones
        self.canvas.select_actor(name)
        
        # Update button text based on whether selection is the base
        if self.robot.base_link and name == self.robot.base_link.name:
            self.set_base_btn.setText("Deselect as Base")
        else:
            self.set_base_btn.setText("Set as Base")

    def _set_rigid_group_ui_state(self, active):
        active = bool(active)
        if hasattr(self, "rigid_group_ok_btn"):
            self.rigid_group_ok_btn.setEnabled(active and len(getattr(self, "_rigid_group_selected_names", [])) >= 2)
        if hasattr(self, "rigid_group_cancel_btn"):
            self.rigid_group_cancel_btn.setEnabled(active)
        if hasattr(self, "rigid_group_delete_btn"):
            self.rigid_group_delete_btn.setEnabled(not active and self._selected_rigid_group_record() is not None)
        if hasattr(self, "rigid_group_btn"):
            self.rigid_group_btn.setText("Collecting..." if active else "Make Rigid Group")
        if hasattr(self, "rigid_group_list_title"):
            self.rigid_group_list_title.setText("Selected Components" if active else "Saved Rigid Groups")
        self._refresh_rigid_group_list()

    def _refresh_rigid_group_list(self):
        if not hasattr(self, "rigid_group_list"):
            return
        self.rigid_group_list.blockSignals(True)
        self.rigid_group_list.clear()
        active = bool(getattr(self, "_rigid_group_selection_active", False))
        if active:
            selected = list(getattr(self, "_rigid_group_selected_names", []))
            anchor = getattr(self, "_rigid_group_anchor_name", None)
            for idx, name in enumerate(selected, start=1):
                label = f"{idx}. {name}"
                if anchor and name == anchor:
                    label += " [anchor]"
                item = QtWidgets.QListWidgetItem(label)
                if name == anchor:
                    item.setBackground(QtGui.QColor("#e8f0fe"))
                self.rigid_group_list.addItem(item)
            if hasattr(self, "rigid_group_ok_btn"):
                self.rigid_group_ok_btn.setEnabled(len(selected) >= 2)
        else:
            self._ensure_rigid_groups_store()
            for group in self.rigid_groups:
                if not isinstance(group, dict):
                    continue
                group_id = str(group.get("group_id", ""))
                anchor = str(group.get("anchor", ""))
                members = [str(name) for name in group.get("members", [])]
                member_text = ", ".join(members) if members else "no members"
                label = f"{anchor} -> {member_text}"
                if group_id:
                    label = f"{group_id}: {label}"
                item = QtWidgets.QListWidgetItem(label)
                item.setData(QtCore.Qt.UserRole, group_id)
                self.rigid_group_list.addItem(item)
        self.rigid_group_list.blockSignals(False)
        if hasattr(self, "rigid_group_delete_btn"):
            self.rigid_group_delete_btn.setEnabled(
                not active and self._selected_rigid_group_record() is not None
            )

    def _ensure_rigid_groups_store(self):
        if not hasattr(self, "rigid_groups") or self.rigid_groups is None:
            self.rigid_groups = []
        return self.rigid_groups

    def _selected_rigid_group_record(self):
        if not hasattr(self, "rigid_group_list"):
            return None
        if getattr(self, "_rigid_group_selection_active", False):
            return None
        item = self.rigid_group_list.currentItem()
        if item is None:
            return None
        group_id = item.data(QtCore.Qt.UserRole)
        if not group_id:
            return None
        for group in self._ensure_rigid_groups_store():
            if str(group.get("group_id", "")) == str(group_id):
                return group
        return None

    def on_rigid_group_selection_changed(self):
        if hasattr(self, "rigid_group_delete_btn"):
            self.rigid_group_delete_btn.setEnabled(
                not getattr(self, "_rigid_group_selection_active", False)
                and self._selected_rigid_group_record() is not None
            )

    def begin_rigid_group_selection(self):
        """Start collecting 3D clicks for a rigid group."""
        self._rigid_group_selection_active = True
        self._rigid_group_selected_names = []
        self._rigid_group_anchor_name = None
        self._set_rigid_group_ui_state(True)
        if hasattr(self.canvas, "start_actor_click_capture"):
            self.canvas.start_actor_click_capture(self._on_rigid_group_actor_clicked)
        if hasattr(self, "show_toast"):
            self.show_toast("Click components in the scene, then press OK.", "info")
        if hasattr(self, "log"):
            self.log("Rigid group selection started. Click objects in the 3D scene.")

    def cancel_rigid_group_selection(self):
        """Abort rigid group selection without creating joints."""
        self._rigid_group_selection_active = False
        self._rigid_group_selected_names = []
        self._rigid_group_anchor_name = None
        if hasattr(self.canvas, "stop_actor_click_capture"):
            self.canvas.stop_actor_click_capture()
        self._set_rigid_group_ui_state(False)
        if hasattr(self, "log"):
            self.log("Rigid group selection cancelled.")

    def _on_rigid_group_actor_clicked(self, name):
        """Collect clicked objects while rigid-group selection is active."""
        if not getattr(self, "_rigid_group_selection_active", False):
            return False
        if name not in getattr(self.robot, "links", {}):
            return True
        if name in getattr(self, "_rigid_group_selected_names", []):
            self._rigid_group_selected_names = [n for n in self._rigid_group_selected_names if n != name]
            if getattr(self, "_rigid_group_anchor_name", None) == name:
                self._rigid_group_anchor_name = self._rigid_group_selected_names[0] if self._rigid_group_selected_names else None
        else:
            self._rigid_group_selected_names.append(name)
            if self._rigid_group_anchor_name is None:
                self._rigid_group_anchor_name = name

        self._refresh_rigid_group_list()
        if hasattr(self, "canvas"):
            self.canvas.select_actor(name)
        if hasattr(self, "log"):
            self.log(f"Rigid group selection: {', '.join(self._rigid_group_selected_names) or 'none'}")
        return True

    def _register_rigid_group(self, anchor_name, member_names, joint_names):
        store = self._ensure_rigid_groups_store()
        group_id = f"rigid_group_{len(store) + 1}"
        member_names = [str(name) for name in member_names if name != anchor_name]
        record = {
            "group_id": group_id,
            "anchor": str(anchor_name),
            "members": member_names,
            "joint_names": [str(name) for name in joint_names],
        }
        store.append(record)
        for joint_name in joint_names:
            joint = self.robot.joints.get(joint_name)
            if joint is None:
                continue
            joint.rigid_group_id = group_id
            joint.rigid_group_anchor = str(anchor_name)
            joint.rigid_group_members = list(member_names)
        if hasattr(self, "_refresh_rigid_group_list"):
            self._refresh_rigid_group_list()
        return record

    def confirm_rigid_group_selection(self):
        """Create the rigid group from the collected scene clicks."""
        if not getattr(self, "_rigid_group_selection_active", False):
            self.show_toast("Click Make Rigid Group first", "warning")
            return

        selected = list(getattr(self, "_rigid_group_selected_names", []))
        if len(selected) < 2:
            self.show_toast("Select at least 2 components", "warning")
            return

        anchor_name = getattr(self, "_rigid_group_anchor_name", None) or selected[0]
        created_joints, skipped = self._create_rigid_group(anchor_name, selected)
        if created_joints:
            self._register_rigid_group(anchor_name, selected, created_joints)
        self.cancel_rigid_group_selection()

        if created_joints:
            self.log(f"Rigid group created: '{anchor_name}' now drives {len(created_joints)} selected component(s).")
            self.show_toast("Rigid group created", "success")
        if skipped:
            skipped_text = ", ".join(skipped)
            self.log(f"Rigid group skipped already-jointed component(s): {skipped_text}")
            self.show_toast("Some selected components already have joints", "warning")

    def delete_selected_rigid_group_relation(self):
        """Delete only the stored rigid-group joints, keeping all components."""
        if getattr(self, "_rigid_group_selection_active", False):
            self.show_toast("Finish or cancel the active selection first", "warning")
            return

        group = self._selected_rigid_group_record()
        if group is None:
            self.show_toast("Select a rigid group first", "warning")
            return

        joint_names = list(group.get("joint_names", []))
        if not joint_names:
            self.show_toast("Rigid group has no stored joints", "warning")
            return

        removed = 0
        for joint_name in joint_names:
            joint = self.robot.joints.get(joint_name)
            if joint is None:
                continue
            child = getattr(joint, "child_link", None)
            if child is not None:
                child_world = np.array(child.t_world, dtype=float).copy()
            else:
                child_world = None
            self.robot.remove_joint(joint_name)
            if child is not None and child_world is not None:
                child.t_offset = child_world.copy()
                child.t_world = child_world.copy()
            removed += 1

        anchor = str(group.get("anchor", ""))
        self.rigid_groups = [entry for entry in self._ensure_rigid_groups_store() if str(entry.get("group_id", "")) != str(group.get("group_id", ""))]
        if hasattr(self, "alignment_cache"):
            to_remove = [
                key for key, value in self.alignment_cache.items()
                if isinstance(value, dict)
                and value.get("manual_rigid_group")
                and str(value.get("rigid_group_anchor", "")) == anchor
            ]
            for key in to_remove:
                self.alignment_cache.pop(key, None)

        self.robot.update_kinematics()
        self.canvas.update_transforms(self.robot)
        self.update_link_colors()
        self._refresh_rigid_group_list()
        if hasattr(self, "joint_tab"):
            self.joint_tab.refresh_links()
            if hasattr(self.joint_tab, "refresh_joints_history"):
                self.joint_tab.refresh_joints_history()
        if hasattr(self, "matrices_tab"):
            self.matrices_tab.refresh_sliders()
            self.matrices_tab.update_display()
        if hasattr(self, "refresh_link_hierarchy"):
            self.refresh_link_hierarchy()

        self.log(f"Rigid group relation deleted: removed {removed} fixed joint(s).")
        self.show_toast("Rigid group relation deleted", "error")

    def _selected_link_names(self):
        names = []
        if not hasattr(self, "links_list"):
            return names
        for item in self.links_list.selectedItems():
            name = item.text().strip()
            if name in getattr(self.robot, "links", {}):
                names.append(name)
        return names

    def _create_rigid_group(self, anchor_name, selected):
        """Bind selected links into one rigid assembly around anchor_name."""
        if not hasattr(self, "robot") or self.robot is None:
            return [], []

        anchor_link = self.robot.links.get(anchor_name)
        if anchor_link is None:
            return [], []

        created_joint_names = []
        skipped = []
        for child_name in selected:
            if child_name == anchor_name:
                continue
            child_link = self.robot.links.get(child_name)
            if child_link is None:
                continue
            parent_joint = getattr(child_link, "parent_joint", None)
            if parent_joint is not None and getattr(parent_joint, "joint_type", None) != "fixed":
                skipped.append(child_name)
                continue

            joint = self.robot.ensure_fixed_joint(
                anchor_name,
                child_name,
                child_world_transform=np.array(child_link.t_world, dtype=float).copy(),
                origin_world=np.array(child_link.t_world, dtype=float)[:3, 3].copy(),
            )
            if joint is None:
                continue

            if hasattr(self, "alignment_cache"):
                self.alignment_cache[(anchor_name, child_name)] = {
                    "contact_point": np.array(child_link.t_world, dtype=float)[:3, 3].tolist(),
                    "manual_rigid_group": True,
                    "rigid_group_anchor": str(anchor_name),
                }
            joint.is_rigid_attachment = True
            joint.rigid_group_anchor = str(anchor_name)
            joint.rigid_group_members = [str(name) for name in selected if name != anchor_name]
            joint.rigid_group_id = None
            created_joint_names.append(joint.name)

        if created_joint_names:
            self.robot.update_kinematics()
            self.canvas.update_transforms(self.robot)
            self.update_link_colors()
            if hasattr(self, "joint_tab"):
                self.joint_tab.refresh_links()
            if hasattr(self, "matrices_tab"):
                self.matrices_tab.refresh_sliders()
                self.matrices_tab.update_display()
        return created_joint_names, skipped

    def create_rigid_group_from_selection(self):
        """Backwards-compatible rigid group action using the current list selection."""
        selected = self._selected_link_names()
        if len(selected) < 2:
            self.show_toast("Select at least 2 components", "warning")
            return

        anchor_item = self.links_list.currentItem() if hasattr(self, "links_list") else None
        anchor_name = anchor_item.text().strip() if anchor_item is not None else selected[0]
        if anchor_name not in selected:
            anchor_name = selected[0]

        created_joints, skipped = self._create_rigid_group(anchor_name, selected)
        if created_joints:
            self._register_rigid_group(anchor_name, selected, created_joints)
            self.log(f"Rigid group created: '{anchor_name}' now drives {len(created_joints)} selected component(s).")
            self.show_toast("Rigid group created", "success")
        if skipped:
            skipped_text = ", ".join(skipped)
            self.log(f"Rigid group skipped already-jointed component(s): {skipped_text}")
            self.show_toast("Some selected components already have joints", "warning")

    def set_as_base(self):
        item = self.links_list.currentItem()
        if not item:
            return
            
        name = item.text()
        if name not in self.robot.links:
            return
            
        link = self.robot.links[name]
        
        # --- COMPLIANCE CHECK: Only un-constrained objects can become the Base ---
        if self.robot.base_link != link:
            is_aligned = False
            if hasattr(self, 'alignment_cache'):
                for (p, c), pt in self.alignment_cache.items():
                    if c == name:
                        is_aligned = True; break
            
            if link.parent_joint:
                self.log(f"⚠️ Locked: '{name}' is jointed. Components with a parent joint cannot be set as the Base.")
                QtWidgets.QMessageBox.warning(self, "Locked", f"'{name}' is part of a joint. Remove the joint before making it the Base.")
                return
            if is_aligned:
                self.log(f"⚠️ Locked: '{name}' is aligned. Undo alignment before making it the Base.")
                QtWidgets.QMessageBox.warning(self, "Locked", f"'{name}' is aligned to another component. Reset alignment first.")
                return
        
        # TOGGLE LOGIC: If it's already the base, unset it
        if self.robot.base_link == link:
            self.robot.base_link = None
            link.is_base = False
            self.canvas.fixed_actors.discard(name)
            self.log(f"BASE UNSET: {name}. Link is now floating.")
            self.set_base_btn.setText("Set as Base")
        else:
            # Preserve the STEP/assembly transform and only change the root link.
            if self.robot.base_link:
                self.canvas.fixed_actors.discard(self.robot.base_link.name)
                self.robot.base_link.is_base = False

            link.is_base = True
            self.robot.base_link = link
            self.canvas.fixed_actors.add(name)
            self.set_base_btn.setText("Deselect as Base")
            self.log(f"BASE SET: {name}")
            self.log("Assembly root updated without recentering the imported STEP transform.")
            self.canvas.plotter.reset_camera()
        
        # 3. Update Robot
        self.robot.update_kinematics()
        self.canvas.update_transforms(self.robot)
        
        # 4. Focus Camera
        self.update_link_colors()

    def go_to_joint_tab(self):
        item = self.links_list.currentItem()
        if not item:
            return
        
        name = item.text()
        # Switch to the Joint tab
        self.switch_panel(1)
        
        # Refresh links first to ensure combo boxes are up to date
        self.joint_tab.refresh_links()
        
        # Pre-select this link as the Child Link
        self.joint_tab.select_child_link(name)
        
        self.log(f"Switched to Joint creation for: {name}")

    def remove_link(self):
        item = self.links_list.currentItem()
        if not item:
            return
        
        name = item.text()
        
        # 1. Remove from Robot Model (Core)
        self.robot.remove_link(name)
        
        # 2. Cleanup and Sync Graphics state
        if self.robot.base_link:
            self.canvas.fixed_actors.discard(self.robot.base_link.name)

        # Remove from Scene (Graphics)
        self.canvas.remove_actor(name)
        
        # 3. Remove from UI List
        row = self.links_list.row(item)
        self.links_list.takeItem(row)
        
        self.log(f"Removed link: {name}")
        
        # Refresh kinematics just in case
        self.robot.update_kinematics()
        self.canvas.update_transforms(self.robot)
        self.update_link_colors()

    def update_link_colors(self):
        """Updates the icons in the link list to show Base (Red) vs Normal/Joint (Green)."""
        root = self.robot.base_link
        
        # Create helper to make colored icons
        def make_icon(color_str):
            pixmap = QtGui.QPixmap(20, 20)
            pixmap.fill(QtGui.QColor(color_str))
            return QtGui.QIcon(pixmap)
            
        red_icon = make_icon("#d32f2f")   # Base Red
        green_icon = make_icon("#388e3c") # Joint Green
        
        for i in range(self.links_list.count()):
            item = self.links_list.item(i)
            name = item.text()
            
            if name in self.robot.links:
                link = self.robot.links[name]
                if link == root:
                    item.setIcon(red_icon)
                    item.setToolTip("Base Link (Fixed/Locked)")
                else:
                    item.setIcon(green_icon)
                    item.setToolTip("Joint/Child Link")

    def change_color(self):
        item = self.links_list.currentItem()
        if not item:
            return
            
        name = item.text()
        if name not in self.robot.links:
            return
            
        link = self.robot.links[name]
        initial_color = QtGui.QColor(link.color)
        
        color = QtWidgets.QColorDialog.getColor(initial_color, self, f"Select Color for {name}")
        if color.isValid():
            hex_color = color.name()
            link.color = hex_color
            if name in self.canvas.actors:
                self.canvas.set_actor_color(name, hex_color)
            self.update_link_colors()
            self.log(f"Changed color of {name} to {hex_color}")

    def apply_manual_scale(self):
        """Manually scales the selected link mesh."""
        item = self.links_list.currentItem()
        if not item:
            return
            
        name = item.text()
        if name not in self.robot.links:
            return
            
        scale = self.scale_spin.value()
        if scale == 1.0:
            return
            
        link = self.robot.links[name]
        try:
            link.mesh.apply_scale(scale)
            # Re-apply transform to refresh visual
            self.canvas.update_link_mesh(name, link.mesh, link.t_world, color=link.color)
            self.log(f"Scaled {name} by {scale}x")
            # Reset spinbox
            self.scale_spin.setValue(1.0)
        except Exception as e:
            self.log(f"Scale Error: {e}")

    def _ask_import_options(self, file_path, detected_unit, detection_source):
        suffix = os.path.splitext(file_path)[1].lower()
        prefs = getattr(self, "import_preferences", {})
        default_unit = prefs.get("last_stl_unit", "mm")
        default_up_axis = prefs.get("last_up_axis", "preserve")

        # STL has no dependable unit metadata, so always confirm it.
        must_confirm = suffix == ".stl" or detected_unit is None
        if not must_confirm:
            return {
                "unit": detected_unit,
                "up_axis": default_up_axis,
            }

        dialog = ImportOptionsDialog(
            self,
            os.path.basename(file_path),
            detected_unit=detected_unit,
            detected_source=detection_source,
            default_unit=detected_unit or default_unit,
            default_up_axis=default_up_axis,
        )
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return None

        selected = dialog.selected_options()
        if suffix == ".stl":
            prefs["last_stl_unit"] = selected["unit"]
        prefs["last_up_axis"] = selected["up_axis"]
        self.import_preferences = prefs
        return selected

    def _extract_scene_parts(self, file_path, scene):
        """Return per-geometry parts from a trimesh scene/assembly import.

        This preserves the actual component names embedded in the scene graph
        rather than collapsing the whole STEP assembly into a single merged mesh.
        """
        if scene is None:
            return []

        parts = []
        try:
            geometry = getattr(scene, "geometry", {})
            graph = getattr(scene, "graph", None)
            node_names = list(getattr(graph, "nodes_geometry", [])) if graph is not None else []
        except Exception:
            geometry = {}
            graph = None
            node_names = []

        if not node_names:
            return []

        for node_name in node_names:
            try:
                transform, geom_name = graph.get(node_name)
            except Exception:
                continue

            geom = geometry.get(geom_name)
            if geom is None:
                continue

            try:
                mesh = geom.copy()
            except Exception:
                mesh = geom

            if not hasattr(mesh, "vertices") or len(mesh.vertices) == 0:
                continue

            parts.append({
                "name": str(node_name),
                "geometry_name": str(geom_name),
                "mesh": mesh.copy(),
                "transform": np.array(transform, dtype=float),
                "source_path": file_path,
            })

        return parts

    def _finalize_loaded_mesh(self, file_path, loaded):
        if isinstance(loaded, tuple):
            loaded = loaded[0]

        if hasattr(loaded, "geometry") and not hasattr(loaded, "vertices"):
            self.log("Detected assembly/scene. Extracting per-component geometry names...")
            scene_parts = self._extract_scene_parts(file_path, loaded)
            if scene_parts:
                return scene_parts
            self.log("Detected assembly/scene. Merging meshes...")
            mesh = loaded.to_mesh()
        else:
            mesh = loaded

        if not hasattr(mesh, "vertices") or len(mesh.vertices) == 0:
            raise ValueError("Imported mesh has 0 vertices")

        return mesh.copy()

    def _rigidize_imported_scene(self, scene_parts):
        """Wire imported STEP parts into one rigid assembly tree."""
        if not scene_parts:
            return None

        valid_parts = []
        for part in scene_parts:
            name = str(part.get("name", "")).strip()
            transform = np.array(part.get("transform", np.eye(4)), dtype=float)
            if not name or name not in self.robot.links:
                continue
            valid_parts.append({"name": name, "transform": transform})

        if len(valid_parts) <= 1:
            return None

        def _score(part):
            mat = np.array(part["transform"], dtype=float)
            return float(np.linalg.norm(mat[:3, 3]))

        root_part = min(valid_parts, key=_score)
        root_name = root_part["name"]
        root_link = self.robot.links.get(root_name)
        if root_link is None:
            return None

        root_world = np.array(root_part["transform"], dtype=float)
        root_world_inv = np.linalg.inv(root_world)
        created = 0

        for part in valid_parts:
            child_name = part["name"]
            if child_name == root_name:
                continue
            child_link = self.robot.links.get(child_name)
            if child_link is None:
                continue
            if getattr(child_link, "parent_joint", None) is not None:
                continue

            joint_name_base = f"rigid__{root_name}__{child_name}"
            joint_name = joint_name_base
            suffix = 1
            while joint_name in self.robot.joints:
                joint_name = f"{joint_name_base}_{suffix}"
                suffix += 1

            joint = self.robot.add_joint(joint_name, root_name, child_name)
            joint.joint_type = "fixed"
            joint.current_value = 0.0
            joint.origin = np.zeros(3, dtype=float)
            child_link.t_offset = root_world_inv @ np.array(part["transform"], dtype=float)
            created += 1

        if created:
            self.log(f"Rigid assembly linked: '{root_name}' now anchors {created} imported component(s).")
            self.robot.update_kinematics()
            self.canvas.update_transforms(self.robot)
        return root_name

    def _prepare_imported_mesh(self, file_path, mesh):
        raw_bounds = np.array(mesh.bounds, dtype=float)
        raw_size = raw_bounds[1] - raw_bounds[0]
        detected_unit, detection_source = detect_file_unit(file_path, mesh)
        import_options = self._ask_import_options(file_path, detected_unit, detection_source)
        if import_options is None:
            return None

        source_unit = import_options["unit"]
        up_axis = import_options["up_axis"]
        scale_to_internal = unit_scale_to_internal(source_unit)

        prepared_mesh = mesh.copy()

        if abs(scale_to_internal - 1.0) > 1e-12:
            prepared_mesh.apply_scale(scale_to_internal)

        axis_rotation = rotation_matrix_for_up_axis(up_axis)
        if not np.allclose(axis_rotation, np.eye(4)):
            prepared_mesh.apply_transform(axis_rotation)

        final_bounds = np.array(prepared_mesh.bounds, dtype=float)
        final_size = final_bounds[1] - final_bounds[0]

        debug = {
            "source_unit": source_unit,
            "detected_unit": detected_unit,
            "detection_source": detection_source,
            "scale_to_internal": scale_to_internal,
            "up_axis": up_axis,
            "raw_bounds": raw_bounds,
            "raw_size": raw_size,
            "final_bounds": final_bounds,
            "final_size": final_size,
            "engine_unit": get_engine_internal_unit(),
        }
        return prepared_mesh, debug

    def _adjust_import_transform(self, transform, import_debug):
        """Apply the same unit and axis conversion to a scene transform."""
        mat = np.array(transform, dtype=float).copy()
        scale = float(import_debug.get("scale_to_internal", 1.0) or 1.0)
        if abs(scale - 1.0) > 1e-12:
            mat[:3, 3] *= scale

        up_axis = str(import_debug.get("up_axis", "preserve") or "preserve")
        axis_rotation = rotation_matrix_for_up_axis(up_axis)
        if not np.allclose(axis_rotation, np.eye(4)):
            mat = axis_rotation @ mat
        return mat

    def _log_import_debug(self, name, debug):
        final_cm = debug["final_size"] / get_engine_units_per_cm()
        axis_label = "preserve native axes" if debug["up_axis"] == "preserve" else f"{debug['up_axis']}-up to z-up"

        self.log(
            f"[Import] {name}: source unit={debug['source_unit']} "
            f"(detected={debug['detected_unit'] or 'manual'}, via {debug['detection_source']})"
        )
        self.log(
            f"[Import] Scale factor to {debug['engine_unit']}: {debug['scale_to_internal']:.6f} "
            f"applied uniformly on X/Y/Z"
        )
        self.log(f"[Import] Up-axis mapping: {axis_label}")
        self.log(
            f"[Import] Bounding box before scaling: "
            f"{debug['raw_size'][0]:.3f} x {debug['raw_size'][1]:.3f} x {debug['raw_size'][2]:.3f} {debug['source_unit']}"
        )
        self.log(
            f"[Import] Bounding box after scaling: "
            f"{debug['final_size'][0]:.3f} x {debug['final_size'][1]:.3f} x {debug['final_size'][2]:.3f} mm "
            f"({final_cm[0]:.3f} x {final_cm[1]:.3f} x {final_cm[2]:.3f} cm)"
        )

    def measure_selected_link(self):
        item = self.links_list.currentItem()
        name = item.text() if item else getattr(self.canvas, "selected_name", None)
        if not name:
            self.show_toast("Select a link first", "warning")
            return
        measurement = self.canvas.measure_actor_bounds(name)
        if not measurement:
            self.show_toast("Unable to measure selection", "error")
            return

        dims_internal = measurement["dims_internal"]
        dims_cm = measurement["dims_cm"]
        self.log(
            f"[Measure] {name}: "
            f"{dims_internal[0]:.3f} x {dims_internal[1]:.3f} x {dims_internal[2]:.3f} {measurement['internal_unit']} "
            f"= {dims_cm[0]:.3f} x {dims_cm[1]:.3f} x {dims_cm[2]:.3f} cm"
        )
        self.show_toast(f"Measured {name}", "success")

    def add_reference_cube(self):
        import trimesh

        mesh = trimesh.creation.box(extents=[100.0, 100.0, 100.0])
        name = "reference_cube_100mm"
        base_name = name
        counter = 1
        while name in self.robot.links:
            name = f"{base_name}_{counter}"
            counter += 1

        link = self.robot.add_link(name, mesh)
        link.color = "#1976d2"
        link.import_metadata = {
            "source_unit": "mm",
            "detected_unit": "mm",
            "detection_source": "generated reference cube",
            "scale_to_internal": 1.0,
            "up_axis": "z",
            "engine_unit": get_engine_internal_unit(),
            "source_path": None,
            "raw_bounds": [[-50.0, -50.0, -50.0], [50.0, 50.0, 50.0]],
            "final_bounds": [[-50.0, -50.0, -50.0], [50.0, 50.0, 50.0]],
            "raw_size": [100.0, 100.0, 100.0],
            "final_size": [100.0, 100.0, 100.0],
        }

        self.add_link_item(name)

        t_import = np.eye(4)
        t_import[:3, 3] = [50.0 * get_engine_units_per_cm(), 50.0 * get_engine_units_per_cm(), 50.0 * get_engine_units_per_cm()]
        link.t_offset = t_import

        self.canvas.update_link_mesh(name, mesh, t_import, color=link.color)
        actor = self.canvas.actors[name]
        self.canvas.ensure_grid_fits_bounds(actor.GetBounds())
        self.canvas.select_actor(name)
        for i in range(self.links_list.count()):
            item = self.links_list.item(i)
            if item and item.text() == name:
                self.links_list.setCurrentItem(item)
                break
        self.canvas.focus_on_actor(name)
        self.update_link_colors()

        self.log("[Validation] Added reference cube with exact size 100.000 x 100.000 x 100.000 mm (10.000 cm)")
        self.measure_selected_link()

    def refresh_sim_objects_list(self):
        """Refresh the simulation object list from all links marked as simulation objects."""
        sim_tab = getattr(self, "simulation_tab", None)
        if sim_tab is None or not hasattr(sim_tab, "objects_list"):
            return

        current_name = None
        current_item = sim_tab.objects_list.currentItem()
        if current_item is not None:
            current_name = current_item.text()

        sim_tab.objects_list.blockSignals(True)
        try:
            sim_tab.objects_list.clear()
            for name, link in self.robot.links.items():
                if getattr(link, "is_sim_obj", False):
                    sim_tab.objects_list.addItem(name)
            if current_name:
                items = sim_tab.objects_list.findItems(current_name, QtCore.Qt.MatchExactly)
                if items:
                    sim_tab.objects_list.setCurrentItem(items[0])
        finally:
            sim_tab.objects_list.blockSignals(False)

    def _next_sim_object_name(self):
        """Return the next sequential simulation object name (ob_1, ob_2, ...)."""
        idx = 1
        while f"ob_{idx}" in self.robot.links:
            idx += 1
        return f"ob_{idx}"

    def create_object_from_panel(self, spec):
        """Create a cube or cylinder from the object panel values and place it in 3D."""
        import trimesh

        if not isinstance(spec, dict):
            self.show_toast("Invalid object specification", "error")
            return False

        obj_type = str(spec.get("type") or "").strip().lower()
        import_data = spec.get("import") or {}

        def _to_float(value, label):
            try:
                value = float(str(value).strip())
            except Exception:
                raise ValueError(f"{label} must be a number.")
            if not np.isfinite(value):
                raise ValueError(f"{label} must be a finite number.")
            return value

        try:
            x_mm = _to_float(import_data.get("x"), "Import X")
            y_mm = _to_float(import_data.get("y"), "Import Y")
            z_mm = _to_float(import_data.get("z"), "Import Z")
        except ValueError as exc:
            self.log(f"Object import failed: {exc}")
            self.show_toast(str(exc), "warning")
            return False

        cm_scale = float(getattr(self.canvas, "grid_units_per_cm", get_engine_units_per_cm()) or get_engine_units_per_cm())
        units_per_mm = cm_scale / 10.0

        try:
            if obj_type == "cube":
                length_mm = _to_float(spec.get("length"), "Cube length")
                breadth_mm = _to_float(spec.get("breadth"), "Cube breadth")
                height_mm = _to_float(spec.get("height"), "Cube height")
                if min(length_mm, breadth_mm, height_mm) <= 0:
                    raise ValueError("Cube dimensions must be greater than zero.")
                mesh = trimesh.creation.box(
                    extents=[
                        length_mm * units_per_mm,
                        breadth_mm * units_per_mm,
                        height_mm * units_per_mm,
                    ]
                )
                mesh.apply_translation([0.0, 0.0, (height_mm * units_per_mm) / 2.0])
                base_name = f"cube_{int(round(length_mm))}x{int(round(breadth_mm))}x{int(round(height_mm))}"
                dims_mm = (length_mm, breadth_mm, height_mm)
            elif obj_type == "cylinder":
                dia_mm = _to_float(spec.get("diameter"), "Cylinder diameter")
                height_mm = _to_float(spec.get("height"), "Cylinder height")
                if min(dia_mm, height_mm) <= 0:
                    raise ValueError("Cylinder dimensions must be greater than zero.")
                mesh = trimesh.creation.cylinder(
                    radius=(dia_mm / 2.0) * units_per_mm,
                    height=height_mm * units_per_mm,
                    sections=48,
                )
                mesh.apply_translation([0.0, 0.0, (height_mm * units_per_mm) / 2.0])
                base_name = f"cylinder_{int(round(dia_mm))}x{int(round(height_mm))}"
                dims_mm = (dia_mm, dia_mm, height_mm)
            else:
                raise ValueError("Choose cube or cylinder first.")
        except ValueError as exc:
            self.log(f"Object import failed: {exc}")
            self.show_toast(str(exc), "warning")
            return False
        except Exception as exc:
            self.log(f"Object import failed: {exc}")
            self.show_toast("Unable to build the shape", "error")
            return False

        name = self._next_sim_object_name()

        link = self.robot.add_link(name, mesh)
        link.is_sim_obj = True
        link.color = "#7B1FA2" if obj_type == "cube" else "#1976d2"
        link.import_metadata = {
            "source_type": "panel_primitive",
            "object_type": obj_type,
            "source_unit": "mm",
            "engine_unit": getattr(self.canvas, "internal_unit_name", "mm"),
            "raw_size": [float(dims_mm[0]), float(dims_mm[1]), float(dims_mm[2])],
            "final_size": [float(dims_mm[0]), float(dims_mm[1]), float(dims_mm[2])],
            "import_point_mm": [x_mm, y_mm, z_mm],
            "import_world_rotation": np.eye(3).tolist(),
        }
        link.pick_pos = [x_mm * units_per_mm, y_mm * units_per_mm, z_mm * units_per_mm]
        link.place_pos = [x_mm * units_per_mm, y_mm * units_per_mm, z_mm * units_per_mm]

        self.add_link_item(name)

        t_import = np.eye(4)
        t_import[:3, 3] = [x_mm * units_per_mm, y_mm * units_per_mm, z_mm * units_per_mm]
        link.t_offset = t_import
        self.canvas.update_link_mesh(name, mesh, t_import, color=link.color)
        self.canvas.ensure_grid_fits_bounds(self.canvas.actors[name].GetBounds())

        self.canvas.select_actor(name)
        for i in range(self.links_list.count()):
            item = self.links_list.item(i)
            if item and item.text() == name:
                self.links_list.setCurrentItem(item)
                break
        self.canvas.focus_on_actor(name)
        self.update_link_colors()
        self.robot.update_kinematics()
        self.canvas.update_transforms(self.robot)

        self.refresh_sim_objects_list()
        exp_tab = getattr(self, "experiment_tab", None)
        if exp_tab is not None and hasattr(exp_tab, "object_tab"):
            exp_tab.object_tab.refresh_object_list()
        sim_tab = getattr(self, "simulation_tab", None)
        if sim_tab is not None:
            sim_list = sim_tab.objects_list
            for i in range(sim_list.count()):
                item = sim_list.item(i)
                if item and item.text() == name:
                    sim_list.setCurrentItem(item)
                    break
            sim_tab.refresh_object_info(name)
            sim_tab.capture_object_to_p1()
            sim_tab.place_x.setValue(sim_tab.pick_x.value())
            sim_tab.place_y.setValue(sim_tab.pick_y.value() + 20.0)
            sim_tab.place_z.setValue(sim_tab.pick_z.value())

        self.log(
            f"Created {obj_type} '{name}' at ({x_mm:.2f}, {y_mm:.2f}, {z_mm:.2f}) mm "
            f"with size {dims_mm[0]:.2f} x {dims_mm[1]:.2f} x {dims_mm[2]:.2f} mm."
        )
        self.show_toast(f"Imported {name}", "success")
        return True

    def _is_simulation_object_import(self):
        """Return whether the current file import belongs to Object Simulation."""
        if getattr(self, "_simulation_object_import_active", False):
            return True
        toggle = getattr(self, "sim_toggle_btn", None)
        return bool(toggle is not None and toggle.isChecked())

    def import_mesh(self):
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Import Mesh", "", "3D Files (*.stl *.step *.stp *.obj)"
        )
        if file_path:
            self.log(f"Importing: {os.path.basename(file_path)}")
            import trimesh
            try:
                loaded = trimesh.load(file_path)
                imported_payload = self._finalize_loaded_mesh(file_path, loaded)

                scene_parts = imported_payload if isinstance(imported_payload, list) else None
                if scene_parts:
                    self.log(f"Detected STEP assembly scene with {len(scene_parts)} part(s). Preserving component names from the file.")
                    imported_parts = []
                    for part in scene_parts:
                        part_name = str(part.get("name", os.path.basename(file_path).split('.')[0]))
                        part_mesh = part.get("mesh")
                        part_transform = np.array(part.get("transform", np.eye(4)), dtype=float)

                        if part_mesh is None or not hasattr(part_mesh, "vertices") or len(part_mesh.vertices) == 0:
                            continue

                        prepared = self._prepare_imported_mesh(file_path, part_mesh)
                        if prepared is None:
                            self.log("Import cancelled.")
                            return
                        mesh, import_debug = prepared

                        colors = ["#e74c3c", "#3498db", "#2ecc71", "#f1c40f", "#9b59b6", "#1abc9c", "#e67e22", "#95a5a6"]
                        link_color = random.choice(colors)

                        name = part_name
                        base_name = name
                        counter = 1
                        while name in self.robot.links:
                            name = f"{base_name}_{counter}"
                            counter += 1

                        link = self.robot.add_link(name, mesh)
                        link.color = link_color
                        part_transform = self._adjust_import_transform(part_transform, import_debug)

                        link.import_metadata = {
                            "source_unit": import_debug["source_unit"],
                            "detected_unit": import_debug["detected_unit"],
                            "detection_source": import_debug["detection_source"],
                            "scale_to_internal": import_debug["scale_to_internal"],
                            "up_axis": import_debug["up_axis"],
                            "engine_unit": import_debug["engine_unit"],
                            "source_path": file_path,
                            "source_component_name": part_name,
                            "source_transform": part_transform.tolist(),
                            "raw_size": import_debug["raw_size"].tolist(),
                            "final_size": import_debug["final_size"].tolist(),
                            "raw_bounds": import_debug["raw_bounds"].tolist(),
                            "final_bounds": import_debug["final_bounds"].tolist(),
                            "import_world_rotation": part_transform[:3, :3].tolist(),
                        }

                        if self._is_simulation_object_import():
                            link.is_sim_obj = True

                        self.add_link_item(name)
                        link.t_offset = part_transform
                        self.canvas.update_link_mesh(name, mesh, part_transform, color=link.color)
                        imported_parts.append({"name": name, "transform": part_transform})

                        actor = self.canvas.actors[name]
                        self.canvas.ensure_grid_fits_bounds(actor.GetBounds())
                        self.log(f"Successfully loaded component '{part_name}' as '{name}'")
                        self._log_import_debug(name, import_debug)

                        self.canvas.select_actor(name)
                        for i in range(self.links_list.count()):
                            item_i = self.links_list.item(i)
                            if item_i and item_i.text() == name:
                                self.links_list.setCurrentItem(item_i)
                                break
                        self.canvas.focus_on_actor(name)
                        self.update_link_colors()

                        if getattr(link, 'is_sim_obj', False):
                            self.refresh_sim_objects_list()
                            sim_list = self.simulation_tab.objects_list
                            for i in range(sim_list.count()):
                                item_i = sim_list.item(i)
                                if item_i and item_i.text() == name:
                                    sim_list.setCurrentItem(item_i)
                                    break
                            self.simulation_tab.refresh_object_info(name)
                            self.simulation_tab.capture_object_to_p1()
                            p1_y = self.simulation_tab.pick_y.value()
                            self.simulation_tab.place_x.setValue(self.simulation_tab.pick_x.value())
                            self.simulation_tab.place_y.setValue(p1_y + 20.0)
                            self.simulation_tab.place_z.setValue(0.0)

                    self._rigidize_imported_scene(imported_parts)
                    self.show_toast(f"Imported {len(imported_parts)} STEP components", "success")
                    return

                mesh = imported_payload
                prepared = self._prepare_imported_mesh(file_path, mesh)
                if prepared is None:
                    self.log("Import cancelled.")
                    return
                mesh, import_debug = prepared

                # Assign a random distinct color
                colors = ["#e74c3c", "#3498db", "#2ecc71", "#f1c40f", "#9b59b6", "#1abc9c", "#e67e22", "#95a5a6"]
                link_color = random.choice(colors)

                name = os.path.basename(file_path).split('.')[0]
                if self._is_simulation_object_import():
                    name = self._next_sim_object_name()
                else:
                    # Handle unique naming for non-simulation imports
                    base_name = name
                    counter = 1
                    while name in self.robot.links:
                        name = f"{base_name}_{counter}"
                        counter += 1

                link = self.robot.add_link(name, mesh)
                link.color = link_color
                t_import = np.eye(4)
                t_import[:3, 3] = [50.0 * get_engine_units_per_cm(), 50.0 * get_engine_units_per_cm(), 50.0 * get_engine_units_per_cm()]

                link.import_metadata = {
                    "source_unit": import_debug["source_unit"],
                    "detected_unit": import_debug["detected_unit"],
                    "detection_source": import_debug["detection_source"],
                    "scale_to_internal": import_debug["scale_to_internal"],
                    "up_axis": import_debug["up_axis"],
                    "engine_unit": import_debug["engine_unit"],
                    "source_path": file_path,
                    "raw_size": import_debug["raw_size"].tolist(),
                    "final_size": import_debug["final_size"].tolist(),
                    "raw_bounds": import_debug["raw_bounds"].tolist(),
                    "final_bounds": import_debug["final_bounds"].tolist(),
                    "import_world_rotation": t_import[:3, :3].tolist(),
                }

                # Tag as Simulation Object if imported in simulation mode
                if self._is_simulation_object_import():
                    link.is_sim_obj = True

                # Use new helper to add row with 'Eye' button
                self.add_link_item(name)

                # Default spawn position: (50, 50, 50) cm
                ratio = get_engine_units_per_cm()
                link.t_offset = t_import

                self.canvas.update_link_mesh(name, mesh, t_import, color=link.color)

                # SELF-ADJUSTING GRAPH:
                # If component is larger than grid, expand the grid automatically
                actor = self.canvas.actors[name]
                self.canvas.ensure_grid_fits_bounds(actor.GetBounds())

                self.log(f"Successfully loaded: {name}")
                self._log_import_debug(name, import_debug)

                # Auto-select and focus
                self.canvas.select_actor(name)
                for i in range(self.links_list.count()):
                    item_i = self.links_list.item(i)
                    if item_i and item_i.text() == name:
                        self.links_list.setCurrentItem(item_i)
                        break
                self.canvas.focus_on_actor(name)

                self.update_link_colors()

                # Refresh Simulation Objects list if needed
                if getattr(link, 'is_sim_obj', False):
                    self.refresh_sim_objects_list()

                    # --- AUTO-SELECT and AUTO-POPULATE DIM on import ---
                    # Find and select the newly-imported item in the sim objects list
                    sim_list = self.simulation_tab.objects_list
                    for i in range(sim_list.count()):
                        item_i = sim_list.item(i)
                        if item_i and item_i.text() == name:
                            sim_list.setCurrentItem(item_i)
                            break

                    # Populate DIM fields and object info immediately
                    self.simulation_tab.refresh_object_info(name)

                    # --- INDUSTRIAL READINESS: Auto-capture P1 and set P2 ---
                    # 1. Capture current bottom-center as P1
                    self.simulation_tab.capture_object_to_p1()

                    # 2. Set default P2 (e.g., 20cm away in Y axis)
                    p1_y = self.simulation_tab.pick_y.value()
                    self.simulation_tab.place_x.setValue(self.simulation_tab.pick_x.value())
                    self.simulation_tab.place_y.setValue(p1_y + 20.0) # Move 20cm north
                    self.simulation_tab.place_z.setValue(0.0) # Place at floor

                    self.log(f"📐 System Ready: Dimensions and P1/P2 auto-populated for '{name}'.")
                    self.show_toast(f"Robot ready to pick {name}", "success")

            except ImportError as ie:
                self.log(f"MISSING DEPENDENCY: {str(ie)}")
                QtWidgets.QMessageBox.critical(self, "Import Error",
                    f"To load STEP files, you need extra libraries.\n\nError: {str(ie)}\n\n"
                    "I am currently trying to install 'cascadio' for you. "
                    "Please restart the app once the installation finishes.")
            except Exception as e:
                self.log(f"Error: {str(e)}")

    def add_link_item(self, name):
        """Helper to add an item to the list with a focus button and imported file label."""
        item = QtWidgets.QListWidgetItem(self.links_list)
        item.setText(name)

        link = self.robot.links.get(name)
        source_path = None
        if link is not None:
            source_path = getattr(link, "import_metadata", {}).get("source_path")

        # Create custom widget for the row
        widget = QtWidgets.QWidget()
        widget_layout = QtWidgets.QHBoxLayout(widget)
        widget_layout.setContentsMargins(12, 8, 10, 8)

        text_layout = QtWidgets.QVBoxLayout()
        text_layout.setContentsMargins(0, 0, 0, 0)
        text_layout.setSpacing(2)

        # Label with component name and imported file reference. Keep the underlying
        # list item text as the logical link name so selection/focus logic still works.
        file_text = "Generated component"
        if source_path:
            file_text = os.path.basename(source_path)

        name_label_text = name
        if file_text and file_text != "Generated component":
            name_label_text = f"{name} — {file_text}"

        name_label = QtWidgets.QLabel(name_label_text)
        name_label.setStyleSheet("border: none; font-size: 15px; font-weight: bold; color: #212121;")
        name_label.setWordWrap(True)
        text_layout.addWidget(name_label)

        file_label = QtWidgets.QLabel(f"Source file: {file_text}")
        file_label.setStyleSheet("border: none; font-size: 11px; color: #5f6b7a;")
        text_layout.addWidget(file_label)

        widget_layout.addLayout(text_layout)
        widget_layout.addStretch()

        # Focus Button — uses Qt standard icon (always visible on Windows)
        focus_btn = QtWidgets.QPushButton()
        focus_btn.setIcon(widget.style().standardIcon(QtWidgets.QStyle.SP_FileDialogContentsView))
        focus_btn.setIconSize(QtCore.QSize(20, 20))
        focus_btn.setToolTip(f"Focus on {name}")
        focus_btn.setAccessibleName(f"Focus {name}")
        focus_btn.setFixedSize(38, 38)
        focus_btn.setCursor(QtCore.Qt.PointingHandCursor)
        focus_btn.setStyleSheet("""
            QPushButton {
                background-color: white;
                border: 2px solid #e0e0e0;
                border-radius: 19px;
                padding: 4px;
            }
            QPushButton:hover {
                background-color: white;
                border-color: #1976d2;
            }
        """)
        focus_btn.clicked.connect(lambda: self.canvas.focus_on_actor(name))
        widget_layout.addWidget(focus_btn)

        item.setToolTip(f"{name}\n{source_path or 'Generated component'}")

        # Set taller row height
        item.setSizeHint(QtCore.QSize(0, 62))

        # Apply to list
        self.links_list.addItem(item)
        self.links_list.setItemWidget(item, widget)
