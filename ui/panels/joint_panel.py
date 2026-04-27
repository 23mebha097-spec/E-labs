from PyQt5 import QtWidgets, QtCore, QtGui
import numpy as np

class TypeOnlyDoubleSpinBox(QtWidgets.QDoubleSpinBox):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
    def stepBy(self, steps): pass
    def wheelEvent(self, event): event.ignore()

class JointPanel(QtWidgets.QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.mw = main_window
        self.selected_object = None
        self.parent_object = None
        self.child_object = None
        self.axis_point1 = None
        self.axis_point2 = None
        
        # Undo/Redo history
        self.history = []  # List of (parent, child) tuples
        self.history_index = -1
        
        # Active joints storage
        self.joints = {}  # {child_object_name: {parent, axis, min, max, current_angle, alignment_point}}
        self.active_joint_control = None  # Currently selected joint for control
        
        self.init_ui()

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        
        # Object List
        self.objects_list = QtWidgets.QListWidget()
        self.objects_list.setStyleSheet("""
            QListWidget {
                background-color: white;
                color: #212121;
                border: none;
                font-size: 14px;
                padding: 5px;
            }
            QListWidget::item {
                padding: 8px;
                border-bottom: 1px solid #e0e0e0;
            }
            QListWidget::item:hover {
                background-color: #f5f5f5;
            }
            QListWidget::item:selected {
                background-color: #1976d2;
                color: white;
            }
            QListWidget::item:selected:hover {
                background-color: #1565c0;
            }
        """)
        self.objects_list.itemClicked.connect(self.on_object_clicked)
        layout.addWidget(self.objects_list)
        # Section 2 is being removed as requested
        self.axis_section = QtWidgets.QWidget()
        self.axis_section.setVisible(False)
        
        # --- ROTATION AXIS & LIMITS SECTION (appears after CREATE JOINT) ---
        self.rotation_section = QtWidgets.QWidget()
        self.rotation_section.setStyleSheet("background-color: white; padding: 10px; border: 1px solid #e0e0e0;")
        self.rotation_section.setVisible(False)
        
        rot_layout = QtWidgets.QVBoxLayout(self.rotation_section)
        rot_layout.setSpacing(10)
        
        # Section header
        rot_header = QtWidgets.QLabel("3. ROTATION AXIS & LIMITS")
        rot_header.setStyleSheet("color: #1976d2; font-size: 14px; font-weight: bold; padding: 5px;")
        rot_layout.addWidget(rot_header)
        
        # Joint name input
        name_layout = QtWidgets.QHBoxLayout()
        name_label = QtWidgets.QLabel("Joint Name:")
        name_label.setStyleSheet("color: #616161; font-size: 12px;")
        name_layout.addWidget(name_label)
        
        self.joint_name_input = QtWidgets.QLineEdit()
        self.joint_name_input.setPlaceholderText("e.g. Shoulder_Pivot")
        self.joint_name_input.setStyleSheet("""
            QLineEdit {
                background-color: white;
                color: #1976d2;
                border: 1px solid #bbb;
                padding: 5px;
                border-radius: 3px;
                font-weight: bold;
            }
        """)
        name_layout.addWidget(self.joint_name_input)
        rot_layout.addLayout(name_layout)
        
        # --- NEW: Industrial Gripper Option ---
        gripper_row = QtWidgets.QHBoxLayout()
        self.gripper_checkbox = QtWidgets.QCheckBox("Gripper Joint")
        self.gripper_checkbox.setToolTip("Mark this joint as an industrial gripper for Pick and Place logic")
        self.gripper_checkbox.setStyleSheet("color: #2e7d32; font-weight: bold; font-size: 13px;")
        gripper_row.addWidget(self.gripper_checkbox)
        
        self.set_lp_btn = QtWidgets.QPushButton("🎯 Set Live Point")
        self.set_lp_btn.setFixedWidth(140)
        self.set_lp_btn.setVisible(False)
        self.set_lp_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.set_lp_btn.setStyleSheet("""
            QPushButton {
                background-color: white;
                color: #d32f2f;
                border: 2px solid #d32f2f;
                border-radius: 4px;
                padding: 4px;
                font-weight: bold;
                font-size: 11px;
            }
            QPushButton:hover { background-color: #ffebee; }
        """)
        self.set_lp_btn.clicked.connect(self.on_set_live_point)
        gripper_row.addWidget(self.set_lp_btn)
        gripper_row.addStretch()
        rot_layout.addLayout(gripper_row)
        
        # Toggle button visibility
        self.gripper_checkbox.toggled.connect(self.set_lp_btn.setVisible)
        self.gripper_checkbox.toggled.connect(self.on_gripper_toggle_sync)
        
        # Axis selection
        axis_label = QtWidgets.QLabel("Select rotation axis:")
        axis_label.setStyleSheet("color: #616161; font-size: 12px; padding: 5px;")
        rot_layout.addWidget(axis_label)
        
        axis_buttons_row = QtWidgets.QHBoxLayout()
        self.axis_group = QtWidgets.QButtonGroup()
        
        self.axis_x_radio = QtWidgets.QRadioButton("X Axis")
        self.axis_x_radio.setStyleSheet("color: #d32f2f; font-size: 12px;")
        self.axis_group.addButton(self.axis_x_radio, 0)
        axis_buttons_row.addWidget(self.axis_x_radio)
        
        self.axis_y_radio = QtWidgets.QRadioButton("Y Axis")
        self.axis_y_radio.setStyleSheet("color: #1976d2; font-size: 12px;")
        self.axis_group.addButton(self.axis_y_radio, 1)
        axis_buttons_row.addWidget(self.axis_y_radio)
        
        self.axis_z_radio = QtWidgets.QRadioButton("Z Axis")
        self.axis_z_radio.setStyleSheet("color: #1565c0; font-size: 12px;")
        self.axis_z_radio.setChecked(True)  # Default to Z
        self.axis_group.addButton(self.axis_z_radio, 2)
        axis_buttons_row.addWidget(self.axis_z_radio)
        
        # Connect axis change to live visuals
        self.axis_x_radio.toggled.connect(lambda: self.show_joint_arrow() if self.axis_x_radio.isChecked() else None)
        self.axis_y_radio.toggled.connect(lambda: self.show_joint_arrow() if self.axis_y_radio.isChecked() else None)
        self.axis_z_radio.toggled.connect(lambda: self.show_joint_arrow() if self.axis_z_radio.isChecked() else None)
        
        rot_layout.addLayout(axis_buttons_row)
        
        # Rotation limits
        limits_label = QtWidgets.QLabel("Rotation limits (degrees):")
        limits_label.setStyleSheet("color: #616161; font-size: 12px; padding: 5px;")
        rot_layout.addWidget(limits_label)
        
        limits_row = QtWidgets.QHBoxLayout()
        
        min_label = QtWidgets.QLabel("Min:")
        min_label.setStyleSheet("color: #616161; font-size: 11px;")
        limits_row.addWidget(min_label)
        
        self.min_limit_spin = TypeOnlyDoubleSpinBox()
        self.min_limit_spin.setRange(-360, 360)
        self.min_limit_spin.setValue(-180)
        self.min_limit_spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.min_limit_spin.setStyleSheet("background-color: white; color: #212121; border: 1px solid #bbb; padding: 5px;")
        self.min_limit_spin.valueChanged.connect(self.update_slider_range)
        limits_row.addWidget(self.min_limit_spin)
        
        max_label = QtWidgets.QLabel("Max:")
        max_label.setStyleSheet("color: #616161; font-size: 11px;")
        limits_row.addWidget(max_label)
        
        self.max_limit_spin = TypeOnlyDoubleSpinBox()
        self.max_limit_spin.setRange(-360, 360)
        self.max_limit_spin.setValue(180)
        self.max_limit_spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.max_limit_spin.setStyleSheet("background-color: white; color: #212121; border: 1px solid #bbb; padding: 5px;")
        self.max_limit_spin.valueChanged.connect(self.update_slider_range)
        limits_row.addWidget(self.max_limit_spin)
        
        rot_layout.addLayout(limits_row)
        
        # Test Rotation Slider
        test_label = QtWidgets.QLabel("Test rotation:")
        test_label.setStyleSheet("color: #616161; font-size: 12px; padding: 5px;")
        rot_layout.addWidget(test_label)
        
        slider_row = QtWidgets.QHBoxLayout()
        
        self.rotation_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.rotation_slider.setRange(-1800, 1800)  # -180 to 180 degrees (x10 for precision)
        self.rotation_slider.setValue(0)
        self.rotation_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 8px;
                background: #f0f0f0;
                border-radius: 4px;
                border: 1px solid #ddd;
            }
            QSlider::sub-page:horizontal {
                background: #bbdefb;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: white;
                border: 2px solid #1976d2;
                width: 16px;
                height: 16px;
                margin-top: -5px;
                margin-bottom: -5px;
                border-radius: 8px;
            }
            QSlider::handle:horizontal:hover {
                background: #e3f2fd;
            }
        """)
        self.rotation_slider.valueChanged.connect(self.on_slider_changed)
        slider_row.addWidget(self.rotation_slider)
        
        # Direct angle input spinbox
        self.rotation_spinbox = TypeOnlyDoubleSpinBox()
        self.rotation_spinbox.setRange(-180, 180)
        self.rotation_spinbox.setValue(0)
        self.rotation_spinbox.setSuffix("°")
        self.rotation_spinbox.setFixedWidth(70)
        self.rotation_spinbox.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.rotation_spinbox.setStyleSheet("""
            QDoubleSpinBox {
                background: white;
                color: #1976d2;
                border: 1px solid #1976d2;
                border-radius: 3px;
                padding: 2px;
                font-weight: bold;
            }
        """)
        self.rotation_spinbox.valueChanged.connect(self.on_spinbox_changed)
        slider_row.addWidget(self.rotation_spinbox)
        
        rot_layout.addLayout(slider_row)
        
        # Confirm button
        self.confirm_joint_btn = QtWidgets.QPushButton("Confirm Joint")
        self.confirm_joint_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.confirm_joint_btn.setStyleSheet("""
            QPushButton {
                background-color: #1976d2;
                color: white;
                border: none;
                border-radius: 6px;
                padding: 10px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1565c0;
            }
        """)
        self.confirm_joint_btn.clicked.connect(self.confirm_joint)
        rot_layout.addWidget(self.confirm_joint_btn)
        
        layout.addWidget(self.rotation_section)
        
        # Parent/Child Selection Buttons
        buttons_container = QtWidgets.QWidget()
        buttons_container.setStyleSheet("background-color: transparent; padding: 10px;")
        buttons_layout = QtWidgets.QHBoxLayout(buttons_container)
        buttons_layout.setSpacing(10)
        
        btn_style = """
            QPushButton {
                background-color: white;
                color: #424242;
                border: 1px solid #e0e0e0;
                border-radius: 6px;
                padding: 10px;
                font-weight: bold;
            }
            QPushButton:hover {
                border: 2px solid #1976d2;
                color: #1976d2;
                background-color: #e3f2fd;
            }
            QPushButton:disabled {
                background-color: #fafafa;
                color: #bdbdbd;
                border: 1px solid #e0e0e0;
            }
        """
        
        # Parent Button
        self.parent_btn = QtWidgets.QPushButton("Parent Object")
        self.parent_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.parent_btn.setStyleSheet(btn_style)
        self.parent_btn.clicked.connect(self.set_as_parent)
        self.parent_btn.setEnabled(False)
        buttons_layout.addWidget(self.parent_btn)
        
        # Child Button
        self.child_btn = QtWidgets.QPushButton("Child Object")
        self.child_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.child_btn.setStyleSheet(btn_style)
        self.child_btn.clicked.connect(self.set_as_child)
        self.child_btn.setEnabled(False)
        buttons_layout.addWidget(self.child_btn)
        
        layout.addWidget(buttons_container)
        
        # --- UNDO/REDO BUTTONS ---
        undo_redo_container = QtWidgets.QWidget()
        undo_redo_container.setStyleSheet("background-color: transparent; padding: 5px;")
        undo_redo_layout = QtWidgets.QHBoxLayout(undo_redo_container)
        undo_redo_layout.setSpacing(10)
        
        undo_redo_style = """
            QPushButton {
                background-color: white;
                color: #424242;
                border: 1px solid #e0e0e0;
                border-radius: 6px;
                padding: 8px;
                font-weight: bold;
            }
            QPushButton:hover {
                border: 2px solid #1976d2;
                color: #1976d2;
                background-color: #e3f2fd;
            }
        """
        
        self.undo_btn = QtWidgets.QPushButton("Undo")
        self.undo_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.undo_btn.setStyleSheet(undo_redo_style)
        self.undo_btn.clicked.connect(self.undo_selection)
        undo_redo_layout.addWidget(self.undo_btn)
        
        self.redo_btn = QtWidgets.QPushButton("Redo")
        self.redo_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.redo_btn.setStyleSheet(undo_redo_style)
        self.redo_btn.clicked.connect(self.redo_selection)
        undo_redo_layout.addWidget(self.redo_btn)
        
        layout.addWidget(undo_redo_container)
        
        # --- JOINT CONTROL SECTION (appears when clicking jointed object) ---
        self.joint_control_section = QtWidgets.QWidget()
        self.joint_control_section.setStyleSheet("background-color: transparent; padding: 10px;")
        self.joint_control_section.setVisible(False)
        
        jc_layout = QtWidgets.QVBoxLayout(self.joint_control_section)
        jc_layout.setSpacing(10)
        
        # Header
        jc_header = QtWidgets.QLabel("Joint Control")
        jc_header.setStyleSheet("color: #1976d2; font-size: 15px; font-weight: bold; padding: 2px;")
        jc_layout.addWidget(jc_header)
        
        # Joint info
        self.joint_info_label = QtWidgets.QLabel("No joint selected")
        self.joint_info_label.setStyleSheet("color: #757575; font-size: 13px; padding: 2px;")
        jc_layout.addWidget(self.joint_info_label)
        
        # Control slider
        jc_slider_label = QtWidgets.QLabel("Rotation:")
        jc_slider_label.setStyleSheet("color: #424242; font-size: 13px; padding: 2px;")
        jc_layout.addWidget(jc_slider_label)
        
        jc_slider_row = QtWidgets.QHBoxLayout()
        
        self.joint_control_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.joint_control_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 8px;
                background: #f0f0f0;
                border-radius: 4px;
                border: 1px solid #ddd;
            }
            QSlider::sub-page:horizontal {
                background: #bbdefb;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: white;
                border: 2px solid #1976d2;
                width: 16px;
                height: 16px;
                margin-top: -5px;
                margin-bottom: -5px;
                border-radius: 8px;
            }
            QSlider::handle:horizontal:hover {
                background: #e3f2fd;
            }
        """)
        self.joint_control_slider.valueChanged.connect(self.on_joint_control_changed)
        jc_slider_row.addWidget(self.joint_control_slider)
        
        self.joint_control_spinbox = TypeOnlyDoubleSpinBox()
        self.joint_control_spinbox.setSuffix("°")
        self.joint_control_spinbox.setFixedWidth(70)
        self.joint_control_spinbox.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.joint_control_spinbox.setStyleSheet("""
            QDoubleSpinBox {
                background: white;
                color: #1976d2;
                border: 1px solid #1976d2;
                border-radius: 3px;
                padding: 2px;
                font-weight: bold;
            }
        """)
        self.joint_control_spinbox.valueChanged.connect(self.on_joint_control_spinbox_changed)
        jc_slider_row.addWidget(self.joint_control_spinbox)
        
        jc_layout.addLayout(jc_slider_row)
        
        layout.addWidget(self.joint_control_section)
        
        # --- 4. CREATED JOINTS SECTION ---
        header_joints = QtWidgets.QLabel("4. CREATED JOINTS")
        header_joints.setStyleSheet("color: #1976d2; font-size: 14px; font-weight: bold; margin-top: 20px; padding: 5px;")
        layout.addWidget(header_joints)
        
        self.joints_history_list = QtWidgets.QListWidget()
        self.joints_history_list.setStyleSheet("""
            QListWidget {
                background-color: white;
                border: 1px solid #e0e0e0;
                border-radius: 5px;
                min-height: 200px;
            }
            QListWidget::item {
                border-bottom: 1px solid #e0e0e0;
                background-color: transparent;
            }
            QListWidget::item:selected {
                background-color: #e3f2fd;
            }
        """)
        layout.addWidget(self.joints_history_list)
        
        # Bottom stretch
        layout.addStretch()

    def refresh_joints_history(self):
        """Refresh the list of created joints with delete buttons"""
        self.joints_history_list.clear()
        
        for child_name, data in self.joints.items():
            item = QtWidgets.QListWidgetItem()
            self.joints_history_list.addItem(item)
            
            # Create custom widget for the item
            widget = QtWidgets.QWidget()
            item_layout = QtWidgets.QHBoxLayout(widget)
            item_layout.setContentsMargins(12, 10, 12, 10)
            item_layout.setSpacing(10)
            
            # Label: Custom Name Only
            display_name = data.get('custom_name', f"{data['parent']} \u2192 {child_name}")
            
            # Check for relations (Master or Slave)
            joint_id = data.get('joint_id', child_name)
            is_master = joint_id in self.mw.robot.joint_relations and self.mw.robot.joint_relations[joint_id]
            
            # Check if this joint is a slave of any other joint
            is_slave = False
            for master, slaves in self.mw.robot.joint_relations.items():
                if any(s_id == joint_id for s_id, r in slaves):
                    is_slave = True
                    break
            
            # Show "R" badge if involved in any relation
            if is_master or is_slave:
                r_badge = QtWidgets.QLabel("R")
                r_badge.setToolTip("Joint Relation Active")
                r_badge.setAlignment(QtCore.Qt.AlignCenter)
                r_badge.setFixedSize(26, 26)
                r_badge.setStyleSheet("""
                    background-color: #673ab7;
                    color: white;
                    border-radius: 11px;
                    font-weight: bold;
                    font-size: 11px;
                """)
                item_layout.addWidget(r_badge)

            label = QtWidgets.QLabel(display_name)
            label.setStyleSheet("color: #212121; font-size: 15px; font-weight: bold;")
            item_layout.addWidget(label)

            # --- Master Only Icons ---
            if is_master:
                # Relationship Count Info
                count_lbl = QtWidgets.QLabel(f"\ud83d\udd17({len(self.mw.robot.joint_relations[joint_id])})")
                count_lbl.setStyleSheet("color: #4caf50; font-size: 10px; font-weight: bold; padding: 2px;")
                item_layout.addWidget(count_lbl)

            # Rename Button
            rename_btn = QtWidgets.QPushButton("Aa")
            rename_btn.setFixedSize(40, 40)
            rename_btn.setCursor(QtCore.Qt.PointingHandCursor)
            rename_btn.setToolTip("Rename joint")
            rename_btn.setAccessibleName("Rename")
            rename_btn.setStyleSheet("""
                QPushButton {
                    background-color: white;
                    color: #757575;
                    border: 2px solid #e0e0e0;
                    border-radius: 20px;
                    font-size: 14px;
                    font-weight: bold;
                }
                QPushButton:hover {
                    background-color: white;
                    color: #1976d2;
                    border-color: #1976d2;
                }
            """)
            rename_btn.clicked.connect(lambda checked, n=child_name: self.rename_joint(n))
            item_layout.addWidget(rename_btn)
            
            # Relation/Edit Button
            rel_text = "R" if is_master else "+R"
            relation_btn = QtWidgets.QPushButton(rel_text) 
            relation_btn.setFixedSize(40, 40)
            relation_btn.setCursor(QtCore.Qt.PointingHandCursor)
            relation_btn.setToolTip("Edit Relation" if is_master else "Add Relation")
            relation_btn.setAccessibleName("Joint Relation")
            
            # Style differently if master
            rel_color = "#9c27b0" if is_master else "#4caf50"
            
            relation_btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: white;
                    color: #757575;
                    border: 2px solid #e0e0e0;
                    border-radius: 20px;
                    font-size: 13px;
                    font-weight: bold;
                }}
                QPushButton:hover {{
                    background-color: white;
                    color: {rel_color};
                    border-color: {rel_color};
                }}
            """)
            relation_btn.clicked.connect(lambda checked, n=child_name: self.add_joint_relation_ui(n))
            item_layout.addWidget(relation_btn)
            
            item_layout.addStretch()
            
            # Axis/Limits info small
            axis_names = {0: "X", 1: "Y", 2: "Z"}
            info = QtWidgets.QLabel(f"Axis: {axis_names[data['axis']]}")
            info.setStyleSheet("color: #757575; font-size: 13px; font-weight: bold; margin-right: 5px;")
            item_layout.addWidget(info)
            
            # Delete Button — red X with circular red border
            del_btn = QtWidgets.QPushButton("X")
            del_btn.setFixedSize(40, 40)
            del_btn.setCursor(QtCore.Qt.PointingHandCursor)
            del_btn.setAccessibleName("Remove")
            del_btn.setToolTip("Remove joint")
            del_btn.setStyleSheet("""
                QPushButton {
                    background-color: white;
                    color: #d32f2f;
                    border: 2px solid #d32f2f;
                    border-radius: 20px;
                    font-weight: bold;
                    font-size: 14px;
                }
                QPushButton:hover {
                    background-color: #d32f2f;
                    color: white;
                }
            """)
            del_btn.clicked.connect(lambda checked, name=child_name: self.delete_joint(name))
            item_layout.addWidget(del_btn)
            
            item.setSizeHint(QtCore.QSize(0, 60))
            self.joints_history_list.setItemWidget(item, widget)

    def delete_joint(self, child_name):
        """Delete a joint and reset the child's transform"""
        if child_name not in self.joints:
            return
            
        joint_data = self.joints[child_name]
        parent_name = joint_data['parent']
        joint_name = joint_data.get('joint_id', f"joint_{parent_name}_{child_name}")
        
        self.mw.log(f"Deleting joint: {joint_name}")
        
        # 1. Remove from Robot Model Core
        self.mw.robot.remove_joint(joint_name)
        
        # 2. Reset world transform to the offset (0 rotation position)
        child_link = self.mw.robot.links[child_name]
        child_link.t_world = child_link.t_offset.copy()
        
        # 3. Remove from UI data structures
        del self.joints[child_name]
        
        # 3. If it was active in control, hide it
        if self.active_joint_control == child_name:
            self.joint_control_section.setVisible(False)
            self.active_joint_control = None
            
        # 4. Refresh UI
        self.refresh_links()
        self.refresh_joints_history()
        
        # Refresh Matrices Panel Sliders
        if hasattr(self.mw, 'matrices_tab'):
            self.mw.matrices_tab.refresh_sliders()
        if hasattr(self.mw, 'experiment_tab'):
            self.mw.experiment_tab.refresh_sliders()
            self.mw.experiment_tab.update_display()
            
        # 5. Update canvas
        self.mw.robot.update_kinematics()
        self.mw.canvas.update_transforms(self.mw.robot)
        self.mw.log(f"Joint deleted successfully.")
        self.mw.show_toast(f"Joint removed", "error")

    def rename_joint(self, child_name):
        """Open a dialog to rename the joint and update internal IDs."""
        if child_name not in self.joints:
            return
            
        data = self.joints[child_name]
        old_custom_name = data.get('custom_name', child_name)
        old_id = data.get('joint_id', old_custom_name)
        
        new_name, ok = QtWidgets.QInputDialog.getText(
            self, "Rename Joint", 
            f"Enter new name for '{old_custom_name}':", 
            text=old_custom_name
        )
        
        if ok and new_name.strip():
            new_custom_name = new_name.strip()
            # Generate sanitized ID for code compatibility
            new_id = new_custom_name.replace(" ", "_").replace("/", "_")
            
            # 1. Update Robot Core dictionary if ID changed
            if new_id != old_id and old_id in self.mw.robot.joints:
                joint_obj = self.mw.robot.joints.pop(old_id)
                joint_obj.name = new_id
                self.mw.robot.joints[new_id] = joint_obj
                self.mw.log(f"Robot core joint ID updated: {old_id} -> {new_id}")
                
            # 2. Update local UI storage
            data['custom_name'] = new_custom_name
            data['joint_id'] = new_id
            
            # 3. Update active control if needed
            if self.active_joint_control == child_name:
                axis_names = {0: "X", 1: "Y", 2: "Z"}
                axis_name = axis_names.get(data['axis'], "?")
                self.joint_info_label.setText(f"Joint: {new_custom_name} | Axis: {axis_name}")

            self.refresh_joints_history()
            self.mw.log(f"Joint renamed to: {new_custom_name}")
            self.mw.show_toast(f"Renamed to {new_custom_name}", "success")

    def add_joint_relation_ui(self, master_child_name):
        """UI to add a relation between joints"""
        if master_child_name not in self.joints:
            return
            
        master_data = self.joints[master_child_name]
        master_id = master_data.get('joint_id', master_child_name)
        
        # Get all other joints
        other_joints = []
        for c_name, data in self.joints.items():
            if c_name != master_child_name:
                display_name = data.get('custom_name', c_name)
                other_joints.append((display_name, data.get('joint_id', c_name), c_name))
        
        if not other_joints:
            QtWidgets.QMessageBox.warning(self, "No Other Joints", "There are no other joints to relate to.")
            return
            
        # Create dialog
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle(f"Add Relation to {master_data.get('custom_name', master_child_name)}")
        dialog.setMinimumWidth(300)
        
        d_layout = QtWidgets.QVBoxLayout(dialog)
        
        label = QtWidgets.QLabel("Select slave joints and ratio (e.g. 1.0 same, -1.0 opposite):")
        label.setWordWrap(True)
        d_layout.addWidget(label)
        
        # List of other joints with checkboxes and ratios
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QtWidgets.QWidget()
        scroll_layout = QtWidgets.QVBoxLayout(scroll_widget)
        
        slave_rows = []
        for display_name, j_id, c_name in other_joints:
            row = QtWidgets.QHBoxLayout()
            cb = QtWidgets.QCheckBox(display_name)
            ratio_spin = TypeOnlyDoubleSpinBox()
            ratio_spin.setRange(-10, 10)
            ratio_spin.setValue(1.0)
            ratio_spin.setSingleStep(0.1)
            ratio_spin.setFixedWidth(60)
            ratio_spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
            
            # Check if relation already exists
            existing_ratio = None
            if master_id in self.mw.robot.joint_relations:
                for s_id, r in self.mw.robot.joint_relations[master_id]:
                    if s_id == j_id:
                        existing_ratio = r
                        break
            
            if existing_ratio is not None:
                cb.setChecked(True)
                ratio_spin.setValue(existing_ratio)
            
            row.addWidget(cb)
            row.addWidget(QtWidgets.QLabel("Ratio:"))
            row.addWidget(ratio_spin)
            scroll_layout.addLayout(row)
            slave_rows.append((cb, ratio_spin, j_id, c_name))
            
        scroll_layout.addStretch()
        scroll.setWidget(scroll_widget)
        d_layout.addWidget(scroll)
        
        # Buttons
        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        btns.accepted.connect(dialog.accept)
        btns.rejected.connect(dialog.reject)
        d_layout.addWidget(btns)
        
        if dialog.exec_() == QtWidgets.QDialog.Accepted:
            # Clear existing relations for this master in the model (we'll rebuild)
            if master_id in self.mw.robot.joint_relations:
                self.mw.robot.joint_relations[master_id] = []
                
            for cb, ratio_spin, j_id, c_name in slave_rows:
                if cb.isChecked():
                    ratio = ratio_spin.value()
                    self.mw.robot.add_joint_relation(master_id, j_id, ratio)
                    self.mw.log(f"Relation added: {master_id} -> {j_id} (ratio: {ratio})")
            
            self.mw.log(f"Joint relations updated for {master_id}.")
            
            # Refresh UI to show "R" badges and hide slave sliders/matrices
            self.refresh_joints_history()
            if hasattr(self.mw, 'matrices_tab'):
                self.mw.matrices_tab.refresh_sliders()
                self.mw.matrices_tab.update_display()

    def on_gripper_toggle_sync(self, checked):
        """Sync the gripper status to the Gripper tab."""
        if not self.selected_object: return
        
        # Find the joint associated with this object
        joint_name = None
        for jn, j in self.mw.robot.joints.items():
            if j.child_link.name == self.selected_object:
                joint_name = jn
                break
        
        if joint_name:
            self.mw.robot.joints[joint_name].is_gripper = checked
            # Refresh Gripper Tab list if visible
            if hasattr(self.mw, 'gripper_tab'):
                self.mw.gripper_tab.refresh_joints()
                # If the gripper tab is currently selecting this joint, sync its checkbox
                items = self.mw.gripper_tab.joints_list.findItems(joint_name, QtCore.Qt.MatchExactly)
                if items:
                    self.mw.gripper_tab.joints_list.setCurrentItem(items[0])
                    self.mw.gripper_tab.mark_gripper_check.blockSignals(True)
                    self.mw.gripper_tab.mark_gripper_check.setChecked(checked)
                    self.mw.gripper_tab.mark_gripper_check.blockSignals(False)

    def select_object(self, name):
        """Selection logic for external calls"""
        self.selected_object = name
        self.parent_btn.setEnabled(True)
        self.child_btn.setEnabled(True)
        self.mw.canvas.select_actor(name)

    def set_as_parent(self):
        """Set selected object as parent"""
        if not self.selected_object:
            return
            
        self.parent_object = self.selected_object
        self.mw.log(f"Parent set to: {self.parent_object}")
        self.save_to_history()
        self.mw.canvas.deselect_all()
        self.refresh_links()
        
        # Section 2 is gone, so we don't call check_show_axis_section
        self.parent_btn.setEnabled(False)
        self.child_btn.setEnabled(False)
        self.selected_object = None
        
        # New: Check for cached alignment
        if self.parent_object and self.child_object:
            self.check_for_cached_alignment()

    def set_as_child(self):
        """Set selected object as child"""
        if not self.selected_object:
            return
        
        if self.selected_object in self.joints:
            self.mw.log(f"Error: {self.selected_object} is already a jointed child.")
            return
            
        self.child_object = self.selected_object
        self.mw.log(f"Child set to: {self.child_object}")
        self.save_to_history()
        self.mw.canvas.deselect_all()
        self.refresh_links()
        
        # Section 2 is gone, so we don't call check_show_axis_section
        self.parent_btn.setEnabled(False)
        self.child_btn.setEnabled(False)
        self.selected_object = None
        
        # New: Check for cached alignment
        if self.parent_object and self.child_object:
            self.check_for_cached_alignment()

    def check_for_cached_alignment(self):
        """Check if an alignment exists for the current parent/child pair"""
        pair = (self.parent_object, self.child_object)
        if pair in self.mw.alignment_cache:
            self.alignment_point = self.mw.alignment_cache[pair]
            self.mw.log(f"Matched alignment point found for {pair}: {self.alignment_point}")
            self.create_joint()
        else:
            self.mw.log(f"No cached alignment found for {pair}. Objects must be aligned in 'Align' tab first.")
    def undo_selection(self):
        """Undo the last parent/child selection"""
        if self.history_index > 0:
            self.history_index -= 1
            parent, child = self.history[self.history_index]
            self.parent_object = parent
            self.child_object = child
            self.refresh_links()
            self.mw.log(f"Undo: Parent={parent}, Child={child}")
        else:
            self.mw.log("Nothing to undo.")

    def redo_selection(self):
        """Redo a previously undone selection"""
        if self.history_index < len(self.history) - 1:
            self.history_index += 1
            parent, child = self.history[self.history_index]
            self.parent_object = parent
            self.child_object = child
            self.refresh_links()
            self.mw.log(f"Redo: Parent={parent}, Child={child}")
        else:
            self.mw.log("Nothing to redo.")

    def save_to_history(self):
        """Save current parent/child state to history"""
        # Remove any "future" history if we're in the middle
        self.history = self.history[:self.history_index + 1]
        
        # Add current state
        self.history.append((self.parent_object, self.child_object))
        self.history_index = len(self.history) - 1

    def on_object_clicked(self, item):
        """When an object is clicked in the list"""
        object_name = item.text().replace("✓ ", "").replace("✓⭕ ", "")  # Remove indicators
        
        # Check if this object has a joint
        if object_name in self.joints:
            # Check if this joint is a slave of any other joint
            joint_id = self.joints[object_name].get('joint_id', object_name)
            is_slave = False
            for master, slaves in self.mw.robot.joint_relations.items():
                if any(s_id == joint_id for s_id, r in slaves):
                    is_slave = True
                    break
            
            if is_slave:
                # Hide joint control for slaves - their movement is driven by the master
                self.joint_control_section.setVisible(False)
                self.active_joint_control = None
                self.mw.log(f"Joint '{object_name}' is a slave relation - control it via master joint.")
            else:
                # Show joint control for this jointed object
                self.show_joint_control(object_name)
            
            # ALLOW jointed objects to be selected as parents!
            self.selected_object = object_name
            self.parent_btn.setEnabled(True)
            self.child_btn.setEnabled(False) # Still keep child disabled (one parent only)
        else:
            # Normal selection for parent/child assignment
            self.selected_object = object_name
            
            # Hide joint control
            self.joint_control_section.setVisible(False)
            self.active_joint_control = None
            
            # Highlight in 3D view (yellow)
            self.mw.canvas.select_actor(self.selected_object)
            
            # Enable buttons
            self.parent_btn.setEnabled(True)
            self.child_btn.setEnabled(True)
            
            self.mw.log(f"Selected: {self.selected_object}")

    def show_joint_control(self, object_name):
        """Show joint control section for a jointed object and sync UI widgets."""
        self.active_joint_control = object_name
        joint_data = self.joints[object_name]
        
        # 1. Sync Axis Selection from metadata
        self.axis_group.blockSignals(True)
        # Use saved index if available, otherwise default to Z (2)
        axis_id = joint_data.get('axis', 2)
        if axis_id == 0: self.axis_x_radio.setChecked(True)
        elif axis_id == 1: self.axis_y_radio.setChecked(True)
        else: self.axis_z_radio.setChecked(True)
        self.axis_group.blockSignals(False)

        # 2. Sync Limits & Name
        self.min_limit_spin.blockSignals(True)
        self.min_limit_spin.setValue(joint_data.get('min', -180))
        self.min_limit_spin.blockSignals(False)
        
        self.max_limit_spin.blockSignals(True)
        self.max_limit_spin.setValue(joint_data.get('max', 180))
        self.max_limit_spin.blockSignals(False)
        
        self.joint_name_input.setText(joint_data.get('custom_name', object_name))

        # 3. Update info label
        axis_names = {0: "X", 1: "Y", 2: "Z"}
        axis_name = axis_names.get(axis_id, "?")
        custom_name = joint_data.get('custom_name', object_name)
        self.joint_info_label.setText(
            f"Joint: {custom_name} | Axis: {axis_name}"
        )
        
        # 4. Setup Control Slider (Section 4)
        min_val = int(joint_data['min'] * 10)
        max_val = int(joint_data['max'] * 10)
        current_val = int(joint_data['current_angle'] * 10)
        
        self.joint_control_slider.blockSignals(True)
        self.joint_control_slider.setRange(min_val, max_val)
        self.joint_control_slider.setValue(current_val)
        self.joint_control_slider.blockSignals(False)
        
        self.joint_control_spinbox.blockSignals(True)
        self.joint_control_spinbox.setRange(joint_data['min'], joint_data['max'])
        self.joint_control_spinbox.setValue(joint_data['current_angle'])
        self.joint_control_spinbox.blockSignals(False)
        
        # Support Gripper state
        self.gripper_checkbox.blockSignals(True)
        self.gripper_checkbox.setChecked(joint_data.get('is_gripper', False))
        self.gripper_checkbox.blockSignals(False)
        self.set_lp_btn.setVisible(self.gripper_checkbox.isChecked())
        
        # 5. Visual Indicators
        self.alignment_point = joint_data.get('alignment_point')
        self.rotation_section.setVisible(True) # Show section 3 so user can see/edit axis
        self.joint_control_section.setVisible(True)
        
        # Determine parent for coordination
        self.parent_object = joint_data.get('parent')
        self.child_object = object_name
        
        # Initialize original transform for rotation math (offset * parent)
        # This prevents AttributeError and handles editing existing joints
        child_link = self.mw.robot.links[object_name]
        parent_link = self.mw.robot.links[self.parent_object]
        self.original_child_transform = parent_link.t_world @ child_link.t_offset
        
        self.show_joint_arrow()
        self.mw.log(f"Joint control active for: {object_name}")

    def create_joint(self):
        """Create the joint between parent and child"""
        if not self.parent_object or not self.child_object or self.alignment_point is None:
            self.mw.log("Error: Parent, child, or alignment point not set.")
            return
        
        self.mw.log(f"Creating joint between {self.parent_object} and {self.child_object}...")
        self.mw.log(f"Joint pivot at: {self.alignment_point}")
        
        # Store original child transform for rotation testing
        child_link = self.mw.robot.links[self.child_object]
        self.original_child_transform = child_link.t_world.copy()
        
        # Show yellow arrow at alignment point
        self.show_joint_arrow()
        
        # Show rotation axis & limits section
        self.rotation_section.setVisible(True)
        
        # Update slider range based on limits
        self.update_slider_range()
        
        # Pre-fill joint name
        default_name = f"joint_{self.parent_object}_{self.child_object}"
        self.joint_name_input.setText(default_name)

    def update_slider_range(self):
        """Update slider range when min/max limits change"""
        if hasattr(self, 'rotation_slider'):
            min_val = int(self.min_limit_spin.value() * 10)
            max_val = int(self.max_limit_spin.value() * 10)
            self.rotation_slider.setRange(min_val, max_val)
            self.rotation_slider.setValue(0)
            
            # Also update spinbox range
            self.rotation_spinbox.setRange(self.min_limit_spin.value(), self.max_limit_spin.value())
            self.rotation_spinbox.setValue(0)

    def on_slider_changed(self, value):
        """Called when slider value changes - update spinbox and rotate"""
        angle_deg = value / 10.0
        
        # Update spinbox without triggering its signal
        self.rotation_spinbox.blockSignals(True)
        self.rotation_spinbox.setValue(angle_deg)
        self.rotation_spinbox.blockSignals(False)
        
        # Apply rotation
        self.test_rotation(value)

    def on_spinbox_changed(self, value):
        """Called when spinbox value changes - update slider and rotate"""
        slider_value = int(value * 10)
        
        # Update slider without triggering its signal
        self.rotation_slider.blockSignals(True)
        self.rotation_slider.setValue(slider_value)
        self.rotation_slider.blockSignals(False)
        
    def test_rotation(self, value):
        """Test rotate the child object based on slider value"""
        if not hasattr(self, 'original_child_transform') or not self.child_object or self.child_object not in self.mw.robot.links:
            return
        
        # Convert slider value to degrees
        angle_deg = value / 10.0
        angle_rad = np.radians(angle_deg)

        # 1. Get Parent Orientation
        parent_link = self.mw.robot.links[self.parent_object]
        R_p = parent_link.t_world[:3, :3]
        
        # 2. Get the currently selected axis Choice
        if self.axis_x_radio.isChecked():
            local_axis = np.array([1, 0, 0])
        elif self.axis_y_radio.isChecked():
            local_axis = np.array([0, 1, 0])
        else:  # Z
            local_axis = np.array([0, 0, 1])
            
        # 3. Transform Local Choice to World Direction
        axis = R_p @ local_axis
        axis = axis / (np.linalg.norm(axis) + 1e-9)

        # 4. Standard Rodrigues Formula (R_world)
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])
        R3x3 = np.eye(3) + np.sin(angle_rad) * K + (1 - np.cos(angle_rad)) * (K @ K)
        R = np.eye(4); R[:3, :3] = R3x3
        
        T_to_origin = np.eye(4); T_to_origin[:3, 3] = -self.alignment_point
        T_from_origin = np.eye(4); T_from_origin[:3, 3] = self.alignment_point
        
        # Apply transformation
        child_link = self.mw.robot.links[self.child_object]
        child_link.t_world = T_from_origin @ R @ T_to_origin @ self.original_child_transform
        
        # 5. Update visual and guides
        self.mw.canvas.update_transforms(self.mw.robot)
        self.show_joint_arrow()

    def show_joint_arrow(self):
        """Display a small RGB axis triad and a yellow joint direction arrow at the pivot."""
        import pyvista as pv
        if not self.parent_object or self.alignment_point is None: return
        
        # Remove any existing indicators
        self.mw.canvas.plotter.remove_actor("joint_arrow")
        self.mw.canvas.plotter.remove_actor("joint_triad_x")
        self.mw.canvas.plotter.remove_actor("joint_triad_y")
        self.mw.canvas.plotter.remove_actor("joint_triad_z")
        
        # 1. Get Parent Orientation
        parent_link = self.mw.robot.links[self.parent_object]
        R_p = parent_link.t_world[:3, :3]
        
        # 2. Get the currently selected axis Choice
        if self.axis_x_radio.isChecked():
            local_axis = np.array([1, 0, 0])
        elif self.axis_y_radio.isChecked():
            local_axis = np.array([0, 1, 0])
        else:  # Z
            local_axis = np.array([0, 0, 1])
            
        # Triangle orientation
        world_axis = R_p @ local_axis
        
        # --- SHOW RGB TRIAD (Local Parent Axes) ---
        # triad_length = 0.5
        # for i, color in enumerate(["red", "green", "blue"]):
        #     l_ax = np.zeros(3); l_ax[i] = 1
        #     w_ax = R_p @ l_ax
        #     line = pv.Line(self.alignment_point, self.alignment_point + w_ax * triad_length)
        #     self.mw.canvas.plotter.add_mesh(line, color=color, line_width=4, name=f"joint_triad_{'xyz'[i]}", pickable=False)

        # --- SHOW MAIN JOINT ARROW (Yellow) ---
        # arrow = pv.Arrow(start=self.alignment_point, direction=world_axis, scale=0.8)
        # self.mw.canvas.plotter.add_mesh(arrow, color="yellow", name="joint_arrow", pickable=False)
        self.mw.canvas.plotter.render()
        if hasattr(self.mw, 'experiment_tab'):
            self.mw.experiment_tab.update_display()
        if hasattr(self.mw, 'matrices_tab'):
            self.mw.matrices_tab.update_display()

    def confirm_joint(self):
        """Finalize the joint with selected axis and limits"""
        # Cleanup triad before proceeding
        self.mw.canvas.plotter.remove_actor("joint_triad_x")
        self.mw.canvas.plotter.remove_actor("joint_triad_y")
        self.mw.canvas.plotter.remove_actor("joint_triad_z")

        # Get selected axis
        if self.axis_x_radio.isChecked():
            axis = 0  # X
            axis_name = "X"
        elif self.axis_y_radio.isChecked():
            axis = 1  # Y
            axis_name = "Y"
        else:  # Z
            axis = 2  # Z
            axis_name = "Z"
        
        # Get limits
        min_limit = self.min_limit_spin.value()
        max_limit = self.max_limit_spin.value()
        
        child_link = self.mw.robot.links[self.child_object]
        parent_link = self.mw.robot.links[self.parent_object]
        
        # Get custom name and sanitize
        custom_name = self.joint_name_input.text().strip()
        if not custom_name:
            custom_name = f"joint_{self.parent_object}_{self.child_object}"
            
        # Robust sanitization: Only replace spaces. Let other chars (like -) stay.
        joint_id = custom_name.replace(" ", "_").replace("/", "_")
        
        # Check for duplicates or empty
        if not joint_id: joint_id = f"joint_{len(self.mw.robot.joints)}"
        
        # --- 1. PROPERLY ADD TO ROBOT MODEL ---
        joint = self.mw.robot.add_joint(joint_id, self.parent_object, self.child_object)
        
        # Calculate pivot point in Parent's Local Frame
        # Math: P_parent = inv(T_parent_world) * P_world
        t_parent_inv = np.linalg.inv(parent_link.t_world)
        pivot_local = (t_parent_inv @ np.append(self.alignment_point, 1))[:3]
        joint.origin = pivot_local
        
        # Set Axis (X, Y, or Z) - Store as local unit vector in parent frame
        axis_vecs = [np.array([1,0,0]), np.array([0,1,0]), np.array([0,0,1])]
        
        # DH COMPLIANCE: In industrial robotics, the rotation axis is the frame axis.
        # We store the local vector so it survives parent re-orientation.
        local_axis_vec = axis_vecs[axis]
        joint.axis = local_axis_vec
        
        # Set Child Static Offset (relative to parent at 0 degrees)
        # Math: Child_Offset = inv(Parent_World) * Original_Aligned_Child_World
        # IMPORTANT: Use original_child_transform to ensure 0 deg = perfectly aligned position
        child_link.t_offset = t_parent_inv @ self.original_child_transform
        
        # Set Joint Limits
        joint.min_limit = min_limit
        joint.max_limit = max_limit
        joint.current_value = 0.0
        
        # Calculate and Store the current WORLD axis for verification and DH tracking
        world_axis_vec = parent_link.t_world[:3, :3] @ local_axis_vec

        # --- 2. LOCAL STORAGE AND LOGGING ---
        # Store for UI tracking and Persistence
        self.joints[self.child_object] = {
            'parent': self.parent_object,
            'axis': axis, # Selection index (X=0, Y=1, Z=2)
            'local_axis_vector': local_axis_vec.tolist(),
            'world_axis_vector': world_axis_vec.tolist(),
            'min': min_limit,
            'max': max_limit,
            'current_angle': 0.0,
            'alignment_point': self.alignment_point.tolist() if isinstance(self.alignment_point, np.ndarray) else self.alignment_point,
            'custom_name': custom_name,
            'joint_id': joint_id,
            'is_gripper': self.gripper_checkbox.isChecked()
        }
        # Update robot model class
        joint.is_gripper = self.gripper_checkbox.isChecked()
        
        ratio = self.mw.canvas.grid_units_per_cm
        pivot_cm = pivot_local / ratio
        
        self.mw.log(f"Joint confirmed and added to Robot model (ID: {joint_id})")
        self.mw.log(f"  Pivot (CM):  X: {pivot_cm[0]:.3f}, Y: {pivot_cm[1]:.3f}, Z: {pivot_cm[2]:.3f}")
        
        # --- 3. AUTO-APPEND TO CODE EDITOR ---
        if hasattr(self.mw, 'experiment_tab') and hasattr(self.mw.experiment_tab, 'program_tab'):
            current_code = self.mw.experiment_tab.program_tab.code_edit.toPlainText()
            new_cmd = f"JOINT {joint_id} 0.00"
            
            if new_cmd not in current_code:
                self.mw.experiment_tab.program_tab.code_edit.appendPlainText(new_cmd)
            else:
                self.mw.experiment_tab.program_tab.code_edit.appendPlainText(new_cmd)
            self.mw.log(f"Auto-generated code: '{new_cmd}' added to Code tab.")
        self.mw.log(f"  Parent: {self.parent_object}")
        self.mw.log(f"  Child: {self.child_object}")
        self.mw.log(f"  Axis: {axis_name}")
        self.mw.log(f"  Limits: {min_limit}° to {max_limit}°")
        self.mw.log(f"  Pivot: {self.alignment_point}")
        
        # Remove arrow
        self.mw.canvas.plotter.remove_actor("joint_arrow")
        self.mw.canvas.plotter.render()
        
        # Reset UI
        self.reset_joint_ui()
        
        # Refresh joints list
        self.refresh_joints_history()
        
        # Refresh Matrices Panel Sliders
        if hasattr(self.mw, 'matrices_tab'):
            self.mw.matrices_tab.refresh_sliders()
        if hasattr(self.mw, 'experiment_tab'):
            self.mw.experiment_tab.refresh_sliders()
            self.mw.experiment_tab.update_display()
        
        self.mw.show_toast(f"Joint '{custom_name}' created", "success")

    def on_joint_control_changed(self, value):
        """Handle joint control slider changes"""
        if not self.active_joint_control:
            return
        
        angle_deg = value / 10.0
        
        # Update spinbox
        self.joint_control_spinbox.blockSignals(True)
        self.joint_control_spinbox.setValue(angle_deg)
        self.joint_control_spinbox.blockSignals(False)
        
        # Apply rotation to joint
        self.apply_joint_rotation(self.active_joint_control, angle_deg)

    def on_joint_control_spinbox_changed(self, value):
        """Handle joint control spinbox changes"""
        if not self.active_joint_control:
            return
        
        slider_value = int(value * 10)
        
        # Update slider
        self.joint_control_slider.blockSignals(True)
        self.joint_control_slider.setValue(slider_value)
        self.joint_control_slider.blockSignals(False)
        
        # Apply rotation to joint
        self.apply_joint_rotation(self.active_joint_control, value)

    def apply_joint_rotation(self, child_name, angle_deg):
        """Apply rotation to a jointed object using the Robot core kinematics"""
        if child_name not in self.mw.robot.links:
            return
            
        child_link = self.mw.robot.links[child_name]
        joint = child_link.parent_joint
        
        if joint:
            # 1. Update the robot model state
            joint.current_value = angle_deg
            
            # 2. Trigger re-calculation of all world transforms
            # This handles multi-link chains correctly (e.g. Base -> Arm1 -> Arm2)
            self.mw.robot.update_kinematics()
            
            # 3. Synchronize local JointPanel data
            if child_name in self.joints:
                self.joints[child_name]['current_angle'] = angle_deg
                
            # 4. Synchronize MatricesPanel if it exists
            if hasattr(self.mw, 'matrices_tab'):
                self.mw.matrices_tab.sync_slider(child_name, angle_deg)
            if hasattr(self.mw, 'experiment_tab'):
                self.mw.experiment_tab.sync_slider(child_name, angle_deg)
                
            # 5. Send command to hardware (ESP32)
            if hasattr(self.mw, 'serial_mgr'):
                # Use joint_id (e.g. joint_1) instead of display name for code consistency
                joint_id = self.joints[child_name].get('joint_id', child_name)
                # Send with current global speed
                speed = float(getattr(self.mw, 'current_speed', 0))
                self.mw.serial_mgr.send_command(joint_id, angle_deg, speed=speed)
                
            # 6. Show Speed Overlay on 3D Canvas
            if hasattr(self.mw, 'show_speed_overlay'):
                self.mw.show_speed_overlay()
                
            # 7. Push updated transforms to the 3D viewer
            self.mw.canvas.update_transforms(self.mw.robot)
            
            # 7b. Update Live Point (LP) coordinates UI
            if hasattr(self.mw, 'update_live_ui'):
                self.mw.update_live_ui()

            # 8. Propagate to related joints (Bidirectional Coupling)
            joint_id = self.joints[child_name].get('joint_id', child_name)
            robot = self.mw.robot
            
            def update_other_joint(target_id, target_val):
                if target_id in robot.joints:
                    target_joint = robot.joints[target_id]
                    target_joint.current_value = target_val
                    
                    # Sync UI metadata for this joint
                    # Find link name
                    l_name = None
                    for name, d in self.joints.items():
                        if d.get('joint_id') == target_id:
                            l_name = name
                            break
                    if l_name:
                        self.joints[l_name]['current_angle'] = target_val
                        # Sync MatricesPanel if exists
                        if hasattr(self.mw, 'matrices_tab'):
                            self.mw.matrices_tab.sync_slider(l_name, target_val)

            # A. If Master moved -> Update all Slaves
            if joint_id in robot.joint_relations:
                for slave_id, ratio in robot.joint_relations[joint_id]:
                    slave_angle = np.clip(angle_deg * ratio, robot.joints[slave_id].min_limit, robot.joints[slave_id].max_limit)
                    update_other_joint(slave_id, slave_angle)
            
            # B. If Slave moved -> Update Master (and its other slaves)
            else:
                for m_id, slaves in robot.joint_relations.items():
                    for s_id, ratio in slaves:
                        if s_id == joint_id and abs(ratio) > 1e-6:
                            m_angle = np.clip(angle_deg / ratio, robot.joints[m_id].min_limit, robot.joints[m_id].max_limit)
                            update_other_joint(m_id, m_angle)
                            # Update siblings
                            for sib_id, sib_ratio in robot.joint_relations[m_id]:
                                if sib_id != joint_id:
                                    sib_angle = np.clip(m_angle * sib_ratio, robot.joints[sib_id].min_limit, robot.joints[sib_id].max_limit)
                                    update_other_joint(sib_id, sib_angle)
                            break
                
            # After updating all related, re-calc kinematics and update canvas once
            self.mw.robot.update_kinematics()
            self.mw.canvas.update_transforms(self.mw.robot)

    def reset_joint_ui(self):
        """Reset the joint creation UI"""
        self.parent_object = None
        self.child_object = None
        self.alignment_point = None
        
        self.axis_section.setVisible(False)
        self.rotation_section.setVisible(False)
        
        self.refresh_links()
        self.mw.log("Joint creation complete. Ready for next joint.")

    def refresh_links(self):
        """Refresh the object list with role indicators"""
        self.objects_list.clear()
        
        # Get all links from robot
        for name in self.mw.robot.links.keys():
            # Create item with colored box indicator and checkmark
            display_text = name
            
            # Check if this object has a joint (jointed child)
            if name in self.joints:
                display_text = f"✓⭕ {name}"  # Special indicator for jointed objects
            # Add checkmark for parent (white) or child (gray)
            elif name == self.parent_object:
                display_text = f"✓ {name}"  # White checkmark for parent
            elif name == self.child_object:
                display_text = f"✓ {name}"  # Gray checkmark for child
            
            item = QtWidgets.QListWidgetItem(display_text)
            
            # Color based on role
            if name in self.joints:
                # Jointed objects get orange color
                item.setForeground(QtGui.QColor("#ff9800"))  # Orange for jointed
                item.setBackground(QtGui.QColor("#fff3e0"))  # Light orange background
            elif name == self.parent_object:
                item.setForeground(QtGui.QColor("#d32f2f"))  # Red text for parent with checkmark
                item.setBackground(QtGui.QColor("#ffebee"))  # Light red background
            elif name == self.child_object:
                item.setForeground(QtGui.QColor("#1976d2"))  # Blue text for child with checkmark
                item.setBackground(QtGui.QColor("#e3f2fd"))  # Light blue background
            else:
                # Default alternating colors
                index = list(self.mw.robot.links.keys()).index(name)
                if index % 2 == 0:
                    item.setForeground(QtGui.QColor("#d32f2f"))  # Red
                else:
                    item.setForeground(QtGui.QColor("#1976d2"))  # Blue
            
            self.objects_list.addItem(item)

    def on_select_gripper_surface(self):
        """Callback for the Gripper Surface button on the canvas. Prepares for surface picking."""
        self.mw.log("Gripper Surface Selection Mode: Please click the inner surface of the gripper in the 3D view.")
        self.mw.show_toast("Select Gripper Inner Surface", "success")
        
        # If there's a canvas method for picking (like focus point), we can call it here.
        # For now, we prepare the system for the feature integration.
        if hasattr(self.mw.canvas, 'start_gripper_surface_picking'):
            self.mw.canvas.start_gripper_surface_picking()

    def on_set_live_point(self):
        """Callback for 'Set Live Point' button. Activates point picking."""
        self.mw.log("📍 Live Point Selection: Click a point on the gripper (TCP) where it should hold objects.")
        self.mw.show_toast("Click to set Live Point (TCP)", "info")
        self.mw.canvas.start_point_picking(self.on_live_point_picked)
        
    def on_live_point_picked(self, world_pt):
        """Processes the picked point and saves it as custom TCP offset."""
        if not self.child_object:
            self.mw.log("⚠️ Error: No child object selected to attach Live Point.")
            return
            
        child_link = self.mw.robot.links[self.child_object]
        
        # Convert world point to local coordinate system of the child link
        # Local = Inv(World_T) * World_P
        inv_world = np.linalg.inv(child_link.t_world)
        local_pt = (inv_world @ np.append(world_pt, 1))[:3]
        
        # Save to robot model
        child_link.custom_tcp_offset = local_pt
        
        # Save to local UI cache for persistence
        if self.child_object in self.joints:
            self.joints[self.child_object]['custom_tcp_offset'] = local_pt.tolist()
            
        self.mw.log(f"✅ Live Point (TCP) set: {np.round(local_pt, 2)} (Local cm)")
        self.mw.show_toast("TCP Position Saved", "success")
        
        # Update UI display in Simulation/Main via update_live_ui
        self.mw.update_live_ui()

