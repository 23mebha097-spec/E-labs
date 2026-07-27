from PyQt5 import QtWidgets, QtCore, QtGui
import numpy as np

from core.kinematics import compute_standard_dh_matrix

class TypeOnlyDoubleSpinBox(QtWidgets.QDoubleSpinBox):
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
        self._gripper_group_control = None
        self._gripper_group_syncing = False
        self._quick_preview_original_transform = None
        self._alignment_point_is_manual = False
        
        self.init_ui()

    def _axis_index_from_vector(self, axis_vector):
        try:
            axis = np.asarray(axis_vector, dtype=float)
            if axis.size != 3 or np.linalg.norm(axis) < 1e-9:
                return 2
        except Exception:
            return 2
        return self.axis_index_from_vector(axis_vector)

    def _axis_name_from_vector(self, axis_vector):
        return {0: "X", 1: "Y", 2: "Z"}.get(self.axis_index_from_vector(axis_vector), "Z")

    def _dh_params_from_transform(self, transform):
        """Infer standard DH parameters from a zero-pose parent->child transform."""
        mat = np.array(transform, dtype=float)
        px = float(mat[0, 3])
        py = float(mat[1, 3])
        pz = float(mat[2, 3])
        a = float(np.hypot(px, py))
        theta0 = float(np.degrees(np.arctan2(py, px))) if a > 1e-9 else 0.0
        alpha = float(np.degrees(np.arctan2(mat[2, 1], mat[2, 2])))
        return {
            "theta0_deg": theta0,
            "d": pz,
            "a": a,
            "alpha_deg": alpha,
        }

    def _dh_zero_pose_matrix(self, transform):
        params = self._dh_params_from_transform(transform)
        return compute_standard_dh_matrix(
            params["theta0_deg"],
            params["d"],
            params["a"],
            params["alpha_deg"],
            q_value=0.0,
            joint_type="revolute",
        )

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.creator_card = QtWidgets.QWidget()
        self.creator_card.setStyleSheet("""
            QWidget {
                background-color: white;
                border: 1px solid #e0e0e0;
                border-radius: 8px;
            }
            QLabel {
                border: none;
                background: transparent;
            }
        """)
        creator_layout = QtWidgets.QVBoxLayout(self.creator_card)
        creator_layout.setContentsMargins(12, 12, 12, 12)
        creator_layout.setSpacing(10)

        creator_title = QtWidgets.QLabel("joint")
        creator_title.setAlignment(QtCore.Qt.AlignCenter)
        creator_title.setStyleSheet("color: #212121; font-size: 28px; font-weight: 500;")
        creator_layout.addWidget(creator_title)

        form_style = """
            QLineEdit, QComboBox {
                background-color: white;
                color: #212121;
                border: 1px solid #bdbdbd;
                border-radius: 4px;
                padding: 6px;
                font-size: 13px;
            }
        """

        self.quick_joint_name_input = QtWidgets.QLineEdit()
        self.quick_joint_name_input.setPlaceholderText("create joint: name")
        self.quick_joint_name_input.setStyleSheet(form_style)
        creator_layout.addWidget(self.quick_joint_name_input)

        self.parent_combo = QtWidgets.QComboBox()
        self.parent_combo.setPlaceholderText("parent")
        self.parent_combo.setStyleSheet(form_style)
        self.parent_combo.currentIndexChanged.connect(self.on_quick_joint_selection_changed)
        creator_layout.addWidget(self.parent_combo)

        self.child_combo = QtWidgets.QComboBox()
        self.child_combo.setPlaceholderText("child")
        self.child_combo.setStyleSheet(form_style)
        self.child_combo.currentIndexChanged.connect(self.on_quick_joint_selection_changed)
        creator_layout.addWidget(self.child_combo)

        self.rotation_face_combo = QtWidgets.QComboBox()
        self.rotation_face_combo.setStyleSheet(form_style)
        self.rotation_face_combo.addItem("rotation face: auto center", None)
        self.rotation_face_combo.addItem("pick parent face", "parent")
        self.rotation_face_combo.addItem("pick child face", "child")
        self.rotation_face_combo.currentIndexChanged.connect(self.on_rotation_face_combo_changed)
        self.rotation_face_combo.view().installEventFilter(self)
        creator_layout.addWidget(self.rotation_face_combo)

        quick_axis_row = QtWidgets.QHBoxLayout()
        quick_axis_label = QtWidgets.QLabel("axis:")
        quick_axis_label.setStyleSheet("color: #212121; font-size: 18px;")
        quick_axis_row.addWidget(quick_axis_label)

        self.quick_axis_group = QtWidgets.QButtonGroup(self)
        quick_axis_style = """
            QRadioButton {
                color: #212121;
                font-size: 17px;
                border: none;
                background: transparent;
            }
        """
        self.quick_axis_x_radio = QtWidgets.QRadioButton("X")
        self.quick_axis_y_radio = QtWidgets.QRadioButton("Y")
        self.quick_axis_z_radio = QtWidgets.QRadioButton("Z")
        self.quick_axis_z_radio.setChecked(True)
        for index, radio in enumerate((self.quick_axis_x_radio, self.quick_axis_y_radio, self.quick_axis_z_radio)):
            radio.setStyleSheet(quick_axis_style)
            self.quick_axis_group.addButton(radio, index)
            radio.toggled.connect(self.on_quick_axis_changed)
            quick_axis_row.addWidget(radio)
        quick_axis_row.addStretch()
        creator_layout.addLayout(quick_axis_row)

        quick_limits_row = QtWidgets.QHBoxLayout()
        quick_limits_label = QtWidgets.QLabel("limit:")
        quick_limits_label.setStyleSheet("color: #212121; font-size: 14px;")
        quick_limits_row.addWidget(quick_limits_label)

        self.quick_min_limit_spin = TypeOnlyDoubleSpinBox()
        self.quick_min_limit_spin.setRange(-360, 360)
        self.quick_min_limit_spin.setValue(-180)
        self.quick_min_limit_spin.setDecimals(1)
        self.quick_min_limit_spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.quick_min_limit_spin.setStyleSheet("background-color: white; color: #212121; border: 1px solid #bbb; padding: 5px;")
        self.quick_min_limit_spin.valueChanged.connect(self.on_quick_limits_changed)
        quick_limits_row.addWidget(self.quick_min_limit_spin)

        self.quick_max_limit_spin = TypeOnlyDoubleSpinBox()
        self.quick_max_limit_spin.setRange(-360, 360)
        self.quick_max_limit_spin.setValue(180)
        self.quick_max_limit_spin.setDecimals(1)
        self.quick_max_limit_spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.quick_max_limit_spin.setStyleSheet("background-color: white; color: #212121; border: 1px solid #bbb; padding: 5px;")
        self.quick_max_limit_spin.valueChanged.connect(self.on_quick_limits_changed)
        quick_limits_row.addWidget(self.quick_max_limit_spin)
        creator_layout.addLayout(quick_limits_row)

        quick_angle_row = QtWidgets.QHBoxLayout()
        quick_angle_label = QtWidgets.QLabel("angle:")
        quick_angle_label.setStyleSheet("color: #212121; font-size: 14px;")
        quick_angle_row.addWidget(quick_angle_label)

        self.quick_angle_spin = TypeOnlyDoubleSpinBox()
        self.quick_angle_spin.setRange(-180, 180)
        self.quick_angle_spin.setValue(0)
        self.quick_angle_spin.setDecimals(1)
        self.quick_angle_spin.setSuffix("°")
        self.quick_angle_spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.quick_angle_spin.setStyleSheet("background-color: white; color: #212121; border: 1px solid #bbb; padding: 5px;")
        self.quick_angle_spin.valueChanged.connect(self.on_quick_angle_changed)
        quick_angle_row.addWidget(self.quick_angle_spin)
        creator_layout.addLayout(quick_angle_row)

        self.quick_joint_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.quick_joint_slider.setRange(-1800, 1800)
        self.quick_joint_slider.setValue(0)
        self.quick_joint_slider.setSingleStep(10)
        self.quick_joint_slider.setPageStep(100)
        self.quick_joint_slider.setTracking(True)
        self.quick_joint_slider.setStyleSheet("""
            QSlider {
                border: none;
                background: transparent;
            }
            QSlider::groove:horizontal {
                height: 8px;
                background: #f5f5f5;
                border: 1px solid #9e9e9e;
                border-radius: 2px;
            }
            QSlider::handle:horizontal {
                background: white;
                border: 1px solid #212121;
                width: 18px;
                height: 18px;
                margin-top: -6px;
                margin-bottom: -6px;
                border-radius: 9px;
            }
        """)
        self.quick_joint_slider.valueChanged.connect(self.on_quick_slider_changed)
        creator_layout.addWidget(self.quick_joint_slider)

        self.quick_create_btn = QtWidgets.QPushButton("Create Joint")
        self.quick_create_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.quick_create_btn.setStyleSheet("""
            QPushButton {
                background-color: #1976d2;
                color: white;
                border: none;
                border-radius: 6px;
                padding: 10px;
                font-size: 13px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1565c0;
            }
        """)
        self.quick_create_btn.clicked.connect(self.create_joint_from_quick_form)
        creator_layout.addWidget(self.quick_create_btn)

        layout.addWidget(self.creator_card)
        
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
        # Kept for internal refresh/selection logic, but removed from the Joint tab UI.
        self.objects_list.hide()
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
        self.joint_setup_header = QtWidgets.QLabel("3. JOINT AXIS & LIMITS")
        self.joint_setup_header.setStyleSheet("color: #1976d2; font-size: 14px; font-weight: bold; padding: 5px;")
        rot_layout.addWidget(self.joint_setup_header)
        
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

        type_layout = QtWidgets.QHBoxLayout()
        type_label = QtWidgets.QLabel("Joint Type:")
        type_label.setStyleSheet("color: #616161; font-size: 12px;")
        type_layout.addWidget(type_label)

        self.joint_type_combo = QtWidgets.QComboBox()
        self.joint_type_combo.addItem("Revolute", "revolute")
        self.joint_type_combo.addItem("Prismatic", "prismatic")
        self.joint_type_combo.setStyleSheet("""
            QComboBox {
                background-color: white;
                color: #1976d2;
                border: 1px solid #bbb;
                padding: 5px;
                border-radius: 3px;
                font-weight: bold;
            }
        """)
        self.joint_type_combo.currentIndexChanged.connect(self.on_joint_type_changed)
        type_layout.addWidget(self.joint_type_combo)
        rot_layout.addLayout(type_layout)

        self.joint_steps_box = QtWidgets.QLabel()
        self.joint_steps_box.setWordWrap(True)
        self.joint_steps_box.setStyleSheet("""
            QLabel {
                background-color: #f5f9ff;
                color: #37474f;
                border: 1px solid #bbdefb;
                border-radius: 5px;
                padding: 8px;
                font-size: 12px;
                line-height: 1.35;
            }
        """)
        rot_layout.addWidget(self.joint_steps_box)
        
        # Axis selection
        self.axis_label = QtWidgets.QLabel("Select joint axis:")
        self.axis_label.setStyleSheet("color: #616161; font-size: 12px; padding: 5px;")
        rot_layout.addWidget(self.axis_label)
        
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
        self.axis_x_radio.toggled.connect(self.on_joint_axis_changed)
        self.axis_y_radio.toggled.connect(self.on_joint_axis_changed)
        self.axis_z_radio.toggled.connect(self.on_joint_axis_changed)
        self.axis_x_radio.toggled.connect(lambda checked: self.on_prismatic_axis_radio("x") if checked else None)
        self.axis_y_radio.toggled.connect(lambda checked: self.on_prismatic_axis_radio("y") if checked else None)
        self.axis_z_radio.toggled.connect(lambda checked: self.on_prismatic_axis_radio("z") if checked else None)
        
        rot_layout.addLayout(axis_buttons_row)

        self.prismatic_direction_section = QtWidgets.QWidget()
        direction_layout = QtWidgets.QVBoxLayout(self.prismatic_direction_section)
        direction_layout.setContentsMargins(0, 0, 0, 0)
        direction_layout.setSpacing(8)

        direction_help = QtWidgets.QLabel("Prismatic direction angle:")
        direction_help.setStyleSheet("color: #616161; font-size: 12px; padding: 5px;")
        direction_layout.addWidget(direction_help)

        direction_row = QtWidgets.QHBoxLayout()
        plane_label = QtWidgets.QLabel("Plane:")
        plane_label.setStyleSheet("color: #616161; font-size: 11px;")
        direction_row.addWidget(plane_label)

        self.prismatic_plane_combo = QtWidgets.QComboBox()
        self.prismatic_plane_combo.addItem("XY plane", "xy")
        self.prismatic_plane_combo.addItem("YZ plane", "yz")
        self.prismatic_plane_combo.addItem("ZX plane", "zx")
        self.prismatic_plane_combo.setStyleSheet("""
            QComboBox {
                background-color: white;
                color: #1976d2;
                border: 1px solid #bbb;
                padding: 5px;
                border-radius: 3px;
                font-weight: bold;
            }
        """)
        self.prismatic_plane_combo.currentIndexChanged.connect(self.on_prismatic_direction_changed)
        direction_row.addWidget(self.prismatic_plane_combo)

        angle_label = QtWidgets.QLabel("Angle:")
        angle_label.setStyleSheet("color: #616161; font-size: 11px;")
        direction_row.addWidget(angle_label)

        self.prismatic_angle_spin = TypeOnlyDoubleSpinBox()
        self.prismatic_angle_spin.setRange(-360, 360)
        self.prismatic_angle_spin.setValue(0)
        self.prismatic_angle_spin.setSuffix("Â°")
        self.prismatic_angle_spin.setDecimals(1)
        self.prismatic_angle_spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.prismatic_angle_spin.setStyleSheet("background-color: white; color: #212121; border: 1px solid #bbb; padding: 5px;")
        self.prismatic_angle_spin.valueChanged.connect(self.on_prismatic_direction_changed)
        direction_row.addWidget(self.prismatic_angle_spin)

        direction_layout.addLayout(direction_row)
        rot_layout.addWidget(self.prismatic_direction_section)
        
        # Rotation limits
        self.limits_label = QtWidgets.QLabel("Rotation limits (degrees):")
        self.limits_label.setStyleSheet("color: #616161; font-size: 12px; padding: 5px;")
        rot_layout.addWidget(self.limits_label)
        
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
        
        # Test Joint Slider
        self.test_label = QtWidgets.QLabel("Test rotation:")
        self.test_label.setStyleSheet("color: #616161; font-size: 12px; padding: 5px;")
        rot_layout.addWidget(self.test_label)
        
        slider_row = QtWidgets.QHBoxLayout()
        
        self.rotation_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.rotation_slider.setRange(-1800, 1800)  # -180 to 180 degrees (x10 for precision)
        self.rotation_slider.setValue(0)
        self.rotation_slider.setSingleStep(10)
        self.rotation_slider.setPageStep(100)
        self.rotation_slider.setTracking(True)
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
        self.rotation_spinbox.setDecimals(1)
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

        # Pick Pivot Button
        self.pick_pivot_btn = QtWidgets.QPushButton("Pick Joint Pivot")
        self.pick_pivot_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.pick_pivot_btn.setStyleSheet(btn_style)
        self.pick_pivot_btn.clicked.connect(self.pick_joint_pivot)
        self.pick_pivot_btn.setEnabled(False)
        buttons_layout.addWidget(self.pick_pivot_btn)
        
        # Hidden legacy controls: parent/child/pivot selection is no longer shown
        # in the Joint tab, but the widgets stay alive for existing callbacks.
        self._legacy_buttons_container = buttons_container
        self._legacy_buttons_container.hide()
        
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
        
        self._legacy_undo_redo_container = undo_redo_container
        self._legacy_undo_redo_container.hide()
        
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
        self.jc_slider_label = QtWidgets.QLabel("Rotation:")
        self.jc_slider_label.setStyleSheet("color: #424242; font-size: 13px; padding: 2px;")
        jc_layout.addWidget(self.jc_slider_label)
        
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
        self.joint_control_spinbox.setDecimals(1)
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
        self.joint_control_section.hide()
        
        # --- 4. CREATED JOINTS SECTION ---
        self.header_joints = QtWidgets.QLabel("4. CREATED JOINTS")
        self.header_joints.setStyleSheet("color: #1976d2; font-size: 14px; font-weight: bold; margin-top: 20px; padding: 5px;")
        self.header_joints.hide()
        
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
        self.joints_history_list.hide()
        layout.addWidget(self.header_joints)
        layout.addWidget(self.joints_history_list)
        
        # Bottom stretch
        self.update_joint_steps_box()
        self.refresh_quick_link_combos()
        layout.addStretch()

    def refresh_quick_link_combos(self):
        """Keep the compact joint form in sync with imported robot links."""
        if not hasattr(self, 'parent_combo') or not hasattr(self, 'child_combo'):
            return

        current_parent = self.parent_combo.currentData()
        current_child = self.child_combo.currentData()
        robot = getattr(self.mw, "robot", None)
        links = getattr(robot, "links", {}) if robot is not None else {}
        link_names = list(links.keys())

        self.parent_combo.blockSignals(True)
        self.parent_combo.clear()
        self.parent_combo.addItem("parent", None)
        for name in link_names:
            self.parent_combo.addItem(name, name)
        if current_parent in link_names:
            self.parent_combo.setCurrentIndex(self.parent_combo.findData(current_parent))
        self.parent_combo.blockSignals(False)

        self.child_combo.blockSignals(True)
        self.child_combo.clear()
        self.child_combo.addItem("child", None)
        child_options = [name for name in link_names if self.is_valid_child_link(name, current_parent)]
        for name in child_options:
            self.child_combo.addItem(name, name)
        if not child_options:
            self.child_combo.addItem("No valid child links", None)
            item = self.child_combo.model().item(self.child_combo.count() - 1)
            if item is not None:
                item.setEnabled(False)
        if current_child in child_options:
            self.child_combo.setCurrentIndex(self.child_combo.findData(current_child))
        self.child_combo.blockSignals(False)

        self.refresh_link_table()

    def is_parented_link(self, name):
        """Return whether a link already has a parent joint."""
        robot = getattr(self.mw, "robot", None)
        link = getattr(robot, "links", {}).get(name) if robot is not None else None
        return bool(link and (link.parent_joint is not None or name in self.joints))

    def is_base_link(self, name):
        robot = getattr(self.mw, "robot", None)
        link = getattr(robot, "links", {}).get(name) if robot is not None else None
        base_link = getattr(robot, "base_link", None)
        base_name = getattr(base_link, "name", None)
        return bool(
            link is not None
            and (
                getattr(link, "is_base", False)
                or (base_name is not None and base_name == name)
            )
        )

    def is_valid_child_link(self, child_name, parent_name=None):
        robot = getattr(self.mw, "robot", None)
        links = getattr(robot, "links", {}) if robot is not None else {}
        if child_name not in links:
            return False
        if self.is_base_link(child_name):
            return False
        if not parent_name:
            return True
        if parent_name not in links:
            return False
        if parent_name and child_name == parent_name:
            return False
        # Re-parenting an existing child is valid, but making a link the child
        # of one of its own descendants would create a closed kinematic loop.
        if parent_name and self.is_descendant_link(parent_name, child_name):
            return False
        return True

    def refresh_link_table(self):
        if not hasattr(self, 'links_table'):
            return

        self.links_table.setRowCount(0)
        for name, link in sorted(self.mw.robot.links.items(), key=lambda item: item[0].lower()):
            row = self.links_table.rowCount()
            self.links_table.insertRow(row)

            status = []
            if link.parent_joint is not None:
                status.append(f"Child of {link.parent_joint.parent_link.name}")
            if link.child_joints:
                status.append(f"Parent of {len(link.child_joints)}")
            if not status:
                status = ["Free"]

            name_item = QtWidgets.QTableWidgetItem(name)
            status_item = QtWidgets.QTableWidgetItem(", ".join(status))
            self.links_table.setItem(row, 0, name_item)
            self.links_table.setItem(row, 1, status_item)

        self.links_table.resizeColumnsToContents()

    def set_selected_link_as_parent(self):
        selected = self.links_table.selectedItems()
        if not selected:
            return
        selected_name = selected[0].text()
        self.select_parent_link(selected_name)

    def set_selected_link_as_child(self):
        selected = self.links_table.selectedItems()
        if not selected:
            return
        selected_name = selected[0].text()
        self.select_child_link(selected_name)

    def on_links_table_selection_changed(self):
        selected = self.links_table.selectedItems()
        if not selected:
            self.set_parent_btn.setEnabled(False)
            self.set_child_btn.setEnabled(False)
            return

        selected_name = selected[0].text()
        self.set_parent_btn.setEnabled(True)
        self.set_child_btn.setEnabled(self.is_valid_child_link(selected_name, self.parent_combo.currentData()))

    def set_quick_combo_value(self, combo, value):
        combo.blockSignals(True)
        index = combo.findData(value)
        combo.setCurrentIndex(index if index >= 0 else 0)
        combo.blockSignals(False)

    def select_parent_link(self, name):
        """Programmatically select a link as the next joint parent, including existing child links."""
        robot = getattr(self.mw, "robot", None)
        if name not in getattr(robot, "links", {}):
            return
        self.set_quick_combo_value(self.parent_combo, name)
        self.refresh_quick_link_combos()
        self.parent_object = name
        if self.child_combo.currentData() and not self.is_valid_child_link(self.child_combo.currentData(), name):
            self.set_quick_combo_value(self.child_combo, None)
            self.child_object = None
        self.alignment_point = self.get_quick_joint_pivot()
        self._alignment_point_is_manual = False
        self.update_pick_pivot_state()
        self.mw.log(f"Parent set to: {name}")

    def select_child_link(self, name):
        """Programmatically select a valid child link, including existing child links for re-parenting."""
        robot = getattr(self.mw, "robot", None)
        if name not in getattr(robot, "links", {}):
            return
        if not self.is_valid_child_link(name, self.parent_combo.currentData()):
            self.mw.log(f"{name} cannot be selected as child for the current parent.")
            return
        self.refresh_quick_link_combos()
        self.set_quick_combo_value(self.child_combo, name)
        self.child_object = name
        self.alignment_point = self.get_quick_joint_pivot()
        self._alignment_point_is_manual = False
        self.update_pick_pivot_state()
        self.mw.log(f"Child set to: {name}")

    def is_descendant_link(self, possible_descendant_name, ancestor_name):
        """Return True if possible_descendant_name is below ancestor_name in the joint tree."""
        if possible_descendant_name == ancestor_name:
            return True
        robot = getattr(self.mw, "robot", None)
        links = getattr(robot, "links", {}) if robot is not None else {}
        ancestor = links.get(ancestor_name)
        if ancestor is None:
            return False

        stack = []
        for joint in getattr(ancestor, "child_joints", []) or []:
            child_link = getattr(joint, "child_link", None)
            if child_link is not None:
                stack.append(child_link)
        seen = set()
        while stack:
            link = stack.pop()
            link_name = getattr(link, "name", None)
            if not link_name or link_name in seen:
                continue
            if link_name == possible_descendant_name:
                return True
            seen.add(link_name)
            for joint in getattr(link, "child_joints", []) or []:
                child_link = getattr(joint, "child_link", None)
                if child_link is not None:
                    stack.append(child_link)
        return False

    def validate_joint_tree_choice(self, parent_name, child_name):
        """Allow chained parents while keeping each child single-parent and acyclic."""
        if not parent_name or not child_name:
            return False, "Select parent and child"
        if parent_name == child_name:
            return False, "Parent and child must differ"
        if parent_name not in self.mw.robot.links or child_name not in self.mw.robot.links:
            return False, "Selected link missing"

        if self.is_base_link(child_name):
            return False, "Base link cannot be selected as child"
        if self.is_descendant_link(parent_name, child_name):
            return False, "This would create a cycle in the joint tree"

        return True, ""

    def rigidize_cached_alignments(self):
        """Make aligned but unjointed components follow their assembled parent."""
        robot = getattr(self.mw, "robot", None)
        if robot is None or not hasattr(robot, "rigidize_alignment_cache"):
            return 0
        created = robot.rigidize_alignment_cache(getattr(self.mw, "alignment_cache", {}))
        if created:
            self.mw.log(f"Rigid attachments recovered: {created} aligned component(s) will follow their assembly parent.")
        return created

    def _rigid_motion_group_names(self, root_link):
        """Return the moving subtree for a link, including already rigid descendants."""
        robot = getattr(self.mw, "robot", None)
        if robot is None or root_link is None:
            return set()

        names = {getattr(root_link, "name", None)}
        for descendant in robot._collect_descendant_links(root_link):
            names.add(getattr(descendant, "name", None))
        names.discard(None)
        return names

    def _best_touching_parent(self, child_link, moving_names):
        """Pick the closest touching moving link that can become the rigid parent."""
        robot = getattr(self.mw, "robot", None)
        if robot is None or child_link is None:
            return None

        best = None
        for parent_name in moving_names:
            parent_link = robot.links.get(parent_name)
            if parent_link is None or parent_link is child_link:
                continue

            pivot = self.get_interference_center(parent_link, child_link)
            if pivot is None:
                continue

            parent_center = self.get_component_center(parent_link)
            child_center = self.get_component_center(child_link)
            if parent_center is not None and child_center is not None:
                score = float(np.linalg.norm(parent_center - child_center))
            else:
                score = 0.0

            if best is None or score < best[0]:
                best = (score, parent_name, np.array(pivot, dtype=float))

        return best

    def rigidize_touching_free_components(self, root_link):
        """
        Make any touching, unjointed components follow a rotated component rigidly.

        This turns "touching + no joint" into a fixed attachment so the next
        rotation keeps the assembly moving as one rigid group.
        """
        robot = getattr(self.mw, "robot", None)
        if robot is None or root_link is None:
            return 0

        moving_names = self._rigid_motion_group_names(root_link)
        if not moving_names:
            return 0

        created = 0
        while True:
            changed = False
            for child_name, child_link in list(robot.links.items()):
                if child_name in moving_names:
                    continue
                if getattr(child_link, "parent_joint", None) is not None:
                    continue

                best = self._best_touching_parent(child_link, moving_names)
                if best is None:
                    continue

                _, parent_name, pivot = best
                joint = robot.ensure_fixed_joint(
                    parent_name,
                    child_name,
                    child_world_transform=np.array(child_link.t_world, dtype=float).copy(),
                    origin_world=pivot,
                )
                if joint is None:
                    continue

                moving_names.add(child_name)
                created += 1
                changed = True

                if hasattr(self.mw, "alignment_cache"):
                    self.mw.alignment_cache[(parent_name, child_name)] = {
                        "contact_point": pivot.tolist(),
                        "auto_rigid": True,
                    }

            if not changed:
                break

        if created:
            robot.update_kinematics()
        return created

    def on_quick_axis_changed(self, checked):
        if not checked:
            return

        axis_id = self.quick_axis_group.checkedId()
        if hasattr(self, 'axis_x_radio'):
            self.axis_x_radio.blockSignals(True)
            self.axis_y_radio.blockSignals(True)
            self.axis_z_radio.blockSignals(True)
            self.axis_x_radio.setChecked(axis_id == 0)
            self.axis_y_radio.setChecked(axis_id == 1)
            self.axis_z_radio.setChecked(axis_id == 2)
            self.axis_x_radio.blockSignals(False)
            self.axis_y_radio.blockSignals(False)
            self.axis_z_radio.blockSignals(False)
        if self.prepare_quick_joint_preview():
            self.reset_joint_preview_to_initial()

    def on_quick_joint_selection_changed(self, *_):
        self.restore_quick_preview_transform()
        parent_changed = self.sender() is self.parent_combo
        self.parent_object = self.parent_combo.currentData()
        self.child_object = self.child_combo.currentData()
        if parent_changed or (self.child_object and not self.is_valid_child_link(self.child_object, self.parent_object)):
            self.child_object = None
            self.refresh_quick_link_combos()
            self.child_object = self.child_combo.currentData()
        self._quick_preview_original_transform = None
        self.alignment_point = self.get_quick_joint_pivot()
        self._alignment_point_is_manual = False
        if hasattr(self, 'rotation_face_combo'):
            self.rotation_face_combo.blockSignals(True)
            self.rotation_face_combo.setCurrentIndex(0)
            self.rotation_face_combo.blockSignals(False)
        self.update_pick_pivot_state()

    def eventFilter(self, obj, event):
        if (
            hasattr(self, 'rotation_face_combo')
            and obj == self.rotation_face_combo.view()
            and event.type() == QtCore.QEvent.MouseButtonDblClick
        ):
            index = self.rotation_face_combo.view().indexAt(event.pos())
            if index.isValid():
                role = self.rotation_face_combo.itemData(index.row())
                selected_role = self.rotation_face_selected_role(role)
                if selected_role:
                    self.start_rotation_face_pick(selected_role)
                    self.rotation_face_combo.hidePopup()
                    return True
        return super().eventFilter(obj, event)

    def rotation_face_selected_role(self, role):
        if isinstance(role, str) and role.startswith("selected:"):
            return role.split(":", 1)[1]
        return None

    def on_rotation_face_combo_changed(self, *_):
        if not hasattr(self, 'rotation_face_combo'):
            return
        role = self.rotation_face_combo.currentData()
        if self.rotation_face_selected_role(role):
            return
        if role is None:
            if self.parent_object and self.child_object:
                self.alignment_point = self.get_quick_joint_pivot()
                self._alignment_point_is_manual = False
                self.show_joint_arrow(render=False)
                self.reset_joint_preview_to_initial()
            return

        self.start_rotation_face_pick(role)

    def start_rotation_face_pick(self, role):
        target_name = self.parent_object if role == "parent" else self.child_object
        if not self.parent_object or not self.child_object or target_name not in self.mw.robot.links:
            self.mw.show_toast("Select parent and child first", "warning")
            self.reset_rotation_face_combo()
            return

        self._pending_rotation_face_role = role
        self.mw.log(f"Rotation face pick mode active. Click a face on {target_name}.")
        self.mw.show_toast(f"Pick {role} rotation face", "info")
        self.mw.canvas.start_face_picking(
            self.on_rotation_face_picked,
            color="#ff9800",
            highlight_prefix="joint_face",
            center_mode="surface",
        )

    def reset_rotation_face_combo(self):
        if not hasattr(self, 'rotation_face_combo'):
            return
        self.rotation_face_combo.blockSignals(True)
        self.rotation_face_combo.setCurrentIndex(0)
        self.rotation_face_combo.blockSignals(False)

    def on_rotation_face_picked(self, link_name, center, normal):
        role = getattr(self, '_pending_rotation_face_role', None)
        expected_name = self.parent_object if role == "parent" else self.child_object
        if link_name != expected_name:
            self.mw.log(f"Picked {link_name}, but expected {expected_name}. Pick the selected {role} face.")
            self.mw.show_toast(f"Pick face on {expected_name}", "warning")
            self.mw.canvas.start_face_picking(
                self.on_rotation_face_picked,
                color="#ff9800",
                highlight_prefix="joint_face",
                center_mode="surface",
            )
            return

        self.alignment_point = np.array(center, dtype=float)
        self._alignment_point_is_manual = True
        if self.child_object in self.mw.robot.links:
            child_link = self.mw.robot.links[self.child_object]
            self._quick_preview_original_transform = child_link.t_world.copy()
            self.original_child_transform = child_link.t_world.copy()

        label = f"rotation face: {role} selected"
        index = self.rotation_face_combo.findText(label)
        if index < 0:
            self.rotation_face_combo.addItem(label, f"selected:{role}")
            index = self.rotation_face_combo.count() - 1
        self.rotation_face_combo.blockSignals(True)
        self.rotation_face_combo.setCurrentIndex(index)
        self.rotation_face_combo.blockSignals(False)

        self.mw.log(f"Rotation face center set at: {np.round(self.alignment_point, 4).tolist()}")
        self.show_joint_arrow(render=False)
        self.reset_joint_preview_to_initial()
        self.mw.canvas.update_transforms(self.mw.robot)

    def restore_quick_preview_transform(self):
        if (
            getattr(self, '_quick_preview_original_transform', None) is not None
            and self.child_object in self.mw.robot.links
            and self.child_object not in self.joints
        ):
            self.mw.robot.links[self.child_object].t_world = self._quick_preview_original_transform.copy()
            self.mw.canvas.update_transforms(self.mw.robot)

    def reset_joint_preview_to_initial(self):
        """Return the preview child to its zero pose whenever the selected axis changes."""
        if (
            not hasattr(self, 'original_child_transform')
            or not self.child_object
            or self.child_object not in self.mw.robot.links
            or self.child_object in self.joints
        ):
            return

        child_link = self.mw.robot.links[self.child_object]
        child_link.t_world = self.original_child_transform.copy()
        self.propagate_transform_recursive(child_link)

        if hasattr(self, 'rotation_slider'):
            self.rotation_slider.blockSignals(True)
            self.rotation_slider.setValue(0)
            self.rotation_slider.blockSignals(False)
        if hasattr(self, 'rotation_spinbox'):
            self.rotation_spinbox.blockSignals(True)
            self.rotation_spinbox.setValue(0)
            self.rotation_spinbox.blockSignals(False)
        if hasattr(self, 'quick_angle_spin'):
            self.quick_angle_spin.blockSignals(True)
            self.quick_angle_spin.setValue(0)
            self.quick_angle_spin.blockSignals(False)
        if hasattr(self, 'quick_joint_slider'):
            self.quick_joint_slider.blockSignals(True)
            self.quick_joint_slider.setValue(0)
            self.quick_joint_slider.blockSignals(False)

        self.show_joint_arrow(render=False)
        self.mw.canvas.update_transforms(self.mw.robot)

    def get_quick_joint_pivot(self):
        """Use the parent/child intersection center in the zero-pose preview state."""
        parent_name = self.parent_combo.currentData() if hasattr(self, 'parent_combo') else self.parent_object
        child_name = self.child_combo.currentData() if hasattr(self, 'child_combo') else self.child_object
        if not parent_name or not child_name:
            return None

        parent_link = self.mw.robot.links.get(parent_name)
        child_link = self.mw.robot.links.get(child_name)
        child_transform = None
        if (
            child_name == self.child_object
            and getattr(self, '_quick_preview_original_transform', None) is not None
        ):
            child_transform = self._quick_preview_original_transform

        interference_center = self.get_interference_center(
            parent_link,
            child_link,
            child_transform=child_transform,
        )
        if interference_center is not None:
            return interference_center

        return self.get_cached_alignment_pivot(parent_name, child_name)

    def get_cached_alignment_pivot(self, parent_name, child_name):
        """Prefer the midpoint of the exact faces used to align this parent/child pair."""
        if not hasattr(self.mw, "alignment_cache"):
            return None

        cache_entry = self.mw.alignment_cache.get((parent_name, child_name))
        if cache_entry is None:
            return None

        if not isinstance(cache_entry, dict):
            return np.array(cache_entry, dtype=float)

        parent_center = cache_entry.get("parent_pick_center", cache_entry.get("contact_point"))
        child_center = cache_entry.get("child_aligned_face_center", cache_entry.get("contact_point"))
        if parent_center is None and child_center is None:
            return None
        if parent_center is None:
            return np.array(child_center, dtype=float)
        if child_center is None:
            return np.array(parent_center, dtype=float)

        return (np.array(parent_center, dtype=float) + np.array(child_center, dtype=float)) / 2.0

    def prepare_quick_joint_preview(self):
        parent_name = self.parent_combo.currentData()
        child_name = self.child_combo.currentData()
        valid, _ = self.validate_joint_tree_choice(parent_name, child_name)
        if not valid:
            return False

        child_link = self.mw.robot.links[child_name]
        if self._quick_preview_original_transform is None:
            self._quick_preview_original_transform = child_link.t_world.copy()

        self.parent_object = parent_name
        self.child_object = child_name
        if self.alignment_point is None or not self._alignment_point_is_manual:
            self.alignment_point = self.get_quick_joint_pivot()
            self._alignment_point_is_manual = False
        if self.alignment_point is None:
            return False

        self.original_child_transform = self._quick_preview_original_transform.copy()
        return True

    def on_quick_slider_changed(self, value):
        if not hasattr(self, '_quick_preview_original_transform'):
            self._quick_preview_original_transform = None
        if not self.prepare_quick_joint_preview():
            return

        self.rotation_slider.blockSignals(True)
        self.rotation_slider.setValue(value)
        self.rotation_slider.blockSignals(False)
        self.rotation_spinbox.blockSignals(True)
        self.rotation_spinbox.setValue(value / 10.0)
        self.rotation_spinbox.blockSignals(False)
        if hasattr(self, 'quick_angle_spin'):
            self.quick_angle_spin.blockSignals(True)
            self.quick_angle_spin.setValue(value / 10.0)
            self.quick_angle_spin.blockSignals(False)

        self.test_rotation(value)

    def create_joint_from_quick_form(self):
        """Create a joint directly from the compact parent/child/axis form."""
        parent_name = self.parent_combo.currentData()
        child_name = self.child_combo.currentData()
        valid, message = self.validate_joint_tree_choice(parent_name, child_name)
        if not valid:
            self.mw.show_toast(message, "warning")
            self.mw.log(f"Error: {message}.")
            return

        if not self.prepare_quick_joint_preview():
            self.mw.show_toast("Could not place joint pivot", "warning")
            self.mw.log("Error: could not calculate a joint pivot for the selected links.")
            return

        axis_id = self.quick_axis_group.checkedId()
        self.axis_x_radio.setChecked(axis_id == 0)
        self.axis_y_radio.setChecked(axis_id == 1)
        self.axis_z_radio.setChecked(axis_id == 2)

        custom_name = self.quick_joint_name_input.text().strip()
        if not custom_name:
            custom_name = f"joint_{parent_name}_{child_name}"
        self.joint_name_input.setText(custom_name)
        self.sync_main_limits_from_quick()

        child_link = self.mw.robot.links[child_name]
        child_link.t_world = self.original_child_transform.copy()
        self.quick_joint_slider.blockSignals(True)
        self.quick_joint_slider.setValue(0)
        self.quick_joint_slider.blockSignals(False)
        self.rotation_slider.blockSignals(True)
        self.rotation_slider.setValue(0)
        self.rotation_slider.blockSignals(False)
        self.rotation_spinbox.blockSignals(True)
        self.rotation_spinbox.setValue(0)
        self.rotation_spinbox.blockSignals(False)
        if hasattr(self, 'quick_angle_spin'):
            self.quick_angle_spin.blockSignals(True)
            self.quick_angle_spin.setValue(0)
            self.quick_angle_spin.blockSignals(False)

        self.confirm_joint()
        self._quick_preview_original_transform = None
        self.quick_joint_name_input.clear()
        self.refresh_quick_link_combos()
        self.set_quick_combo_value(self.parent_combo, child_name)
        self.set_quick_combo_value(self.child_combo, None)
        self.parent_object = child_name
        self.child_object = None
        self.alignment_point = None
        self._alignment_point_is_manual = False
        self.update_pick_pivot_state()

    def refresh_joints_history(self):
        """Refresh the list of created joints with delete buttons"""
        self.joints_history_list.clear()
        self._gripper_group_control = self._build_gripper_group_control()
        has_joints = bool(self.joints)
        if hasattr(self, 'header_joints'):
            self.header_joints.setVisible(has_joints)
        if hasattr(self, 'joints_history_list'):
            self.joints_history_list.setVisible(has_joints)

        if not has_joints:
            return

        if self._gripper_group_control:
            self._add_gripper_group_row()
        
        for child_name, data in self.joints.items():
            if self._gripper_group_control and child_name in self._gripper_group_control["hidden_child_names"]:
                continue

            item = QtWidgets.QListWidgetItem()
            self.joints_history_list.addItem(item)
            
            # Create custom widget for the item
            widget = QtWidgets.QWidget()
            item_layout = QtWidgets.QHBoxLayout(widget)
            item_layout.setContentsMargins(12, 10, 12, 10)
            item_layout.setSpacing(10)
            
            # Label: Custom Name Only
            display_name = data.get('custom_name', f"{data['parent']} \u2192 {child_name}")
            joint_id = data.get('joint_id', child_name)
            
            label = QtWidgets.QLabel(display_name)
            label.setMaximumWidth(48)
            label.setStyleSheet("color: #212121; font-size: 15px; font-weight: bold;")
            item_layout.addWidget(label)

            # Rename Button
            rename_btn = QtWidgets.QPushButton("Edit")
            rename_btn.setFixedSize(34, 30)
            rename_btn.setCursor(QtCore.Qt.PointingHandCursor)
            rename_btn.setToolTip("Rename joint")
            rename_btn.setAccessibleName("Rename")
            rename_btn.setStyleSheet("""
                QPushButton {
                    background-color: white;
                    color: #757575;
                    border: 2px solid #e0e0e0;
                    border-radius: 15px;
                    font-size: 10px;
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
            
            # Keep the live slider usable in the narrow side panel.
            
            # Axis/Limits info small
            axis_names = {0: "X", 1: "Y", 2: "Z"}
            if data.get('joint_type') == "prismatic" and data.get('prismatic_plane'):
                info_text = f"{data['prismatic_plane'].upper()} {data.get('prismatic_angle', 0.0):.1f} deg"
            else:
                info_text = f"Axis: {axis_names[data['axis']]}"
            info = QtWidgets.QLabel(info_text)
            info.setMaximumWidth(52)
            info.setStyleSheet("color: #757575; font-size: 13px; font-weight: bold; margin-right: 5px;")
            item_layout.addWidget(info)

            joint_obj = self.mw.robot.joints.get(joint_id)
            joint_type = getattr(joint_obj, 'joint_type', data.get('joint_type', 'revolute'))
            min_val = float(getattr(joint_obj, 'min_limit', data.get('min', 0 if joint_type == "prismatic" else -180)))
            max_val = float(getattr(joint_obj, 'max_limit', data.get('max', 10 if joint_type == "prismatic" else 180)))
            if min_val >= max_val:
                min_val, max_val = (0.0, 10.0) if joint_type == "prismatic" else (-180.0, 180.0)
                data['min'] = min_val
                data['max'] = max_val
                if joint_obj is not None:
                    joint_obj.min_limit = min_val
                    joint_obj.max_limit = max_val
            current_value = float(self.get_joint_value(data))
            current_value = float(np.clip(current_value, min_val, max_val))

            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(int(min_val * 10), int(max_val * 10))
            slider.setValue(int(current_value * 10))
            slider.setSingleStep(10)
            slider.setPageStep(100)
            slider.setTracking(True)
            slider.setMinimumWidth(120)
            slider.setStyleSheet("""
                QSlider::groove:horizontal {
                    height: 7px;
                    background: #f0f0f0;
                    border: 1px solid #d0d0d0;
                    border-radius: 3px;
                }
                QSlider::sub-page:horizontal {
                    background: #bbdefb;
                    border-radius: 3px;
                }
                QSlider::handle:horizontal {
                    background: white;
                    border: 2px solid #1976d2;
                    width: 15px;
                    height: 15px;
                    margin-top: -5px;
                    margin-bottom: -5px;
                    border-radius: 7px;
                }
            """)
            item_layout.addWidget(slider, 1)

            value_spin = TypeOnlyDoubleSpinBox()
            value_spin.setRange(min_val, max_val)
            value_spin.setDecimals(1)
            value_spin.setValue(current_value)
            value_spin.setSuffix(self.joint_value_suffix(joint_type))
            value_spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
            value_spin.setFixedWidth(68)
            value_spin.setStyleSheet("""
                QDoubleSpinBox {
                    background: white;
                    color: #1976d2;
                    border: 1px solid #1976d2;
                    border-radius: 3px;
                    padding: 2px;
                    font-weight: bold;
                }
            """)
            item_layout.addWidget(value_spin)

            slider.valueChanged.connect(
                lambda value, spin=value_spin, name=child_name: self.on_history_joint_slider_changed(name, value, spin)
            )
            value_spin.valueChanged.connect(
                lambda value, s=slider, name=child_name: self.on_history_joint_spinbox_changed(name, value, s)
            )
            
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

    def _build_gripper_group_control(self):
        """Return a shared control model for all gripper joints, or None if not needed."""
        if not hasattr(self.mw, "robot") or self.mw.robot is None:
            return None

        gripper_child_names = []
        gripper_joint_ids = []
        child_name_by_joint_id = {}
        for child_name, data in self.joints.items():
            joint_id = data.get("joint_id", child_name)
            joint = self.mw.robot.joints.get(joint_id)
            if joint is None or not getattr(joint, "is_gripper", False):
                continue
            gripper_child_names.append(child_name)
            gripper_joint_ids.append(joint_id)
            child_name_by_joint_id[joint_id] = child_name

        if not gripper_joint_ids:
            return None

        slave_ids = {
            slave_id
            for slaves in self.mw.robot.joint_relations.values()
            for slave_id, _ratio in slaves
        }
        root_joint_ids = [joint_id for joint_id in gripper_joint_ids if joint_id not in slave_ids]
        if not root_joint_ids:
            root_joint_ids = list(dict.fromkeys(gripper_joint_ids))

        hidden_joint_ids = set()
        stack = list(root_joint_ids)
        while stack:
            joint_id = stack.pop()
            if joint_id in hidden_joint_ids:
                continue
            hidden_joint_ids.add(joint_id)
            for slave_id, _ratio in self.mw.robot.joint_relations.get(joint_id, []):
                stack.append(slave_id)

        hidden_joint_ids.update(gripper_joint_ids)
        hidden_child_names = [
            child_name
            for child_name, data in self.joints.items()
            if data.get("joint_id", child_name) in hidden_joint_ids
        ]

        root_child_names = [
            child_name_by_joint_id[joint_id]
            for joint_id in root_joint_ids
            if joint_id in child_name_by_joint_id
        ]

        if not root_child_names:
            return None

        return {
            "root_child_names": root_child_names,
            "hidden_child_names": set(hidden_child_names),
            "joint_ids": list(dict.fromkeys(gripper_joint_ids)),
        }

    def _add_gripper_group_row(self):
        """Add a single shared slider for the whole gripper assembly."""
        group = self._gripper_group_control
        if not group:
            return

        root_child_names = list(group.get("root_child_names", []))
        if not root_child_names:
            return

        item = QtWidgets.QListWidgetItem()
        self.joints_history_list.addItem(item)

        widget = QtWidgets.QWidget()
        item_layout = QtWidgets.QHBoxLayout(widget)
        item_layout.setContentsMargins(12, 10, 12, 10)
        item_layout.setSpacing(10)

        label = QtWidgets.QLabel("Gripper Slider")
        label.setMaximumWidth(120)
        label.setStyleSheet("color: #212121; font-size: 15px; font-weight: bold;")
        item_layout.addWidget(label)

        jaw_count = len(group.get("joint_ids", root_child_names))
        info = QtWidgets.QLabel(f"{jaw_count} linked jaws")
        info.setMaximumWidth(110)
        info.setStyleSheet("color: #757575; font-size: 13px; font-weight: bold; margin-right: 5px;")
        item_layout.addWidget(info)

        joint_names = []
        for child_name in root_child_names:
            data = self.joints.get(child_name, {})
            joint_id = data.get("joint_id", child_name)
            joint_obj = self.mw.robot.joints.get(joint_id)
            if joint_obj is None:
                continue
            joint_names.append(child_name)

        if not joint_names:
            return

        current_value = 0.0
        if hasattr(self.mw, "get_gripper_opening_percent"):
            current_value = float(self.mw.get_gripper_opening_percent())

        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(0, 100)
        slider.setValue(int(round(current_value)))
        slider.setSingleStep(1)
        slider.setPageStep(10)
        slider.setTracking(True)
        slider.setMinimumWidth(120)
        slider.setToolTip("0% closes all jaws; 100% opens all jaws")
        slider.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 7px;
                background: #f0f0f0;
                border: 1px solid #d0d0d0;
                border-radius: 3px;
            }
            QSlider::sub-page:horizontal {
                background: #bbdefb;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: white;
                border: 2px solid #1976d2;
                width: 15px;
                height: 15px;
                margin-top: -5px;
                margin-bottom: -5px;
                border-radius: 7px;
            }
        """)
        item_layout.addWidget(slider, 1)

        value_spin = TypeOnlyDoubleSpinBox()
        value_spin.setRange(0.0, 100.0)
        value_spin.setDecimals(0)
        value_spin.setValue(current_value)
        value_spin.setSuffix("%")
        value_spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        value_spin.setFixedWidth(68)
        value_spin.setStyleSheet("""
            QDoubleSpinBox {
                background: white;
                color: #1976d2;
                border: 1px solid #1976d2;
                border-radius: 3px;
                padding: 2px;
                font-weight: bold;
            }
        """)
        item_layout.addWidget(value_spin)

        slider.valueChanged.connect(
            lambda value, spin=value_spin, names=joint_names: self.on_gripper_group_slider_changed(names, value, spin)
        )
        value_spin.valueChanged.connect(
            lambda value, s=slider, names=joint_names: self.on_gripper_group_spinbox_changed(names, value, s)
        )

        note = QtWidgets.QLabel("all jaws")
        note.setFixedWidth(60)
        note.setAlignment(QtCore.Qt.AlignCenter)
        note.setStyleSheet("color: #757575; font-size: 11px; font-weight: bold;")
        item_layout.addWidget(note)

        item.setSizeHint(QtCore.QSize(0, 60))
        self.joints_history_list.setItemWidget(item, widget)
        self._gripper_group_control["slider"] = slider
        self._gripper_group_control["spinbox"] = value_spin

    def on_gripper_group_slider_changed(self, child_names, value, spinbox):
        opening_percent = float(value)
        self._gripper_group_syncing = True
        spinbox.blockSignals(True)
        spinbox.setValue(opening_percent)
        spinbox.blockSignals(False)

        if hasattr(self.mw, "set_gripper_opening_percent"):
            self.mw.set_gripper_opening_percent(opening_percent)
        else:
            for child_name in child_names:
                self.apply_joint_rotation(child_name, opening_percent)
        self._gripper_group_syncing = False

    def on_gripper_group_spinbox_changed(self, child_names, value, slider):
        slider_value = int(round(value))
        self._gripper_group_syncing = True
        slider.blockSignals(True)
        slider.setValue(slider_value)
        slider.blockSignals(False)

        if hasattr(self.mw, "set_gripper_opening_percent"):
            self.mw.set_gripper_opening_percent(float(value))
        else:
            for child_name in child_names:
                self.apply_joint_rotation(child_name, float(value))
        self._gripper_group_syncing = False

    def on_history_joint_slider_changed(self, child_name, value, spinbox):
        joint_value = value / 10.0
        spinbox.blockSignals(True)
        spinbox.setValue(joint_value)
        spinbox.blockSignals(False)
        self.apply_joint_rotation(child_name, joint_value)

    def on_history_joint_spinbox_changed(self, child_name, value, slider):
        slider_value = int(value * 10)
        slider.blockSignals(True)
        slider.setValue(slider_value)
        slider.blockSignals(False)
        self.apply_joint_rotation(child_name, value)

    def delete_joint(self, child_name):
        """Delete only the joint and keep the child component in place."""
        if child_name not in self.joints:
            return
            
        joint_data = self.joints[child_name]
        parent_name = joint_data['parent']
        joint_name = joint_data.get('joint_id', f"joint_{parent_name}_{child_name}")
        
        self.mw.log(f"Deleting joint: {joint_name}")
        
        child_link = self.mw.robot.links[child_name]
        child_world = np.array(child_link.t_world, dtype=float).copy()

        # 1. Remove from Robot Model Core
        self.mw.robot.remove_joint(joint_name)

        # 2. Preserve the current pose so deleting the joint does not make the
        # component jump back to its original import pose.
        child_link = self.mw.robot.links[child_name]
        child_link.t_offset = child_world.copy()
        child_link.t_world = child_world.copy()
        
        # 3. Remove from UI data structures
        del self.joints[child_name]
        
        # 3. If it was active in control, hide it
        if self.active_joint_control == child_name:
            self.joint_control_section.setVisible(False)
            self.active_joint_control = None
            
        # 4. Refresh UI
        self.refresh_links()
        self.refresh_joints_history()
        if hasattr(self.mw, "refresh_link_hierarchy"):
            self.mw.refresh_link_hierarchy()
        
        # Refresh Matrices Panel Sliders
        if hasattr(self.mw, 'matrices_tab'):
            self.mw.matrices_tab.refresh_sliders()
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
            if not new_id:
                self.mw.show_toast("Joint name cannot be empty", "warning")
                return
            if new_id != old_id and new_id in self.mw.robot.joints:
                self.mw.show_toast("Joint name already exists", "warning")
                return
            
            # 1. Update Robot Core dictionary if ID changed
            if new_id != old_id and old_id in self.mw.robot.joints:
                joint_obj = self.mw.robot.joints.pop(old_id)
                joint_obj.name = new_id
                self.mw.robot.joints[new_id] = joint_obj
                if old_id in self.mw.robot.joint_relations:
                    self.mw.robot.joint_relations[new_id] = self.mw.robot.joint_relations.pop(old_id)
                for master_id, slaves in list(self.mw.robot.joint_relations.items()):
                    self.mw.robot.joint_relations[master_id] = [
                        (new_id if slave_id == old_id else slave_id, ratio)
                        for slave_id, ratio in slaves
                    ]
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
            if hasattr(self.mw, "refresh_link_hierarchy"):
                self.mw.refresh_link_hierarchy()
            self.mw.log(f"Joint renamed to: {new_custom_name}")
            self.mw.show_toast(f"Renamed to {new_custom_name}", "success")

    def add_joint_relation_ui(self, master_child_name):
        """UI to add a relation between joints"""
        return
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

    def select_object(self, name):
        """Selection logic for external calls"""
        self.selected_object = name
        self.parent_btn.setEnabled(True)
        self.child_btn.setEnabled(True)
        self.update_pick_pivot_state()
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
        self.update_pick_pivot_state()
        
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
        self.update_pick_pivot_state()
        
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
            self.alignment_point = self.get_quick_joint_pivot()
            if self.current_joint_type() == "prismatic":
                center_point = self.get_prismatic_center_point()
                if center_point is not None:
                    self.alignment_point = center_point
            self._alignment_point_is_manual = False
            self.mw.log(f"Matched alignment point found for {pair}: {self.alignment_point}")
            self.create_joint()
        else:
            self.mw.log(f"No cached alignment found for {pair}. Pick a joint pivot or use the center point when creating the joint.")

    def pick_joint_pivot(self):
        """Allow the user to click a point in the 3D view for the joint pivot."""
        if not self.parent_object or not self.child_object:
            self.mw.log("Select both a parent and a child before picking a joint pivot.")
            self.mw.show_toast("Select parent and child first", "warning")
            return
        self.mw.log("Joint pivot pick mode active. Click a point on the child or parent in the 3D view.")
        self.mw.show_toast("Pick joint pivot point", "info")
        self.mw.canvas.start_point_picking(self.on_joint_point_picked)

    def on_joint_point_picked(self, point):
        self.alignment_point = np.array(point)
        self._alignment_point_is_manual = True
        self.mw.log(f"Joint pivot set at: {np.round(self.alignment_point, 4).tolist()}")
        if self.child_object in self.mw.robot.links:
            child_link = self.mw.robot.links[self.child_object]
            self.original_child_transform = child_link.t_world.copy()
        self.show_joint_arrow()
        self.update_pick_pivot_state()

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
        self.update_pick_pivot_state()
        
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
                self.active_joint_control = None
                self.mw.log(f"Joint '{object_name}' is a slave relation - control it via master joint.")
            else:
                self.active_joint_control = object_name
            
            # ALLOW jointed objects to be selected as parents!
            self.selected_object = object_name
            self.parent_btn.setEnabled(True)
            self.child_btn.setEnabled(False) # Still keep child disabled (one parent only)
        else:
            # Normal selection for parent/child assignment
            self.selected_object = object_name
            self.active_joint_control = None
            
            # Highlight in 3D view (yellow)
            self.mw.canvas.select_actor(self.selected_object)
            
            # Enable buttons
            self.parent_btn.setEnabled(True)
            self.child_btn.setEnabled(True)
            
            self.mw.log(f"Selected: {self.selected_object}")

    def current_joint_type(self):
        if hasattr(self, 'joint_type_combo'):
            return self.joint_type_combo.currentData() or "revolute"
        return "revolute"

    def joint_value_suffix(self, joint_type=None):
        return " cm" if (joint_type or self.current_joint_type()) == "prismatic" else "°"

    def joint_value_label(self, joint_type=None):
        return "Translation" if (joint_type or self.current_joint_type()) == "prismatic" else "Rotation"

    def get_joint_value(self, joint_data):
        return joint_data.get('current_value', joint_data.get('current_angle', 0.0))

    def prismatic_scene_units(self, value_cm):
        return value_cm * float(getattr(self.mw.canvas, "grid_units_per_cm", 10.0))

    def get_component_world_bounds(self, link, transform=None):
        """Return world-space AABB min/max for a transformed component mesh."""
        if link is None or link.mesh is None:
            return None
        matrix = transform if transform is not None else link.t_world
        try:
            corners = np.array([
                [x, y, z, 1.0]
                for x in (link.mesh.bounds[0][0], link.mesh.bounds[1][0])
                for y in (link.mesh.bounds[0][1], link.mesh.bounds[1][1])
                for z in (link.mesh.bounds[0][2], link.mesh.bounds[1][2])
            ])
            world_corners = (matrix @ corners.T).T[:, :3]
            return world_corners.min(axis=0), world_corners.max(axis=0)
        except Exception:
            pos = matrix[:3, 3].copy()
            return pos, pos

    def get_component_center(self, link, transform=None):
        """Return the world-space center of a component's transformed mesh bounds."""
        bounds = self.get_component_world_bounds(link, transform=transform)
        if bounds is None:
            return None
        bounds_min, bounds_max = bounds
        return (bounds_min + bounds_max) / 2.0

    def get_component_world_mesh(self, link, transform=None):
        """Return a copy of the component mesh in world coordinates."""
        if link is None or link.mesh is None:
            return None
        try:
            matrix = transform if transform is not None else link.t_world
            mesh = link.mesh.copy()
            mesh.apply_transform(matrix)
            return mesh
        except Exception:
            return None

    def points_in_bounds(self, points, bounds_min, bounds_max, tolerance=1e-6):
        if points is None or len(points) == 0:
            return np.empty((0, 3), dtype=float)
        pts = np.asarray(points, dtype=float)
        mask = np.all((pts >= bounds_min - tolerance) & (pts <= bounds_max + tolerance), axis=1)
        return pts[mask]

    def limited_mesh_points(self, mesh, limit=1600):
        """Return deterministic surface-ish mesh points without making large rows slow."""
        if mesh is None:
            return np.empty((0, 3), dtype=float)
        points = np.asarray(mesh.vertices, dtype=float)
        if len(points) > limit:
            step = max(1, len(points) // limit)
            points = points[::step][:limit]
        try:
            sample_count = min(limit, max(200, len(points) // 2))
            sampled = np.asarray(mesh.sample(sample_count), dtype=float)
            if len(sampled):
                points = np.vstack((points, sampled))
        except Exception:
            pass
        return points

    def nearest_surface_pairs(self, source_points, target_points):
        """Return nearest target point and distance for each source point."""
        if len(source_points) == 0 or len(target_points) == 0:
            return np.empty(0), np.empty((0, 3), dtype=float)

        try:
            from scipy.spatial import cKDTree
            distances, indices = cKDTree(target_points).query(source_points, k=1)
            return np.asarray(distances, dtype=float), target_points[indices]
        except Exception:
            distances = np.full(len(source_points), np.inf, dtype=float)
            nearest = np.empty_like(source_points)
            chunk_size = 256
            for start in range(0, len(source_points), chunk_size):
                stop = min(start + chunk_size, len(source_points))
                chunk = source_points[start:stop]
                diff = chunk[:, None, :] - target_points[None, :, :]
                dist2 = np.einsum("ijk,ijk->ij", diff, diff)
                indices = np.argmin(dist2, axis=1)
                distances[start:stop] = np.sqrt(dist2[np.arange(stop - start), indices])
                nearest[start:stop] = target_points[indices]
            return distances, nearest

    def get_surface_contact_center(self, parent_mesh, child_mesh):
        """Find the midpoint of the closest parent/child surface patch."""
        parent_points = self.limited_mesh_points(parent_mesh, limit=2600)
        child_points = self.limited_mesh_points(child_mesh, limit=2600)
        if len(parent_points) == 0 or len(child_points) == 0:
            return None

        parent_distances, parent_nearest_child = self.nearest_surface_pairs(parent_points, child_points)
        child_distances, child_nearest_parent = self.nearest_surface_pairs(child_points, parent_points)
        all_distances = np.concatenate((parent_distances, child_distances))
        all_distances = all_distances[np.isfinite(all_distances)]
        if len(all_distances) == 0:
            return None

        min_distance = float(np.min(all_distances))
        parent_bounds = np.array(parent_mesh.bounds, dtype=float)
        child_bounds = np.array(child_mesh.bounds, dtype=float)
        combined_min = np.minimum(parent_bounds[0], child_bounds[0])
        combined_max = np.maximum(parent_bounds[1], child_bounds[1])
        scene_diag = float(np.linalg.norm(combined_max - combined_min))
        band = max(1e-4, scene_diag * 0.006, min_distance * 0.35)
        threshold = min_distance + band

        parent_mask = parent_distances <= threshold
        child_mask = child_distances <= threshold
        contact_midpoints = []
        if np.any(parent_mask):
            contact_midpoints.append((parent_points[parent_mask] + parent_nearest_child[parent_mask]) / 2.0)
        if np.any(child_mask):
            contact_midpoints.append((child_points[child_mask] + child_nearest_parent[child_mask]) / 2.0)
        if not contact_midpoints:
            return None

        return np.vstack(contact_midpoints).mean(axis=0)

    def get_mesh_intersection_center(self, parent_link, child_link, parent_transform=None, child_transform=None):
        """Return a stable center for the parent/child contact or overlap region."""
        parent_mesh = self.get_component_world_mesh(parent_link, transform=parent_transform)
        child_mesh = self.get_component_world_mesh(child_link, transform=child_transform)
        if parent_mesh is None or child_mesh is None:
            return None

        parent_bounds = self.get_component_world_bounds(parent_link, transform=parent_transform)
        child_bounds = self.get_component_world_bounds(child_link, transform=child_transform)
        if parent_bounds is None or child_bounds is None:
            return None

        parent_min, parent_max = parent_bounds
        child_min, child_max = child_bounds
        overlap_min = np.maximum(parent_min, child_min)
        overlap_max = np.minimum(parent_max, child_max)
        overlap_size = overlap_max - overlap_min
        if np.all(overlap_size >= -1e-6):
            # The axis-aligned overlap box gives the exact center of the shared
            # parent/child region, avoiding noisy sampled surface centroids.
            return (overlap_min + overlap_max) / 2.0

        contact_center = self.get_surface_contact_center(parent_mesh, child_mesh)
        if contact_center is not None:
            return contact_center
        return None

    def get_interference_center(self, parent_link, child_link, parent_transform=None, child_transform=None):
        """Return the center of the actual parent/child mesh intersection."""
        return self.get_mesh_intersection_center(
            parent_link,
            child_link,
            parent_transform=parent_transform,
            child_transform=child_transform,
        )

    def refresh_alignment_point_from_intersection(self):
        """Move the rotational pivot to the current parent/child intersection."""
        if not self.parent_object or not self.child_object:
            return False

        parent_link = self.mw.robot.links.get(self.parent_object)
        child_link = self.mw.robot.links.get(self.child_object)
        child_transform = getattr(self, 'original_child_transform', None)
        pivot = self.get_interference_center(
            parent_link,
            child_link,
            child_transform=child_transform,
        )
        if pivot is None:
            pivot = self.get_cached_alignment_pivot(self.parent_object, self.child_object)
            if pivot is None:
                return False

        self.alignment_point = pivot
        return True

    def get_prismatic_center_point(self):
        """Center point between the aligned parent and child components."""
        if not self.parent_object or not self.child_object:
            return None
        parent_link = self.mw.robot.links.get(self.parent_object)
        child_link = self.mw.robot.links.get(self.child_object)
        parent_center = self.get_component_center(parent_link)
        child_center = self.get_component_center(child_link)
        if parent_center is None or child_center is None:
            return None
        return (parent_center + child_center) / 2.0

    def get_selected_local_axis(self):
        """Return the selected joint axis in the parent frame."""
        if self.current_joint_type() == "prismatic" and hasattr(self, 'prismatic_plane_combo'):
            angle = np.radians(self.prismatic_angle_spin.value())
            plane = self.prismatic_plane_combo.currentData() or "xy"
            if plane == "xy":
                axis = np.array([np.cos(angle), np.sin(angle), 0.0])
            elif plane == "yz":
                axis = np.array([0.0, np.cos(angle), np.sin(angle)])
            else:  # zx
                axis = np.array([np.sin(angle), 0.0, np.cos(angle)])
            return axis / (np.linalg.norm(axis) + 1e-9)

        if self.axis_x_radio.isChecked():
            return np.array([1.0, 0.0, 0.0])
        if self.axis_y_radio.isChecked():
            return np.array([0.0, 1.0, 0.0])
        return np.array([0.0, 0.0, 1.0])

    def axis_index_from_vector(self, axis_vec):
        """Best principal axis label for compact UI display."""
        return int(np.argmax(np.abs(axis_vec)))

    def on_prismatic_direction_changed(self, *_):
        self.show_joint_arrow()
        if hasattr(self, 'original_child_transform'):
            self.test_rotation(self.rotation_slider.value())

    def on_joint_axis_changed(self, checked):
        if not checked:
            return
        self.reset_joint_preview_to_initial()

    def on_prismatic_axis_radio(self, axis_name):
        if self.current_joint_type() != "prismatic" or not hasattr(self, 'prismatic_plane_combo'):
            return
        presets = {
            "x": ("xy", 0.0),
            "y": ("yz", 0.0),
            "z": ("zx", 0.0),
        }
        plane, angle = presets[axis_name]
        self.prismatic_plane_combo.blockSignals(True)
        plane_index = self.prismatic_plane_combo.findData(plane)
        self.prismatic_plane_combo.setCurrentIndex(plane_index if plane_index >= 0 else 0)
        self.prismatic_plane_combo.blockSignals(False)
        self.prismatic_angle_spin.blockSignals(True)
        self.prismatic_angle_spin.setValue(angle)
        self.prismatic_angle_spin.blockSignals(False)
        self.on_prismatic_direction_changed()

    def on_joint_type_changed(self, *_):
        joint_type = self.current_joint_type()
        label = self.joint_value_label(joint_type)

        if hasattr(self, "axis_x_radio") and hasattr(self, "axis_y_radio") and hasattr(self, "axis_z_radio"):
            self.axis_x_radio.setEnabled(True)
            self.axis_y_radio.setEnabled(True)
            self.axis_z_radio.setEnabled(True)

        self.update_joint_steps_box(joint_type)
        self.prismatic_direction_section.setVisible(joint_type == "prismatic")
        if joint_type == "prismatic":
            center_point = self.get_prismatic_center_point()
            if center_point is not None:
                self.alignment_point = center_point
        self.limits_label.setText(
            "Translation limits (cm):" if joint_type == "prismatic" else "Rotation limits (degrees):"
        )
        self.test_label.setText(f"Test {label.lower()}:")
        self.jc_slider_label.setText(f"{label}:")
        self.rotation_spinbox.setSuffix(self.joint_value_suffix(joint_type))
        self.joint_control_spinbox.setSuffix(self.joint_value_suffix(joint_type))

        self.min_limit_spin.blockSignals(True)
        self.max_limit_spin.blockSignals(True)
        if hasattr(self, 'quick_min_limit_spin') and hasattr(self, 'quick_max_limit_spin'):
            self.quick_min_limit_spin.blockSignals(True)
            self.quick_max_limit_spin.blockSignals(True)
        if joint_type == "prismatic":
            self.min_limit_spin.setRange(-10000, 10000)
            self.max_limit_spin.setRange(-10000, 10000)
            if hasattr(self, 'quick_min_limit_spin') and hasattr(self, 'quick_max_limit_spin'):
                self.quick_min_limit_spin.setRange(-10000, 10000)
                self.quick_max_limit_spin.setRange(-10000, 10000)
            if self.min_limit_spin.value() == -180 and self.max_limit_spin.value() == 180:
                self.min_limit_spin.setValue(0)
                self.max_limit_spin.setValue(10)
        else:
            self.min_limit_spin.setRange(-360, 360)
            self.max_limit_spin.setRange(-360, 360)
            if hasattr(self, 'quick_min_limit_spin') and hasattr(self, 'quick_max_limit_spin'):
                self.quick_min_limit_spin.setRange(-360, 360)
                self.quick_max_limit_spin.setRange(-360, 360)
            if self.min_limit_spin.value() == 0 and self.max_limit_spin.value() == 10:
                self.min_limit_spin.setValue(-180)
                self.max_limit_spin.setValue(180)
        self.min_limit_spin.blockSignals(False)
        self.max_limit_spin.blockSignals(False)
        if hasattr(self, 'quick_min_limit_spin') and hasattr(self, 'quick_max_limit_spin'):
            self.quick_min_limit_spin.blockSignals(False)
            self.quick_max_limit_spin.blockSignals(False)
        self.update_slider_range()
        self.show_joint_arrow()

    def update_joint_steps_box(self, joint_type=None):
        """Show joint-specific creation steps and kinematic interpretation."""
        joint_type = joint_type or self.current_joint_type()
        if joint_type == "prismatic":
            self.joint_steps_box.setText(
                "Prismatic joint creation steps:\n"
                "1. Select parent and child links.\n"
                "2. Pick the joint pivot or use the center point.\n"
                "3. Choose the slide axis: X, Y, or Z.\n"
                "4. Optionally choose XY, YZ, or ZX plane and enter the direction angle.\n"
                "5. Set translation limits in cm to match the 3D engine grid. The joint variable is d.\n"
                "6. Use Test translation to slide the child along the selected direction.\n"
                "7. Confirm Joint to lock rotations and allow only linear motion.\n\n"
                "Kinematic logic: joint_type = PRISMATIC, motion = linear, "
                "blocked = all rotations + translations perpendicular to the selected axis."
            )
        else:
            self.joint_steps_box.setText(
                "Revolute joint creation steps:\n"
                "1. Select parent and child links.\n"
                "2. Pick the joint pivot or use the center point.\n"
                "3. Choose the rotation axis: X, Y, or Z.\n"
                "4. Set rotation limits in degrees. The joint variable is theta.\n"
                "5. Use Test rotation to rotate the child around the selected axis.\n"
                "6. Confirm Joint to lock translations and allow only angular motion."
            )

    def show_joint_control(self, object_name):
        """Compatibility stub retained after removing the on-screen joint control card."""
        self.active_joint_control = object_name
        return

    def create_joint(self):
        """Create the joint between parent and child"""
        valid, message = self.validate_joint_tree_choice(self.parent_object, self.child_object)
        if not valid:
            self.mw.log(f"Error: {message}.")
            self.mw.show_toast(message, "warning")
            return

        child_link = self.mw.robot.links[self.child_object]
        self.original_child_transform = child_link.t_world.copy()

        if self.current_joint_type() == "prismatic":
            center_point = self.get_prismatic_center_point()
            if center_point is not None:
                self.alignment_point = center_point
                self._alignment_point_is_manual = False
        else:
            # Keep the exact pivot prepared by alignment/quick creation or picked
            # by the user. Recomputing here can move the hinge center slightly.
            if self.alignment_point is None:
                self.refresh_alignment_point_from_intersection()
                self._alignment_point_is_manual = False
        
        if self.alignment_point is None:
            self.mw.log("Error: parent/child intersection not found. Please align the components or pick a pivot point.")
            self.mw.show_toast("No parent/child intersection found", "warning")
            return

        self.mw.log(f"Creating joint between {self.parent_object} and {self.child_object}...")
        self.mw.log(f"Joint pivot at: {self.alignment_point}")
        
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
            if hasattr(self, 'quick_joint_slider'):
                self.quick_joint_slider.blockSignals(True)
                self.quick_joint_slider.setRange(min_val, max_val)
                self.quick_joint_slider.setValue(0)
                self.quick_joint_slider.blockSignals(False)
            if hasattr(self, 'quick_angle_spin'):
                self.quick_angle_spin.blockSignals(True)
                self.quick_angle_spin.setRange(self.min_limit_spin.value(), self.max_limit_spin.value())
                self.quick_angle_spin.setValue(0)
                self.quick_angle_spin.blockSignals(False)
            if hasattr(self, 'quick_min_limit_spin') and hasattr(self, 'quick_max_limit_spin'):
                self.quick_min_limit_spin.blockSignals(True)
                self.quick_max_limit_spin.blockSignals(True)
                self.quick_min_limit_spin.setValue(self.min_limit_spin.value())
                self.quick_max_limit_spin.setValue(self.max_limit_spin.value())
                self.quick_min_limit_spin.blockSignals(False)
                self.quick_max_limit_spin.blockSignals(False)
            
            # Also update spinbox range
            self.rotation_spinbox.setRange(self.min_limit_spin.value(), self.max_limit_spin.value())
            self.rotation_spinbox.setValue(0)

    def sync_main_limits_from_quick(self):
        """Copy compact form limits into the joint creation controls."""
        if not hasattr(self, 'min_limit_spin') or not hasattr(self, 'max_limit_spin'):
            return

        self.min_limit_spin.blockSignals(True)
        self.max_limit_spin.blockSignals(True)
        self.min_limit_spin.setValue(self.quick_min_limit_spin.value())
        self.max_limit_spin.setValue(self.quick_max_limit_spin.value())
        self.min_limit_spin.blockSignals(False)
        self.max_limit_spin.blockSignals(False)
        self.update_slider_range()

    def on_quick_limits_changed(self):
        """Keep compact limit fields in sync with preview and creation."""
        if not hasattr(self, 'quick_joint_slider'):
            return

        min_val = int(self.quick_min_limit_spin.value() * 10)
        max_val = int(self.quick_max_limit_spin.value() * 10)
        if min_val < max_val:
            self.quick_joint_slider.blockSignals(True)
            self.quick_joint_slider.setRange(min_val, max_val)
            self.quick_joint_slider.setValue(0)
            self.quick_joint_slider.blockSignals(False)
            if hasattr(self, 'quick_angle_spin'):
                self.quick_angle_spin.blockSignals(True)
                self.quick_angle_spin.setRange(self.quick_min_limit_spin.value(), self.quick_max_limit_spin.value())
                self.quick_angle_spin.setValue(0)
                self.quick_angle_spin.blockSignals(False)
            self.sync_main_limits_from_quick()

    def on_quick_angle_changed(self, value):
        """Jump the compact joint preview directly to a typed angle."""
        slider_value = int(round(value * 10))
        if hasattr(self, 'quick_joint_slider'):
            self.quick_joint_slider.blockSignals(True)
            self.quick_joint_slider.setValue(slider_value)
            self.quick_joint_slider.blockSignals(False)
        if hasattr(self, 'rotation_slider'):
            self.rotation_slider.blockSignals(True)
            self.rotation_slider.setValue(slider_value)
            self.rotation_slider.blockSignals(False)
        if hasattr(self, 'rotation_spinbox'):
            self.rotation_spinbox.blockSignals(True)
            self.rotation_spinbox.setValue(value)
            self.rotation_spinbox.blockSignals(False)
        self.test_rotation(slider_value)

    def on_slider_changed(self, value):
        """Called when slider value changes - update spinbox and rotate"""
        angle_deg = value / 10.0
        
        # Update spinbox without triggering its signal
        self.rotation_spinbox.blockSignals(True)
        self.rotation_spinbox.setValue(angle_deg)
        self.rotation_spinbox.blockSignals(False)

        if hasattr(self, 'quick_joint_slider'):
            self.quick_joint_slider.blockSignals(True)
            self.quick_joint_slider.setValue(value)
            self.quick_joint_slider.blockSignals(False)
        if hasattr(self, 'quick_angle_spin'):
            self.quick_angle_spin.blockSignals(True)
            self.quick_angle_spin.setValue(angle_deg)
            self.quick_angle_spin.blockSignals(False)
        
        # Apply rotation
        self.test_rotation(value)

    def on_spinbox_changed(self, value):
        """Called when spinbox value changes - update slider and rotate"""
        slider_value = int(value * 10)
        
        # Update slider without triggering its signal
        self.rotation_slider.blockSignals(True)
        self.rotation_slider.setValue(slider_value)
        self.rotation_slider.blockSignals(False)
        if hasattr(self, 'quick_joint_slider'):
            self.quick_joint_slider.blockSignals(True)
            self.quick_joint_slider.setValue(slider_value)
            self.quick_joint_slider.blockSignals(False)
        if hasattr(self, 'quick_angle_spin'):
            self.quick_angle_spin.blockSignals(True)
            self.quick_angle_spin.setValue(value)
            self.quick_angle_spin.blockSignals(False)
        self.test_rotation(slider_value)
        
    def test_rotation(self, value):
        """Test move the child object based on slider value"""
        if not hasattr(self, 'original_child_transform') or not self.child_object or self.child_object not in self.mw.robot.links:
            return
        
        joint_value = value / 10.0

        # 1. Get Parent Orientation
        parent_link = self.mw.robot.links[self.parent_object]
        child_link = self.mw.robot.links[self.child_object]
        if self.current_joint_type() == "revolute":
            local_axis = self.get_selected_local_axis()
            R_p = parent_link.t_world[:3, :3]
            axis = R_p @ local_axis
            axis = axis / (np.linalg.norm(axis) + 1e-9)

            angle_rad = np.radians(joint_value)
            K = np.array([
                [0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0]
            ])
            R3x3 = np.eye(3) + np.sin(angle_rad) * K + (1 - np.cos(angle_rad)) * (K @ K)
            R = np.eye(4)
            R[:3, :3] = R3x3

            T_to_origin = np.eye(4)
            T_to_origin[:3, 3] = -np.array(self.alignment_point, dtype=float)
            T_from_origin = np.eye(4)
            T_from_origin[:3, 3] = np.array(self.alignment_point, dtype=float)
            child_link.t_world = T_from_origin @ R @ T_to_origin @ self.original_child_transform
        else:
            local_axis = self.get_selected_local_axis()
            R_p = parent_link.t_world[:3, :3]
            axis = R_p @ local_axis
            axis = axis / (np.linalg.norm(axis) + 1e-9)
            T_slide = np.eye(4)
            T_slide[:3, 3] = axis * self.prismatic_scene_units(joint_value)
            child_link.t_world = T_slide @ self.original_child_transform
        
        # 5. Recursively update descendents for visual feedback
        self.propagate_transform_recursive(child_link)
        
        # 6. Update visual and guides in one render pass so the slider and 3D view stay in sync.
        self.show_joint_arrow(render=False)
        self.mw.canvas.update_transforms(self.mw.robot)

    def show_joint_arrow(self, render=True):
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
        
        # 2. Get the currently selected axis choice in the parent frame.
        local_axis = self.get_selected_local_axis()
            
        # Triangle orientation
        world_axis = R_p @ local_axis
        
        # --- SHOW RGB TRIAD (Local Parent Axes) ---
        triad_length = 0.5 * (self.mw.canvas.grid_units_per_cm)
        for i, color in enumerate(["red", "green", "blue"]):
            l_ax = np.zeros(3); l_ax[i] = 1
            w_ax = R_p @ l_ax
            line = pv.Line(self.alignment_point, self.alignment_point + w_ax * triad_length)
            self.mw.canvas.plotter.add_mesh(line, color=color, line_width=4, name=f"joint_triad_{'xyz'[i]}", pickable=False)

        # --- SHOW MAIN JOINT ARROW (Yellow) ---
        arrow = pv.Arrow(start=self.alignment_point, direction=world_axis, scale=0.8 * self.mw.canvas.grid_units_per_cm)
        self.mw.canvas.plotter.add_mesh(arrow, color="yellow", name="joint_arrow", pickable=False)
        if render:
            self.mw.canvas.plotter.render()

    def confirm_joint(self):
        """Finalize the joint with selected axis and limits"""
        # Cleanup triad before proceeding
        self.mw.canvas.plotter.remove_actor("joint_triad_x")
        self.mw.canvas.plotter.remove_actor("joint_triad_y")
        self.mw.canvas.plotter.remove_actor("joint_triad_z")

        # Get limits
        min_limit = self.min_limit_spin.value()
        max_limit = self.max_limit_spin.value()
        joint_type = self.current_joint_type()
        if min_limit >= max_limit:
            min_limit, max_limit = (0.0, 10.0) if joint_type == "prismatic" else (-180.0, 180.0)
            self.min_limit_spin.blockSignals(True)
            self.max_limit_spin.blockSignals(True)
            self.min_limit_spin.setValue(min_limit)
            self.max_limit_spin.setValue(max_limit)
            self.min_limit_spin.blockSignals(False)
            self.max_limit_spin.blockSignals(False)

        if joint_type == "prismatic":
            center_point = self.get_prismatic_center_point()
            if center_point is not None:
                self.alignment_point = center_point
                self._alignment_point_is_manual = False
        else:
            # Do not overwrite the selected revolute pivot at confirmation time.
            # The chosen point must remain the fixed joint origin.
            if self.alignment_point is None:
                self.refresh_alignment_point_from_intersection()
                self._alignment_point_is_manual = False

        local_axis_vec = self.get_selected_local_axis()
        axis = self.axis_index_from_vector(local_axis_vec)
        axis_names = ["X", "Y", "Z"]
        axis_name = axis_names[axis]
        
        child_link = self.mw.robot.links[self.child_object]
        parent_link = self.mw.robot.links[self.parent_object]
        zero_child_world = np.array(
            getattr(self, 'original_child_transform', child_link.t_world),
            dtype=float,
        ).copy()
        zero_parent_world = np.array(parent_link.t_world, dtype=float).copy()
        
        # Get custom name and sanitize
        custom_name = self.joint_name_input.text().strip()
        if not custom_name:
            custom_name = f"joint_{self.parent_object}_{self.child_object}"
            
        # Robust sanitization: Only replace spaces. Let other chars (like -) stay.
        joint_id = custom_name.replace(" ", "_").replace("/", "_")
        
        # Check for duplicates or empty
        if not joint_id: joint_id = f"joint_{len(self.mw.robot.joints)}"
        
        # --- 1. PROPERLY ADD TO ROBOT MODEL ---
        joint = self.mw.robot.add_joint(joint_id, self.parent_object, self.child_object, joint_type=joint_type)
        
        # Calculate pivot point in Parent's Local Frame
        # Math: P_parent = inv(T_parent_world) * P_world
        t_parent_inv = np.linalg.inv(zero_parent_world)
        pivot_local = (t_parent_inv @ np.append(self.alignment_point, 1))[:3]
        joint.origin = pivot_local
        
        joint.linear_units_per_cm = float(getattr(self.mw.canvas, "grid_units_per_cm", 10.0))

        if joint_type == "revolute":
            joint.axis = local_axis_vec
            joint.axis_name = axis_name
            child_link.t_offset = t_parent_inv @ zero_child_world
        else:
            # Keep the existing prismatic motion model for now.
            joint.axis = local_axis_vec
            joint.axis_name = axis_name # "X", "Y", or "Z"
            # Set Child Static Offset (relative to parent at 0 degrees)
            # Math: Child_Offset = inv(Parent_World) * Original_Aligned_Child_World
            # IMPORTANT: Use original_child_transform to ensure 0 deg = perfectly aligned position
            child_link.t_offset = t_parent_inv @ zero_child_world
        
        # Set Joint Limits
        joint.min_limit = min_limit
        joint.max_limit = max_limit
        joint.current_value = 0.0
        self.rigidize_cached_alignments()
        self.mw.robot.update_kinematics()
        self.mw.canvas.update_transforms(self.mw.robot, render=False)
        
        # Calculate and Store the current WORLD axis for verification and DH tracking
        world_axis_vec = parent_link.t_world[:3, :3] @ local_axis_vec

        # --- 2. LOCAL STORAGE AND LOGGING ---
        # Store for UI tracking and Persistence
        self.joints[self.child_object] = {
            'parent': self.parent_object,
            'axis': axis, # Selection index (X=0, Y=1, Z=2)
            'local_axis_vector': local_axis_vec.tolist(),
            'world_axis_vector': world_axis_vec.tolist(),
            'joint_type': joint_type,
            'prismatic_plane': self.prismatic_plane_combo.currentData() if joint_type == "prismatic" else None,
            'prismatic_angle': self.prismatic_angle_spin.value() if joint_type == "prismatic" else 0.0,
            'min': min_limit,
            'max': max_limit,
            'current_value': 0.0,
            'current_angle': 0.0,
            'alignment_point': self.alignment_point.tolist() if isinstance(self.alignment_point, np.ndarray) else self.alignment_point,
            'custom_name': custom_name,
            'joint_id': joint_id
        }
        
        self.mw.log(f"Joint confirmed and added to Robot model (ID: {joint_id})")
        
        # --- 3. AUTO-APPEND TO CODE EDITOR ---
        new_cmd = f"{joint_id} 0"
        if hasattr(self.mw, 'program_tab'):
            current_code = self.mw.program_tab.code_edit.toPlainText()
            # If default text is there, clear it or append
            if "Example Program" in current_code and len(current_code.splitlines()) < 10:
                self.mw.program_tab.code_edit.appendPlainText(new_cmd)
            else:
                self.mw.program_tab.code_edit.appendPlainText(new_cmd)
            self.mw.log(f"Auto-generated code: '{new_cmd}' added to Code tab.")
        self.mw.log(f"  Parent: {self.parent_object}")
        self.mw.log(f"  Child: {self.child_object}")
        self.mw.log(f"  Type: {joint_type.capitalize()}")
        self.mw.log(f"  Axis: {axis_name} ({np.round(local_axis_vec, 4).tolist()})")
        self.mw.log(f"  Limits: {min_limit}{self.joint_value_suffix(joint_type)} to {max_limit}{self.joint_value_suffix(joint_type)}")
        if joint_type == "prismatic":
            plane = (self.prismatic_plane_combo.currentData() or "xy").upper()
            angle = self.prismatic_angle_spin.value()
            self.mw.log(f"  Variable: d (linear displacement in cm along {plane} angle {angle:.1f} deg)")
            self.mw.log(f"  Engine scale: 1 cm = {joint.linear_units_per_cm:g} scene units")
            self.mw.log(f"  Motion: child link slides along the parent-local direction vector; rotations are blocked.")
        else:
            self.mw.log(f"  Variable: theta (angular displacement about {axis_name})")
            self.mw.log(f"  Motion: child link rotates about parent-local {axis_name}; translations are blocked.")
        self.mw.log(f"  Pivot: {self.alignment_point}")
        
        # Remove arrow
        self.mw.canvas.plotter.remove_actor("joint_arrow")
        self.mw.canvas.plotter.render()
        
        # Reset UI
        self.reset_joint_ui()
        
        # Refresh joints list
        self.refresh_joints_history()
        if hasattr(self.mw, "refresh_link_hierarchy"):
            self.mw.refresh_link_hierarchy()
        
        # Refresh Matrices Panel Sliders
        if hasattr(self.mw, 'matrices_tab'):
            self.mw.matrices_tab.refresh_sliders()
        
        self.mw.show_toast(f"Joint '{custom_name}' created", "success")

    def on_joint_control_changed(self, value):
        """Handle joint control slider changes"""
        if not self.active_joint_control:
            return
        
        joint_value = value / 10.0
        
        # Update spinbox
        self.joint_control_spinbox.blockSignals(True)
        self.joint_control_spinbox.setValue(joint_value)
        self.joint_control_spinbox.blockSignals(False)
        
        # Apply value to joint
        self.apply_joint_rotation(self.active_joint_control, joint_value)

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

    def apply_joint_rotation(self, child_name, joint_value):
        """Apply a value to a jointed object using the Robot core kinematics"""
        if child_name not in self.mw.robot.links:
            return

        self.rigidize_cached_alignments()
            
        child_link = self.mw.robot.links[child_name]
        joint = child_link.parent_joint
        
        if joint:
            # 1. Update the robot model state
            joint.current_value = joint_value
            
            # 2. Trigger re-calculation of all world transforms
            self.mw.robot.update_kinematics()

            # 2b. Any free components that are touching this moving part become rigid followers.
            rigid_created = self.rigidize_touching_free_components(child_link)
            if rigid_created:
                self.mw.log(
                    f"Rigid follow-through: {rigid_created} touching component(s) were fixed to '{child_name}'."
                )
            
            # 3. Synchronize local JointPanel data
            if child_name in self.joints:
                self.joints[child_name]['current_value'] = joint_value
                self.joints[child_name]['current_angle'] = joint_value
                # Ensure the pivot point for show_joint_arrow is updated from current robot state
                # Pivot in parent local frame -> world
                p_l = joint.parent_link
                self.parent_object = p_l.name
                self.alignment_point = (p_l.t_world @ np.append(joint.origin, 1.0))[:3]
                self.show_joint_arrow()
                
            # 4. Synchronize MatricesPanel if it exists
            if hasattr(self.mw, 'matrices_tab'):
                self.mw.matrices_tab.sync_slider(child_name, joint_value)

            if self._gripper_group_control and not self._gripper_group_syncing:
                controlled_names = set(self._gripper_group_control.get("root_child_names", [])) | set(
                    self._gripper_group_control.get("hidden_child_names", set())
                )
                if child_name in controlled_names:
                    slider = self._gripper_group_control.get("slider")
                    spinbox = self._gripper_group_control.get("spinbox")
                    opening_percent = float(joint_value)
                    if hasattr(self.mw, "get_gripper_opening_percent"):
                        opening_percent = float(self.mw.get_gripper_opening_percent())
                    if slider is not None:
                        slider.blockSignals(True)
                        slider.setValue(int(round(opening_percent)))
                        slider.blockSignals(False)
                    if spinbox is not None:
                        spinbox.blockSignals(True)
                        spinbox.setValue(opening_percent)
                        spinbox.blockSignals(False)
                
            # 5. Send command to hardware (ESP32)
            if hasattr(self.mw, 'serial_mgr'):
                # Use joint_id (e.g. joint_1) instead of display name for code consistency
                joint_id = self.joints[child_name].get('joint_id', child_name)
                # Send with current global speed
                speed = float(getattr(self.mw, 'current_speed', 0))
                self.mw.serial_mgr.send_command(joint_id, joint_value, speed=speed)
                
            # 6. Show Speed Overlay on 3D Canvas
            if hasattr(self.mw, 'show_speed_overlay'):
                self.mw.show_speed_overlay()
                
            # 7. Push updated transforms to the 3D viewer
            self.mw.canvas.update_transforms(self.mw.robot)
            
            # 7b. Update Live Point (LP) coordinates UI
            if hasattr(self.mw, 'update_live_ui'):
                self.mw.update_live_ui()

            # 8. Propagate to related joints
            joint_id = self.joints[child_name].get('joint_id', child_name)
            if joint_id in self.mw.robot.joint_relations:
                for slave_id, ratio in self.mw.robot.joint_relations[joint_id]:
                    slave_angle = joint_value * ratio
                    # Find which child link this slave_id belongs to
                    slave_child_name = None
                    for c_n, data in self.joints.items():
                        if data.get('joint_id') == slave_id:
                            slave_child_name = c_n
                            break
                    
                    if slave_child_name:
                        # Avoid infinite recursion if there are circular relations (though we should avoid them)
                        # We use a simpler update for slaves to avoid re-triggering this method
                        slave_joint = self.mw.robot.joints.get(slave_id)
                        if slave_joint:
                            slave_joint.current_value = slave_angle
                            self.joints[slave_child_name]['current_value'] = slave_angle
                            self.joints[slave_child_name]['current_angle'] = slave_angle
                            
                            # Update MatricesPanel if it exists
                            if hasattr(self.mw, 'matrices_tab'):
                                self.mw.matrices_tab.sync_slider(slave_child_name, slave_angle)
                
                # After updating all slaves, re-calc kinematics and update canvas once
                self.mw.robot.update_kinematics()
                self.mw.canvas.update_transforms(self.mw.robot)

            if hasattr(self.mw, "refresh_link_hierarchy"):
                self.mw.refresh_link_hierarchy()

    def propagate_transform_recursive(self, parent_link):
        """Correctly propagate t_world updates down children when a joint is NOT yet in robot.joints."""
        for joint in parent_link.child_joints:
            child = joint.child_link
            # joint_matrix = T(o) @ R @ T(-o)
            joint_matrix = joint.get_matrix()
            child.t_world = parent_link.t_world @ joint_matrix @ child.t_offset
            self.propagate_transform_recursive(child)

    def reset_joint_ui(self):
        """Reset the joint creation UI"""
        self.parent_object = None
        self.child_object = None
        self.alignment_point = None
        self._alignment_point_is_manual = False
        
        self.axis_section.setVisible(False)
        self.rotation_section.setVisible(False)
        self.pick_pivot_btn.setEnabled(False)
        if hasattr(self, 'header_joints'):
            self.header_joints.setVisible(False)
        if hasattr(self, 'joints_history_list'):
            self.joints_history_list.setVisible(False)
        
        self.refresh_links()
        self.update_pick_pivot_state()
        self.mw.log("Joint creation complete. Ready for next joint.")

    def update_pick_pivot_state(self):
        """Enable pivot picking only when both parent and child are selected."""
        enabled = bool(self.parent_object and self.child_object)
        if hasattr(self, 'pick_pivot_btn'):
            self.pick_pivot_btn.setEnabled(enabled)

    def refresh_links(self):
        """Refresh the object list with role indicators"""
        self.refresh_quick_link_combos()
        self.objects_list.clear()
        robot = getattr(self.mw, "robot", None)
        links = getattr(robot, "links", {})
        
        # Get all links from robot
        for name in links.keys():
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
                index = list(links.keys()).index(name)
                if index % 2 == 0:
                    item.setForeground(QtGui.QColor("#d32f2f"))  # Red
                else:
                    item.setForeground(QtGui.QColor("#1976d2"))  # Blue
            
            self.objects_list.addItem(item)

