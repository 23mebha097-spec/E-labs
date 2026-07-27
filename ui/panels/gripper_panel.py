from PyQt5 import QtWidgets, QtCore, QtGui
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

    def _coord_spinbox_style(self, color):
        return f"""
            QDoubleSpinBox {{
                background: white;
                color: {color};
                border: 1px solid #ddd;
                border-radius: 4px;
                font-size: 12px;
                padding: 4px 6px;
                font-weight: bold;
            }}
            QDoubleSpinBox:focus {{
                border-color: {color};
            }}
        """

    def _create_coord_spinbox(self, color="#1565c0"):
        sb = TypeOnlyDoubleSpinBox()
        sb.setRange(-9999, 9999)
        sb.setDecimals(2)
        sb.setSuffix(" cm")
        sb.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        sb.setStyleSheet(self._coord_spinbox_style(color))
        return sb

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(12)

        header = QtWidgets.QLabel("END-EFFECTOR CONTROL")
        header.setStyleSheet(
            "font-weight: bold; font-size: 16px; color: #2e7d32; margin-bottom: 5px;"
        )
        layout.addWidget(header)

        tool_group = QtWidgets.QGroupBox("1. SELECT TOOL")
        tool_group.setStyleSheet(self._group_style())
        tool_layout = QtWidgets.QHBoxLayout(tool_group)

        self.tool_combo = QtWidgets.QComboBox()
        self.tool_combo.addItems(["Gripper Tool", "Welding Tool", "Painting Tool"])
        self.tool_combo.setFixedHeight(32)
        self.tool_combo.setStyleSheet("font-size: 13px; padding: 5px;")
        tool_layout.addWidget(self.tool_combo)
        # Preview selection text, but apply UI changes only after OK is clicked
        self.tool_combo.currentIndexChanged.connect(
            lambda _index: self.tool_selection_status.setText(
                f"Selected: {self.tool_combo.currentText()} (press OK)"
            )
        )

        self.tool_ok_btn = QtWidgets.QPushButton("OK")
        self.tool_ok_btn.setFixedSize(80, 32)
        self.tool_ok_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.tool_ok_btn.setStyleSheet("""
            QPushButton {
                background-color: #2e7d32;
                color: white;
                border: none;
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #1b5e20; }
            QPushButton:pressed { background-color: #145a2d; }
        """)
        self.tool_ok_btn.clicked.connect(self.on_select_tool_ok)
        tool_layout.addWidget(self.tool_ok_btn)

        self.tool_selection_status = QtWidgets.QLabel("No tool selected")
        self.tool_selection_status.setStyleSheet("color: #616161; font-size: 12px; margin-left: 10px;")
        tool_layout.addWidget(self.tool_selection_status)

        layout.addWidget(tool_group)

        self.end_effector_summary_group = QtWidgets.QGroupBox("END-EFFECTOR SUMMARY")
        self.end_effector_summary_group.setStyleSheet(self._group_style())
        summary_layout = QtWidgets.QVBoxLayout(self.end_effector_summary_group)

        self.end_effector_summary_tool = QtWidgets.QLabel("selected tool: -")
        self.end_effector_summary_tool.setStyleSheet("color: #2e7d32; font-size: 14px; font-weight: bold;")
        summary_layout.addWidget(self.end_effector_summary_tool)

        self.end_effector_summary_detail = QtWidgets.QLabel("tool info: -")
        self.end_effector_summary_detail.setStyleSheet("color: #2e7d32; font-size: 14px; font-weight: bold;")
        summary_layout.addWidget(self.end_effector_summary_detail)

        self.end_effector_summary_note = QtWidgets.QLabel("Save a tool to lock in the summary.")
        self.end_effector_summary_note.setWordWrap(True)
        self.end_effector_summary_note.setStyleSheet("color: #616161; font-size: 12px;")
        summary_layout.addWidget(self.end_effector_summary_note)

        self.delete_end_effector_btn = QtWidgets.QPushButton("Delete End-Effector")
        self.delete_end_effector_btn.setFixedHeight(34)
        self.delete_end_effector_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.delete_end_effector_btn.setToolTip(
            "Remove the saved end-effector configuration and choose another tool"
        )
        self.delete_end_effector_btn.setStyleSheet("""
            QPushButton {
                background-color: white;
                color: #c62828;
                border: 1px solid #c62828;
                border-radius: 6px;
                font-weight: bold;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #ffebee;
                border-color: #b71c1c;
            }
            QPushButton:pressed {
                background-color: #ffcdd2;
            }
        """)
        self.delete_end_effector_btn.clicked.connect(self.on_delete_end_effector)
        summary_layout.addWidget(self.delete_end_effector_btn)

        self.end_effector_summary_group.setVisible(False)
        layout.addWidget(self.end_effector_summary_group)

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
        self.make_robo_btn.setVisible(False)
        layout.addWidget(self.make_robo_btn)

        self.selection_group = QtWidgets.QGroupBox("2. SELECT JOINTS")
        self.selection_group.setStyleSheet(self._group_style())
        sel_layout = QtWidgets.QVBoxLayout(self.selection_group)

        self.joints_list = QtWidgets.QListWidget()
        self.joints_list.setFixedHeight(120)
        self.joints_list.setSelectionMode(QtWidgets.QAbstractItemView.MultiSelection)
        self.joints_list.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.joints_list.setStyleSheet(self._surface_list_style())
        self.joints_list.itemClicked.connect(self.on_joint_selected)
        self.joints_list.itemSelectionChanged.connect(self.on_joint_list_selection_changed)
        sel_layout.addWidget(self.joints_list)

        self.gripper_compile_btn = QtWidgets.QPushButton("Compile")
        self.gripper_compile_btn.setFixedHeight(34)
        self.gripper_compile_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.gripper_compile_btn.setStyleSheet("""
            QPushButton {
                background-color: #1976d2;
                color: white;
                border: none;
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #1565c0; }
            QPushButton:pressed { background-color: #0d47a1; }
        """)
        self.gripper_compile_btn.clicked.connect(self.on_gripper_compile_clicked)
        sel_layout.addWidget(self.gripper_compile_btn)

        self.gripper_face_status = QtWidgets.QLabel(
            "Select the jaw joints and press Compile. Contact faces are detected automatically."
        )
        self.gripper_face_status.setWordWrap(True)
        self.gripper_face_status.setStyleSheet("color: #616161; font-size: 12px;")
        sel_layout.addWidget(self.gripper_face_status)

        self.face_selection_table = QtWidgets.QTableWidget(0, 2)
        self.face_selection_table.setHorizontalHeaderLabels(["Joint", "Selected Face"])
        self.face_selection_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.face_selection_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.face_selection_table.setFocusPolicy(QtCore.Qt.NoFocus)
        self.face_selection_table.setMinimumHeight(120)
        self.face_selection_table.verticalHeader().setVisible(False)
        self.face_selection_table.verticalHeader().setDefaultSectionSize(38)
        self.face_selection_table.horizontalHeader().setStretchLastSection(True)
        self.face_selection_table.setStyleSheet("""
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
        sel_layout.addWidget(self.face_selection_table)

        self.gripper_alignment_btn = QtWidgets.QPushButton("Select Gripping / Alignment Face")
        self.gripper_alignment_btn.setFixedHeight(34)
        self.gripper_alignment_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.gripper_alignment_btn.setToolTip(
            "Select a gripper-tool face manually. Its center becomes the Live Point and "
            "its plane remains parallel to the object's base during Pick & Place."
        )
        self.gripper_alignment_btn.setStyleSheet("""
            QPushButton {
                background-color: white;
                color: #1565c0;
                border: 1px solid #1976d2;
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #e3f2fd; }
            QPushButton:pressed { background-color: #bbdefb; }
            QPushButton:disabled { color: #9e9e9e; border-color: #bdbdbd; }
        """)
        self.gripper_alignment_btn.clicked.connect(self.on_select_gripper_alignment_face)
        self.gripper_alignment_btn.setEnabled(False)
        sel_layout.addWidget(self.gripper_alignment_btn)

        self.gripper_alignment_status = QtWidgets.QLabel(
            "Gripping face and Live Point: not selected"
        )
        self.gripper_alignment_status.setWordWrap(True)
        self.gripper_alignment_status.setStyleSheet("color: #616161; font-size: 12px;")
        sel_layout.addWidget(self.gripper_alignment_status)

        self.gripper_live_point_preview = QtWidgets.QLabel("Live Point Preview: not set")
        self.gripper_live_point_preview.setWordWrap(True)
        self.gripper_live_point_preview.setStyleSheet("color: #2e7d32; font-size: 12px; font-weight: bold;")
        sel_layout.addWidget(self.gripper_live_point_preview)

        self.gripper_opening_label = QtWidgets.QLabel("Opening Preview")
        self.gripper_opening_label.setStyleSheet("color: #616161; font-size: 12px; font-weight: bold;")
        sel_layout.addWidget(self.gripper_opening_label)

        min_opening_layout = QtWidgets.QHBoxLayout()
        min_opening_layout.addWidget(QtWidgets.QLabel("Min Opening (deg)"))
        self.gripper_min_input = QtWidgets.QSpinBox()
        self.gripper_min_input.setRange(0, 180)
        self.gripper_min_input.setValue(0)
        self.gripper_min_input.valueChanged.connect(self.on_gripper_opening_slider_changed)
        min_opening_layout.addWidget(self.gripper_min_input)
        sel_layout.addLayout(min_opening_layout)

        max_opening_layout = QtWidgets.QHBoxLayout()
        max_opening_layout.addWidget(QtWidgets.QLabel("Max Opening (deg)"))
        self.gripper_max_input = QtWidgets.QSpinBox()
        self.gripper_max_input.setRange(0, 180)
        self.gripper_max_input.setValue(40)
        self.gripper_max_input.valueChanged.connect(self.on_gripper_opening_slider_changed)
        max_opening_layout.addWidget(self.gripper_max_input)
        sel_layout.addLayout(max_opening_layout)

        self.gripper_save_btn = QtWidgets.QPushButton("Save")
        self.gripper_save_btn.setFixedHeight(34)
        self.gripper_save_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.gripper_save_btn.setStyleSheet("""
            QPushButton {
                background-color: #2e7d32;
                color: white;
                border: none;
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #1b5e20; }
            QPushButton:pressed { background-color: #145a2d; }
        """)
        self.gripper_save_btn.clicked.connect(self.on_gripper_save_clicked)
        self.gripper_save_btn.setEnabled(False)
        sel_layout.addWidget(self.gripper_save_btn)
        self.gripper_save_btn.setVisible(False)

        self.mark_gripper_check = QtWidgets.QCheckBox("Mark as Gripper")
        self.mark_gripper_check.setStyleSheet(
            "font-weight: bold; color: #2e7d32; padding: 5px;"
        )
        self.mark_gripper_check.toggled.connect(self.on_mark_toggled)
        self.mark_gripper_check.setVisible(False)
        sel_layout.addWidget(self.mark_gripper_check)

        self._gripper_face_selection_queue = []
        self._pending_gripper_contact_joint_name = None
        self._gripper_face_selection_data = {}
        self._gripper_selection_joint_names = []
        self._gripper_live_point_world = None
        self._gripper_joint_endpoints = {}
        self._gripper_alignment_face_data = None
        self._gripper_tool_selected = False
        self._gripper_confirmation_mode = False

        layout.addWidget(self.selection_group)

        self.control_group = QtWidgets.QGroupBox("2. MANUAL ACTIONS")
        self.control_group.setStyleSheet(self._group_style())
        ctrl_layout = QtWidgets.QVBoxLayout(self.control_group)

        self.gripper_opening_control_label = QtWidgets.QLabel("Gripper Opening (all jaws): 0% - Closed")
        self.gripper_opening_control_label.setStyleSheet("font-weight: bold; color: #2e7d32;")
        ctrl_layout.addWidget(self.gripper_opening_control_label)
        self.stroke_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.stroke_slider.setRange(0, 100)
        self.stroke_slider.setToolTip("One shared control: 0% closes every jaw and 100% opens every jaw")
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

        layout.addWidget(self.control_group)


        # --- WELDING TOOL SETUP (visible only when Welding Tool selected) ---
        self.welding_group = QtWidgets.QGroupBox("WELDING TOOL SETUP")
        self.welding_group.setStyleSheet(self._group_style())
        weld_layout = QtWidgets.QVBoxLayout(self.welding_group)

        face_label = QtWidgets.QLabel("Selected Face (Live Point)")
        face_label.setStyleSheet("font-weight: bold; color: #424242;")
        weld_layout.addWidget(face_label)

        self.weld_face_name = QtWidgets.QLineEdit()
        self.weld_face_name.setPlaceholderText("Face_12")
        self.weld_face_name.setFixedHeight(28)
        weld_layout.addWidget(self.weld_face_name)

        self.weld_compile_btn = QtWidgets.QPushButton("Compile")
        self.weld_compile_btn.setFixedHeight(32)
        self.weld_compile_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.weld_compile_btn.setStyleSheet("""
            QPushButton {
                background-color: #1976d2;
                color: white;
                border: none;
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #1565c0; }
            QPushButton:pressed { background-color: #0d47a1; }
        """)
        self.weld_compile_btn.clicked.connect(self.on_weld_tool_compile)
        weld_layout.addWidget(self.weld_compile_btn)

        self.weld_compile_status = QtWidgets.QLabel("Enter the tool filename and press Compile.")
        self.weld_compile_status.setStyleSheet("color: #616161; font-size: 12px; margin-top: 4px;")
        weld_layout.addWidget(self.weld_compile_status)

        # 2. Tool Direction
        dir_label = QtWidgets.QLabel("2. Tool Direction")
        dir_label.setStyleSheet("font-weight: bold; color: #424242;")
        weld_layout.addWidget(dir_label)

        self.weld_dir_combo = QtWidgets.QComboBox()
        self.weld_dir_combo.addItems(["Normal of Selected Face"])
        self.weld_dir_combo.setFixedHeight(28)
        weld_layout.addWidget(self.weld_dir_combo)

        # 3. Tool Information (Auto Calculated)
        info_label = QtWidgets.QLabel("3. Tool Information (Auto Calculated)")
        info_label.setStyleSheet("font-weight: bold; color: #424242;")
        weld_layout.addWidget(info_label)

        coord_layout = QtWidgets.QGridLayout()
        coord_layout.setSpacing(6)
        coord_layout.addWidget(QtWidgets.QLabel("Live Point (TCP) Position (mm)"), 0, 0, 1, 3)
        self.weld_tcp_x = QtWidgets.QLineEdit(); self.weld_tcp_y = QtWidgets.QLineEdit(); self.weld_tcp_z = QtWidgets.QLineEdit()
        for widget in (self.weld_tcp_x, self.weld_tcp_y, self.weld_tcp_z):
            widget.setFixedHeight(26); widget.setReadOnly(True)
        coord_layout.addWidget(QtWidgets.QLabel("X"), 1, 0); coord_layout.addWidget(self.weld_tcp_x, 1, 1)
        coord_layout.addWidget(QtWidgets.QLabel("Y"), 2, 0); coord_layout.addWidget(self.weld_tcp_y, 2, 1)
        coord_layout.addWidget(QtWidgets.QLabel("Z"), 3, 0); coord_layout.addWidget(self.weld_tcp_z, 3, 1)

        coord_layout.addWidget(QtWidgets.QLabel("Tool Axis (Direction Vector)"), 4, 0, 1, 3)
        self.weld_axis_x = QtWidgets.QLineEdit(); self.weld_axis_y = QtWidgets.QLineEdit(); self.weld_axis_z = QtWidgets.QLineEdit()
        for widget in (self.weld_axis_x, self.weld_axis_y, self.weld_axis_z):
            widget.setFixedHeight(26); widget.setReadOnly(True)
        coord_layout.addWidget(QtWidgets.QLabel("X"), 5, 0); coord_layout.addWidget(self.weld_axis_x, 5, 1)
        coord_layout.addWidget(QtWidgets.QLabel("Y"), 6, 0); coord_layout.addWidget(self.weld_axis_y, 6, 1)
        coord_layout.addWidget(QtWidgets.QLabel("Z"), 7, 0); coord_layout.addWidget(self.weld_axis_z, 7, 1)

        weld_layout.addLayout(coord_layout)
        layout.addWidget(self.welding_group)

        # --- PAINTING TOOL SETUP (visible only when Painting Tool selected) ---
        self.painting_group = QtWidgets.QGroupBox("PAINTING TOOL SETUP")
        self.painting_group.setStyleSheet(self._group_style())
        paint_layout = QtWidgets.QVBoxLayout(self.painting_group)

        nozzle_face_label = QtWidgets.QLabel("1. Nozzle Face (Spray Face)")
        nozzle_face_label.setStyleSheet("font-weight: bold; color: #424242;")
        paint_layout.addWidget(nozzle_face_label)

        self.paint_face_status = QtWidgets.QLabel("Nozzle face selected: nil")
        self.paint_face_status.setStyleSheet("color: #388e3c; font-size: 12px; margin-top: 6px;")
        paint_layout.addWidget(self.paint_face_status)

        self.paint_compile_btn = QtWidgets.QPushButton("Compile")
        self.paint_compile_btn.setFixedHeight(34)
        self.paint_compile_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.paint_compile_btn.setStyleSheet("""
            QPushButton {
                background-color: #1976d2;
                color: white;
                border: none;
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #1565c0; }
            QPushButton:pressed { background-color: #0d47a1; }
        """)
        self.paint_compile_btn.clicked.connect(self.on_paint_tool_compile)
        paint_layout.addWidget(self.paint_compile_btn)

        self.paint_tool_status = QtWidgets.QLineEdit()
        self.paint_tool_status.setReadOnly(True)
        self.paint_tool_status.setText("Click a nozzle face, then press Compile.")
        self.paint_tool_status.setFixedHeight(28)
        paint_layout.addWidget(self.paint_tool_status)

        self.paint_tcp_position_label = QtWidgets.QLabel("2. TCP Position (Auto)")
        self.paint_tcp_position_label.setStyleSheet("font-weight: bold; color: #424242;")
        paint_layout.addWidget(self.paint_tcp_position_label)

        paint_coord_layout = QtWidgets.QGridLayout()
        paint_coord_layout.setSpacing(6)
        paint_coord_layout.addWidget(QtWidgets.QLabel("X"), 0, 0)
        paint_coord_layout.addWidget(QtWidgets.QLabel("Y"), 1, 0)
        paint_coord_layout.addWidget(QtWidgets.QLabel("Z"), 2, 0)
        self.paint_tcp_x = QtWidgets.QLineEdit()
        self.paint_tcp_y = QtWidgets.QLineEdit()
        self.paint_tcp_z = QtWidgets.QLineEdit()
        for widget in (self.paint_tcp_x, self.paint_tcp_y, self.paint_tcp_z):
            widget.setReadOnly(True)
            widget.setFixedHeight(26)
            widget.setText("nil")
        paint_coord_layout.addWidget(self.paint_tcp_x, 0, 1)
        paint_coord_layout.addWidget(self.paint_tcp_y, 1, 1)
        paint_coord_layout.addWidget(self.paint_tcp_z, 2, 1)
        paint_layout.addLayout(paint_coord_layout)

        self.paint_tcp_direction_label = QtWidgets.QLabel("3. TCP Direction (Auto)")
        self.paint_tcp_direction_label.setStyleSheet("font-weight: bold; color: #424242;")
        paint_layout.addWidget(self.paint_tcp_direction_label)

        paint_dir_layout = QtWidgets.QGridLayout()
        paint_dir_layout.setSpacing(6)
        paint_dir_layout.addWidget(QtWidgets.QLabel("X"), 0, 0)
        paint_dir_layout.addWidget(QtWidgets.QLabel("Y"), 1, 0)
        paint_dir_layout.addWidget(QtWidgets.QLabel("Z"), 2, 0)
        self.paint_dir_x = QtWidgets.QLineEdit()
        self.paint_dir_y = QtWidgets.QLineEdit()
        self.paint_dir_z = QtWidgets.QLineEdit()
        for widget in (self.paint_dir_x, self.paint_dir_y, self.paint_dir_z):
            widget.setReadOnly(True)
            widget.setFixedHeight(26)
            widget.setText("nil")
        paint_dir_layout.addWidget(self.paint_dir_x, 0, 1)
        paint_dir_layout.addWidget(self.paint_dir_y, 1, 1)
        paint_dir_layout.addWidget(self.paint_dir_z, 2, 1)
        paint_layout.addLayout(paint_dir_layout)

        paint_note = QtWidgets.QLabel(
            "TCP and direction are calculated automatically from the selected nozzle face."
        )
        paint_note.setWordWrap(True)
        paint_note.setStyleSheet("color: #757575; font-size: 12px;")
        paint_layout.addWidget(paint_note)

        layout.addWidget(self.painting_group)

        self.tool_save_btn = QtWidgets.QPushButton("Save")
        self.tool_save_btn.setFixedHeight(36)
        self.tool_save_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.tool_save_btn.setStyleSheet("""
            QPushButton {
                background-color: #2e7d32;
                color: white;
                border: none;
                border-radius: 6px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover { background-color: #1b5e20; }
            QPushButton:pressed { background-color: #145a2d; }
            QPushButton:disabled {
                background-color: #c8e6c9;
                color: #7f8b7f;
            }
        """)
        self.tool_save_btn.clicked.connect(self.on_tool_save_clicked)
        self.tool_save_btn.setVisible(False)
        layout.addWidget(self.tool_save_btn)

        # hide welding and painting groups by default; shown only when selected
        self.welding_group.setVisible(False)
        self.painting_group.setVisible(False)
        self.selection_group.setVisible(False)
        self.control_group.setVisible(False)
        layout.addStretch()

        layout.addStretch()

    def _selected_joint_name(self):
        item = self.joints_list.currentItem()
        if not item:
            return None
        return item.data(QtCore.Qt.UserRole)

    def _apply_tool_selection(self, selected_tool):
        """Enable/disable UI elements according to the selected end-effector tool."""
        tool_key = (selected_tool or "").strip().lower()
        is_gripper = tool_key == "gripper tool"
        weld_visible = tool_key == "welding tool"
        paint_visible = tool_key == "painting tool"
        show_gripper_workflow = is_gripper and not self._gripper_confirmation_mode

        # Gripper-specific widgets
        if hasattr(self, 'joints_list'):
            self.joints_list.setEnabled(is_gripper)
        if hasattr(self, 'mark_gripper_check'):
            # If tool is not gripper, ensure nothing remains marked
            if not is_gripper:
                try:
                    # unmark currently displayed selection without emitting signals
                    self.mark_gripper_check.blockSignals(True)
                    self.mark_gripper_check.setChecked(False)
                finally:
                    self.mark_gripper_check.blockSignals(False)
            self.mark_gripper_check.setEnabled(is_gripper)
        if hasattr(self, 'stroke_slider'):
            self.stroke_slider.setEnabled(is_gripper)

        # Welding tool UI
        if hasattr(self, 'welding_group'):
            self.welding_group.setVisible(weld_visible)
            if weld_visible:
                self.weld_face_name.setText("nil")
                self.weld_tcp_x.setText("nil")
                self.weld_tcp_y.setText("nil")
                self.weld_tcp_z.setText("nil")
                self.weld_axis_x.setText("nil")
                self.weld_axis_y.setText("nil")
                self.weld_axis_z.setText("nil")
                if hasattr(self, 'weld_compile_status'):
                    self.weld_compile_status.setText("Click a face in the 3D view to set the welding live point.")
                try:
                    self._start_weld_face_picking()
                except Exception:
                    pass

        # Painting tool UI
        if hasattr(self, 'painting_group'):
            self.painting_group.setVisible(paint_visible)
            if paint_visible:
                self.paint_tool_status.setText("Click a nozzle face in the 3D view.")
                self.paint_face_status.setText("Nozzle face selected: nil")
                self.paint_tcp_x.setText("nil")
                self.paint_tcp_y.setText("nil")
                self.paint_tcp_z.setText("nil")
                self.paint_dir_x.setText("nil")
                self.paint_dir_y.setText("nil")
                self.paint_dir_z.setText("nil")
                try:
                    self.mw.canvas.start_face_picking(self.on_paint_nozzle_face_picked, color="green")
                    self.mw.log("Painting tool active. Click a face to assign nozzle TCP.")
                    self.mw.show_toast("Select the nozzle face in 3D view", "info")
                except Exception:
                    pass

        # Show only the selected tool's relevant controls
        if hasattr(self, 'selection_group'):
            self.selection_group.setVisible(show_gripper_workflow)
        if hasattr(self, 'control_group'):
            self.control_group.setVisible(show_gripper_workflow)

        # when welding or painting is visible, keep gripper controls disabled
        if weld_visible or paint_visible:
            try:
                if hasattr(self, 'joints_list'):
                    self.joints_list.setEnabled(False)
                if hasattr(self, 'mark_gripper_check'):
                    self.mark_gripper_check.setEnabled(False)
                if hasattr(self, 'stroke_slider'):
                    self.stroke_slider.setEnabled(False)
            except Exception:
                pass


    def _selected_group_members(self):
        item = self.joints_list.currentItem()
        if not item:
            return []

        members = item.data(QtCore.Qt.UserRole + 1)
        if not isinstance(members, list) or not members:
            selected = item.data(QtCore.Qt.UserRole)
            members = [selected] if isinstance(selected, str) else []

        return [name for name in members if isinstance(name, str) and name in self.mw.robot.joints]

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

        for joint_name in sorted(robot.joints.keys(), key=self._joint_name_sort_key):
            joint = robot.joints[joint_name]
            tooltip = (
                f"{joint_name}: {joint.parent_link.name} -> {joint.child_link.name}"
            )
            entries.append(
                {
                    "primary_name": joint_name,
                    "members": [joint_name],
                    "display_name": joint_name,
                    "tooltip": tooltip,
                }
            )

        return entries

    def on_make_robo(self):
        self.mw.log("?? FINALIZING ASSEMBLY: Building Robot Kinematic Tree...")
        success = self.mw.make_robot()
        if success:
            self.refresh_joints()
            if hasattr(self.mw, 'experiment_tab'):
                self.mw.experiment_tab.update_display()

            # Auto-attach the live point to the finalized TCP result.
            self._auto_lock_live_point()

            self.mw.show_toast("Assembly Finalized ? | Live Point Locked to TCP", "success")
        else:
            self.mw.show_toast("Assembly Failed", "error")

    def _auto_lock_live_point(self):
        """Lock the live point to the finalized rigid TCP so it moves with the robot."""
        try:
            if hasattr(self.mw, '_configure_default_tcp') and self.mw._configure_default_tcp():
                tcp_name = getattr(self.mw, 'custom_tcp_name', None)
                sim_panel = getattr(self.mw, 'simulation_tab', None)
                if sim_panel is not None:
                    sim_panel.live_point_locked = False
                    sim_panel.locked_live_point = None

                self.mw.live_point_locked = False
                self.mw.locked_live_point = None
                self.mw.locked_live_point_link_name = tcp_name
                self.mw.locked_live_point_local = None
                self.mw.log(f"Live Point locked to rigid TCP frame '{tcp_name}' and will move with the robot.")
                if hasattr(self.mw, 'update_live_ui'):
                    self.mw.update_live_ui(render=False)
                return

            self.mw.log("Could not auto-lock live point: no rigid TCP link was found.")
        except Exception as e:
            self.mw.log(f"Could not auto-lock live point: {e}")

    def refresh_sliders(self):
        self.refresh_joints()

    def refresh_joints(self):
        """Update the list of available joints for Gripper Tool selection."""
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

    def _selected_gripper_joint_names(self):
        selected_items = self.joints_list.selectedItems()
        return [
            item.data(QtCore.Qt.UserRole)
            for item in selected_items
            if item.data(QtCore.Qt.UserRole)
        ]

    def _has_valid_gripper_tool_selection(self):
        selected_tool = self.tool_combo.currentText().strip()
        selected_status = self.tool_selection_status.text().strip()
        return selected_tool == "Gripper Tool" and selected_status.startswith("Tool selected:")

    def _refresh_gripper_save_state(self):
        selected_names = self._selected_gripper_joint_names()
        has_valid_tool = self._has_valid_gripper_tool_selection()
        has_required_jaw_count = len(selected_names) >= 2
        has_one_face_per_joint = all(
            isinstance(self._gripper_face_selection_data.get(name), dict)
            and self._gripper_face_selection_data.get(name)
            for name in selected_names
        )
        has_alignment_face = isinstance(self._gripper_alignment_face_data, dict)
        can_save = (
            has_valid_tool
            and has_required_jaw_count
            and has_one_face_per_joint
            and has_alignment_face
        )
        self.gripper_save_btn.setEnabled(can_save)
        self._refresh_tool_save_state()

    def _refresh_tool_save_state(self):
        selected_tool = self.tool_combo.currentText().strip()
        tool_key = selected_tool.lower()
        save_btn = getattr(self, "tool_save_btn", None)
        if save_btn is None:
            return

        if self._gripper_confirmation_mode or not selected_tool:
            save_btn.setVisible(False)
            save_btn.setEnabled(False)
            return

        save_btn.setVisible(True)
        if tool_key == "gripper tool":
            save_btn.setEnabled(self.gripper_save_btn.isEnabled())
        elif tool_key == "welding tool":
            save_btn.setEnabled(self.weld_face_name.text().strip().lower() not in ("", "nil"))
        elif tool_key == "painting tool":
            paint_face = self.paint_face_status.text().strip().lower()
            save_btn.setEnabled("nil" not in paint_face)
        else:
            save_btn.setEnabled(False)

    def _set_gripper_confirmation_mode(self, enabled):
        """Show a compact confirmation view after Save, or restore the editable form."""
        self._gripper_confirmation_mode = bool(enabled)
        for widget_name in ("selection_group", "control_group", "welding_group", "painting_group"):
            widget = getattr(self, widget_name, None)
            if widget is not None:
                widget.setVisible(not enabled)
        if hasattr(self, "end_effector_summary_group"):
            self.end_effector_summary_group.setVisible(enabled)
        if hasattr(self, "make_robo_btn"):
            self.make_robo_btn.setVisible(enabled)
        if hasattr(self, "tool_save_btn"):
            self.tool_save_btn.setVisible(not enabled and bool(self.tool_combo.currentText().strip()))

    def on_delete_end_effector(self):
        """Clear the active tool and return to end-effector selection mode."""
        payload = getattr(self.mw, "end_effector_tool_config", None)
        tool_type = ""
        if isinstance(payload, dict):
            tool_type = str(payload.get("EndEffector", {}).get("ToolType", "")).strip()
        if not tool_type:
            tool_type = self.tool_combo.currentText().strip()

        tool_key = tool_type.lower()
        config_attribute = {
            "gripper tool": "gripper_tool_config",
            "welding tool": "welding_tool_config",
            "painting tool": "paint_tool_config",
        }.get(tool_key)
        if config_attribute:
            setattr(self.mw, config_attribute, None)
        self.mw.end_effector_tool_config = None

        if tool_key == "gripper tool" and isinstance(payload, dict):
            jaws = payload.get("EndEffector", {}).get("Jaws", [])
            jaw_names = {
                str(jaw.get("JointID"))
                for jaw in jaws
                if isinstance(jaw, dict) and jaw.get("JointID")
            }
            for joint_name in jaw_names:
                joint = self.mw.robot.joints.get(joint_name)
                if joint is not None:
                    joint.is_gripper = False
            self.mw.active_gripper_joint_names = []
            self.mw.active_gripper_joint_name = None

        tcp_names = {
            getattr(self.mw, "custom_tcp_name", None),
            getattr(self.mw, "locked_live_point_link_name", None),
        }
        for tcp_name in tcp_names:
            if tcp_name and tcp_name in self.mw.robot.links:
                tcp_link = self.mw.robot.links[tcp_name]
                tcp_link.custom_tcp_offset = None
                tcp_link.custom_tcp_rpy_deg = [0.0, 0.0, 0.0]
                tcp_link.auto_tcp_offset = None

        self.mw.custom_tcp_name = None
        self.mw.locked_live_point_link_name = None
        self.mw.locked_live_point_local = None
        self.mw.live_point_locked = False
        self.mw.locked_live_point = None
        self.mw.robot_finalized = False

        sim_panel = getattr(self.mw, "simulation_tab", None)
        if sim_panel is not None:
            sim_panel.live_point_locked = False
            sim_panel.locked_live_point = None

        self._gripper_tool_selected = False
        self._gripper_face_selection_queue = []
        self._pending_gripper_contact_joint_name = None
        self._gripper_face_selection_data = {}
        self._gripper_selection_joint_names = []
        self._gripper_live_point_world = None
        self._gripper_joint_endpoints = {}
        self._gripper_alignment_face_data = None
        self.gripper_alignment_btn.setEnabled(False)
        self.gripper_alignment_status.setText("Gripping face and Live Point: not selected")
        self.tool_selection_status.setText("No tool selected")
        self._update_end_effector_summary(tool_name=None, jaw_count=None, saved=False)
        self._set_gripper_confirmation_mode(False)
        for widget_name in ("selection_group", "control_group", "welding_group", "painting_group"):
            widget = getattr(self, widget_name, None)
            if widget is not None:
                widget.setVisible(False)
        self.tool_save_btn.setVisible(False)
        self.make_robo_btn.setVisible(False)

        canvas = getattr(self.mw, "canvas", None)
        if canvas is not None:
            if hasattr(canvas, "clear_live_point_marker"):
                canvas.clear_live_point_marker()
            if hasattr(canvas, "clear_live_tcp_marker"):
                canvas.clear_live_tcp_marker()

        joint_tab = getattr(self.mw, "joint_tab", None)
        if joint_tab is not None:
            if hasattr(joint_tab, "refresh_joints_history"):
                joint_tab.refresh_joints_history()
            if hasattr(joint_tab, "refresh_links"):
                joint_tab.refresh_links()

        if hasattr(self.mw, "update_live_ui"):
            self.mw.update_live_ui(render=False)
        self.mw.log(f"End-effector deleted: {tool_type or 'saved tool'}.")
        self.mw.show_toast("End-effector removed. Select a new tool.", "success")

    def _build_end_effector_payload(self):
        selected_names = self._selected_gripper_joint_names()
        endpoint_angles = self._calculate_gripper_endpoint_angles(selected_names)
        jaws = []
        for joint_name in selected_names:
            face_info = self._gripper_face_selection_data.get(joint_name)
            if not isinstance(face_info, dict):
                continue
            jaws.append({
                "JointID": joint_name,
                "FaceID": face_info.get("link_name", ""),
                "FaceCenter": np.asarray(face_info.get("local_center", [0.0, 0.0, 0.0]), dtype=float).tolist(),
                "FaceNormal": np.asarray(face_info.get("local_normal", [0.0, 0.0, 1.0]), dtype=float).tolist(),
                "FaceCenterWorld": np.asarray(face_info.get("world_center", [0.0, 0.0, 0.0]), dtype=float).tolist(),
                "FaceNormalWorld": np.asarray(face_info.get("world_normal", [0.0, 0.0, 1.0]), dtype=float).tolist(),
                "FaceCenterLocal": np.asarray(face_info.get("local_center", [0.0, 0.0, 0.0]), dtype=float).tolist(),
                "FaceNormalLocal": np.asarray(face_info.get("local_normal", [0.0, 0.0, 1.0]), dtype=float).tolist(),
                "ClosedAngle": float(endpoint_angles.get(joint_name, {}).get("closed", self.gripper_min_input.value())),
                "OpenAngle": float(endpoint_angles.get(joint_name, {}).get("open", self.gripper_max_input.value())),
            })

        alignment_face = None
        if isinstance(self._gripper_alignment_face_data, dict):
            alignment_face = {
                "LinkID": self._gripper_alignment_face_data.get("link_name", ""),
                "TCPLink": self._gripper_alignment_face_data.get("tcp_link_name", ""),
                "FaceCenterTCPLocal": np.asarray(
                    self._gripper_alignment_face_data.get("tcp_local_center", [0.0, 0.0, 0.0]),
                    dtype=float,
                ).tolist(),
                "FaceNormalTCPLocal": np.asarray(
                    self._gripper_alignment_face_data.get("tcp_local_normal", [0.0, 0.0, 1.0]),
                    dtype=float,
                ).tolist(),
                "FaceCenterLinkLocal": np.asarray(
                    self._gripper_alignment_face_data.get("link_local_center", [0.0, 0.0, 0.0]),
                    dtype=float,
                ).tolist(),
                "FaceNormalLinkLocal": np.asarray(
                    self._gripper_alignment_face_data.get("link_local_normal", [0.0, 0.0, 1.0]),
                    dtype=float,
                ).tolist(),
            }

        return {
            "EndEffector": {
                "ToolType": self.tool_combo.currentText().strip(),
                "LivePoint": np.asarray(self._gripper_live_point_world, dtype=float).tolist() if self._gripper_live_point_world is not None else None,
                "TCPLink": getattr(self.mw, "custom_tcp_name", None),
                "MinOpening": int(self.gripper_min_input.value()),
                "MaxOpening": int(self.gripper_max_input.value()),
                "JawCount": len(jaws),
                "Jaws": jaws,
                "BaseAlignmentFace": alignment_face,
            }
        }

    def restore_saved_gripper_config(self, payload):
        """Restore the gripper summary and editable data from a project payload."""
        if not isinstance(payload, dict):
            return
        definition = payload.get("EndEffector", payload)
        if str(definition.get("ToolType", "")).strip().lower() != "gripper tool":
            return

        jaw_names = [
            str(jaw.get("JointID"))
            for jaw in definition.get("Jaws", [])
            if isinstance(jaw, dict) and jaw.get("JointID") in self.mw.robot.joints
        ]
        self._gripper_selection_joint_names = jaw_names
        self._set_active_gripper_context(jaw_names)
        self._gripper_face_selection_data = {}
        self._gripper_joint_endpoints = {}
        for jaw in definition.get("Jaws", []):
            if not isinstance(jaw, dict):
                continue
            joint_name = str(jaw.get("JointID", ""))
            joint = self.mw.robot.joints.get(joint_name)
            if joint is None or joint.child_link is None:
                continue
            local_center = np.asarray(
                jaw.get("FaceCenterLocal", jaw.get("FaceCenter", [0.0, 0.0, 0.0])),
                dtype=float,
            )
            local_normal = np.asarray(
                jaw.get("FaceNormalLocal", jaw.get("FaceNormal", [0.0, 0.0, 1.0])),
                dtype=float,
            )
            child_world = np.asarray(joint.child_link.t_world, dtype=float)
            self._gripper_face_selection_data[joint_name] = {
                "link_name": jaw.get("FaceID", joint.child_link.name),
                "surface_name": "Saved Contact Face",
                "local_center": local_center,
                "local_normal": local_normal,
                "world_center": (child_world @ np.append(local_center, 1.0))[:3],
                "world_normal": child_world[:3, :3] @ local_normal,
            }
            self._gripper_joint_endpoints[joint_name] = {
                "closed": float(jaw.get("ClosedAngle", definition.get("MinOpening", 0.0))),
                "open": float(jaw.get("OpenAngle", definition.get("MaxOpening", 90.0))),
            }

        alignment = definition.get("BaseAlignmentFace")
        self._gripper_alignment_face_data = None
        if isinstance(alignment, dict):
            tcp_name = alignment.get("TCPLink") or definition.get("TCPLink")
            tcp_link = self.mw.robot.links.get(tcp_name)
            local_center = np.asarray(alignment.get("FaceCenterTCPLocal", [0.0, 0.0, 0.0]), dtype=float)
            local_normal = np.asarray(alignment.get("FaceNormalTCPLocal", [0.0, 0.0, 1.0]), dtype=float)
            jaw_centers = [
                np.asarray(face_info["world_center"], dtype=float).reshape(3)
                for face_info in self._gripper_face_selection_data.values()
                if isinstance(face_info, dict) and face_info.get("world_center") is not None
            ]
            midpoint_world = np.mean(np.asarray(jaw_centers, dtype=float), axis=0) if jaw_centers else None
            if tcp_link is not None:
                tcp_pose = np.asarray(self.mw.robot.get_tcp_world_pose(tcp_link), dtype=float)
                if midpoint_world is not None:
                    link_local_center = (
                        np.linalg.inv(np.asarray(tcp_link.t_world, dtype=float))
                        @ np.append(midpoint_world, 1.0)
                    )[:3]
                    self.mw.robot.set_tcp_transform(
                        tcp_link.name, position=link_local_center
                    )
                    self.mw.robot.ensure_tcp_transform(tcp_link)
                    self.mw.custom_tcp_name = tcp_link.name
                    world_center = midpoint_world
                    saved_link_center = link_local_center
                    tcp_local_center = np.zeros(3, dtype=float)
                else:
                    link_local_center = alignment.get("FaceCenterLinkLocal")
                    if link_local_center is not None:
                        try:
                            link_local_center = np.asarray(
                                link_local_center, dtype=float
                            ).reshape(3)
                            self.mw.robot.set_tcp_transform(
                                tcp_link.name, position=link_local_center
                            )
                            self.mw.robot.ensure_tcp_transform(tcp_link)
                            self.mw.custom_tcp_name = tcp_link.name
                        except (TypeError, ValueError):
                            link_local_center = None
                    world_center = (tcp_pose @ np.append(local_center, 1.0))[:3]
                    if link_local_center is None:
                        saved_link_center = getattr(
                            tcp_link, "custom_tcp_offset", None
                        )
                        if saved_link_center is None:
                            saved_link_center = np.zeros(3, dtype=float)
                    else:
                        saved_link_center = link_local_center
                    tcp_local_center = local_center
                tcp_local_rotation = np.asarray(
                    self.mw.robot.get_tcp_local_transform(tcp_link), dtype=float
                )[:3, :3]
                self._gripper_alignment_face_data = {
                    "link_name": alignment.get("LinkID", ""),
                    "tcp_link_name": tcp_link.name,
                    "tcp_local_center": tcp_local_center,
                    "tcp_local_normal": local_normal,
                    "link_local_center": np.asarray(
                        saved_link_center, dtype=float
                    ),
                    "link_local_normal": np.asarray(
                        alignment.get(
                            "FaceNormalLinkLocal",
                            tcp_local_rotation @ local_normal,
                        ),
                        dtype=float,
                    ),
                    "world_center": world_center,
                    "world_normal": tcp_pose[:3, :3] @ local_normal,
                }
                self._gripper_live_point_world = world_center.copy()
                self.gripper_live_point_preview.setText(
                    f"Live Point Preview: ({world_center[0]:.2f}, "
                    f"{world_center[1]:.2f}, {world_center[2]:.2f})"
                )

        self.gripper_min_input.blockSignals(True)
        self.gripper_max_input.blockSignals(True)
        self.gripper_min_input.setValue(int(definition.get("MinOpening", 0)))
        self.gripper_max_input.setValue(int(definition.get("MaxOpening", 90)))
        self.gripper_min_input.blockSignals(False)
        self.gripper_max_input.blockSignals(False)
        self.tool_combo.setCurrentText("Gripper Tool")
        self.tool_selection_status.setText("Tool selected: Gripper Tool")
        self._gripper_tool_selected = True
        self.gripper_alignment_btn.setEnabled(bool(jaw_names))
        if self._gripper_alignment_face_data is not None:
            self.gripper_alignment_status.setText(
                f"Gripping face / Live Point: {alignment.get('LinkID', 'saved face')}"
            )
            self.gripper_alignment_status.setStyleSheet("color: #388e3c; font-size: 12px;")
        self._update_end_effector_summary(
            tool_name="Gripper Tool",
            jaw_count=len(jaw_names),
            saved=True,
        )
        self._set_gripper_confirmation_mode(True)

    def on_select_tool_ok(self):
        selected_tool = self.tool_combo.currentText()
        self.tool_selection_status.setText(f"Tool selected: {selected_tool}")
        self._gripper_tool_selected = selected_tool == "Gripper Tool"
        self.mw.log(f"End-effector tool selected: {selected_tool}")
        self._set_gripper_confirmation_mode(False)
        # Apply UI changes based on chosen tool
        try:
            self._apply_tool_selection(selected_tool)
        except Exception:
            pass
        self._update_end_effector_summary(tool_name=selected_tool, jaw_count=None, saved=False)
        self._refresh_gripper_save_state()
        self._refresh_tool_save_state()

    def on_weld_tool_compile(self):
        tool_name = self.tool_combo.currentText().strip()
        if not tool_name:
            self.weld_compile_status.setText("Please enter the welding tool filename first.")
            self.weld_compile_status.setStyleSheet("color: #d32f2f; font-size: 12px; margin-top: 4px;")
            return

        self._start_weld_face_picking()
        self.weld_compile_status.setText(f"Compiled tool: {tool_name}. Click a face in the 3D view to set Live Point.")
        self.weld_compile_status.setStyleSheet("color: #388e3c; font-size: 12px; margin-top: 4px;")
        self._refresh_tool_save_state()

    def _start_weld_face_picking(self):
        tool_name = self.tool_combo.currentText().strip() or "Welding Tool"
        if hasattr(self.mw.canvas, 'start_face_picking'):
            self.mw.canvas.start_face_picking(self.on_weld_face_picked, color="red")
            self.mw.log(f"{tool_name} active. Click a face on the model to assign the Live Point.")
            self.mw.show_toast("Select a face for the welding tool Live Point", "info")

    def on_weld_face_picked(self, link_name, world_center=None, world_normal=None):
        self.weld_face_name.setText(link_name or "nil")
        if world_center is not None:
            world_center = np.asarray(world_center, dtype=float).reshape(3)
            self.weld_tcp_x.setText(f"{world_center[0]:.2f}")
            self.weld_tcp_y.setText(f"{world_center[1]:.2f}")
            self.weld_tcp_z.setText(f"{world_center[2]:.2f}")
        else:
            self.weld_tcp_x.setText("nil")
            self.weld_tcp_y.setText("nil")
            self.weld_tcp_z.setText("nil")

        if world_normal is not None:
            world_normal = np.asarray(world_normal, dtype=float).reshape(3)
            self.weld_axis_x.setText(f"{world_normal[0]:.2f}")
            self.weld_axis_y.setText(f"{world_normal[1]:.2f}")
            self.weld_axis_z.setText(f"{world_normal[2]:.2f}")
        else:
            self.weld_axis_x.setText("nil")
            self.weld_axis_y.setText("nil")
            self.weld_axis_z.setText("nil")

        if link_name and hasattr(self.mw, 'robot') and link_name in self.mw.robot.links:
            picked_link = self.mw.robot.links[link_name]
            if hasattr(self.mw, '_resolve_rigid_tcp_link'):
                tcp_link = self.mw._resolve_rigid_tcp_link(picked_link)
            else:
                tcp_link = picked_link

            if world_center is None:
                try:
                    world_center = np.asarray(tcp_link.t_world[:3, 3], dtype=float)
                except Exception:
                    world_center = None

            if world_center is not None:
                try:
                    inv_world = np.linalg.inv(np.asarray(tcp_link.t_world, dtype=float))
                    local_point = (inv_world @ np.append(world_center, 1.0))[:3]
                except Exception:
                    local_point = np.asarray(world_center, dtype=float).reshape(3)

                self.mw.custom_tcp_name = tcp_link.name
                self.mw.robot.set_tcp_transform(tcp_link.name, position=local_point)
                self.mw.robot.ensure_tcp_transform(tcp_link)
                self.mw.log(f"Welding tool live point bound to TCP link '{tcp_link.name}'.")
                self.mw.show_toast(f"Live Point set to {tcp_link.name}", "success")
                if hasattr(self.mw, 'update_live_ui'):
                    self.mw.update_live_ui()
            else:
                self.mw.log("Welding tool face selected, but could not resolve a valid TCP world point.")
                self.mw.show_toast("Live Point assignment incomplete", "warning")
        else:
            self.mw.log(f"Welding tool face selected for '{link_name}', but link was not found in the robot.")
            self.mw.show_toast("Selected face is not part of the robot model", "error")

        self.mw.log(f"Welding tool live point assigned to face on '{link_name}'.")
        self._refresh_tool_save_state()

    def on_paint_tool_compile(self):
        selected_face = self.paint_face_status.text().replace("Nozzle face selected:", "").strip()
        if not selected_face or selected_face.lower() == "nil":
            self.paint_tool_status.setText("Please pick a nozzle face before compiling.")
            self.paint_tool_status.setStyleSheet("color: #d32f2f; font-size: 12px; margin-top: 4px;")
            return

        tool_name = self.tool_combo.currentText().strip() or "Painting Tool"
        self.paint_tool_status.setText(f"Compiled paint tool: {tool_name}. Nozzle face locked.")
        self.paint_tool_status.setStyleSheet("color: #388e3c; font-size: 12px; margin-top: 4px;")
        self.mw.log(f"Paint tool compiled with nozzle face '{selected_face}'.")
        self.mw.show_toast("Painting nozzle face compiled", "success")
        self._refresh_tool_save_state()

    def on_paint_pick_nozzle(self):
        self.paint_tool_status.setText("Pick a nozzle face on the robot model.")
        self.paint_tool_status.setStyleSheet("color: #388e3c; font-size: 12px; margin-top: 4px;")

        if hasattr(self.mw.canvas, 'start_face_picking'):
            self.mw.canvas.start_face_picking(
                self.on_paint_nozzle_face_picked,
                color="green",
            )
            self.mw.log("Painting tool active. Click a face to assign nozzle TCP.")
            self.mw.show_toast("Select the nozzle face in 3D view", "info")

    def on_paint_nozzle_face_picked(self, link_name, world_center=None, world_normal=None):
        self.paint_face_status.setText(f"Nozzle face selected: {link_name or 'nil'}")
        self.paint_tool_status.setText("Painting nozzle face assigned.")
        self.paint_tool_status.setStyleSheet("color: #388e3c; font-size: 12px; margin-top: 4px;")

        if world_center is not None:
            world_center = np.asarray(world_center, dtype=float).reshape(3)
            self.paint_tcp_x.setText(f"{world_center[0]:.2f}")
            self.paint_tcp_y.setText(f"{world_center[1]:.2f}")
            self.paint_tcp_z.setText(f"{world_center[2]:.2f}")
        else:
            self.paint_tcp_x.setText("nil")
            self.paint_tcp_y.setText("nil")
            self.paint_tcp_z.setText("nil")

        if world_normal is not None:
            world_normal = np.asarray(world_normal, dtype=float).reshape(3)
            self.paint_dir_x.setText(f"{world_normal[0]:.2f}")
            self.paint_dir_y.setText(f"{world_normal[1]:.2f}")
            self.paint_dir_z.setText(f"{world_normal[2]:.2f}")
        else:
            self.paint_dir_x.setText("nil")
            self.paint_dir_y.setText("nil")
            self.paint_dir_z.setText("nil")

        if link_name and hasattr(self.mw, 'robot') and link_name in self.mw.robot.links:
            picked_link = self.mw.robot.links[link_name]
            if hasattr(self.mw, '_resolve_rigid_tcp_link'):
                tcp_link = self.mw._resolve_rigid_tcp_link(picked_link)
            else:
                tcp_link = picked_link

            if world_center is None:
                try:
                    world_center = np.asarray(tcp_link.t_world[:3, 3], dtype=float)
                except Exception:
                    world_center = None

            if world_normal is not None and world_center is not None:
                try:
                    inv_world = np.linalg.inv(np.asarray(tcp_link.t_world, dtype=float))
                    local_point = (inv_world @ np.append(world_center, 1.0))[:3]
                except Exception:
                    local_point = np.asarray(world_center, dtype=float).reshape(3)

                self.mw.custom_tcp_name = tcp_link.name
                self.mw.robot.set_tcp_transform(tcp_link.name, position=local_point)
                self.mw.robot.ensure_tcp_transform(tcp_link)
                self.mw.log(f"Paint tool nozzle TCP set to '{tcp_link.name}'.")
                self.mw.show_toast(f"Painting TCP set to {tcp_link.name}", "success")
                if hasattr(self.mw, 'update_live_ui'):
                    self.mw.update_live_ui()
        else:
            self.mw.log(f"Paint nozzle face selected for '{link_name}', but link was not found in the robot.")
            self.mw.show_toast("Selected nozzle face is not part of the robot model", "error")

        self.mw.log(f"Painting tool nozzle face assigned on '{link_name}'.")
        self._refresh_tool_save_state()

    def on_tool_save_clicked(self):
        selected_tool = self.tool_combo.currentText().strip().lower()
        if selected_tool == "gripper tool":
            self.on_gripper_save_clicked()
        elif selected_tool == "welding tool":
            self.on_weld_tool_save_clicked()
        elif selected_tool == "painting tool":
            self.on_paint_tool_save_clicked()

    def _build_weld_tool_payload(self):
        return {
            "EndEffector": {
                "ToolType": "Welding Tool",
                "ToolName": self.tool_combo.currentText().strip(),
                "SelectedFace": self.weld_face_name.text().strip(),
                "LivePoint": [
                    self.weld_tcp_x.text().strip(),
                    self.weld_tcp_y.text().strip(),
                    self.weld_tcp_z.text().strip(),
                ],
                "ToolAxis": [
                    self.weld_axis_x.text().strip(),
                    self.weld_axis_y.text().strip(),
                    self.weld_axis_z.text().strip(),
                ],
                "DirectionMode": self.weld_dir_combo.currentText().strip(),
            }
        }

    def _gripper_tcp_anchor_link(self, joint_names=None):
        """Return the rigid flange shared by the selected gripper jaws."""
        anchors = []
        for joint_name in joint_names or self._gripper_selection_joint_names:
            joint = self.mw.robot.joints.get(joint_name)
            if joint is None or joint.child_link is None:
                continue
            anchor = joint.parent_link
            if hasattr(self.mw, "_resolve_rigid_tcp_link"):
                anchor = self.mw._resolve_rigid_tcp_link(joint.child_link) or anchor
            if anchor is not None and anchor not in anchors:
                anchors.append(anchor)
        if not anchors:
            return None
        return max(
            anchors,
            key=lambda link: len(self.mw.robot.get_kinematic_chain(link)),
        )

    def _gripper_alignment_allowed_link_names(self):
        """Return tool links that may provide the object-base alignment face."""
        allowed = set()
        anchor = self._gripper_tcp_anchor_link()
        if anchor is not None:
            allowed.add(anchor.name)

        for joint_name in self._gripper_selection_joint_names:
            joint = self.mw.robot.joints.get(joint_name)
            if joint is None or joint.child_link is None:
                continue
            stack = [joint.child_link]
            while stack:
                link = stack.pop()
                if link.name in allowed:
                    continue
                allowed.add(link.name)
                for child_joint in getattr(link, "child_joints", []):
                    if child_joint.child_link is not None:
                        stack.append(child_joint.child_link)
        return allowed

    def on_select_gripper_alignment_face(self):
        """Ask the user to choose the gripping face that defines the TCP."""
        if len(self._gripper_selection_joint_names) < 2:
            self.gripper_alignment_status.setText(
                "Compile at least two gripper joints before selecting the alignment face."
            )
            self.gripper_alignment_status.setStyleSheet("color: #d32f2f; font-size: 12px;")
            return
        if not hasattr(self.mw, "canvas") or not hasattr(self.mw.canvas, "start_face_picking"):
            return

        self.gripper_alignment_status.setText(
            "Click a gripper-tool face. Its normal will align with the object's base; "
            "the Live Point will stay at the jaw centroid."
        )
        self.gripper_alignment_status.setStyleSheet("color: #d97706; font-size: 12px;")
        self.mw.canvas.start_face_picking(
            self._on_gripper_alignment_face_selected,
            color="orange",
        )
        self.mw.show_toast("Select the gripping face and Live Point", "info")

    def _on_gripper_alignment_face_selected(self, link_name, world_center=None, world_normal=None):
        """Store the selected face in the rigid TCP coordinate frame."""
        if (
            link_name not in self._gripper_alignment_allowed_link_names()
            or world_center is None
            or world_normal is None
        ):
            self.gripper_alignment_status.setText(
                "Select a face belonging to the compiled gripper tool."
            )
            self.gripper_alignment_status.setStyleSheet("color: #d32f2f; font-size: 12px;")
            self._refresh_gripper_save_state()
            return

        tcp_link = self._gripper_tcp_anchor_link()
        if tcp_link is None:
            self.gripper_alignment_status.setText("The rigid gripper TCP could not be resolved.")
            self.gripper_alignment_status.setStyleSheet("color: #d32f2f; font-size: 12px;")
            return

        world_center = np.asarray(world_center, dtype=float).reshape(3)
        world_normal = np.asarray(world_normal, dtype=float).reshape(3)
        normal_length = float(np.linalg.norm(world_normal))
        if normal_length <= 1e-9:
            self.gripper_alignment_status.setText("The selected face has no valid normal.")
            self.gripper_alignment_status.setStyleSheet("color: #d32f2f; font-size: 12px;")
            return
        world_normal /= normal_length

        self._gripper_alignment_face_data = {
            "link_name": link_name,
            "tcp_link_name": tcp_link.name,
            "world_center": world_center,
            "world_normal": world_normal,
        }
        tcp_link = self._bind_gripper_live_point_to_selected_face()
        if tcp_link is None:
            self.gripper_alignment_status.setText(
                "The selected face center could not be assigned as the Live Point."
            )
            self.gripper_alignment_status.setStyleSheet(
                "color: #d32f2f; font-size: 12px;"
            )
            return
        tcp_local_normal = self._gripper_alignment_face_data["tcp_local_normal"]
        self.gripper_alignment_status.setText(
            f"Gripping face / alignment normal: {link_name} (TCP {tcp_link.name})"
        )
        self.gripper_alignment_status.setStyleSheet("color: #388e3c; font-size: 12px;")
        self._refresh_gripper_save_state()
        self.mw.log(
            f"Gripping face selected on '{link_name}'. Its normal will guide the alignment; "
            f"the Live Point will be centered between all selected jaw faces."
        )
        self.mw.show_toast("Gripping face selected for alignment", "success")

    def _selected_gripper_face_centroid_world(self, joint_names):
        """Return the world-space centroid of all selected jaw face centers."""
        face_centers = []
        for joint_name in joint_names or []:
            joint = self.mw.robot.joints.get(joint_name)
            face_info = self._gripper_face_selection_data.get(joint_name)
            if joint is None or joint.child_link is None or not isinstance(face_info, dict):
                continue

            local_center = face_info.get("local_center")
            if local_center is None:
                continue

            try:
                local_center = np.asarray(local_center, dtype=float).reshape(3)
            except (TypeError, ValueError):
                continue

            face_centers.append(
                (
                    np.asarray(joint.child_link.t_world, dtype=float)
                    @ np.append(local_center, 1.0)
                )[:3]
            )

        if not face_centers:
            return None

        return np.mean(np.asarray(face_centers, dtype=float), axis=0)

    def _mesh_face_centers(self, mesh):
        """Return face-center points for meshes that expose triangle or cell geometry."""
        if mesh is None:
            return None

        try:
            import trimesh
            if isinstance(mesh, trimesh.Trimesh):
                if hasattr(mesh, "triangles_center"):
                    return np.asarray(mesh.triangles_center, dtype=float)
                if hasattr(mesh, "faces") and hasattr(mesh, "vertices"):
                    faces = np.asarray(mesh.faces, dtype=int)
                    verts = np.asarray(mesh.vertices, dtype=float)
                    if faces.ndim == 2 and faces.shape[1] >= 3:
                        return np.mean(verts[faces[:, :3]], axis=1)
        except Exception:
            pass

        try:
            import pyvista as pv
            if hasattr(mesh, "cell_centers"):
                centers = mesh.cell_centers().points
                if centers is not None and len(centers):
                    return np.asarray(centers, dtype=float)
        except Exception:
            pass

        if hasattr(mesh, "faces") and hasattr(mesh, "vertices"):
            faces = np.asarray(mesh.faces)
            verts = np.asarray(mesh.vertices, dtype=float)
            if faces.ndim == 2 and faces.shape[1] >= 3:
                return np.mean(verts[faces[:, :3]], axis=1)
            if faces.ndim == 1 and faces.size > 0:
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

    def _bind_gripper_live_point_to_selected_face(self):
        """Bind the jaw-face centroid to the rigid TCP link."""
        face_data = self._gripper_alignment_face_data
        if not isinstance(face_data, dict):
            return None

        tcp_name = face_data.get("tcp_link_name")
        tcp_link = self.mw.robot.links.get(tcp_name)
        if tcp_link is None:
            tcp_link = self._gripper_tcp_anchor_link()
        world_center = self._selected_gripper_face_centroid_world(
            self._gripper_selection_joint_names
        )
        world_normal = face_data.get("world_normal")
        if tcp_link is None or world_center is None or world_normal is None:
            return None

        world_center = np.asarray(world_center, dtype=float).reshape(3)
        world_normal = np.asarray(world_normal, dtype=float).reshape(3)
        normal_length = float(np.linalg.norm(world_normal))
        if normal_length <= 1e-9:
            return None
        world_normal /= normal_length

        link_world = np.asarray(tcp_link.t_world, dtype=float)
        inverse_link_world = np.linalg.inv(link_world)
        link_local_center = (
            inverse_link_world @ np.append(world_center, 1.0)
        )[:3]
        link_local_normal = link_world[:3, :3].T @ world_normal
        link_local_normal /= max(float(np.linalg.norm(link_local_normal)), 1e-12)

        self.mw.robot.set_tcp_transform(
            tcp_link.name, position=link_local_center
        )
        self.mw.robot.ensure_tcp_transform(tcp_link)
        self.mw.custom_tcp_name = tcp_link.name

        tcp_pose = np.asarray(
            self.mw.robot.get_tcp_world_pose(tcp_link), dtype=float
        )
        inverse_tcp = np.linalg.inv(tcp_pose)
        tcp_local_center = (
            inverse_tcp @ np.append(world_center, 1.0)
        )[:3]
        tcp_local_normal = tcp_pose[:3, :3].T @ world_normal
        tcp_local_normal /= max(float(np.linalg.norm(tcp_local_normal)), 1e-12)

        face_data.update({
            "tcp_link_name": tcp_link.name,
            "world_center": world_center,
            "world_normal": world_normal,
            "tcp_local_center": tcp_local_center,
            "tcp_local_normal": tcp_local_normal,
            "link_local_center": link_local_center,
            "link_local_normal": link_local_normal,
        })
        self._gripper_live_point_world = world_center.copy()
        self.gripper_live_point_preview.setText(
            f"Live Point Preview: ({world_center[0]:.2f}, "
            f"{world_center[1]:.2f}, {world_center[2]:.2f})"
        )

        self.mw.live_point_locked = False
        self.mw.locked_live_point = None
        self.mw.locked_live_point_link_name = tcp_link.name
        self.mw.locked_live_point_local = None
        simulation_panel = getattr(self.mw, "simulation_tab", None)
        if simulation_panel is not None:
            simulation_panel.live_point_locked = False
            simulation_panel.locked_live_point = None
        if hasattr(self.mw, "update_live_ui"):
            self.mw.update_live_ui(render=False)
        return tcp_link

    def _bind_gripper_live_point_to_flange(self, joint_names):
        """Bind the midpoint between selected jaw faces to the rigid gripper flange."""
        current_face_centers = []
        for joint_name in joint_names or []:
            joint = self.mw.robot.joints.get(joint_name)
            face_info = self._gripper_face_selection_data.get(joint_name)
            if joint is None or joint.child_link is None or not isinstance(face_info, dict):
                continue

            local_center = face_info.get("local_center")
            if local_center is not None:
                current_face_centers.append(
                    (
                        np.asarray(joint.child_link.t_world, dtype=float)
                        @ np.append(np.asarray(local_center, dtype=float).reshape(3), 1.0)
                    )[:3]
                )

        tcp_link = self._gripper_tcp_anchor_link(joint_names)
        if tcp_link is None or not current_face_centers:
            return None

        midpoint_world = np.mean(np.asarray(current_face_centers, dtype=float), axis=0)
        inverse_tcp_world = np.linalg.inv(np.asarray(tcp_link.t_world, dtype=float))
        midpoint_local = (inverse_tcp_world @ np.append(midpoint_world, 1.0))[:3]

        self._gripper_live_point_world = midpoint_world
        self.mw.robot.set_tcp_transform(tcp_link.name, position=midpoint_local)
        self.mw.robot.ensure_tcp_transform(tcp_link)
        self.mw.custom_tcp_name = tcp_link.name
        self.mw.log(
            f"Gripper TCP bound to '{tcp_link.name}' at the midpoint of {len(current_face_centers)} jaw faces."
        )
        return tcp_link

    def _calculate_gripper_endpoint_angles(self, joint_names):
        """Find a moving closed/open endpoint for every jaw, honoring joint relations."""
        joint_names = [
            name for name in dict.fromkeys(joint_names or [])
            if name in self.mw.robot.joints
        ]
        min_angle = float(self.gripper_min_input.value())
        max_angle = float(self.gripper_max_input.value())
        if min_angle > max_angle:
            min_angle, max_angle = max_angle, min_angle

        selected_set = set(joint_names)
        slave_relations = {}
        for master_name, slaves in self.mw.robot.joint_relations.items():
            for slave_name, ratio in slaves:
                if master_name in selected_set and slave_name in selected_set:
                    slave_relations[slave_name] = (master_name, float(ratio))

        candidate_angles = {}
        for name in joint_names:
            joint = self.mw.robot.joints[name]
            first = float(np.clip(min_angle, joint.min_limit, joint.max_limit))
            second = float(np.clip(max_angle, joint.min_limit, joint.max_limit))

            relation = slave_relations.get(name)
            if relation is not None:
                master_name, ratio = relation
                master = self.mw.robot.joints[master_name]
                master_first = float(np.clip(min_angle, master.min_limit, master.max_limit))
                master_second = float(np.clip(max_angle, master.min_limit, master.max_limit))
                first = float(np.clip(master_first * ratio, joint.min_limit, joint.max_limit))
                second = float(np.clip(master_second * ratio, joint.min_limit, joint.max_limit))

            # A positive UI range must still move a negative-range mirrored jaw.
            if abs(second - first) < 1e-9 and max_angle > min_angle:
                mirrored_first = float(np.clip(-min_angle, joint.min_limit, joint.max_limit))
                mirrored_second = float(np.clip(-max_angle, joint.min_limit, joint.max_limit))
                if abs(mirrored_second - mirrored_first) > 1e-9:
                    first, second = mirrored_first, mirrored_second
                elif joint.max_limit > joint.min_limit:
                    first = float(joint.min_limit)
                    second = float(joint.max_limit)
            candidate_angles[name] = (first, second)

        fallback = {
            name: {"closed": values[0], "open": values[1]}
            for name, values in candidate_angles.items()
        }
        if len(joint_names) < 2:
            self._gripper_joint_endpoints = fallback
            return fallback

        for name in joint_names:
            face_info = self._gripper_face_selection_data.get(name)
            joint = self.mw.robot.joints[name]
            if not isinstance(face_info, dict) or face_info.get("local_center") is None or joint.child_link is None:
                self._gripper_joint_endpoints = fallback
                return fallback

        saved_values = {
            name: float(joint.current_value)
            for name, joint in self.mw.robot.joints.items()
        }
        open_indices = {}
        try:
            # Hold every jaw at the midpoint while evaluating one jaw at a time.
            for name, (first, second) in candidate_angles.items():
                self.mw.robot.joints[name].current_value = 0.5 * (first + second)
            self.mw.robot.update_kinematics()

            for name in joint_names:
                joint = self.mw.robot.joints[name]
                scores = []
                for angle in candidate_angles[name]:
                    joint.current_value = float(angle)
                    self.mw.robot.update_kinematics()

                    local_center = np.asarray(
                        self._gripper_face_selection_data[name]["local_center"],
                        dtype=float,
                    ).reshape(3)
                    jaw_center = (
                        np.asarray(joint.child_link.t_world, dtype=float)
                        @ np.append(local_center, 1.0)
                    )[:3]
                    other_centers = []
                    for other_name in joint_names:
                        if other_name == name:
                            continue
                        other_joint = self.mw.robot.joints[other_name]
                        other_local = np.asarray(
                            self._gripper_face_selection_data[other_name]["local_center"],
                            dtype=float,
                        ).reshape(3)
                        other_centers.append(
                            (
                                np.asarray(other_joint.child_link.t_world, dtype=float)
                                @ np.append(other_local, 1.0)
                            )[:3]
                        )
                    scores.append(float(np.mean([
                        np.linalg.norm(jaw_center - other_center)
                        for other_center in other_centers
                    ])))

                open_indices[name] = 1 if scores[1] >= scores[0] else 0
                first, second = candidate_angles[name]
                joint.current_value = 0.5 * (first + second)
                self.mw.robot.update_kinematics()
        finally:
            for name, value in saved_values.items():
                joint = self.mw.robot.joints.get(name)
                if joint is not None:
                    joint.current_value = value
            self.mw.robot.update_kinematics()

        if len(open_indices) != len(joint_names):
            self._gripper_joint_endpoints = fallback
            return fallback

        # Relation slaves must follow the same normalized phase as their master.
        for slave_name, (master_name, _ratio) in slave_relations.items():
            if master_name in open_indices:
                open_indices[slave_name] = open_indices[master_name]

        endpoints = {}
        for name, values in candidate_angles.items():
            open_index = open_indices[name]
            endpoints[name] = {
                "closed": float(values[1 - open_index]),
                "open": float(values[open_index]),
            }
        self._gripper_joint_endpoints = endpoints
        return endpoints

    def _manual_gripper_joint_names(self):
        if self._gripper_selection_joint_names:
            return list(self._gripper_selection_joint_names)
        selected = [
            item.data(QtCore.Qt.UserRole)
            for item in self.joints_list.selectedItems()
            if item.data(QtCore.Qt.UserRole)
        ]
        current = self.joints_list.currentItem()
        if not selected and current is not None and current.data(QtCore.Qt.UserRole):
            selected = [current.data(QtCore.Qt.UserRole)]
        return list(dict.fromkeys(selected))

    def _apply_gripper_opening_percent(self, percent, recalculate=False):
        """Apply one normalized slider value to every configured jaw."""
        joint_names = self._manual_gripper_joint_names()
        if not joint_names:
            return {}
        if recalculate or any(name not in self._gripper_joint_endpoints for name in joint_names):
            endpoints = self._calculate_gripper_endpoint_angles(joint_names)
        else:
            endpoints = self._gripper_joint_endpoints

        fraction = float(np.clip(percent, 0.0, 100.0)) / 100.0
        targets = {}
        for name in joint_names:
            bounds = endpoints.get(name)
            if not bounds:
                continue
            target = bounds["closed"] + fraction * (bounds["open"] - bounds["closed"])
            targets[name] = float(target)
            self._apply_uniform_gripper_opening(name, target)

        self.mw.robot.update_kinematics()
        if hasattr(self.mw, "canvas") and hasattr(self.mw.canvas, "update_transforms"):
            self.mw.canvas.update_transforms(self.mw.robot)
        return targets

    def on_weld_tool_save_clicked(self):
        tool_name = self.tool_combo.currentText().strip()
        face_name = self.weld_face_name.text().strip()
        if not tool_name:
            self.weld_compile_status.setText("Please enter the welding tool filename first.")
            self.weld_compile_status.setStyleSheet("color: #d32f2f; font-size: 12px; margin-top: 4px;")
            return
        if not face_name or face_name.lower() == "nil":
            self.weld_compile_status.setText("Please pick a face in the 3D view before saving.")
            self.weld_compile_status.setStyleSheet("color: #d32f2f; font-size: 12px; margin-top: 4px;")
            return

        payload = self._build_weld_tool_payload()
        self.mw.welding_tool_config = payload
        self.mw.end_effector_tool_config = payload
        self.tool_selection_status.setText("Tool selected: Welding Tool")
        self._gripper_tool_selected = False
        self._set_gripper_confirmation_mode(True)
        self._update_end_effector_summary(
            tool_name="Welding Tool",
            detail_text=f"selected face: {face_name}",
            saved=True,
        )
        self.weld_compile_status.setText("Welding tool selected and saved successfully.")
        self.weld_compile_status.setStyleSheet("color: #388e3c; font-size: 12px; margin-top: 4px;")
        self.mw.log(f"Welding Tool saved: {payload}")
        self.mw.show_toast("Welding tool saved", "success")
        self._refresh_tool_save_state()

    def _build_paint_tool_payload(self):
        return {
            "EndEffector": {
                "ToolType": "Painting Tool",
                "ToolName": self.tool_combo.currentText().strip(),
                "NozzleFace": self.paint_face_status.text().replace("Nozzle face selected:", "").strip(),
                "TCP": [
                    self.paint_tcp_x.text().strip(),
                    self.paint_tcp_y.text().strip(),
                    self.paint_tcp_z.text().strip(),
                ],
                "Direction": [
                    self.paint_dir_x.text().strip(),
                    self.paint_dir_y.text().strip(),
                    self.paint_dir_z.text().strip(),
                ],
            }
        }

    def on_paint_tool_save_clicked(self):
        tool_name = self.tool_combo.currentText().strip()
        face_text = self.paint_face_status.text().strip().lower()
        selected_face = self.paint_face_status.text().replace("Nozzle face selected:", "").strip()
        if not tool_name:
            self.paint_tool_status.setText("Select the painting tool before saving.")
            self.paint_tool_status.setStyleSheet("color: #d32f2f; font-size: 12px; margin-top: 4px;")
            return
        if "nil" in face_text:
            self.paint_tool_status.setText("Please pick a nozzle face before saving.")
            self.paint_tool_status.setStyleSheet("color: #d32f2f; font-size: 12px; margin-top: 4px;")
            return

        payload = self._build_paint_tool_payload()
        self.mw.paint_tool_config = payload
        self.mw.end_effector_tool_config = payload
        self.tool_selection_status.setText("Tool selected: Painting Tool")
        self._gripper_tool_selected = False
        self._set_gripper_confirmation_mode(True)
        self._update_end_effector_summary(
            tool_name="Painting Tool",
            detail_text=f"nozzle face: {selected_face}",
            saved=True,
        )
        self.paint_tool_status.setText("Painting tool selected and saved successfully.")
        self.paint_tool_status.setStyleSheet("color: #388e3c; font-size: 12px; margin-top: 4px;")
        self.mw.log(f"Painting Tool saved: {payload}")
        self.mw.show_toast("Painting tool saved", "success")
        self._refresh_tool_save_state()

    def on_joint_selected(self, item):
        if item is None:
            return
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
            if hasattr(joint, 'contact_face_selection') and isinstance(joint.contact_face_selection, dict):
                pass
        else:
            self.refresh_contact_surface_ui(name)

    def on_joint_list_selection_changed(self):
        selected_items = self.joints_list.selectedItems()
        if not selected_items:
            self.gripper_face_status.setText(
                "Select the jaw joints and press Compile. Contact faces are detected automatically."
            )
            return

        selected_names = [item.data(QtCore.Qt.UserRole) for item in selected_items if item.data(QtCore.Qt.UserRole)]
        self.gripper_face_status.setText(
            f"Selected gripper joints: {', '.join(selected_names)}. Compile to create a gripper group."
        )

    def _refresh_face_selection_table(self):
        self.face_selection_table.setRowCount(0)
        for joint_name in self._gripper_selection_joint_names:
            face_info = self._gripper_face_selection_data.get(joint_name)
            row = self.face_selection_table.rowCount()
            self.face_selection_table.insertRow(row)
            self.face_selection_table.setItem(row, 0, QtWidgets.QTableWidgetItem(str(joint_name)))
            if isinstance(face_info, dict):
                face_text = f"{face_info.get('link_name', '')} - {face_info.get('surface_name', 'Auto Contact')}"
            else:
                face_text = "Not detected"

            face_cell = QtWidgets.QWidget()
            face_layout = QtWidgets.QHBoxLayout(face_cell)
            face_layout.setContentsMargins(6, 2, 4, 2)
            face_layout.setSpacing(6)
            face_label = QtWidgets.QLabel(face_text)
            face_label.setToolTip(face_text)
            face_label.setStyleSheet("color: #424242; font-size: 11px;")
            face_layout.addWidget(face_label, 1)

            select_button = QtWidgets.QPushButton("Select Face")
            select_button.setObjectName(
                f"select_gripper_contact_face_{joint_name}"
            )
            select_button.setToolTip(
                f"Select the contact face for {joint_name} that will touch the object."
            )
            select_button.setFixedSize(82, 27)
            select_button.setCursor(QtCore.Qt.PointingHandCursor)
            select_button.setStyleSheet("""
                QPushButton {
                    background-color: #1976d2;
                    color: white;
                    border: none;
                    border-radius: 4px;
                    font-size: 10px;
                    font-weight: bold;
                }
                QPushButton:hover { background-color: #1565c0; }
                QPushButton:pressed { background-color: #0d47a1; }
            """)
            select_button.clicked.connect(
                lambda _checked=False, name=joint_name:
                self._start_gripper_contact_face_selection(name)
            )
            face_layout.addWidget(select_button)
            self.face_selection_table.setCellWidget(row, 1, face_cell)
        self._refresh_gripper_save_state()

    def _start_gripper_contact_face_selection(self, joint_name):
        """Start one manual contact-face pick for a specific gripper joint."""
        joint = self.mw.robot.joints.get(joint_name)
        if (
            joint is None
            or joint.child_link is None
            or joint_name not in self._gripper_selection_joint_names
        ):
            return
        if not hasattr(self.mw, "canvas") or not hasattr(
            self.mw.canvas, "start_face_picking"
        ):
            return

        self._pending_gripper_contact_joint_name = joint_name
        self.gripper_face_status.setText(
            f"Select the contact face on '{joint.child_link.name}' for {joint_name}. "
            "Choose the surface that will touch the object."
        )
        self.gripper_face_status.setStyleSheet(
            "color: #d97706; font-size: 12px;"
        )
        self.mw.canvas.start_face_picking(
            self._on_gripper_face_selected,
            color="cyan",
        )
        self.mw.show_toast(
            f"Select the object-contact face for {joint_name}", "info"
        )

    def _gripper_joint_world_center(self, joint_name):
        joint = self.mw.robot.joints.get(joint_name)
        link = joint.child_link if joint is not None else None
        mesh = getattr(link, "mesh", None) if link is not None else None
        if link is None:
            return None

        # Prefer the actual stored contact surface center when it exists.
        # The mesh bounding-box center can be far from the jaw's true grip point
        # for curved or offset grippers, which shifts the live point.
        for attr_name in (
            "contact_surface_center_local",
            "gripping_surface_center_local",
            "paired_gripping_surface_center_local",
        ):
            local_center = getattr(joint, attr_name, None)
            if local_center is None:
                continue
            try:
                local_center = np.asarray(local_center, dtype=float).reshape(3)
            except (TypeError, ValueError):
                continue
            return (np.asarray(link.t_world, dtype=float) @ np.append(local_center, 1.0))[:3]

        if mesh is None:
            return None
        face_centers = self._mesh_face_centers(mesh)
        if face_centers is not None and len(face_centers):
            local_center = np.mean(np.asarray(face_centers, dtype=float), axis=0)
        else:
            bounds = np.asarray(mesh.bounds, dtype=float)
            if bounds.shape != (2, 3):
                return None
            local_center = np.mean(bounds, axis=0)
        return (np.asarray(link.t_world, dtype=float) @ np.append(local_center, 1.0))[:3]

    def _auto_detect_gripper_contact_faces(self, joint_names):
        """Choose the surface of each selected jaw that points into the jaw group."""
        jaw_centers = {
            name: self._gripper_joint_world_center(name)
            for name in joint_names
        }
        valid_centers = [center for center in jaw_centers.values() if center is not None]
        if len(valid_centers) < 2:
            return {}, list(joint_names)
        group_center = np.mean(np.asarray(valid_centers, dtype=float), axis=0)

        detected = {}
        failed = []
        for joint_name in joint_names:
            jaw_center = jaw_centers.get(joint_name)
            joint = self.mw.robot.joints.get(joint_name)
            candidates = self._get_surface_candidates(joint_name)
            direct_child_name = getattr(getattr(joint, "child_link", None), "name", None)
            direct_candidates = [
                candidate for candidate in candidates
                if candidate.get("link_name") == direct_child_name
            ]
            if direct_candidates:
                candidates = direct_candidates
            if jaw_center is None or not candidates:
                failed.append(joint_name)
                continue

            inward = group_center - jaw_center
            inward_length = float(np.linalg.norm(inward))
            if inward_length <= 1e-9:
                failed.append(joint_name)
                continue
            inward /= inward_length

            def candidate_rank(candidate):
                center = np.asarray(candidate.get("world_center", jaw_center), dtype=float)
                normal = np.asarray(candidate.get("world_normal", np.zeros(3)), dtype=float)
                normal_length = float(np.linalg.norm(normal))
                alignment = float(np.dot(normal / normal_length, inward)) if normal_length > 1e-9 else -1.0
                inward_projection = float(np.dot(center - jaw_center, inward))
                return inward_projection, alignment, float(candidate.get("area", 0.0))

            candidate = max(candidates, key=candidate_rank)
            local_normal = np.asarray(candidate.get("local_normal", [0.0, 0.0, 1.0]), dtype=float).reshape(3)
            world_normal = np.asarray(candidate.get("world_normal", [0.0, 0.0, 1.0]), dtype=float).reshape(3)
            if float(np.dot(world_normal, inward)) < 0.0:
                local_normal = -local_normal
                world_normal = -world_normal

            detected[joint_name] = {
                "link_name": candidate.get("link_name", self.mw.robot.joints[joint_name].child_link.name),
                "surface_name": "Auto Contact Face",
                "world_center": np.asarray(candidate.get("world_center"), dtype=float).reshape(3),
                "world_normal": world_normal,
                "local_center": np.asarray(candidate.get("local_center"), dtype=float).reshape(3),
                "local_normal": local_normal,
            }
        return detected, failed

    def _resolve_gripper_selection_joint(self, link_name):
        """Return the selected gripper joint that owns the clicked link, if any."""
        for joint_name in self._gripper_selection_joint_names:
            joint = self.mw.robot.joints.get(joint_name)
            if joint is None or joint.child_link is None:
                continue
            if getattr(joint.child_link, 'name', None) == link_name:
                return joint_name
        return None

    def on_gripper_compile_clicked(self):
        selected_items = self.joints_list.selectedItems()
        if not selected_items:
            self.gripper_face_status.setText("Please select at least two joints before compiling.")
            self.gripper_face_status.setStyleSheet("color: #d32f2f; font-size: 12px;")
            return

        selected_names = [item.data(QtCore.Qt.UserRole) for item in selected_items if item.data(QtCore.Qt.UserRole)]
        if len(selected_names) < 2:
            self.gripper_face_status.setText("Please select at least two joints to create a gripper group.")
            self.gripper_face_status.setStyleSheet("color: #d32f2f; font-size: 12px;")
            return

        for joint_name in selected_names:
            joint = self.mw.robot.joints.get(joint_name)
            if joint is not None:
                joint.is_gripper = True
        self._set_active_gripper_context(selected_names)

        self._gripper_selection_joint_names = list(selected_names)
        self._pending_gripper_contact_joint_name = None
        detected_faces, failed_names = self._auto_detect_gripper_contact_faces(selected_names)
        self._gripper_face_selection_queue = list(failed_names)
        self._gripper_face_selection_data = {
            joint_name: detected_faces.get(joint_name)
            for joint_name in selected_names
        }
        self._gripper_joint_endpoints = {}
        self._gripper_alignment_face_data = None
        self.gripper_alignment_btn.setEnabled(True)
        self.gripper_alignment_status.setText(
            "Gripping face alignment not selected. The Live Point will be centered between all jaw faces."
        )
        self.gripper_alignment_status.setStyleSheet("color: #d97706; font-size: 12px;")

        if not failed_names:
            face_centers = [detected_faces[name]["world_center"] for name in selected_names]
            midpoint = np.mean(np.asarray(face_centers, dtype=float), axis=0)
            self._gripper_live_point_world = midpoint
            self._calculate_gripper_endpoint_angles(selected_names)
            self.gripper_live_point_preview.setText(
                f"Live Point Preview: ({midpoint[0]:.2f}, {midpoint[1]:.2f}, {midpoint[2]:.2f})"
            )

        self._refresh_face_selection_table()
        if not failed_names:
            self.gripper_face_status.setText(
                f"Compile complete. Contact faces detected automatically for: {', '.join(selected_names)}. "
                "Now select the gripping face whose center will become the Live Point."
            )
            self.gripper_face_status.setStyleSheet("color: #388e3c; font-size: 12px;")
            self.mw.show_toast("Gripper contact faces detected automatically", "success")
            self.mw.log(f"Gripper group auto-compiled from joints: {', '.join(selected_names)}")
            return

        self.gripper_face_status.setText(
            f"Automatic face detection needs a mesh for: {', '.join(failed_names)}. Click only those jaw faces."
        )
        self.gripper_face_status.setStyleSheet("color: #d97706; font-size: 12px;")
        self.mw.show_toast("Some jaw faces need manual selection", "warning")
        if hasattr(self.mw, "canvas") and hasattr(self.mw.canvas, 'start_face_picking'):
            self.mw.canvas.start_face_picking(
                self._on_gripper_face_selected,
                color="cyan",
                keep_active=True,
            )

    def _on_gripper_face_selected(self, link_name, world_center=None, world_normal=None):
        if not self._gripper_selection_joint_names:
            return

        if not link_name or world_center is None or world_normal is None:
            self.gripper_face_status.setText("Please select a valid face on the jaw surface.")
            self.gripper_face_status.setStyleSheet("color: #d32f2f; font-size: 12px;")
            return

        selected = self._pending_gripper_contact_joint_name
        if selected is not None:
            pending_joint = self.mw.robot.joints.get(selected)
            expected_link_name = getattr(
                getattr(pending_joint, "child_link", None), "name", None
            )
            if link_name != expected_link_name:
                self.gripper_face_status.setText(
                    f"Select a face on '{expected_link_name}' for {selected}; "
                    f"'{link_name}' belongs to another component."
                )
                self.gripper_face_status.setStyleSheet(
                    "color: #d32f2f; font-size: 12px;"
                )
                return
        else:
            selected = self._resolve_gripper_selection_joint(link_name)
        if selected is None:
            self.gripper_face_status.setText(
                f"Clicked face '{link_name}' is not part of the currently selected gripper joints."
            )
            self.gripper_face_status.setStyleSheet("color: #d32f2f; font-size: 12px;")
            return

        joint = self.mw.robot.joints.get(selected)
        child_link = joint.child_link if joint is not None else None
        world_center = np.asarray(world_center, dtype=float).reshape(3)
        world_normal = np.asarray(world_normal, dtype=float).reshape(3)
        if child_link is None:
            return
        inverse_world = np.linalg.inv(np.asarray(child_link.t_world, dtype=float))
        local_center = (inverse_world @ np.append(world_center, 1.0))[:3]
        local_normal = inverse_world[:3, :3] @ world_normal
        normal_length = float(np.linalg.norm(local_normal))
        if normal_length > 1e-9:
            local_normal = local_normal / normal_length

        face_entry = {
            "link_name": link_name,
            "surface_name": "Manual Contact Face",
            "world_center": world_center,
            "world_normal": world_normal,
            "local_center": local_center,
            "local_normal": local_normal,
        }
        self._gripper_face_selection_data[selected] = face_entry
        self._pending_gripper_contact_joint_name = None
        self._refresh_face_selection_table()

        self.gripper_face_status.setText(
            f"Face assigned for {selected}. One face is now stored for this jaw."
        )
        self.gripper_face_status.setStyleSheet("color: #616161; font-size: 12px;")

        if self._gripper_face_selection_data and all(self._gripper_face_selection_data.get(name) for name in self._gripper_selection_joint_names):
            face_centers = []
            for joint_name in self._gripper_selection_joint_names:
                face_info = self._gripper_face_selection_data.get(joint_name)
                if isinstance(face_info, dict):
                    face_centers.append(face_info["world_center"])
            if face_centers:
                midpoint = np.mean(np.asarray(face_centers, dtype=float), axis=0)
                self._gripper_live_point_world = midpoint
                self._calculate_gripper_endpoint_angles(self._gripper_selection_joint_names)
                self.gripper_live_point_preview.setText(
                    f"Live Point Preview: ({midpoint[0]:.2f}, {midpoint[1]:.2f}, {midpoint[2]:.2f})"
                )

    def on_gripper_opening_slider_changed(self):
        min_val = int(self.gripper_min_input.value())
        max_val = int(self.gripper_max_input.value())
        if min_val > max_val:
            self.gripper_max_input.setValue(min_val)
            max_val = min_val

        targets = self._apply_gripper_opening_percent(100, recalculate=True)

        self.gripper_opening_label.setText(
            f"Opening Preview: Min {min_val}, Max {max_val} ({len(targets)} jaws)"
        )

    def _apply_uniform_gripper_opening(self, joint_name, target_deg):
        """Apply the same opening angle to one gripper joint and its linked relation chain."""
        joint = self.mw.robot.joints.get(joint_name)
        if joint is None:
            return

        target = float(np.clip(target_deg, joint.min_limit, joint.max_limit))
        joint.current_value = target
        self._propagate_relation(joint_name, target)

    def on_gripper_save_clicked(self):
        if not self._has_valid_gripper_tool_selection():
            self.gripper_face_status.setText("Please select the Gripper Tool before saving.")
            self.gripper_face_status.setStyleSheet("color: #d32f2f; font-size: 12px;")
            self.gripper_save_btn.setEnabled(False)
            return

        selected_names = self._selected_gripper_joint_names()
        if len(selected_names) < 2:
            self.gripper_face_status.setText("A gripper group needs at least two joints.")
            self.gripper_face_status.setStyleSheet("color: #d32f2f; font-size: 12px;")
            self.gripper_save_btn.setEnabled(False)
            return

        if any(not isinstance(self._gripper_face_selection_data.get(name), dict) or not self._gripper_face_selection_data.get(name) for name in selected_names):
            self.gripper_face_status.setText("Assign exactly one face to every selected gripper joint before saving.")
            self.gripper_face_status.setStyleSheet("color: #d32f2f; font-size: 12px;")
            self.gripper_save_btn.setEnabled(False)
            return

        if not isinstance(self._gripper_alignment_face_data, dict):
            self.gripper_alignment_status.setText(
                "Select one gripping face for alignment; the Live Point will be the jaw centroid."
            )
            self.gripper_alignment_status.setStyleSheet("color: #d32f2f; font-size: 12px;")
            self.gripper_save_btn.setEnabled(False)
            return

        if self._bind_gripper_live_point_to_selected_face() is None:
            self.gripper_alignment_status.setText(
                "The jaw-face centroid could not be saved as the Live Point."
            )
            self.gripper_alignment_status.setStyleSheet(
                "color: #d32f2f; font-size: 12px;"
            )
            return
        payload = self._build_end_effector_payload()
        self.mw.gripper_tool_config = payload
        self.mw.end_effector_tool_config = payload
        self._gripper_tool_selected = True
        self.tool_combo.setCurrentText("Gripper Tool")
        self.tool_selection_status.setText("Tool selected: Gripper Tool")
        self._set_gripper_confirmation_mode(True)
        jaw_count = int(payload.get("EndEffector", {}).get("JawCount", 0))
        self._update_end_effector_summary(tool_name="Gripper Tool", jaw_count=jaw_count, saved=True)
        self.mw.log(f"Gripper Tool saved: {payload}")
        self.mw.log("Gripper Tool selected and saved.")
        self.gripper_face_status.setText("Gripper Tool selected and saved successfully.")
        self.gripper_face_status.setStyleSheet("color: #388e3c; font-size: 12px;")
        self.mw.show_toast("Gripper Tool configuration saved", "success")

    def _update_end_effector_summary(self, tool_name=None, jaw_count=None, detail_text=None, saved=False):
        """Refresh the summary card shown under the end-effector tool picker."""
        if hasattr(self, "end_effector_summary_tool"):
            if tool_name:
                display_tool = "gripper" if str(tool_name).strip().lower() == "gripper tool" else str(tool_name).strip().lower()
                self.end_effector_summary_tool.setText(f"selected tool: {display_tool}")
            else:
                self.end_effector_summary_tool.setText("selected tool: -")

        if hasattr(self, "end_effector_summary_detail"):
            if detail_text:
                self.end_effector_summary_detail.setText(f"tool info: {detail_text}")
            elif jaw_count is None:
                self.end_effector_summary_detail.setText("tool info: -")
            else:
                self.end_effector_summary_detail.setText(f"tool info: no of jaws: {int(jaw_count)}")

        if hasattr(self, "end_effector_summary_note"):
            if saved and tool_name:
                self.end_effector_summary_note.setText("Saved selection is ready to use in the selected tool mode.")
            else:
                self.end_effector_summary_note.setText("Press Save to lock the current tool configuration.")

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
        face_centers = self._mesh_face_centers(mesh)
        if face_centers is not None and len(face_centers):
            local_center = np.mean(np.asarray(face_centers, dtype=float), axis=0)
        else:
            bounds = np.array(mesh.bounds, dtype=float)
            local_center = (bounds[0] + bounds[1]) / 2.0
        bounds = np.array(mesh.bounds, dtype=float)
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

            face_centers = self._mesh_face_centers(link.mesh)
            if face_centers is not None and len(face_centers):
                local_center = np.mean(np.asarray(face_centers, dtype=float), axis=0)
            else:
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
        self.mw.log(
            "Contact surface selection via the canvas button has been removed."
        )
        self.mw.show_toast(
            "Select Gripper Surface has been removed",
            "info",
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
        targets = self._apply_gripper_opening_percent(value)
        if not targets:
            return

        state = "Closed" if value <= 0 else "Open" if value >= 100 else "Opening"
        self.gripper_opening_control_label.setText(
            f"Gripper Opening (all jaws): {int(value)}% - {state}"
        )

        # Send to Hardware (Digital Twin Sync)
        if hasattr(self.mw, 'serial_mgr') and self.mw.serial_mgr.is_connected:
            speed = float(getattr(self.mw, 'current_speed', 50))
            for joint_name, target in targets.items():
                self.mw.serial_mgr.send_command(joint_name, target, speed=speed)

        self.refresh_contact_surface_ui(next(iter(targets)))

    # ========================================
    # CONTACT FACE SELECTION HANDLERS
    # ========================================

    # Contact face support removed from this panel.
