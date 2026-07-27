from PyQt5 import QtWidgets, QtCore, QtGui
import numpy as np
import trimesh


class TypeOnlyDoubleSpinBox(QtWidgets.QDoubleSpinBox):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
    def stepBy(self, steps): pass
    def wheelEvent(self, event): event.ignore()

class SimulationPanel(QtWidgets.QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.sliders = {}
        self.matrix_labels = {}
        
        self._target_gripper_angles = {} # For smooth finger animation
        self._env_collision_manager = None # Performance Cache for Rigid Rigidity
        self._pick_place_tcp_orientation = None
        self._pick_place_original_object_rotation = None
        self.grip_original_rotation = None
        self.grip_translation_offset = None
        
        # Live Point Locking
        self.live_point_locked = False  # Is LP currently locked?
        self.locked_live_point = None   # Fixed [x, y, z] in cm when locked
        
        self.init_ui()

    def init_ui(self):
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setContentsMargins(10, 10, 10, 10)
        self.layout.setSpacing(10)

        title = QtWidgets.QLabel("SIMULATION MODE")
        title.setStyleSheet("font-weight: bold; font-size: 16px; color: #1976d2; margin-bottom: 10px;")
        title.setAlignment(QtCore.Qt.AlignCenter)
        self.layout.addWidget(title)
        
        # --- TAB NAVIGATION ---
        tab_layout = QtWidgets.QHBoxLayout()
        tab_layout.setSpacing(10)
        
        self.joints_btn = self.create_tab_button("Joints", "assets/panel.png")
        self.matrices_btn = self.create_tab_button("Matrices", "assets/matrices.png")
        self.objects_btn = self.create_tab_button("Objects", "assets/simulation.png")
        
        self.joints_btn.clicked.connect(lambda: self.switch_view(0))
        self.matrices_btn.clicked.connect(lambda: self.switch_view(1))
        self.objects_btn.clicked.connect(lambda: self.switch_view(2))
        
        tab_layout.addWidget(self.joints_btn)
        tab_layout.addWidget(self.matrices_btn)
        tab_layout.addWidget(self.objects_btn)
        self.layout.addLayout(tab_layout)
        
        # --- STACKED VIEW ---
        self.stack = QtWidgets.QStackedWidget()
        self.layout.addWidget(self.stack)
        
        # 1. Joints View (Sliders)
        self.joints_view = QtWidgets.QWidget()
        self.joints_layout = QtWidgets.QVBoxLayout(self.joints_view)
        self.joints_layout.setContentsMargins(0,0,0,0)
        
        # Scroll Area for sliders
        scroll_joints = QtWidgets.QScrollArea()
        scroll_joints.setWidgetResizable(True)
        scroll_joints.setFrameShape(QtWidgets.QFrame.NoFrame)
        
        self.scroll_content = QtWidgets.QWidget()
        self.scroll_layout = QtWidgets.QVBoxLayout(self.scroll_content)
        self.scroll_layout.setAlignment(QtCore.Qt.AlignTop)
        self.scroll_layout.setSpacing(15)
        
        scroll_joints.setWidget(self.scroll_content)
        self.joints_layout.addWidget(scroll_joints)
        self.stack.addWidget(self.joints_view)
        
        # 2. Matrices View
        self.matrices_view = QtWidgets.QWidget()
        self.matrices_layout = QtWidgets.QVBoxLayout(self.matrices_view)
        self.matrices_layout.setContentsMargins(0,0,0,0)
        
        scroll_matrices = QtWidgets.QScrollArea()
        scroll_matrices.setWidgetResizable(True)
        scroll_matrices.setFrameShape(QtWidgets.QFrame.NoFrame)
        
        self.matrices_content = QtWidgets.QWidget()
        self.matrices_scroll_layout = QtWidgets.QVBoxLayout(self.matrices_content)
        self.matrices_scroll_layout.setAlignment(QtCore.Qt.AlignTop)
        self.matrices_scroll_layout.setSpacing(15)
        
        scroll_matrices.setWidget(self.matrices_content)
        self.matrices_layout.addWidget(scroll_matrices)
        self.stack.addWidget(self.matrices_view)

        # 3. Simulation Objects View (Consolidated from floating panel)
        self.objects_view = QtWidgets.QWidget()
        self.objects_layout = QtWidgets.QVBoxLayout(self.objects_view)
        self.objects_layout.setContentsMargins(0, 5, 0, 0)
        self.objects_layout.setSpacing(10)

        # Header Buttons
        btn_container = QtWidgets.QWidget()
        btn_layout = QtWidgets.QVBoxLayout(btn_container)
        btn_layout.setContentsMargins(0, 0, 0, 0)
        btn_layout.setSpacing(8)

        self.import_btn = QtWidgets.QPushButton("📦 Import Object")
        self.import_btn.setFixedHeight(45)
        self.import_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.import_btn.setStyleSheet("""
            QPushButton {
                background-color: white;
                color: #1976d2;
                border: 2px solid #1976d2;
                border-radius: 8px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover { background-color: #e3f2fd; }
        """)
        self.import_btn.clicked.connect(self.import_simulation_object)
        btn_layout.addWidget(self.import_btn)

        operation_group = QtWidgets.QGroupBox("OBJECT OPERATION")
        operation_group.setStyleSheet("""
            QGroupBox {
                border: 1px solid #cfd8dc;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 12px;
                font-weight: bold;
                color: #37474f;
            }
        """)
        operation_layout = QtWidgets.QVBoxLayout(operation_group)
        operation_layout.setSpacing(7)

        self.operation_combo = QtWidgets.QComboBox()
        self.operation_combo.setFixedHeight(36)
        self.operation_combo.addItem("Pick & Place", "pick_place")
        self.operation_combo.addItem("Welding", "welding")
        self.operation_combo.addItem("Painting", "painting")
        self.operation_combo.setStyleSheet("""
            QComboBox {
                background: white;
                border: 1px solid #90a4ae;
                border-radius: 6px;
                padding: 5px 10px;
                font-size: 13px;
                font-weight: bold;
                color: #263238;
            }
        """)
        self.operation_combo.currentIndexChanged.connect(self._on_operation_changed)
        operation_layout.addWidget(self.operation_combo)

        self.operation_help = QtWidgets.QLabel()
        self.operation_help.setWordWrap(True)
        self.operation_help.setStyleSheet("color: #607d8b; font-size: 11px;")
        operation_layout.addWidget(self.operation_help)

        process_row = QtWidgets.QHBoxLayout()
        self.process_points_label = QtWidgets.QLabel("Path points")
        self.process_points_label.setStyleSheet("color: #455a64; font-size: 12px;")
        self.process_points_sb = QtWidgets.QSpinBox()
        self.process_points_sb.setRange(2, 40)
        self.process_points_sb.setValue(8)
        self.process_points_sb.setToolTip("Number of evenly spaced tool positions between P1 and P2")
        process_row.addWidget(self.process_points_label)
        process_row.addWidget(self.process_points_sb)
        process_row.addStretch()
        operation_layout.addLayout(process_row)

        paint_row = QtWidgets.QHBoxLayout()
        self.paint_color_label = QtWidgets.QLabel("Paint colour")
        self.paint_color_label.setStyleSheet("color: #455a64; font-size: 12px;")
        self.paint_color_combo = QtWidgets.QComboBox()
        self.paint_color_combo.addItem("Safety Yellow", "#f9a825")
        self.paint_color_combo.addItem("Signal Blue", "#1976d2")
        self.paint_color_combo.addItem("Machine Red", "#d32f2f")
        self.paint_color_combo.addItem("Industrial Green", "#388e3c")
        paint_row.addWidget(self.paint_color_label)
        paint_row.addWidget(self.paint_color_combo, 1)
        operation_layout.addLayout(paint_row)

        self.operation_status = QtWidgets.QLabel("Ready to configure Pick & Place")
        self.operation_status.setWordWrap(True)
        self.operation_status.setStyleSheet(
            "background: #eceff1; color: #455a64; border-radius: 5px; padding: 7px; font-size: 11px;"
        )
        operation_layout.addWidget(self.operation_status)
        btn_layout.addWidget(operation_group)

        self.update_btn = QtWidgets.QPushButton("🔄 Update Position")
        self.update_btn.setFixedHeight(45)
        self.update_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.update_btn.setToolTip("Automatically move the selected object to P1 coordinates")
        self.update_btn.setStyleSheet("""
            QPushButton {
                background-color: #388e3c;
                color: white;
                border: none;
                border-radius: 8px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover { background-color: #2e7d32; }
        """)
        self.update_btn.clicked.connect(self.update_object_position)
        btn_layout.addWidget(self.update_btn)

        self.pick_place_btn = QtWidgets.QPushButton("Run Pick & Place")
        self.pick_place_btn.setFixedHeight(45)
        self.pick_place_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.pick_place_btn.setToolTip("Run the pick-and-place sequence using the selected object, P1, and P2")
        self.pick_place_btn.setStyleSheet("""
            QPushButton {
                background-color: #388e3c;
                color: white;
                border: none;
                border-radius: 8px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover { background-color: #2e7d32; }
            QPushButton:pressed { background-color: #1b5e20; }
        """)
        self.pick_place_btn.clicked.connect(self.run_selected_operation)
        btn_layout.addWidget(self.pick_place_btn)

        self.start_btn = QtWidgets.QPushButton("Stop Operation")
        self.start_btn.setFixedHeight(38)
        self.start_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.start_btn.setToolTip("Stop the active object operation")
        self.start_btn.setEnabled(False)
        self.start_btn.setStyleSheet("""
            QPushButton {
                background-color: white;
                color: #c62828;
                border: 1px solid #c62828;
                border-radius: 8px;
                font-weight: bold;
                font-size: 13px;
            }
            QPushButton:hover { background-color: #ffebee; }
            QPushButton:disabled { color: #b0bec5; border-color: #cfd8dc; }
        """)
        self.start_btn.clicked.connect(self.stop_current_operation)
        btn_layout.addWidget(self.start_btn)

        self.objects_layout.addWidget(btn_container)

        # Simulation State
        self.is_sim_active = False
        self.gripped_object = None
        self.grip_offset = None # Relative transform
        self.grip_translation_offset = None
        
        self.sim_timer = QtCore.QTimer(self)
        self.sim_timer.timeout.connect(self._on_sim_tick)
        
        # Sequenced Motion State
        self.sim_state = "IDLE" 
        self.target_joint_values = {} 
        self.active_joint_index = 0
        self.current_tcp = None
        self.motion_speed = 5.0 # Initial default
        self.active_operation = None
        self._process_path_cm = []
        self._process_path_index = 0
        self._process_trace_points_world = []
        # Objects List
        list_label = QtWidgets.QLabel("Simulation Objects:")
        list_label.setStyleSheet("font-weight: bold; color: #424242; font-size: 13px;")
        self.objects_layout.addWidget(list_label)

        self.objects_list = QtWidgets.QListWidget()
        self.objects_list.setFixedHeight(180)
        self.objects_list.setStyleSheet("""
            QListWidget {
                border: 1px solid #ddd;
                border-radius: 6px;
                background: white;
            }
            QListWidget::item { padding: 8px; border-bottom: 1px solid #f0f0f0; }
            QListWidget::item:selected { background: #e3f2fd; color: #1976d2; }
        """)
        self.objects_list.itemClicked.connect(self.main_window.on_sim_object_clicked)
        self.objects_layout.addWidget(self.objects_list)

        # --- OBJECT PROPERTIES PANEL ---
        self.prop_group = QtWidgets.QGroupBox("Object Info")
        self.prop_group.setStyleSheet("""
            QGroupBox {
                border: 1px solid #ddd;
                border-radius: 6px;
                margin-top: 10px;
                padding-top: 12px;
                font-weight: bold;
                color: #555;
            }
        """)
        prop_vbox = QtWidgets.QVBoxLayout(self.prop_group)
        prop_vbox.setSpacing(5)
        
        self.dim_label = QtWidgets.QLabel("Dimensions: ---")
        self.dim_label.setStyleSheet("font-size: 11px; color: #1976d2; font-weight: bold;")
        prop_vbox.addWidget(self.dim_label)
        
        self.pos_label = QtWidgets.QLabel("Current Pos: ---")
        self.pos_label.setStyleSheet("font-size: 11px; color: #424242;")
        prop_vbox.addWidget(self.pos_label)
        
        self.capture_btn = QtWidgets.QPushButton("🎯 Set Object as P1")
        self.capture_btn.setFixedHeight(30)
        self.capture_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.capture_btn.setStyleSheet("""
            QPushButton {
                background-color: #f5f5f5;
                color: #1976d2;
                border: 1px solid #1976d2;
                border-radius: 4px;
                font-size: 11px;
                font-weight: bold;
                margin-top: 5px;
            }
            QPushButton:hover { background-color: #e3f2fd; }
        """)
        self.capture_btn.clicked.connect(self.capture_object_to_p1)
        prop_vbox.addWidget(self.capture_btn)
        
        self.set_lp_btn = QtWidgets.QPushButton("🎯 Set as Live Point (TCP)")
        self.set_lp_btn.setFixedHeight(30)
        self.set_lp_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.set_lp_btn.setToolTip("Set the selected object as the Live Point (Tool Center Point)")
        self.set_lp_btn.setStyleSheet("""
            QPushButton {
                background-color: #f5f5f5;
                color: #d32f2f;
                border: 1px solid #d32f2f;
                border-radius: 4px;
                font-size: 11px;
                font-weight: bold;
                margin-top: 5px;
            }
            QPushButton:hover { background-color: #ffebee; }
        """)
        self.set_lp_btn.clicked.connect(self.set_custom_lp)
        prop_vbox.addWidget(self.set_lp_btn)
        
        self.objects_layout.addWidget(self.prop_group)

        # Coordinate Grid
        coord_container = QtWidgets.QWidget()
        coord_layout = QtWidgets.QVBoxLayout(coord_container)
        coord_layout.setContentsMargins(5, 5, 5, 5)
        coord_layout.setSpacing(5)

        points_grid = QtWidgets.QGridLayout()
        points_grid.setSpacing(6)

        # Exposing widgets to main_window for Mixin access
        self.main_window.sim_objects_list = self.objects_list

        # P1 Row
        self.p1_label = QtWidgets.QLabel("P1")
        self.p1_label.setStyleSheet("font-weight: bold; color: #1976d2; font-size: 13px;")
        self.pick_x = self.create_coord_sb("#1976d2")
        self.pick_y = self.create_coord_sb("#1976d2")
        self.pick_z = self.create_coord_sb("#1976d2")
        
        points_grid.addWidget(self.p1_label, 0, 0)
        points_grid.addWidget(self.pick_x, 0, 1)
        points_grid.addWidget(self.pick_y, 0, 2)
        points_grid.addWidget(self.pick_z, 0, 3)

        # P2 Row
        self.p2_label = QtWidgets.QLabel("P2")
        self.p2_label.setStyleSheet("font-weight: bold; color: #388E3C; font-size: 13px;")
        self.place_x = self.create_coord_sb("#388E3C")
        self.place_y = self.create_coord_sb("#388E3C")
        self.place_z = self.create_coord_sb("#388E3C")
        
        points_grid.addWidget(self.p2_label, 1, 0)
        points_grid.addWidget(self.place_x, 1, 1)
        points_grid.addWidget(self.place_y, 1, 2)
        points_grid.addWidget(self.place_z, 1, 3)

        # LP Row
        lp_lbl = QtWidgets.QLabel("LP")
        lp_lbl.setStyleSheet("font-weight: bold; color: #D32F2F; font-size: 13px;")
        self.live_x = self.create_coord_sb("#D32F2F")
        self.live_y = self.create_coord_sb("#D32F2F")
        self.live_z = self.create_coord_sb("#D32F2F")
        for sb in [self.live_x, self.live_y, self.live_z]:
            sb.setReadOnly(True)

        # Lock Live Point Button
        self.lock_lp_btn = QtWidgets.QPushButton("🔓 Lock")
        self.lock_lp_btn.setFixedWidth(50)
        self.lock_lp_btn.setFixedHeight(32)
        self.lock_lp_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.lock_lp_btn.setToolTip("Click to lock the current live point. Click again to unlock.")
        self.lock_lp_btn.setStyleSheet("""
            QPushButton {
                background-color: #f5f5f5;
                color: #D32F2F;
                border: 1px solid #D32F2F;
                border-radius: 4px;
                font-size: 10px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #ffebee; }
            QPushButton:pressed { background-color: #D32F2F; color: white; }
        """)
        self.lock_lp_btn.clicked.connect(self.toggle_lock_live_point)

        points_grid.addWidget(lp_lbl, 2, 0)
        points_grid.addWidget(self.live_x, 2, 1)
        points_grid.addWidget(self.live_y, 2, 2)
        points_grid.addWidget(self.live_z, 2, 3)
        points_grid.addWidget(self.lock_lp_btn, 2, 4)

        # DIM Row (New: Industrial Dimensions)
        dim_lbl = QtWidgets.QLabel("DIM")
        dim_lbl.setStyleSheet("font-weight: bold; color: #7B1FA2; font-size: 13px;")
        dim_lbl.setToolTip("Object Dimensions (Length, Width, Height) in cm")
        self.obj_width = self.create_coord_sb("#7B1FA2")
        self.obj_depth = self.create_coord_sb("#7B1FA2")
        self.obj_height = self.create_coord_sb("#7B1FA2")
        
        points_grid.addWidget(dim_lbl, 3, 0)
        points_grid.addWidget(self.obj_width, 3, 1)
        points_grid.addWidget(self.obj_depth, 3, 2)
        points_grid.addWidget(self.obj_height, 3, 3)

        # SPEED Row
        speed_lbl = QtWidgets.QLabel("SPD")
        speed_lbl.setStyleSheet("font-weight: bold; color: #ff9800; font-size: 13px;")
        speed_lbl.setToolTip("Motion Speed (Degrees per Tick)")
        self.motion_speed_sb = QtWidgets.QDoubleSpinBox()
        self.motion_speed_sb.setRange(0.1, 20.0)
        self.motion_speed_sb.setValue(5.0)
        self.motion_speed_sb.setSuffix(" °/t")
        self.motion_speed_sb.setStyleSheet("""
            QDoubleSpinBox {
                background: white;
                color: #ff9800;
                border: 1px solid #ddd;
                border-radius: 4px;
                font-size: 12px;
                padding: 2px 4px;
                font-weight: bold;
            }
        """)
        self.motion_speed_sb.valueChanged.connect(self.update_motion_speed)
        
        points_grid.addWidget(speed_lbl, 4, 0)
        points_grid.addWidget(self.motion_speed_sb, 4, 1, 1, 3)

        # Back-link coordinates back to main_window for Mixin methods
        self.main_window.pick_x, self.main_window.pick_y, self.main_window.pick_z = self.pick_x, self.pick_y, self.pick_z
        self.main_window.place_x, self.main_window.place_y, self.main_window.place_z = self.place_x, self.place_y, self.place_z
        self.main_window.live_x, self.main_window.live_y, self.main_window.live_z = self.live_x, self.live_y, self.live_z
        self.main_window.obj_width, self.main_window.obj_depth, self.main_window.obj_height = self.obj_width, self.obj_depth, self.obj_height

        coord_layout.addLayout(points_grid)
        self.objects_layout.addWidget(coord_container)
        self.objects_layout.addStretch()

        self.stack.addWidget(self.objects_view)
        
        # Initial State
        self.switch_view(2)
        self._on_operation_changed()

    def create_coord_sb(self, color):
        sb = TypeOnlyDoubleSpinBox()
        sb.setRange(-9999, 9999)
        sb.setSuffix(" cm")
        sb.setFixedWidth(78)
        sb.setFixedHeight(32)
        sb.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        sb.setStyleSheet(f"""
            QDoubleSpinBox {{
                background: white;
                color: {color};
                border: 1px solid #ddd;
                border-radius: 4px;
                font-size: 12px;
                padding: 2px 4px;
                font-weight: bold;
            }}
            QDoubleSpinBox:focus {{ border-color: {color}; }}
        """)
        sb.valueChanged.connect(self.main_window.save_sim_object_coords)
        return sb

    def refresh_links(self):
        """Refresh simulation objects whenever this workspace becomes visible."""
        if hasattr(self.main_window, "refresh_sim_objects_list"):
            self.main_window.refresh_sim_objects_list()

    def import_simulation_object(self):
        """Import a mesh and mark it as an object used by simulation tasks."""
        self.main_window._simulation_object_import_active = True
        try:
            self.main_window.import_mesh()
        finally:
            self.main_window._simulation_object_import_active = False

    def selected_operation(self):
        return self.operation_combo.currentData() or "pick_place"

    def _on_operation_changed(self, *_):
        operation = self.selected_operation()
        settings = {
            "pick_place": (
                "Run Pick & Place",
                "P1 is the object's pick position and P2 is its place position.",
                "P1",
                "P2",
            ),
            "welding": (
                "Run Welding",
                "The welding TCP approaches P1, follows a straight weld path to P2, then retracts.",
                "START",
                "END",
            ),
            "painting": (
                "Run Painting",
                "The paint nozzle approaches P1, follows the surface path to P2, and applies the selected colour.",
                "START",
                "END",
            ),
        }
        button_text, help_text, p1_text, p2_text = settings[operation]
        self.pick_place_btn.setText(button_text)
        self.operation_help.setText(help_text)
        self.p1_label.setText(p1_text)
        self.p2_label.setText(p2_text)
        is_surface_operation = operation in ("welding", "painting")
        self.process_points_label.setVisible(is_surface_operation)
        self.process_points_sb.setVisible(is_surface_operation)
        self.paint_color_label.setVisible(operation == "painting")
        self.paint_color_combo.setVisible(operation == "painting")
        self.update_btn.setVisible(operation == "pick_place")
        self.capture_btn.setText(
            "Set Object as P1" if operation == "pick_place" else "Set Path from Object Surface"
        )
        display_name = self.operation_combo.currentText()
        self.operation_status.setText(f"Ready to configure {display_name}")

    def _set_operation_running(self, running, message=None):
        self.pick_place_btn.setEnabled(not running)
        self.operation_combo.setEnabled(not running)
        self.start_btn.setEnabled(running)
        controlled_widgets = (
            "import_btn",
            "update_btn",
            "objects_list",
            "pick_x",
            "pick_y",
            "pick_z",
            "place_x",
            "place_y",
            "place_z",
            "process_points_sb",
            "paint_color_combo",
        )
        if running:
            self._operation_widget_enabled_state = {
                name: getattr(self, name).isEnabled()
                for name in controlled_widgets
                if getattr(self, name, None) is not None
            }
        previous_states = getattr(self, "_operation_widget_enabled_state", {})
        for widget_name in controlled_widgets:
            widget = getattr(self, widget_name, None)
            if widget is not None:
                widget.setEnabled(False if running else previous_states.get(widget_name, True))
        if message:
            color = "#e8f5e9" if running else "#eceff1"
            text_color = "#2e7d32" if running else "#455a64"
            self.operation_status.setStyleSheet(
                f"background: {color}; color: {text_color}; border-radius: 5px; padding: 7px; font-size: 11px;"
            )
            self.operation_status.setText(message)

    def run_selected_operation(self):
        operation = self.selected_operation()
        if operation == "pick_place":
            self.run_pick_place_task()
        else:
            self.run_surface_operation(operation)

    def stop_current_operation(self):
        if not self.is_sim_active:
            return
        self.toggle_pick_place_sim(False)

    @staticmethod
    def _build_surface_path(start_cm, end_cm, point_count):
        """Build an inclusive straight-line process path in centimetres."""
        start = np.asarray(start_cm, dtype=float).reshape(3)
        end = np.asarray(end_cm, dtype=float).reshape(3)
        count = max(2, int(point_count))
        return [start + (end - start) * alpha for alpha in np.linspace(0.0, 1.0, count)]

    def _validate_operation_tool(self, operation):
        if operation == "pick_place":
            config = getattr(self.main_window, "gripper_tool_config", None)
            label = "Gripper Tool"
        elif operation == "welding":
            config = getattr(self.main_window, "welding_tool_config", None)
            label = "Welding Tool"
        elif operation == "painting":
            config = getattr(self.main_window, "paint_tool_config", None)
            label = "Painting Tool"
        else:
            return False

        tool_type = ""
        if isinstance(config, dict):
            tool_type = str(config.get("EndEffector", {}).get("ToolType", ""))
        active_config = getattr(self.main_window, "end_effector_tool_config", None)
        active_tool_type = ""
        if isinstance(active_config, dict):
            active_tool_type = str(active_config.get("EndEffector", {}).get("ToolType", ""))
        if tool_type.lower() != label.lower() or active_tool_type.lower() != label.lower():
            self.main_window.log(f"Select and save the {label} in End-Effector before running this operation.")
            self.main_window.show_toast(f"Save the {label} first", "warning")
            return False
        return True

    def run_surface_operation(self, operation):
        """Start a welding or painting path over the selected simulation object."""
        if self.is_sim_active:
            self.main_window.show_toast("An object operation is already running", "warning")
            return
        if operation not in ("welding", "painting"):
            return
        if not self._validate_operation_tool(operation):
            return

        obj_name = self._selected_sim_object_name()
        if not obj_name:
            self.main_window.log("Select an object before starting the surface operation.")
            self.main_window.show_toast("Select an object first", "warning")
            return

        tcp_link = self._get_tcp_link()
        if tcp_link is None:
            self.main_window.log("No TCP (Live Point) link is available for the selected tool.")
            self.main_window.show_toast("No TCP found", "warning")
            return

        start_cm = [self.pick_x.value(), self.pick_y.value(), self.pick_z.value()]
        end_cm = [self.place_x.value(), self.place_y.value(), self.place_z.value()]
        if np.linalg.norm(np.asarray(end_cm) - np.asarray(start_cm)) < 1e-6:
            self.main_window.log("The process start and end points must be different.")
            self.main_window.show_toast("Set different START and END points", "warning")
            return

        self.current_task_object = obj_name
        self.main_window.current_task_object = obj_name
        self.active_operation = operation
        self._process_path_cm = self._build_surface_path(start_cm, end_cm, self.process_points_sb.value())
        self._process_path_index = 0
        self._process_trace_points_world = []
        self._initial_joint_state = {
            name: joint.current_value for name, joint in self.main_window.robot.joints.items()
        }
        self.gripped_object = None
        self.grip_offset = None
        self.target_joint_values = {}
        self.is_sim_active = True
        self.sim_state = "SOLVE_PROCESS_APPROACH"
        operation_name = "Welding" if operation == "welding" else "Painting"
        self._set_operation_running(True, f"{operation_name} is running: approaching START")
        self.main_window.log("─" * 50)
        self.main_window.log(f"STARTING {operation_name.upper()} OPERATION")
        self.main_window.log(f"   Object      : {obj_name}")
        self.main_window.log(f"   Path points : {len(self._process_path_cm)}")
        self.main_window.log(f"   TCP Link    : {tcp_link.name}")
        self.main_window.log("─" * 50)
        self.sim_timer.start(50)
        self.main_window.show_toast(f"{operation_name} operation started", "info")

    def update_object_position(self):
        """Moves the selected simulation object to P1 coordinates and compiles the path for Pick and Place."""
        # Auto-switch to objects tab so user can see coordinates
        self.switch_view(2)
        
        current_item = self.objects_list.currentItem()
        if not current_item:
            self.main_window.log("⚠️ Select an object from the list first.")
            self.main_window.show_toast("No object selected", "warning")
            return
            
        name = current_item.text()
        if name in self.main_window.robot.links:
            link = self.main_window.robot.links[name]
            
            # --- COMPLIANCE CHECK: Base, Aligned, or Jointed cannot be moved ---
            is_aligned = False
            if hasattr(self.main_window, 'alignment_cache'):
                for (p, c), pt in self.main_window.alignment_cache.items():
                    if c == name:
                        is_aligned = True; break
            
            if link.is_base:
                reason = "Base"
            elif link.parent_joint:
                reason = "Jointed"
            elif is_aligned:
                reason = "Aligned"
            else:
                reason = None
                
            if reason:
                self.main_window.log(f"⚠️ Locked: '{name}' is {reason} and cannot be moved.")
                self.main_window.show_toast(f"{reason} is fixed", "warning")
                return

            ratio = self.main_window.canvas.grid_units_per_cm
            
            # Target P1 Position (scaled to graph units)
            px = self.pick_x.value() * ratio
            py = self.pick_y.value() * ratio
            pz = self.pick_z.value() * ratio
            
            # --- COMPILE PROCESS FOR P1 AND P2 ---
            tcp_link = self._get_tcp_link()
            if tcp_link:
                self.main_window.log("-----------------------------------------")
                self.main_window.log("🛠️ COMPILING PROCESS: P1 -> P2 Path Planning")
                self.main_window.log("-----------------------------------------")
                
                start_vals = {n: j.current_value for n, j in self.main_window.robot.joints.items()}
                _, tool_local, gap = self.main_window.get_link_tool_point(tcp_link)
                tol = 0.5 * ratio  # 0.5 cm in canvas units
                
                # Fetch object height offset for realistic targets
                _, z_offset, _ = self._get_object_grip_width()
                
                # 1. Compile P1
                p1_target = np.array([px, py, pz + z_offset])
                reached_p1 = self.main_window.robot.inverse_kinematics(
                    p1_target, tcp_link, max_iters=300, tolerance=tol, tool_offset=tool_local)
                if reached_p1:
                    self.main_window.log("🧠 Path to reach P1 (Pick Position):")
                    chain_p1 = self.main_window.robot.get_kinematic_chain(tcp_link)
                    for i, j in enumerate(chain_p1):
                        self.main_window.log(f"   Step [{i+1}] {j.name} → {j.current_value:.2f}°")
                else:
                    self.main_window.log("⚠ Error: P1 Object Center is unreachable!")
                
                # Restore to calculate P2 independently
                for n, val in start_vals.items():
                    self.main_window.robot.joints[n].current_value = val
                self.main_window.robot.update_kinematics()
                
                # 2. Compile P2
                p2_target = np.array([
                    self.place_x.value() * ratio, 
                    self.place_y.value() * ratio, 
                    self.place_z.value() * ratio + z_offset
                ])
                reached_p2 = self.main_window.robot.inverse_kinematics(
                    p2_target, tcp_link, max_iters=300, tolerance=tol, tool_offset=tool_local)
                if reached_p2:
                    self.main_window.log("🧠 Path to reach P2 (Place Position):")
                    chain_p2 = self.main_window.robot.get_kinematic_chain(tcp_link)
                    for i, j in enumerate(chain_p2):
                        self.main_window.log(f"   Step [{i+1}] {j.name} → {j.current_value:.2f}°")
                else:
                    self.main_window.log("⚠ Error: P2 Object Center is unreachable!")
                
                self.main_window.log("-----------------------------------------")
                
                # Restore state again before moving object
                for n, val in start_vals.items():
                    self.main_window.robot.joints[n].current_value = val
                self.main_window.robot.update_kinematics()
            
            # Apply transformation
            # We want the BOTTOM of the mesh to sit at (px, py, pz).
            # If the mesh's local min-Z is 'min_z', then the origin must be at 'pz - min_z'.
            t_new = np.identity(4)
            t_new[:3, :3] = link.t_offset[:3, :3] # keep rotation
            
            origin_z = pz
            if link.mesh:
                local_min_z = link.mesh.bounds[0][2]
                origin_z = pz - local_min_z
            
            t_new[:3, 3] = [px, py, origin_z]
            link.t_offset = t_new
            
            # Update visuals
            self.main_window.robot.update_kinematics()
            self.main_window.canvas.update_transforms(self.main_window.robot)
            self.main_window.log(f"✅ Object '{name}' moved to P1: ({self.pick_x.value()}, {self.pick_y.value()}, {self.pick_z.value()}) cm")
            self.main_window.show_toast(f"Moved {name} to P1 & Compiled", "success")
            # Refresh info
            self.refresh_object_info(name)

    def capture_object_to_p1(self):
        """Capture a grip point or a top-surface process path from the object."""
        name = self._selected_sim_object_name()
        if not name:
            return

        if name not in self.main_window.robot.links:
            return

        link = self.main_window.robot.links[name]
        ratio = self.main_window.canvas.grid_units_per_cm

        is_surface_operation = self.selected_operation() in ("welding", "painting")
        if link.mesh:
            b = link.mesh.bounds
            center_y = (b[0][1] + b[1][1]) / 2.0
            center_x = (b[0][0] + b[1][0]) / 2.0
            if is_surface_operation:
                local_start = np.array([b[0][0], center_y, b[1][2]])
                local_end = np.array([b[1][0], center_y, b[1][2]])
                world_start = (link.t_world @ np.append(local_start, 1.0))[:3]
                world_end = (link.t_world @ np.append(local_end, 1.0))[:3]
            else:
                local_start = np.array([center_x, center_y, b[0][2]])
                world_start = (link.t_world @ np.append(local_start, 1.0))[:3]
                world_end = None
        else:
            world_start = link.t_world[:3, 3]
            world_end = None

        pos_cm = world_start / ratio

        self.pick_x.setValue(pos_cm[0])
        self.pick_y.setValue(pos_cm[1])
        self.pick_z.setValue(pos_cm[2])

        if world_end is not None:
            end_cm = world_end / ratio
            self.place_x.setValue(end_cm[0])
            self.place_y.setValue(end_cm[1])
            self.place_z.setValue(end_cm[2])
            self.main_window.log(
                f"Surface path set for '{name}': START ({pos_cm[0]:.1f}, {pos_cm[1]:.1f}, {pos_cm[2]:.1f}) cm "
                f"to END ({end_cm[0]:.1f}, {end_cm[1]:.1f}, {end_cm[2]:.1f}) cm"
            )
        else:
            self.main_window.log(
                f"P1 set to bottom-center of '{name}': "
                f"({pos_cm[0]:.1f}, {pos_cm[1]:.1f}, {pos_cm[2]:.1f}) cm"
            )

        self.main_window.save_sim_object_coords()

    def refresh_object_info(self, name):
        """Updates the info labels and automated DIM fields for the given object."""
        if name not in self.main_window.robot.links:
            return
            
        link = self.main_window.robot.links[name]
        ratio = self.main_window.canvas.grid_units_per_cm
        
        # Dimensions
        if link.mesh:
            b = link.mesh.bounds
            size = (b[1] - b[0]) / ratio
            self.dim_label.setText(f"Dimensions: {size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f} cm")
            
            # --- AUTO-POPULATE INDUSTRIAL DIM FIELDS ---
            self.obj_width.setValue(size[0])
            self.obj_depth.setValue(size[1])
            self.obj_height.setValue(size[2])
        else:
            self.dim_label.setText("Dimensions: N/A")
            
        # Position
        pos = link.t_world[:3, 3] / ratio
        self.pos_label.setText(f"Current Pos: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) cm")

    def _selected_sim_object_name(self):
        """Return the selected imported object name, falling back to task state."""
        if hasattr(self, "objects_list"):
            current_item = self.objects_list.currentItem()
            if current_item is not None:
                name = current_item.text()
                if name in self.main_window.robot.links:
                    return name

        task_name = getattr(self, "current_task_object", None)
        if isinstance(task_name, str) and task_name in self.main_window.robot.links:
            return task_name

        return None

    def run_pick_place_task(self):
        """Start the current pick-and-place sequence after validating inputs."""
        if self.is_sim_active:
            self.main_window.log("⚠️ Pick-and-place is already running.")
            self.main_window.show_toast("Pick-and-place is already running", "warning")
            return

        if not self._validate_operation_tool("pick_place"):
            return

        obj_name = self._selected_sim_object_name()
        if not obj_name:
            self.main_window.log("⚠️ No simulation object selected. Please select an object first.")
            self.main_window.show_toast("Select an object first", "warning")
            return

        if obj_name not in self.main_window.robot.links:
            self.main_window.log("⚠️ Selected object not found in the robot model.")
            self.main_window.show_toast("Selected object is not in the robot model", "error")
            return

        items = self.objects_list.findItems(obj_name, QtCore.Qt.MatchExactly)
        if items:
            self.objects_list.setCurrentItem(items[0])
        self.current_task_object = obj_name
        self.main_window.current_task_object = obj_name

        obj_link = self.main_window.robot.links[obj_name]
        if self._known_primitive_dimensions_world(obj_link) is not None:
            # Panel-created cubes/cylinders carry authoritative dimensions and
            # transforms, so always refresh P1 from the object's current base center.
            self.refresh_object_info(obj_name)
            self.capture_object_to_p1()

        if hasattr(self.main_window, "ensure_saved_gripper_tcp"):
            repaired_tcp = self.main_window.ensure_saved_gripper_tcp()
            if repaired_tcp is not None:
                self.main_window.log(
                    f"🎯 Pick TCP synchronized to the midpoint of the saved gripper jaws on '{repaired_tcp.name}'."
                )

        tcp_link = self._get_tcp_link()
        if not tcp_link:
            self.main_window.log("⚠️ No TCP (Live Point) link found on robot.")
            self.main_window.show_toast("No TCP found", "warning")
            return

        if self.obj_height.value() == 0.0 and self.obj_width.value() == 0.0:
            self.refresh_object_info(obj_name)

        self.main_window.log("🤖 Launching pick-and-place task...")
        self.main_window.log(f"   Object : {obj_name}")
        self.main_window.log(
            f"   P1     : ({self.pick_x.value():.1f}, {self.pick_y.value():.1f}, {self.pick_z.value():.1f}) cm"
        )
        self.main_window.log(
            f"   P2     : ({self.place_x.value():.1f}, {self.place_y.value():.1f}, {self.place_z.value():.1f}) cm"
        )

        self.toggle_pick_place_sim(True)
        if self.is_sim_active:
            self.main_window.show_toast("Pick-and-place task started", "info")

    def toggle_pick_place_sim(self, checked):
        """Enable automated pick-and-place monitoring with sequential motion."""
        if checked:
            # === PRE-FLIGHT VALIDATION ===
            # 1. Verify an object is selected
            obj_name = self._selected_sim_object_name()
            if not obj_name:
                self.main_window.log("⚠️ No simulation object selected. Please select an object from the list first.")
                self.main_window.show_toast("Select an object first!", "warning")
                return

            if obj_name not in self.main_window.robot.links:
                self.main_window.log("⚠️ Selected object not found in robot model.")
                return

            # 2. Refresh dimensions from mesh if DIM fields are still zero
            if self.obj_height.value() == 0.0 and self.obj_width.value() == 0.0:
                self.refresh_object_info(obj_name)
                self.main_window.log(f"📐 Auto-populated dimensions for '{obj_name}' before simulation.")

            # 3. Verify TCP link is available
            tcp_link = self._get_tcp_link()
            if not tcp_link:
                self.main_window.log("⚠️ No TCP (Live Point) link found on robot. Cannot start simulation.")
                self.main_window.show_toast("No TCP found!", "warning")
                return

            obj_link = self.main_window.robot.links[obj_name]
            self._pick_place_original_object_rotation = np.asarray(
                obj_link.t_world[:3, :3],
                dtype=float,
            ).copy()
            self._pick_place_tcp_orientation = self._build_pick_place_alignment_orientation(
                tcp_link,
                obj_link,
            )

            # === START SEQUENCE ===
            self.is_sim_active = True
            self.active_operation = "pick_place"
            self.main_window.log("─" * 50)
            self.main_window.log("🚀 STARTING PICK-AND-PLACE SEQUENCE")
            ratio = self.main_window.canvas.grid_units_per_cm
            self.main_window.log(f"   Object : {obj_name}")
            self.main_window.log(f"   DIM    : {self.obj_width.value():.1f} x {self.obj_depth.value():.1f} x {self.obj_height.value():.1f} cm")
            self.main_window.log(f"   P1 (Pick)  : ({self.pick_x.value():.1f}, {self.pick_y.value():.1f}, {self.pick_z.value():.1f}) cm")
            self.main_window.log(f"   P2 (Place) : ({self.place_x.value():.1f}, {self.place_y.value():.1f}, {self.place_z.value():.1f}) cm")
            self.main_window.log(f"   TCP Link   : {tcp_link.name}")
            self.main_window.log("─" * 50)

            self._set_operation_running(True, "Pick & Place is running: opening the gripper")
            self.main_window.log(
                "   Flow      : align gripper -> approach with clearance -> cover object -> close jaws -> keep pose locked -> place with IK/FK"
            )

            # === Snapshot initial joint state so we can return later ===
            self._initial_joint_state = {
                n: j.current_value
                for n, j in self.main_window.robot.joints.items()
            }

            # Reset Sequence
            self.sim_state = "OPEN_GRIPPER"   # first: open gripper to object width
            self.main_window.log("📍 Initializing motion sequence from Robot Base...")
            self.gripped_object = None
            self.grip_offset = None
            self.grip_translation_offset = None
            self.target_joint_values = {}
            self._target_gripper_angles = {}  # for smooth animation
            self._gripper_contact_joint_names = set()
            self.active_joint_index = 0

            self.sim_timer.start(50)  # Ticking every 50 ms
        else:
            operation_name = (self.active_operation or "object").replace("_", " ").title()
            self.main_window.log(f"{operation_name} operation stopped.")
            self.sim_timer.stop()
            self.is_sim_active = False
            self.sim_state = "IDLE"
            self.active_operation = None
            self._set_operation_running(False, "Operation stopped. Ready to run again.")

            # Reset state
            self.gripped_object = None
            self.grip_offset = None
            self.grip_translation_offset = None
            self._pick_place_tcp_orientation = None
            self._pick_place_original_object_rotation = None
            self.grip_original_rotation = None
            if hasattr(self.main_window, "canvas") and hasattr(self.main_window.canvas, "clear_highlights"):
                self.main_window.canvas.clear_highlights()
            if hasattr(self.main_window, "canvas") and hasattr(self.main_window.canvas, "plotter") and hasattr(self.main_window.canvas.plotter, "render"):
                self.main_window.canvas.plotter.render()
            
    def _on_sim_tick(self):
        if not self.is_sim_active:
            return

        # 1. Identify TCP link
        tcp_link = self._get_tcp_link()
        if not tcp_link:
            return

        if self.active_operation in ("welding", "painting"):
            self._on_surface_operation_tick(tcp_link)
            self._sync_all_sliders()
            self.main_window.canvas.update_transforms(self.main_window.robot, render=False)
            self.main_window.update_live_ui(render=False)
            self.main_window.canvas.plotter.render()
            return

        # 2. STATE MACHINE (Industrial Sequence)
        # ──────────────────────────────────────────────────────────────────
        #  OPEN_GRIPPER      → size gripper to fit around object (with clearance)
        #  SOLVE_APPROACH_P1 → plan path to Safe Point (5cm above P1)
        #  MOVE_APPROACH_P1  → travel to safe approach point
        #  SOLVE_PICK_P1     → plan descent to exact P1
        #  MOVE_PICK_P1      → descend vertically to grip object
        #  GRIP              → close fingers to snugly grip the object
        #  SOLVE_LIFT_P1     → plan path back to Safe Point (5cm above P1)
        #  MOVE_LIFT_P1      → lift object vertically from surface
        #  SOLVE_APPROACH_P2 → plan path to Safe Point (5cm above P2)
        #  MOVE_APPROACH_P2  → transit to safe place point
        #  SOLVE_PLACE_P2    → plan descent to exact P2
        #  MOVE_PLACE_P2     → descend to place object at destination
        #  RELEASE           → open fingers, drop object at P2
        #  SOLVE_RETRACT_P2  → plan path back to Safe Point (5cm above P2)
        #  MOVE_RETRACT_P2   → retract vertically from destination
        #  DONE              → sequence complete
        # ──────────────────────────────────────────────────────────────────

        # Pick-and-place sequence:
        # align -> approach -> cover object -> close jaws -> carry with locked pose -> place
        if self.sim_state == "OPEN_GRIPPER":
            if not self._target_gripper_angles:
                grip_width, _, _ = self._get_object_grip_width()
                if grip_width > 0:
                    # Open to object width + 2 cm clearance so we don't hit it on approach
                    self._presise_gripper_for_approach()
                    self.main_window.log("👐 Opening gripper wide enough to clear the object...")
                else:
                    # No width info — open fully
                    self._target_gripper_angles = self.main_window._control_gripper_fingers(
                        close=False, apply=False
                    )
                    # If still empty (no gripper joints), skip immediately
                    if not self._target_gripper_angles:
                        self.main_window.log("ℹ️ No gripper joints found — skipping OPEN_GRIPPER.")
                        self.sim_state = (
                            "SOLVE_ALIGN_TOOL"
                            if self._pick_place_tcp_orientation is not None
                            else "SOLVE_APPROACH_P1"
                        )
                        return
                    self.main_window.log("👐 Opening gripper fully before approach...")

            done = self._move_gripper_smoothly(tcp_link)
            if done:
                self.main_window.log("✅ Gripper open. Commencing movement from Base reference to P1...")
                self._target_gripper_angles = {}
                self.sim_state = (
                    "SOLVE_ALIGN_TOOL"
                    if self._pick_place_tcp_orientation is not None
                    else "SOLVE_APPROACH_P1"
                )

        elif self.sim_state == "SOLVE_ALIGN_TOOL":
            self._solve_initial_gripper_alignment(tcp_link)

        elif self.sim_state == "MOVE_ALIGN_TOOL":
            if self._handle_sequential_motion():
                self.main_window.log(
                    "Selected gripper face is aligned with the object's base at the safe point. "
                    "Descending to grip the object..."
                )
                self.main_window.log("Alignment is locked for the rest of the pick-and-place sequence.")
                self._set_operation_running(
                    True,
                    "Pick & Place is running: tool aligned, descending to grip",
                )
                self.sim_state = "SOLVE_PICK_P1"

        elif self.sim_state == "SOLVE_APPROACH_P1":
            self._handle_state_solve(
                "P1",
                tcp_link,
                next_state="MOVE_APPROACH_P1",
                z_offset_cm=5.0,
                preserve_pick_place_orientation=self._pick_place_tcp_orientation is not None,
            )

        elif self.sim_state == "MOVE_APPROACH_P1":
            if self._handle_sequential_motion():
                self.main_window.log("📍 Reached safe approach point. Descending to P1...")
                self.sim_state = "SOLVE_PICK_P1"

        elif self.sim_state == "SOLVE_PICK_P1":
            self._handle_state_solve(
                "P1",
                tcp_link,
                next_state="MOVE_PICK_P1",
                z_offset_cm=0.0,
                preserve_pick_place_orientation=self._pick_place_tcp_orientation is not None,
            )

        elif self.sim_state == "MOVE_PICK_P1":
            if self._handle_sequential_motion():
                self.main_window.log("📍 Reached P1. Closing gripper to grip object...")
                self.main_window.log("Rotating all gripper joints together to catch the object.")
                self.sim_state = "GRIP"

        elif self.sim_state == "GRIP":
            if not self._target_gripper_angles:
                self._prepare_grip_targets(tcp_link)
            
            if self._move_gripper_smoothly(tcp_link):
                self._target_gripper_angles = {}
                if self._finalize_grip(tcp_link):
                    self.main_window.log("🧲 Object gripped between the configured jaws. Lifting from P1...")
                    self.main_window.log("The gripper-object alignment will remain unchanged during transport.")
                    self.sim_state = "SOLVE_LIFT_P1"
                else:
                    self.main_window.log("Pick aborted: both configured jaws did not contact the object.")
                    self.main_window.show_toast("Grip failed: object is not between the jaws", "warning")
                    self.target_joint_values = dict(self._initial_joint_state)
                    self.joint_chain = self.main_window.robot.get_kinematic_chain(tcp_link)
                    self.sim_state = "AUTO_RETURN"

        elif self.sim_state == "SOLVE_LIFT_P1":
            self._handle_state_solve(
                "P1",
                tcp_link,
                next_state="MOVE_LIFT_P1",
                z_offset_cm=5.0,
                preserve_pick_place_orientation=self._pick_place_tcp_orientation is not None,
            )

        elif self.sim_state == "MOVE_LIFT_P1":
            self._carry_gripped_object(tcp_link)
            if self._handle_sequential_motion():
                self.main_window.log("📍 Lift complete. Moving to P2 approach...")
                self.sim_state = "SOLVE_APPROACH_P2"

        elif self.sim_state == "SOLVE_APPROACH_P2":
            self._handle_state_solve(
                "P2",
                tcp_link,
                next_state="MOVE_APPROACH_P2",
                z_offset_cm=5.0,
                preserve_pick_place_orientation=self._pick_place_tcp_orientation is not None,
            )

        elif self.sim_state == "MOVE_APPROACH_P2":
            self._carry_gripped_object(tcp_link)
            if self._handle_sequential_motion():
                self.main_window.log("📍 Reached P2 approach point. Descending to place...")
                self.main_window.log("Descending with the same gripper orientation for IK/FK-consistent placement.")
                self.sim_state = "SOLVE_PLACE_P2"

        elif self.sim_state == "SOLVE_PLACE_P2":
            self._handle_state_solve(
                "P2",
                tcp_link,
                next_state="MOVE_PLACE_P2",
                z_offset_cm=0.0,
                preserve_pick_place_orientation=self._pick_place_tcp_orientation is not None,
            )

        elif self.sim_state == "MOVE_PLACE_P2":
            self._carry_gripped_object(tcp_link)
            if self._handle_sequential_motion():
                self.main_window.log("📍 Reached P2. Opening gripper to release object...")
                self.main_window.log("Using the solved FK pose to keep the final placement aligned.")
                self.sim_state = "RELEASE"

        elif self.sim_state == "RELEASE":
            if not self._target_gripper_angles:
                self._prepare_release_targets()
            
            if self._move_gripper_smoothly(tcp_link):
                self._finalize_release()
                self.main_window.log("📦 Object released. Retracting from P2...")
                self._target_gripper_angles = {}
                self.sim_state = "SOLVE_RETRACT_P2"

        elif self.sim_state == "SOLVE_RETRACT_P2":
            self._handle_state_solve(
                "P2",
                tcp_link,
                next_state="MOVE_RETRACT_P2",
                z_offset_cm=5.0,
                preserve_pick_place_orientation=self._pick_place_tcp_orientation is not None,
            )

        elif self.sim_state == "MOVE_RETRACT_P2":
            if self._handle_sequential_motion():
                self.main_window.log("📍 Retract complete. Auto-returning to start position...")
                
                # Setup targets for return
                if hasattr(self, '_initial_joint_state') and self._initial_joint_state:
                    self.target_joint_values = dict(self._initial_joint_state)
                    self.joint_chain = self.main_window.robot.get_kinematic_chain(tcp_link)
                    self.sim_state = "AUTO_RETURN"
                else:
                    self.sim_state = "DONE"

        elif self.sim_state == "AUTO_RETURN":
            if self._handle_sequential_motion():
                self.main_window.log("✨ Pick-and-Place sequence complete. All units at initial positions.")
                self.sim_state = "DONE"

        elif self.sim_state == "DONE":
            self.sim_timer.stop()
            self._finish_return()
            self.sim_state = "IDLE"
            return 

        # Sync UI after every tick
        self._sync_all_sliders()
        self.main_window.canvas.update_transforms(self.main_window.robot, render=False)
        self.main_window.update_live_ui(render=False)
        self.main_window.canvas.plotter.render()

    def _on_surface_operation_tick(self, tcp_link):
        """Advance one timer step of a welding or painting path."""
        operation_name = "Welding" if self.active_operation == "welding" else "Painting"

        if self.sim_state == "SOLVE_PROCESS_APPROACH":
            approach = np.array(self._process_path_cm[0], dtype=float)
            approach[2] += 5.0
            self._handle_state_solve(
                "START approach",
                tcp_link,
                next_state="MOVE_PROCESS_APPROACH",
                target_cm_override=approach,
                align_to_object=False,
            )

        elif self.sim_state == "MOVE_PROCESS_APPROACH":
            if self._handle_sequential_motion():
                self.operation_status.setText(f"{operation_name} is running: processing path point 1")
                self.sim_state = "SOLVE_PROCESS_POINT"

        elif self.sim_state == "SOLVE_PROCESS_POINT":
            point_number = self._process_path_index + 1
            target = self._process_path_cm[self._process_path_index]
            self._handle_state_solve(
                f"PATH {point_number}",
                tcp_link,
                next_state="MOVE_PROCESS_POINT",
                target_cm_override=target,
                align_to_object=False,
            )

        elif self.sim_state == "MOVE_PROCESS_POINT":
            if self._handle_sequential_motion():
                self._append_process_trace_point(self._process_path_cm[self._process_path_index])
                self._process_path_index += 1
                if self._process_path_index < len(self._process_path_cm):
                    self.operation_status.setText(
                        f"{operation_name} is running: processing path point "
                        f"{self._process_path_index + 1} of {len(self._process_path_cm)}"
                    )
                    self.sim_state = "SOLVE_PROCESS_POINT"
                else:
                    self.operation_status.setText(f"{operation_name} path complete: retracting tool")
                    self.sim_state = "SOLVE_PROCESS_RETRACT"

        elif self.sim_state == "SOLVE_PROCESS_RETRACT":
            retract = np.array(self._process_path_cm[-1], dtype=float)
            retract[2] += 5.0
            self._handle_state_solve(
                "END retract",
                tcp_link,
                next_state="MOVE_PROCESS_RETRACT",
                target_cm_override=retract,
                align_to_object=False,
            )

        elif self.sim_state == "MOVE_PROCESS_RETRACT":
            if self._handle_sequential_motion():
                self.target_joint_values = dict(self._initial_joint_state)
                self.joint_chain = self.main_window.robot.get_kinematic_chain(tcp_link)
                self.operation_status.setText(f"{operation_name} complete: returning robot to start")
                self.sim_state = "AUTO_RETURN"

        elif self.sim_state == "AUTO_RETURN":
            if self._handle_sequential_motion():
                self.sim_state = "DONE"

        elif self.sim_state == "DONE":
            self.sim_timer.stop()
            self._complete_surface_operation()
            self._finish_return()
            self.sim_state = "IDLE"

    def _append_process_trace_point(self, point_cm):
        ratio = self.main_window.canvas.grid_units_per_cm
        point_world = np.asarray(point_cm, dtype=float) * ratio
        self._process_trace_points_world.append(point_world)
        if len(self._process_trace_points_world) < 2:
            return

        try:
            import pyvista as pv

            start = self._process_trace_points_world[-2]
            end = self._process_trace_points_world[-1]
            color = "#ff6f00" if self.active_operation == "welding" else self.paint_color_combo.currentData()
            trace_name = f"object_operation_trace_{self.active_operation}_{len(self._process_trace_points_world)}"
            self.main_window.canvas.plotter.add_mesh(
                pv.Line(start, end),
                color=color,
                line_width=5 if self.active_operation == "welding" else 8,
                name=trace_name,
            )
        except Exception as exc:
            self.main_window.log(f"Operation trace could not be drawn: {exc}")

    def _complete_surface_operation(self):
        operation = self.active_operation
        obj_name = self._selected_sim_object_name()
        link = self.main_window.robot.links.get(obj_name) if obj_name else None
        result = {
            "operation": operation,
            "start_cm": np.asarray(self._process_path_cm[0]).tolist(),
            "end_cm": np.asarray(self._process_path_cm[-1]).tolist(),
            "path_points": len(self._process_path_cm),
        }
        if link is not None:
            history = list(getattr(link, "simulation_operations", []))
            history.append(result)
            link.simulation_operations = history

        if operation == "painting" and link is not None:
            paint_color = self.paint_color_combo.currentData()
            link.color = paint_color
            if hasattr(self.main_window.canvas, "set_actor_color"):
                self.main_window.canvas.set_actor_color(link.name, paint_color)
            result["colour"] = paint_color
            self.main_window.log(f"Painting complete: '{link.name}' changed to {paint_color}.")
        else:
            self.main_window.log(f"Welding complete on '{obj_name}'.")

        operation_name = "Painting" if operation == "painting" else "Welding"
        self.main_window.show_toast(f"{operation_name} operation complete", "success")

    def _known_primitive_dimensions_world(self, obj_link):
        """Return known cube/cylinder dimensions in scene units, if available."""
        metadata = getattr(obj_link, "import_metadata", {})
        if not isinstance(metadata, dict):
            return None
        object_type = str(metadata.get("object_type", "")).strip().lower()
        if object_type not in ("cube", "cylinder"):
            return None
        raw_size = metadata.get("final_size") or metadata.get("raw_size")
        try:
            dimensions_mm = np.asarray(raw_size, dtype=float).reshape(3)
        except Exception:
            return None
        if not np.all(np.isfinite(dimensions_mm)) or np.any(dimensions_mm <= 0):
            return None
        units_per_mm = self.main_window.canvas.grid_units_per_cm / 10.0
        return object_type, dimensions_mm * units_per_mm

    def _get_object_grip_width(self):
        """
        Measures the object's thickness along the gripper's opening axis
        and the world-space height of the selected sim object.
        Returns (grip_size_world, z_offset_world, obj_link)
        """
        obj_name = self._selected_sim_object_name()
        if not obj_name:
            return 0.0, 0.0, None
        if obj_name not in self.main_window.robot.links:
            return 0.0, 0.0, None

        obj_link = self.main_window.robot.links[obj_name]
        if not obj_link.mesh:
            return 0.0, 0.0, obj_link

        ratio = self.main_window.canvas.grid_units_per_cm

        known_geometry = self._known_primitive_dimensions_world(obj_link)
        if known_geometry is not None:
            object_type, dimensions_world = known_geometry
            width, depth, height = dimensions_world
            grip_width = width if object_type == "cylinder" else max(width, depth)
            self.main_window.log(
                f"📐 Known {object_type}: using stored dimensions "
                f"({width/ratio:.1f}x{depth/ratio:.1f}x{height/ratio:.1f} cm)."
            )
            return float(grip_width), float(height / 2.0), obj_link
        
        # --- NEW: Prioritize Manual User Inputs (Industrial Standard) ---
        m_w = self.obj_width.value() * ratio
        m_d = self.obj_depth.value() * ratio
        m_h = self.obj_height.value() * ratio
        
        if m_h > 0 or m_w > 0:
            # Use manual height for z_offset (centrally gripped)
            z_offset = m_h / 2.0
            # Use max of width/depth for grip width safety if mesh detection fails
            manual_grip_width = max(m_w, m_d)
            self.main_window.log(f"📐 Balancing: Using manual dimensions ({m_w/ratio:.1f}x{m_d/ratio:.1f}x{m_h/ratio:.1f} cm) for center-of-mass alignment.")
            return manual_grip_width, z_offset, obj_link

        # --- FALLBACK: Geometric detection from mesh ---
        # 1. Height calculation (consistent)
        raw_size = obj_link.mesh.bounds[1] - obj_link.mesh.bounds[0]
        R_obj = obj_link.t_world[:3, :3]
        world_extents = np.abs(R_obj @ raw_size)
        z_offset = world_extents[2] / 2.0

        # 2. Geometric Grip Width Calculation
        # To "hold perfectly", we must measure the object across the gripper's unique openings.
        tcp_link = self._get_tcp_link()
        grip_width = 0.0
        
        if tcp_link:
            _, _, geo_data = self.main_window.get_link_tool_point(tcp_link, return_vec=True)
            
            # --- Project all object mesh vertices for geometric measurement ---
            # Vertices in world space
            verts_world = (obj_link.t_world[:3, :3] @ obj_link.mesh.vertices.T).T + obj_link.t_world[:3, 3]
            
            if isinstance(geo_data, dict) and "fingers_world" in geo_data:
                # N-FINGER LOGIC: 
                # For each finger, measure thickness along the radial axis (Centroid -> Finger)
                # and tangential axes (Finger -> Finger).
                max_observed = 0.0
                centers = geo_data["fingers_world"]
                centroid = np.mean(centers, axis=0)
                
                # Axes to check:
                check_axes = []
                # Radial axes
                for c in centers:
                    v = c - centroid
                    if np.linalg.norm(v) > 1e-3:
                        check_axes.append(v / np.linalg.norm(v))
                
                # Tangential axes (Finger to Finger)
                for i in range(len(centers)):
                    for j in range(i + 1, len(centers)):
                        v = centers[i] - centers[j]
                        if np.linalg.norm(v) > 1e-3:
                            check_axes.append(v / np.linalg.norm(v))
                
                # Use the primary axis from the data if available
                if "primary_axis" in geo_data:
                    v = geo_data["primary_axis"]
                    check_axes.append(v / np.linalg.norm(v))
                
                # Hold the object "between" them: 
                # The effective grip width is the maximum chord of the object among all these axes.
                for axis in check_axes:
                    projections = verts_world @ axis
                    max_observed = max(max_observed, np.ptp(projections))
                
                grip_width = max_observed
            else:
                # FALLBACK: Use simple primary axis if data is just a vector
                grip_axis = geo_data if geo_data is not None else tcp_link.t_world[:3, 0]
                if np.linalg.norm(grip_axis) < 1e-3: grip_axis = np.array([1,0,0])
                grip_axis /= np.linalg.norm(grip_axis)
                
                projections = verts_world @ grip_axis
                grip_width = np.ptp(projections)
        else:
            # Fallback to world-space bounding box
            grip_width = max(world_extents[0], world_extents[1])


        return grip_width, z_offset, obj_link

    def _imported_object_base_rotation(self, obj_link):
        """Return the saved import rotation used as the object's stable base side."""
        metadata = getattr(obj_link, "import_metadata", {})
        if not isinstance(metadata, dict):
            return None
        rotation = metadata.get("import_world_rotation")
        if rotation is None:
            return None
        try:
            rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
        except Exception:
            return None
        if not np.all(np.isfinite(rotation)):
            return None
        return rotation

    def _ground_aligned_object_rotation(self, obj_link):
        """Return an object rotation whose base face stays parallel to the ground."""
        base_rotation = self._imported_object_base_rotation(obj_link)
        if base_rotation is None:
            base_rotation = np.asarray(getattr(obj_link, "t_world", np.eye(4)), dtype=float)[:3, :3]

        base_rotation = np.asarray(base_rotation, dtype=float).reshape(3, 3)
        if not np.all(np.isfinite(base_rotation)):
            return np.eye(3)

        # Preserve the object's heading as much as possible while forcing the
        # base face normal to point downward.
        heading = base_rotation @ np.array([1.0, 0.0, 0.0], dtype=float)
        heading[2] = 0.0
        heading_norm = float(np.linalg.norm(heading))
        if heading_norm <= 1e-9:
            heading = base_rotation @ np.array([0.0, 1.0, 0.0], dtype=float)
            heading[2] = 0.0
            heading_norm = float(np.linalg.norm(heading))
        if heading_norm <= 1e-9:
            heading = np.array([1.0, 0.0, 0.0], dtype=float)
            heading_norm = 1.0
        x_axis = heading / heading_norm
        z_axis = np.array([0.0, 0.0, 1.0], dtype=float)
        y_axis = np.cross(z_axis, x_axis)
        y_norm = float(np.linalg.norm(y_axis))
        if y_norm <= 1e-9:
            y_axis = np.array([0.0, 1.0, 0.0], dtype=float)
            y_norm = 1.0
        y_axis /= y_norm
        x_axis = np.cross(y_axis, z_axis)
        x_axis /= max(float(np.linalg.norm(x_axis)), 1e-9)

        return np.column_stack((x_axis, y_axis, z_axis))


    def _presise_gripper_for_approach(self):
        """Opens gripper fully before commencing movement to P1."""
        self._target_gripper_angles = self.main_window._control_gripper_fingers(
            close=False, apply=False
        )
        
        if self._target_gripper_angles:
            self.main_window.log("👐 Opening gripper fully for a safe approach to P1...")
            for j_name, angle in self._target_gripper_angles.items():
                self.main_window.log(f"   ∟ Main '{j_name}' target: {angle:.2f}°")



    def _prepare_grip_targets(self, tcp_link):
        """Calculates targets to close gripper snugly around the object."""
        ratio = self.main_window.canvas.grid_units_per_cm
        grip_width, _, _ = self._get_object_grip_width()
        
        # --- IMPROVED: Real-Gap Catching Logic ---
        # Instead of generic over-closing, we use the measured object thickness
        # as the target for our inner finger clearance (the real space between).
        # This ensures fingers stop exactly at the outer surface.
        # We add a tiny "TIGHTEN" factor (0.5mm) to ensure it "can not be loosen" as requested.
        TIGHTEN_FACTOR = 0.05 * ratio # 0.5mm extra squeeze
        target_gap = max(0.0, grip_width - TIGHTEN_FACTOR)
        
        if target_gap <= 0:
            target_gap = 0.05 * ratio # default safety min
            
        self.main_window.log(f"🧲 Calculating Degrees: Targeting inner space of {target_gap/ratio:.2f} cm for a secure, tight grip.")

        self._target_gripper_angles = self.main_window._control_gripper_fingers(
            close=True, target_gap_world=target_gap, apply=False
        )
        
        if self._target_gripper_angles:
            for j_name, angle in self._target_gripper_angles.items():
                self.main_window.log(f"   ∟ Main '{j_name}' calculated target: {angle:.2f}°")
                for s_id, ratio in self.main_window.robot.joint_relations.get(j_name, []):
                    self.main_window.log(f"      ∟ Slave Folding Joint '{s_id}' target: {angle * ratio:.2f}°")

    def _log_joint_angles(self, prefix="Joint angles"):
        """Print all joint angles to the terminal in one compact line."""
        robot = getattr(self.main_window, "robot", None)
        if robot is None or not getattr(robot, "joints", None):
            return
        parts = []
        for name, joint in robot.joints.items():
            parts.append(f"{name}={float(joint.current_value):.2f}°")
        self.main_window.log(f"{prefix}: " + " | ".join(parts))

    def _sync_object_to_jaw_contact(self, tcp_link):
        """Visually keep the workpiece centered between the touching jaw faces."""
        obj_name = getattr(self, "current_task_object", None)
        if not isinstance(obj_name, str) or obj_name not in self.main_window.robot.links:
            return

        obj_link = self.main_window.robot.links[obj_name]
        if not obj_link.mesh:
            return

        contact_names = sorted(
            self._gripper_contact_joint_names
            or self._contacting_configured_gripper_joints()
            or []
        )
        if len(contact_names) < 2:
            return

        contact_points = []
        object_center = np.asarray(obj_link.t_world[:3, 3], dtype=float)
        for joint_name in contact_names:
            contact_point = self._jaw_contact_point_world(joint_name, object_center)
            if contact_point is not None:
                contact_points.append(np.asarray(contact_point, dtype=float))

        if len(contact_points) < 2:
            return

        midpoint = np.mean(np.asarray(contact_points, dtype=float), axis=0)
        R_obj = self._ground_aligned_object_rotation(obj_link)

        local_center = np.asarray(obj_link.mesh.centroid, dtype=float).reshape(3)
        snapped_pose = np.eye(4)
        snapped_pose[:3, :3] = R_obj
        snapped_pose[:3, 3] = midpoint - R_obj @ local_center
        obj_link.t_offset = snapped_pose
        self.main_window.robot.update_kinematics()
        self.main_window.canvas.update_transforms(self.main_window.robot)
        self.main_window.simulation_tab.refresh_object_info(obj_name)


    def _finalize_grip(self, tcp_link):
        """Actually attaches the object to the robot after gripper finished closing."""
        _, _, obj_link = self._get_object_grip_width()
        if not obj_link or not obj_link.mesh:
            return False

        contact_names = self._contacting_configured_gripper_joints()
        contact_names.update(getattr(self, "_gripper_contact_joint_names", set()))
        configured_names = set(self.main_window._configured_gripper_joint_names())
        valid_contacts = contact_names.intersection(configured_names)
        if len(valid_contacts) < 2:
            self.main_window.log(
                f"Grip validation failed: {len(valid_contacts)} of {len(configured_names)} configured jaw joints touch the object."
            )
            return False

        if not self._object_is_between_jaws(obj_link, valid_contacts):
            self.main_window.log("Grip validation failed: jaw contacts are not on opposing sides of the object.")
            return False

        # 1. Compute the exact TCP (centroid of fingers) at this moment
        world_tcp, local_tcp, geo_data = self.main_window.get_link_tool_point(tcp_link, return_vec=True)

        # 2. Lock the object in its current snapped pose so the jaw faces keep
        # touching it visually while the grip closes.
        t_obj_perfect = obj_link.t_world.copy()
        t_obj_perfect[:3, :3] = self._ground_aligned_object_rotation(obj_link)

        # Store relative offset from Hand (TCP Link) to the snapped object pose
        inv_hand = np.linalg.inv(tcp_link.t_world)
        self.grip_offset = inv_hand @ t_obj_perfect
        self.gripped_object = obj_link.name
        self.grip_original_rotation = R_obj.copy()
        tcp_world = np.asarray(tcp_link.t_world, dtype=float)
        self.grip_translation_offset = t_obj_perfect[:3, 3] - tcp_world[:3, 3]
        
        # Apply immediately to the link offset
        carried_pose = np.eye(4)
        carried_pose[:3, :3] = self._ground_aligned_object_rotation(obj_link)
        carried_pose[:3, 3] = tcp_world[:3, 3] + self.grip_translation_offset
        obj_link.t_offset = carried_pose
        self.main_window.robot.update_kinematics()
        
        # --- PERFECT GRIP FEEDBACK ---
        self.main_window.log(f"✅ PERFECT GRIP: '{obj_link.name}' is now physically held by {len(tcp_link.child_joints)} finger components.")
        if isinstance(geo_data, dict):
            self.main_window.log(f"   Shape Data  : Reach={geo_data.get('finger_depth', 0)/10.0:.1f} cm | Gap={geo_data.get('real_gap', 0)/10.0:.1f} cm")
        
        # Visual Signal: Flash green to confirm surface contact
        orig_color = obj_link.color if hasattr(obj_link, 'color') else "silver"
        self.main_window.canvas.set_actor_color(self.gripped_object, "#4caf50")
        QtCore.QTimer.singleShot(500, lambda: self.main_window.canvas.set_actor_color(self.gripped_object, orig_color))
        
        self.main_window.show_toast(f"Held '{obj_link.name}' between fingers", "success")
        return True


    def _prepare_release_targets(self):
        """Calculates targets to open gripper fully."""
        self._target_gripper_angles = self.main_window._control_gripper_fingers(
            close=False, apply=False
        )

    def _finalize_release(self):
        """Drops the object at P2."""
        self._do_release()

    def _move_gripper_smoothly(self, tcp_link=None):
        """Moves gripper joints toward targets incrementally. Returns True if all reached."""
        if not self._target_gripper_angles:
            return True

        step_degrees = 2.0
        old_values = {
            name: float(self.main_window.robot.joints[name].current_value)
            for name in self._target_gripper_angles
            if name in self.main_window.robot.joints
        }
        proposed_values = {}
        for joint_name, target in self._target_gripper_angles.items():
            joint = self.main_window.robot.joints.get(joint_name)
            if joint is None:
                continue
            difference = float(target) - float(joint.current_value)
            if abs(difference) <= step_degrees:
                proposed_values[joint_name] = float(target)
            else:
                proposed_values[joint_name] = float(joint.current_value + np.sign(difference) * step_degrees)

        # Apply all jaws together. A relation master is not allowed to reset a
        # slave that has its own calculated target in this same map.
        if hasattr(self.main_window, "_apply_gripper_target_values"):
            self.main_window._apply_gripper_target_values(proposed_values)
        else:
            for joint_name, value in proposed_values.items():
                self.main_window.robot.joints[joint_name].current_value = value
        self.main_window.robot.update_kinematics()

        if self.sim_state == "GRIP":
            if tcp_link is None:
                tcp_link = self._get_tcp_link()
            try:
                if tcp_link is not None:
                    self._sync_object_to_jaw_contact(tcp_link)
            except Exception as exc:
                self.main_window.log(f"Grip contact sync skipped: {exc}")
            for joint_name in list(self._target_gripper_angles):
                if not self._check_gripper_collision(joint_name):
                    continue
                contacted_joints = self._contacting_configured_gripper_joints()
                proposed_values[joint_name] = old_values.get(
                    joint_name,
                    self.main_window.robot.joints[joint_name].current_value,
                )
                self._gripper_contact_joint_names.update(contacted_joints or {joint_name})
                del self._target_gripper_angles[joint_name]
                self.main_window.log(
                    f"📐 Contact: '{joint_name}' stopped at the rigid object surface."
                )

            if hasattr(self.main_window, "_apply_gripper_target_values"):
                self.main_window._apply_gripper_target_values(proposed_values)
            self.main_window.robot.update_kinematics()
            try:
                if tcp_link is not None:
                    self._sync_object_to_jaw_contact(tcp_link)
                self._log_joint_angles("Grip joint angles")
            except Exception as exc:
                self.main_window.log(f"Grip update logging skipped: {exc}")

        all_done = all(
            abs(float(target) - float(self.main_window.robot.joints[name].current_value)) < 1e-6
            for name, target in self._target_gripper_angles.items()
            if name in self.main_window.robot.joints
        )
        return all_done or not self._target_gripper_angles

    @staticmethod
    def _world_mesh_bounds(link):
        """Return axis-aligned world bounds for a link mesh."""
        mesh = getattr(link, "mesh", None)
        if mesh is None:
            return None
        vertices = np.asarray(getattr(mesh, "vertices", []), dtype=float)
        if vertices.ndim != 2 or vertices.shape[0] == 0 or vertices.shape[1] < 3:
            return None
        transform = np.asarray(link.t_world, dtype=float)
        world_vertices = (transform[:3, :3] @ vertices[:, :3].T).T + transform[:3, 3]
        return np.vstack((world_vertices.min(axis=0), world_vertices.max(axis=0)))

    def _gripper_joint_child_links(self, joint_name):
        """Return the child jaw assembly driven by one configured control joint."""
        joint = self.main_window.robot.joints.get(joint_name)
        if joint is None or joint.child_link is None:
            return []

        # A relation controls motion, not geometric ownership. Including a
        # relation slave here made the master appear to touch whichever side
        # the slave touched, so two contacts could incorrectly become one.
        configured_names = set(self.main_window._configured_gripper_joint_names())
        links = []
        seen = set()
        stack = [joint.child_link]
        while stack:
            link = stack.pop()
            if link.name in seen:
                continue
            seen.add(link.name)
            links.append(link)
            for child_joint in link.child_joints:
                if child_joint.name in configured_names and child_joint.name != joint_name:
                    continue
                if child_joint.child_link is not None:
                    stack.append(child_joint.child_link)
        return links

    @staticmethod
    def _world_mesh_vertices(link):
        """Return a link mesh's vertices transformed into world coordinates."""
        mesh = getattr(link, "mesh", None)
        vertices = np.asarray(getattr(mesh, "vertices", []), dtype=float)
        if vertices.ndim != 2 or vertices.shape[0] == 0 or vertices.shape[1] < 3:
            return np.empty((0, 3), dtype=float)
        transform = np.asarray(link.t_world, dtype=float)
        return (transform[:3, :3] @ vertices[:, :3].T).T + transform[:3, 3]

    def _jaw_contact_point_world(self, joint_name, object_center):
        """Estimate the physical jaw point nearest the workpiece center."""
        jaw_map = {}
        if hasattr(self.main_window, "_saved_gripper_jaw_map"):
            jaw_map = self.main_window._saved_gripper_jaw_map()
        jaw = jaw_map.get(joint_name, {})
        joint = self.main_window.robot.joints.get(joint_name)
        local_center = jaw.get("FaceCenterLocal")
        if joint is not None and joint.child_link is not None and local_center is not None:
            try:
                local_center = np.asarray(local_center, dtype=float).reshape(3)
                transform = np.asarray(joint.child_link.t_world, dtype=float)
                return (transform @ np.append(local_center, 1.0))[:3]
            except (TypeError, ValueError, np.linalg.LinAlgError):
                pass

        closest_point = None
        closest_distance = np.inf
        for link in self._gripper_joint_child_links(joint_name):
            mesh = getattr(link, "mesh", None)
            if mesh is None:
                continue
            transform = np.asarray(link.t_world, dtype=float)
            try:
                local_center = (
                    np.linalg.inv(transform) @ np.append(object_center, 1.0)
                )[:3]
                local_points, distances, _ = trimesh.proximity.closest_point_naive(
                    mesh, np.asarray([local_center], dtype=float)
                )
                candidate = (transform @ np.append(local_points[0], 1.0))[:3]
                distance = float(distances[0])
            except Exception:
                vertices = self._world_mesh_vertices(link)
                if len(vertices) == 0:
                    continue
                index = int(np.argmin(np.linalg.norm(vertices - object_center, axis=1)))
                candidate = vertices[index]
                distance = float(np.linalg.norm(candidate - object_center))
            if distance < closest_distance:
                closest_point = np.asarray(candidate, dtype=float)
                closest_distance = distance
        return closest_point

    def _joint_child_touches_object(self, joint_name, obj_link):
        """Check contact between one selected joint's child jaw and the workpiece."""
        jaw_links = self._gripper_joint_child_links(joint_name)
        if not jaw_links:
            return False

        try:
            manager = trimesh.collision.CollisionManager()
            manager.add_object("TARGET", obj_link.mesh, obj_link.t_world)
            for jaw_link in jaw_links:
                if jaw_link.mesh is not None and manager.in_collision_single(jaw_link.mesh, jaw_link.t_world):
                    return True
        except Exception:
            pass

        object_bounds = self._world_mesh_bounds(obj_link)
        if object_bounds is None:
            return False
        tolerance = 0.05 * self.main_window.canvas.grid_units_per_cm
        for jaw_link in jaw_links:
            jaw_bounds = self._world_mesh_bounds(jaw_link)
            if jaw_bounds is None:
                continue
            separated = np.any(
                (jaw_bounds[1] < object_bounds[0] - tolerance)
                | (jaw_bounds[0] > object_bounds[1] + tolerance)
            )
            if not separated:
                return True
        return False

    def _contacting_configured_gripper_joints(self):
        obj_name = self._selected_sim_object_name()
        obj_link = self.main_window.robot.links.get(obj_name) if obj_name else None
        if obj_link is None or obj_link.mesh is None:
            return set()
        return {
            joint_name
            for joint_name in self.main_window._configured_gripper_joint_names()
            if self._joint_child_touches_object(joint_name, obj_link)
        }

    def _object_is_between_jaws(self, obj_link, contact_names):
        mesh = getattr(obj_link, "mesh", None)
        if mesh is None:
            return False
        transform = np.asarray(obj_link.t_world, dtype=float)
        object_center = (transform @ np.append(np.asarray(mesh.centroid, dtype=float), 1.0))[:3]
        contacts = []
        for joint_name in contact_names:
            contact_point = self._jaw_contact_point_world(joint_name, object_center)
            if contact_point is None:
                continue
            vector = contact_point - object_center
            norm = np.linalg.norm(vector)
            if norm > 1e-9:
                contacts.append((joint_name, contact_point, vector / norm))

        for first_index in range(len(contacts)):
            for second_index in range(first_index + 1, len(contacts)):
                _, first_point, first_direction = contacts[first_index]
                _, second_point, second_direction = contacts[second_index]
                if float(np.dot(first_direction, second_direction)) >= -0.2:
                    continue

                # Opposing directions are necessary, but the center must also
                # project between the two contact locations along their span.
                span = second_point - first_point
                span_length_sq = float(np.dot(span, span))
                if span_length_sq <= 1e-9:
                    continue
                center_fraction = float(np.dot(object_center - first_point, span) / span_length_sq)
                if -0.05 <= center_fraction <= 1.05:
                    return True
        return False

    def _check_gripper_collision(self, joint_name=None):
        """Check contact for one selected gripper joint's child jaw assembly."""
        obj_name = self._selected_sim_object_name()
        if not obj_name:
            return False
        obj_link = self.main_window.robot.links.get(obj_name)
        if not obj_link or not obj_link.mesh:
            return False
        if joint_name is not None:
            return self._joint_child_touches_object(joint_name, obj_link)
        return bool(self._contacting_configured_gripper_joints())



    def _do_grip(self, tcp_link):
        # Redundant: replaced by _prepare_grip_targets and _finalize_grip
        pass

    def _carry_gripped_object(self, tcp_link):
        """Updates the gripped object's position every tick so it follows the TCP."""
        if not self.gripped_object:
            return
        if self.gripped_object not in self.main_window.robot.links:
            return
        if self.grip_translation_offset is None:
            if self.grip_offset is None:
                return
            self.grip_translation_offset = np.asarray(self.grip_offset, dtype=float)[:3, 3].copy()
        if self.grip_translation_offset is None:
            return

        obj_link = self.main_window.robot.links[self.gripped_object]
        tcp_world = np.asarray(tcp_link.t_world, dtype=float)
        carried_pose = np.eye(4)
        carried_pose[:3, :3] = self._ground_aligned_object_rotation(obj_link)
        carried_pose[:3, 3] = tcp_world[:3, 3] + self.grip_translation_offset
        obj_link.t_offset = carried_pose
        self.main_window.robot.update_kinematics()
        self.main_window.canvas.update_transforms(self.main_window.robot)
        self.main_window.simulation_tab.refresh_object_info(self.gripped_object)

    def _do_release(self):
        """Opens gripper and drops the gripped object at P2 with its ORIGINAL orientation."""
        # Open fingers
        self.main_window._control_gripper_fingers(close=False)
        self.main_window.robot.update_kinematics()

        if not self.gripped_object:
            return
        if self.gripped_object not in self.main_window.robot.links:
            return

        obj_link = self.main_window.robot.links[self.gripped_object]
        ratio = self.main_window.canvas.grid_units_per_cm

        # Build final transform:
        #   - Translation: P2 coordinates from the spinboxes (canvas units)
        #   - Rotation: preserve the object's import-side orientation
        t_release = np.eye(4)
        t_release[:3, :3] = self._ground_aligned_object_rotation(obj_link)

        # Place at P2 world position
        # Align mesh BASE with P2 coordinates
        p2_cm = np.array([self.place_x.value(), self.place_y.value(), self.place_z.value()])
        p2_world = p2_cm * ratio
        
        if obj_link.mesh:
            bounds = np.asarray(obj_link.mesh.bounds, dtype=float)
            base_center_local = np.array([
                0.5 * (bounds[0, 0] + bounds[1, 0]),
                0.5 * (bounds[0, 1] + bounds[1, 1]),
                bounds[0, 2],
            ])
            t_release[:3, 3] = p2_world - t_release[:3, :3] @ base_center_local
        else:
            t_release[:3, 3] = p2_world
        obj_link.t_offset = t_release
        self.main_window.robot.update_kinematics()
        self.main_window.canvas.update_transforms(self.main_window.robot)

        self.main_window.log(f"📦 RELEASED: '{self.gripped_object}' placed at P2 with original orientation.")
        self.main_window.show_toast(f"Placed {self.gripped_object} at P2", "success")

        self.gripped_object = None
        self.grip_offset = None
        self.grip_original_rotation = None
        self.grip_translation_offset = None

    def _on_task_completed(self):
        """Show a completion dialog and restore initial joint state when OK pressed."""
        self.main_window.log("🎉 Task Completed! Robot reached P2 successfully.")
        self.main_window.show_toast("Task Completed!", "success", duration=5000)

        # --- Build & Show dialog ---
        dlg = QtWidgets.QDialog(self.main_window)
        dlg.setWindowTitle("Task Completed")
        dlg.setFixedSize(360, 180)
        dlg.setStyleSheet("""
            QDialog  { background: #ffffff; }
            QLabel   { font-size: 15px; color: #212121; }
            QPushButton {
                background-color: #1976d2;
                color: white;
                border: none;
                border-radius: 8px;
                font-weight: bold;
                font-size: 14px;
                padding: 8px 30px;
            }
            QPushButton:hover { background-color: #1565c0; }
        """)

        layout = QtWidgets.QVBoxLayout(dlg)
        layout.setContentsMargins(24, 24, 24, 16)
        layout.setSpacing(16)

        # Icon + message row
        icon_row = QtWidgets.QHBoxLayout()
        icon_lbl = QtWidgets.QLabel("🎉")
        icon_lbl.setStyleSheet("font-size: 36px; color: #388e3c;")
        icon_row.addWidget(icon_lbl)

        msg_lbl = QtWidgets.QLabel(
            "<b>Task Completed!</b><br>"
            "<span style='font-size:13px; color:#555;'>"
            "The robot reached <b>P2</b> successfully.<br>"
            "Press <b>OK</b> to return to the initial position."
            "</span>"
        )
        msg_lbl.setWordWrap(True)
        icon_row.addWidget(msg_lbl, 1)
        layout.addLayout(icon_row)

        btn_row = QtWidgets.QHBoxLayout()
        btn_row.addStretch()
        ok_btn = QtWidgets.QPushButton("OK  ↩  Return to Start")
        ok_btn.setCursor(QtCore.Qt.PointingHandCursor)
        ok_btn.clicked.connect(dlg.accept)
        btn_row.addWidget(ok_btn)
        layout.addLayout(btn_row)

        dlg.exec_()   # Blocks until user presses OK

        # === Restore initial joint state ===
        self._return_to_initial_position()

    def _return_to_initial_position(self):
        """Smoothly animates joints back to the snapshot taken before simulation started."""
        if not hasattr(self, '_initial_joint_state') or not self._initial_joint_state:
            self.main_window.log("⚠️ No initial state snapshot found.")
            self._finish_return()
            return

        self.main_window.log("↩ Returning to initial position...")

        # Reuse the existing motion machinery
        self.target_joint_values = dict(self._initial_joint_state)
        self.joint_chain = self._get_tcp_chain_ordered()

        self._return_timer = QtCore.QTimer(self)
        self._return_timer.timeout.connect(self._on_return_tick)
        self._return_timer.start(50)

    def _get_tcp_chain_ordered(self):
        """Returns joint chain base->TCP (same order used for motion)."""
        tcp_link = self._get_tcp_link()
        if not tcp_link:
            return list(self.main_window.robot.joints.values())
        chain = self.main_window.robot.get_kinematic_chain(tcp_link)
        return chain  # already base->TCP

    def _on_return_tick(self):
        """Single tick of the return-to-home animation."""
        all_done = True
        for joint in self.joint_chain:
            target  = self.target_joint_values.get(joint.name, joint.current_value)
            diff    = target - joint.current_value
            if abs(diff) < 0.08:
                joint.current_value = target
                self._update_joint_and_slaves(joint, target)
                continue
            all_done = False
            RAMP, MIN_S = 15.0, 0.5
            step_mag = max(MIN_S, self.motion_speed * min(1.0, abs(diff) / RAMP))
            step = step_mag if diff > 0 else -step_mag
            new_val = target if abs(step) > abs(diff) else joint.current_value + step
            joint.current_value = new_val
            self._update_joint_and_slaves(joint, new_val)

        self._sync_all_sliders()
        self.main_window.canvas.update_transforms(self.main_window.robot)
        self.main_window.update_live_ui()

        if all_done:
            self._return_timer.stop()
            self._finish_return()

    def _finish_return(self):
        """Called after return animation completes."""
        operation_name = (self.active_operation or "simulation").replace("_", " ").title()
        self.is_sim_active = False
        self.active_operation = None
        self._set_operation_running(False, f"{operation_name} complete. Robot returned to start.")
        self.main_window.log(f"✅ Returned to initial position. {operation_name} complete.")
        self.main_window.show_toast("Back at start position", "success")

    def set_custom_lp(self):
        """Activates face picking mode to set the Live Point (TCP)."""
        self.main_window.log("Click a face on the robot to set the Live Point (TCP).")
        self.main_window.show_toast("Click a robot face in 3D view", "info")
        self.main_window.canvas.start_face_picking(self._on_custom_lp_picked, color="red")

    def _on_custom_lp_picked(self, name, world_center=None, world_normal=None):
        """Callback for when a robot face is clicked to become the Live Point."""
        if name in self.main_window.robot.links:
            picked_link = self.main_window.robot.links[name]
            link = self.main_window._resolve_rigid_tcp_link(picked_link) if hasattr(self.main_window, '_resolve_rigid_tcp_link') else picked_link
            if world_center is None:
                world_center = np.array(link.t_world[:3, 3], dtype=float)
            else:
                world_center = np.array(world_center, dtype=float).reshape(3)

            try:
                local_point = (np.linalg.inv(np.asarray(link.t_world, dtype=float)) @ np.append(world_center, 1.0))[:3]
            except Exception:
                local_point = np.array(world_center, dtype=float)

            committed_name = getattr(self.main_window, "locked_live_point_link_name", None)
            if committed_name and committed_name in self.main_window.robot.links:
                self.main_window.log(f"Live Point remains committed to '{committed_name}' from Make Robo.")
                self.main_window.show_toast(f"Live Point locked to {committed_name}", "info")
                self.main_window.update_live_ui()
                return

            self.main_window.custom_tcp_name = link.name
            self.main_window.robot.set_tcp_transform(link.name, position=local_point)
            self.main_window.robot.ensure_tcp_transform(link)
            self.main_window.log(f"Live Point (TCP) manually set to: '{link.name}' at {np.round(world_center, 2).tolist()} via 3D click.")
            self.main_window.show_toast(f"Live Point set to {link.name}", "success")
            self.main_window.update_live_ui()

            # Select it in the UI list too
            items = self.objects_list.findItems(name, QtCore.Qt.MatchExactly)
            if items:
                self.objects_list.setCurrentItem(items[0])

    def _get_tcp_link(self):
        """
        Identifies the Tool Center Point (TCP) link for the robot.
        Prioritizes user's custom selection, then 'Hand' link, then leaf link.
        """
        robot = self.main_window.robot
        
        committed_tcp_name = getattr(self.main_window, "locked_live_point_link_name", None)
        if committed_tcp_name and committed_tcp_name in robot.links:
            link = robot.links[committed_tcp_name]
            if hasattr(self.main_window, '_resolve_rigid_tcp_link'):
                link = self.main_window._resolve_rigid_tcp_link(link)
            return link

        # 1. Custom TCP Priority
        custom_tcp_name = getattr(self.main_window, "custom_tcp_name", None)
        if custom_tcp_name and custom_tcp_name in robot.links:
            link = robot.links[custom_tcp_name]
            if hasattr(self.main_window, '_resolve_rigid_tcp_link'):
                link = self.main_window._resolve_rigid_tcp_link(link)
            return link

        for link in robot.links.values():
            if getattr(link, 'custom_tcp_offset', None) is not None:
                if hasattr(self.main_window, '_resolve_rigid_tcp_link'):
                    link = self.main_window._resolve_rigid_tcp_link(link)
                return link

        # 2. Gripper Designation Priority
        for joint in robot.joints.values():
            if getattr(joint, 'is_gripper', False) and joint.parent_link is not None:
                return joint.parent_link

        # 3. Master R-relation Priority
        rel_joints = set()
        for master, slaves in robot.joint_relations.items():
            rel_joints.add(master)
            for s_id, _ in slaves:
                rel_joints.add(s_id)
        
        if rel_joints:
            parent_counts = {}
            for j_name in rel_joints:
                joint = robot.joints.get(j_name)
                if joint:
                    p_name = joint.parent_link.name
                    parent_counts[p_name] = parent_counts.get(p_name, 0) + 1
            
            if parent_counts:
                best_hand_name = max(parent_counts, key=parent_counts.get)
                return robot.links[best_hand_name]

        # 4. Leaf link priority
        for link in robot.links.values():
            if link.parent_joint and not link.child_joints:
                return link
                
        return next((l for l in robot.links.values() if not l.is_base), None)

    def _saved_gripper_alignment_face(self):
        """Return the saved tool face used for object-base plane alignment."""
        payload = getattr(self.main_window, "gripper_tool_config", None)
        if not isinstance(payload, dict):
            payload = getattr(self.main_window, "end_effector_tool_config", None)
        if not isinstance(payload, dict):
            return None
        definition = payload.get("EndEffector", payload)
        if not isinstance(definition, dict):
            return None
        face = definition.get("BaseAlignmentFace")
        return face if isinstance(face, dict) else None

    @staticmethod
    def _rotation_between_vectors(source, target):
        """Return the shortest rotation that maps one unit direction to another."""
        source = np.asarray(source, dtype=float).reshape(3)
        target = np.asarray(target, dtype=float).reshape(3)
        source_length = float(np.linalg.norm(source))
        target_length = float(np.linalg.norm(target))
        if source_length <= 1e-9 or target_length <= 1e-9:
            return None
        source /= source_length
        target /= target_length

        cosine = float(np.clip(np.dot(source, target), -1.0, 1.0))
        if cosine >= 1.0 - 1e-9:
            return np.eye(3)
        if cosine <= -1.0 + 1e-9:
            basis = np.eye(3)[int(np.argmin(np.abs(source)))]
            axis = np.cross(source, basis)
            axis /= np.linalg.norm(axis)
            return (-np.eye(3)) + (2.0 * np.outer(axis, axis))

        cross = np.cross(source, target)
        sine = float(np.linalg.norm(cross))
        skew = np.array([
            [0.0, -cross[2], cross[1]],
            [cross[2], 0.0, -cross[0]],
            [-cross[1], cross[0], 0.0],
        ])
        return np.eye(3) + skew + (skew @ skew) * ((1.0 - cosine) / (sine * sine))

    def _pick_place_alignment_axes(self, obj_link):
        """Return the selected TCP-local face normal and saved object-base normal."""
        face = self._saved_gripper_alignment_face()
        if face is None or obj_link is None:
            return None
        try:
            local_normal = np.asarray(
                face["FaceNormalTCPLocal"], dtype=float
            ).reshape(3)
        except (KeyError, TypeError, ValueError):
            return None
        local_length = float(np.linalg.norm(local_normal))
        if local_length <= 1e-9:
            return None
        local_normal /= local_length

        object_rotation = self._pick_place_original_object_rotation
        if object_rotation is None:
            object_rotation = np.asarray(obj_link.t_world[:3, :3], dtype=float)
        object_base_normal = np.asarray(object_rotation, dtype=float) @ np.array(
            [0.0, 0.0, -1.0]
        )
        normal_length = float(np.linalg.norm(object_base_normal))
        if normal_length <= 1e-9:
            return None
        return local_normal, object_base_normal / normal_length

    def _build_pick_place_alignment_orientation(
        self, tcp_link, obj_link, reference_rotation=None
    ):
        """Align the selected tool-face plane with the object's original base plane."""
        face = self._saved_gripper_alignment_face()
        axes = self._pick_place_alignment_axes(obj_link)
        if face is None or axes is None:
            if face is not None:
                self.main_window.log(
                    "Base-alignment face is invalid; Pick & Place will use position-only IK."
                )
            return None
        local_normal, object_base_normal = axes
        if reference_rotation is None:
            tcp_pose = np.asarray(
                self.main_window.robot.get_tcp_world_pose(tcp_link), dtype=float
            )
            current_rotation = tcp_pose[:3, :3]
        else:
            current_rotation = np.asarray(reference_rotation, dtype=float).reshape(3, 3)
        current_face_normal = current_rotation @ local_normal

        # Face-plane alignment accepts either normal direction. Choose the one
        # requiring the smaller tool rotation and preserve roll as much as possible.
        if float(np.dot(current_face_normal, -object_base_normal)) > float(
            np.dot(current_face_normal, object_base_normal)
        ):
            object_base_normal = -object_base_normal

        delta = self._rotation_between_vectors(current_face_normal, object_base_normal)
        if delta is None:
            return None
        target_rotation = delta @ current_rotation
        self.main_window.log(
            f"Gripper alignment active: face '{face.get('LinkID', 'selected face')}' "
            "will remain parallel to the object's base."
        )
        return target_rotation

    def _solve_initial_gripper_alignment(self, tcp_link):
        """Move to the safe P1 waypoint while aligning the selected tool face."""
        if self._pick_place_tcp_orientation is None:
            self.sim_state = "SOLVE_APPROACH_P1"
            return

        self.main_window.log(
            "Moving to the safe point above P1 while aligning the selected gripper "
            "face with the object's base..."
        )
        self._set_operation_running(
            True,
            "Pick & Place is running: approaching P1 and aligning the tool face",
        )
        self._handle_state_solve(
            "P1",
            tcp_link,
            next_state="MOVE_ALIGN_TOOL",
            z_offset_cm=5.0,
            preserve_pick_place_orientation=True,
        )

    def _handle_state_solve(
        self,
        target_name,
        tcp_link,
        next_state,
        z_offset_cm=0.0,
        target_cm_override=None,
        align_to_object=True,
        preserve_pick_place_orientation=False,
    ):
        ratio = self.main_window.canvas.grid_units_per_cm  # canvas units per cm

        # Target in canvas units (raw world space)
        if target_cm_override is not None:
            target_cm = np.asarray(target_cm_override, dtype=float).reshape(3).copy()
        elif target_name == "P1":
            target_cm = np.array([self.pick_x.value(), self.pick_y.value(), self.pick_z.value()])
        else:
            target_cm = np.array([self.place_x.value(), self.place_y.value(), self.place_z.value()])

        # Apply industry Z offset (approach/lift/retract)
        target_cm[2] += z_offset_cm
        
        target_world = target_cm * ratio  # Convert cm → canvas units

        # ADJUST TARGET FOR OBJECT BOTTOM-CENTER:
        # P1/P2 are locations for the object's BASE. 
        # The robot's TCP targets the object's CENTER by default.
        final_z_offset = 0.0
        grip_obj_link = None
        if align_to_object:
            _, base_z_offset, grip_obj_link = self._get_object_grip_width()
            _, _, geo_data = self.main_window.get_link_tool_point(tcp_link, return_vec=True)
            final_z_offset = base_z_offset
            if isinstance(geo_data, dict) and "finger_depth" in geo_data:
                reach = geo_data["finger_depth"]
            # To "cover" the object: we want the finger midpoint to be at 
            # some depth relative to the object's height. 
            # If reach > object_height: reach down so tips are at bottom (cover everything).
            # If reach < object_height: reach down to max depth.
                final_z_offset = reach / 2.0
                self.main_window.log(
                    f"📐 Coverage Mode: Setting Z-offset to {final_z_offset/ratio:.1f} cm to envelope object."
                )

        target_world[2] += final_z_offset 
        
        if final_z_offset > 0:
            self.main_window.log(f"🧠 Balancing Analysis: Targeting center-of-mass at Z={target_world[2]/ratio:.1f} cm for stable placement.")
        else:
            self.main_window.log(f"🧠 Balancing Analysis: Targeting object base for direct surface placement.")

        # Current TCP position for reference logging
        _, tool_local, gap = self.main_window.get_link_tool_point(tcp_link)
        self.main_window.robot.update_kinematics()
        tcp_now_world = (tcp_link.t_world @ np.append(tool_local, 1.0))[:3]
        tcp_now_cm = tcp_now_world / ratio

        self.main_window.log(
            f"📍 [{target_name}] Target: ({target_cm[0]:.1f}, {target_cm[1]:.1f}, {target_cm[2]:.1f}) cm  |  "
            f"TCP Position: ({tcp_now_cm[0]:.1f}, {tcp_now_cm[1]:.1f}, {tcp_now_cm[2]:.1f}) cm"
        )

        # Snapshot current joint state so we can revert after planning
        start_vals = {n: j.current_value for n, j in self.main_window.robot.joints.items()}

        # Tolerance: 0.5 cm expressed in canvas units
        tolerance_world = 0.5 * ratio

        robot = self.main_window.robot
        target_tcp_pose = robot.get_tcp_world_pose(tcp_link).copy()
        target_tcp_pose[:3, 3] = target_world

        if preserve_pick_place_orientation and self._pick_place_tcp_orientation is not None:
            target_tcp_pose[:3, :3] = np.asarray(self._pick_place_tcp_orientation, dtype=float).reshape(3, 3)

        ik_kwargs = {
            "max_iters": 1500,
            "position_tolerance": tolerance_world,
            "orientation_tolerance": 0.1,
            "orientation_weight": 0.0,
            "joint_change_weight": 0.2,
        }
        if preserve_pick_place_orientation and self._pick_place_tcp_orientation is not None:
            ik_kwargs["orientation_weight"] = 0.25
            ik_kwargs["orientation_tolerance"] = 0.2
        axis_axes = self._pick_place_alignment_axes(grip_obj_link) if align_to_object and grip_obj_link is not None else None
        use_axis_solver = (
            align_to_object
            and hasattr(robot, "inverse_kinematics_axis")
            and axis_axes is not None
            and grip_obj_link is not None
        )

        try:
            if use_axis_solver and axis_axes is not None:
                local_axis, target_axis = axis_axes
                reached, ik_info = robot.inverse_kinematics_axis(
                    target_world,
                    tcp_link,
                    local_axis,
                    target_axis,
                    max_iters=ik_kwargs["max_iters"],
                    position_tolerance=tolerance_world,
                    axis_tolerance=np.deg2rad(7.5),
                    axis_weight=0.8,
                    joint_change_weight=0.2,
                )
                if isinstance(ik_info, dict) and "tcp_pose" in ik_info:
                    target_tcp_pose = np.asarray(ik_info["tcp_pose"], dtype=float).copy()
            else:
                reached, ik_info = robot.inverse_kinematics_pose(
                    target_tcp_pose,
                    tcp_link,
                    **ik_kwargs,
                )
        except Exception as exc:
            if (
                not use_axis_solver
                and preserve_pick_place_orientation
                and self._pick_place_tcp_orientation is not None
                and hasattr(robot, "inverse_kinematics_axis")
                and align_to_object
                and grip_obj_link is not None
                and axis_axes is None
            ):
                axis_axes = self._pick_place_alignment_axes(grip_obj_link)
            if preserve_pick_place_orientation and self._pick_place_tcp_orientation is not None and axis_axes is not None and hasattr(robot, "inverse_kinematics_axis"):
                self.main_window.log(
                    f"Aligned IK failed for {target_name}, retrying with axis-based solve: {exc}"
                )
                try:
                    local_axis, target_axis = axis_axes
                    reached, ik_info = robot.inverse_kinematics_axis(
                        target_world,
                        tcp_link,
                        local_axis,
                        target_axis,
                        max_iters=900,
                        position_tolerance=tolerance_world,
                        axis_tolerance=np.deg2rad(7.5),
                        axis_weight=0.8,
                        joint_change_weight=0.2,
                    )
                    if isinstance(ik_info, dict) and "tcp_pose" in ik_info:
                        target_tcp_pose = np.asarray(ik_info["tcp_pose"], dtype=float).copy()
                except Exception as fallback_exc:
                    self.main_window.log(f"Axis-based IK failed for {target_name}: {fallback_exc}")
                    self.toggle_pick_place_sim(False)
                    return
            else:
                self.main_window.log(f"Position-only IK failed for {target_name}: {exc}")
                self.toggle_pick_place_sim(False)
                return

        fk_pose = robot.get_tcp_world_pose(tcp_link)
        fk_position_error = float(np.linalg.norm(fk_pose[:3, 3] - target_world))
        fk_orientation_error = float(
            np.arccos(
                np.clip(
                    (np.trace(target_tcp_pose[:3, :3].T @ fk_pose[:3, :3]) - 1.0) / 2.0,
                    -1.0,
                    1.0,
                )
            )
        )
        if isinstance(ik_info, dict):
            ik_info["fk_position_error"] = fk_position_error
            ik_info["fk_orientation_error"] = fk_orientation_error

        if gap:
            self.main_window.log(
                f"🤏 Gripper gap: {gap/ratio:.1f} cm — IK aligns to midpoint of fingers."
            )

        if not reached:
            self.main_window.log(f"⚠ Warning: {target_name} might be outside workspace! (best effort)")
            self.main_window.show_toast(f"{target_name} partially reachable", "warning")
        else:
            self.main_window.log(f"✅ IK Solved for {target_name} successfully.")
        if isinstance(ik_info, dict) and "fk_position_error" in ik_info:
            self.main_window.log(
                f"   FK verify: position error {ik_info['fk_position_error']/ratio:.2f} cm | "
                f"orientation error {np.rad2deg(ik_info['fk_orientation_error']):.1f}°"
            )

        # Capture solved joint angles as targets
        self.target_joint_values = {
            n: j.current_value for n, j in self.main_window.robot.joints.items()
        }
        self.joint_chain = self.main_window.robot.get_kinematic_chain(tcp_link)  # base → TCP

        planned_motion = sum(
            abs(self.target_joint_values.get(joint.name, joint.current_value) - start_vals.get(joint.name, joint.current_value))
            for joint in self.joint_chain
        )
        position_error = float(ik_info.get("fk_position_error", ik_info.get("position_error", np.inf))) if isinstance(ik_info, dict) else np.inf

        # Revert robot to start state — actual movement happens in MOVE state
        for n, val in start_vals.items():
            self.main_window.robot.joints[n].current_value = val
        self.main_window.robot.update_kinematics()

        if not self.joint_chain:
            self.main_window.log("❌ Pick motion stopped: the gripper TCP has no arm-joint chain to the robot base.")
            self.main_window.show_toast("No arm joints connected to the gripper TCP", "error")
            self.toggle_pick_place_sim(False)
            return
        if planned_motion < 1e-3 and position_error > tolerance_world:
            self.main_window.log(
                f"❌ Pick motion stopped: IK could not produce arm movement for {target_name} "
                f"(position error {position_error/ratio:.2f} cm)."
            )
            self.main_window.show_toast("Object is outside the robot workspace", "warning")
            self.toggle_pick_place_sim(False)
            return

        self._log_joint_angles(f"{target_name} IK joint angles")

        self.sim_state = next_state
        self.main_window.log(f"🧠 Motion Plan for {target_name} (reached={reached}):")
        for i, joint in enumerate(self.joint_chain):
            deg = self.target_joint_values.get(joint.name, 0)
            self.main_window.log(f"   [{i+1}] {joint.name} → {deg:.2f}°")
        
        # --- NEW: PERFECT GRIP FEEDBACK ---
        # Get actual finger count and shape data from the tool analysis
        if align_to_object:
            _, _, geo_report = self.main_window.get_link_tool_point(tcp_link, return_vec=True)
            if isinstance(geo_report, dict):
                finger_count = len(geo_report.get('fingers_world', []))
                self.main_window.log(f"🤏 Gripper Configuration: {finger_count} relationed components detected.")
                self.main_window.log(f"   Shape Data  : Reach={geo_report.get('finger_depth', 0)/ratio:.1f} cm | Gap={geo_report.get('real_gap', 0)/ratio:.1f} cm")
                self.main_window.log(f"   Grip Strategy: Centroid-averaging midpoint TCP.")
            else:
                self.main_window.log(f"🤏 Gripper Configuration: Standard leaf gripper detected.")

    def _handle_sequential_motion(self):
        """
        Moves joints simultaneously toward their target angles.
        Uses a smooth trapezoidal speed profile:
          - Accelerates when far from target (large diff)
          - Decelerates within the last few degrees (smooth arrival, no snap)
          - Snaps to exact target angle when within a tiny dead-zone
        Returns True when ALL joints have reached their targets.
        """
        all_done = True

        for joint in self.joint_chain:
            target  = self.target_joint_values.get(joint.name, joint.current_value)
            current = joint.current_value
            diff    = target - current

            # 1. Dead-zone snap
            if abs(diff) < 0.08:
                if joint.current_value != target:
                    old_snap_val = joint.current_value
                    joint.current_value = target
                    self._update_joint_and_slaves(joint, target)
                    
                    # RIGID BLOCKING: revert if snap causes collision
                    if self._check_global_collision():
                        joint.current_value = old_snap_val
                        self._update_joint_and_slaves(joint, old_snap_val)
                        all_done = False
                    continue
                continue

            all_done = False

            # --- Trapezoidal speed profile ---
            RAMP_DIST  = 15.0   
            MIN_SPEED  = 0.5    
            if abs(diff) >= RAMP_DIST:
                step_mag = self.motion_speed
            else:
                step_mag = max(MIN_SPEED, self.motion_speed * (abs(diff) / RAMP_DIST))

            step = step_mag if diff > 0 else -step_mag

            if abs(step) > abs(diff):
                new_val = target
            else:
                new_val = np.clip(current + step, joint.min_limit, joint.max_limit)

            # 2. PROPOSE MOVEMENT
            old_move_val = joint.current_value
            joint.current_value = new_val
            self._update_joint_and_slaves(joint, new_val)
            
            # RIGID BLOCKING: If we hit a simulation object, REVERT.
            if self._check_global_collision():
                joint.current_value = old_move_val
                self._update_joint_and_slaves(joint, old_move_val)
                # Note: we don't return True here; other joints might still be able to move
                # unless they are downstream in the chain.


        return all_done

    def _check_global_collision(self):
        """Checks if any robot part intersections with any independent simulation object mesh."""
        excluded_names = {self.gripped_object}
        if self.active_operation in ("pick_place", "welding", "painting"):
            excluded_names.add(self._selected_sim_object_name())

        # The active workpiece must be reachable by the process tool. Other objects
        # remain collision obstacles throughout the operation.
        sim_objs = [
            link for link in self.main_window.robot.links.values()
            if getattr(link, "is_sim_obj", False) and link.name not in excluded_names
        ]
        if not sim_objs: return False
        
        # 2. Gather robot links
        robot_links = [l for l in self.main_window.robot.links.values() 
                       if not getattr(l, 'is_sim_obj', False)]
        
        # Rebuild when an obstacle moves so collision transforms never become stale.
        signature = tuple(
            (obj.name, tuple(np.round(np.asarray(obj.t_world).reshape(-1), 6)))
            for obj in sim_objs
        )
        if self._env_collision_manager is None or signature != getattr(self, "_env_collision_signature", None):
            try:
                self._env_collision_manager = trimesh.collision.CollisionManager()
                for i, obj in enumerate(sim_objs):
                    if obj.mesh:
                        self._env_collision_manager.add_object(f"EXTERNAL_{i}", obj.mesh, obj.t_world)
                self._env_collision_signature = signature
            except Exception as exc:
                if not getattr(self, "_collision_backend_warning_shown", False):
                    self.main_window.log(f"Collision checking is unavailable: {exc}")
                    self._collision_backend_warning_shown = True
                self._env_collision_manager = None
                return False
                
        # 4. Check each robot link against the environment
        for link in robot_links:
            if link.mesh:
                # We only care about robot <-> environment collisions here
                if self._env_collision_manager.in_collision_single(link.mesh, link.t_world):
                    self.main_window.log(f"💥 Collision: Robot link '{link.name}' hit a rigid environment object.")
                    return True
                    
        # 5. Check the gripped object (if any) against the environment
        if self.gripped_object:
            gripped_link = self.main_window.robot.links.get(self.gripped_object)
            if gripped_link and gripped_link.mesh:
                if self._env_collision_manager.in_collision_single(gripped_link.mesh, gripped_link.t_world):
                    self.main_window.log(f"💥 Collision: Gripped object '{self.gripped_object}' hit another rigid object.")
                    return True

        return False


    def _update_joint_and_slaves(self, joint, val):
        """Propagates a joint value to all slave joints and refreshes kinematics."""
        if joint.name in self.main_window.robot.joint_relations:
            for slave_id, ratio in self.main_window.robot.joint_relations[joint.name]:
                slave_joint = self.main_window.robot.joints.get(slave_id)
                if slave_joint:
                    slave_joint.current_value = np.clip(
                        val * ratio,
                        slave_joint.min_limit,
                        slave_joint.max_limit
                    )
        self.main_window.robot.update_kinematics()

    def _sync_all_sliders(self):
        for name, data in self.sliders.items():
            joint = data['joint']
            val = joint.current_value
            data['slider'].blockSignals(True)
            data['slider'].setValue(int(val))
            data['slider'].blockSignals(False)
            data['spinbox'].blockSignals(True)
            data['spinbox'].setValue(float(val))
            data['spinbox'].blockSignals(False)

    def _on_sim_tick_old(self):
        # [Legacy method content replaced by state machine]
        pass

    def create_tab_button(self, text, icon_path):
        btn = QtWidgets.QPushButton(text)
        btn.setIcon(QtGui.QIcon(icon_path))
        btn.setCursor(QtCore.Qt.PointingHandCursor)
        btn.setFixedHeight(40)
        btn.setStyleSheet("""
            QPushButton {
                background-color: #f5f5f5;
                color: black;
                font-weight: bold;
                border: 1px solid #bbb;
                border-radius: 8px;
                padding: 5px;
                text-align: left;
                padding-left: 15px;
            }
            QPushButton:hover {
                background-color: #e0e0e0;
            }
        """)
        return btn

    def switch_view(self, index):
        self.stack.setCurrentIndex(index)
        
        # Style active button
        active_style = """
            QPushButton {
                background-color: #1976d2;
                color: black;
                font-weight: bold;
                border: 1px solid #0d47a1;
                border-radius: 8px;
                padding: 5px;
                text-align: left;
                padding-left: 15px;
            }
        """
        inactive_style = """
            QPushButton {
                background-color: #f5f5f5;
                color: black;
                font-weight: bold;
                border: 1px solid #bbb;
                border-radius: 8px;
                padding: 5px;
                text-align: left;
                padding-left: 15px;
            }
            QPushButton:hover {
                background-color: #e0e0e0;
            }
        """
        
        for button_index, button in enumerate((self.joints_btn, self.matrices_btn, self.objects_btn)):
            button.setStyleSheet(active_style if button_index == index else inactive_style)

        if index == 0:
            self.refresh_joints()
        elif index == 1:
            self.refresh_matrices()
        elif hasattr(self.main_window, "refresh_sim_objects_list"):
            self.main_window.refresh_sim_objects_list()

    def refresh_joints(self):
        # Reset ghost angle tracking dict on each refresh
        self._last_ghost_angle = {}  # joint_name -> last angle a ghost was snapped
        # Clear existing items in Joint View
        while self.scroll_layout.count():
            item = self.scroll_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        
        self.sliders = {}
        robot = self.main_window.robot
        
        if not robot.joints:
            no_joints_label = QtWidgets.QLabel("No joints found. Create joints in 'Joint' tab first.")
            no_joints_label.setStyleSheet("color: #757575; font-style: italic;")
            no_joints_label.setAlignment(QtCore.Qt.AlignCenter)
            self.scroll_layout.addWidget(no_joints_label)
            return

        for name, joint in robot.joints.items():
            # Skip slave joints - we only show master/independent controls
            is_slave = False
            for master, slaves in robot.joint_relations.items():
                if any(s_id == name for s_id, r in slaves):
                    is_slave = True
                    break
            if is_slave:
                continue

            container = QtWidgets.QWidget()
            layout = QtWidgets.QVBoxLayout(container)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(5)
            
            # Header
            header = QtWidgets.QLabel(f"{name} ({joint.joint_type})")
            header.setStyleSheet("font-weight: bold;")
            layout.addWidget(header)
            
            # Sub-header
            sub_header = QtWidgets.QLabel(f"{joint.parent_link.name} -> {joint.child_link.name}")
            sub_header.setStyleSheet("font-size: 10px; color: #666;")
            layout.addWidget(sub_header)
            # Slider
            slider_layout = QtWidgets.QHBoxLayout()
            
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setMinimum(int(joint.min_limit))
            slider.setMaximum(int(joint.max_limit))
            slider.setValue(int(joint.current_value))
            slider.setCursor(QtCore.Qt.PointingHandCursor)
            slider.setStyleSheet("""
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
            
            slider_layout.addWidget(slider)
            
            # Manual Spinbox
            val_spin = TypeOnlyDoubleSpinBox()
            val_spin.setRange(joint.min_limit, joint.max_limit)
            val_spin.setValue(joint.current_value)
            val_spin.setSuffix("°")
            val_spin.setFixedWidth(70)
            val_spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
            val_spin.setStyleSheet("""
                QDoubleSpinBox {
                    background: white;
                    color: #1976d2;
                    border: 1px solid #1976d2;
                    border-radius: 3px;
                    padding: 2px;
                    font-weight: bold;
                }
            """)
            slider_layout.addWidget(val_spin)
            
            layout.addLayout(slider_layout)
            
            # Separator
            line = QtWidgets.QFrame()
            line.setFrameShape(QtWidgets.QFrame.HLine)
            line.setFrameShadow(QtWidgets.QFrame.Sunken)
            line.setStyleSheet("color: #ddd;")
            layout.addWidget(line)
            
            self.scroll_layout.addWidget(container)
            
            self.sliders[name] = {
                'slider': slider,
                'spinbox': val_spin,
                'joint': joint
            }
            
            slider.valueChanged.connect(lambda val, n=name: self.on_slider_change(n, val))
            val_spin.valueChanged.connect(lambda val, n=name: self.on_slider_change(n, val))

    def refresh_matrices(self):
        # Clear existing items in Matrices View
        while self.matrices_scroll_layout.count():
            item = self.matrices_scroll_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
                
        self.matrix_labels = {}
        robot = self.main_window.robot
        
        if not robot.joints:
            label = QtWidgets.QLabel("No joints/matrices available.")
            label.setAlignment(QtCore.Qt.AlignCenter)
            self.matrices_scroll_layout.addWidget(label)
            return

        for name, joint in robot.joints.items():
            # Skip slave joints - we only show master/independent matrices
            is_slave = False
            for master, slaves in robot.joint_relations.items():
                if any(s_id == name for s_id, r in slaves):
                    is_slave = True
                    break
            if is_slave:
                continue

            container = QtWidgets.QWidget()
            layout = QtWidgets.QVBoxLayout(container)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(5)
            
            header = QtWidgets.QLabel(f"Matrix: {name} (cm)")
            header.setStyleSheet("font-weight: bold; color: #1565c0;")
            layout.addWidget(header)
            
            # Get Matrix string
            matrix = joint.get_matrix()
            mat_str = self.format_matrix(matrix)
            
            mat_label = QtWidgets.QLabel(mat_str)
            mat_label.setStyleSheet("font-family: Consolas; font-size: 24px; font-weight: bold; color: #1976d2; background: #fff; padding: 15px; border: 1px solid #ddd;")
            layout.addWidget(mat_label)
            
            self.matrices_scroll_layout.addWidget(container)
            self.matrix_labels[name] = mat_label

    def format_matrix(self, matrix):
        # Scale translation to CM based on adjustable graph ratio
        ratio = self.main_window.canvas.grid_units_per_cm
        mat_cm = np.copy(matrix)
        mat_cm[:3, 3] /= ratio
        
        lines = []
        for row in mat_cm:
            line = "  ".join([f"{val:6.2f}" for val in row])
            lines.append(f"[ {line} ]")
        return "\n".join(lines)

    def on_slider_change(self, name, value):
        if name in self.sliders:
            data = self.sliders[name]
            joint = data['joint']
            
            # Update Joint Model
            joint.current_value = float(value)
            
            # Propagation to related slave joints
            if name in self.main_window.robot.joint_relations:
                for slave_id, ratio in self.main_window.robot.joint_relations[name]:
                    slave_joint = self.main_window.robot.joints.get(slave_id)
                    if slave_joint:
                        slave_joint.current_value = float(value) * ratio
            
            # Update Spinbox and Slider without infinite loop
            if data['slider'].value() != int(value):
                data['slider'].blockSignals(True)
                data['slider'].setValue(int(value))
                data['slider'].blockSignals(False)
            if data['spinbox'].value() != float(value):
                data['spinbox'].blockSignals(True)
                data['spinbox'].setValue(float(value))
                data['spinbox'].blockSignals(False)
            
            # Update Robot Kinematics
            self.main_window.robot.update_kinematics()
            
            # Update Graphics
            self.main_window.canvas.update_transforms(self.main_window.robot)
            
            # RESTORE FIXED TARGET MARKER (if locked)
            self.restore_fixed_target_marker()
            
            # Update Live Point Coordinates UI
            if hasattr(self.main_window, 'update_live_ui'):
                self.main_window.update_live_ui()


            # --- GHOST SHADOW TRAIL ---
            # Sample a ghost every GHOST_STEP degrees of movement
            try:
                GHOST_STEP = 3  # degrees between ghost snapshots
                _last = self._last_ghost_angle.get(name, None)
                _cur_angle = float(value)
                if _last is None or abs(_cur_angle - _last) >= GHOST_STEP:
                    import numpy as _np2
                    
                    # 1. Master Joint Trail
                    _link = joint.child_link
                    _mesh = _link.mesh
                    _transform = _np2.copy(_link.t_world)
                    _col = getattr(_link, 'color', '#888888') or '#888888'
                    self.main_window.canvas.add_joint_ghost(
                        _link.name,
                        mesh=_mesh, transform=_transform,
                        color=_col
                    )
                    
                    # 2. Related (Slave) Joint Trails
                    if name in self.main_window.robot.joint_relations:
                        for slave_id, ratio in self.main_window.robot.joint_relations[name]:
                            slave_joint = self.main_window.robot.joints.get(slave_id)
                            if slave_joint:
                                s_link = slave_joint.child_link
                                s_mesh = s_link.mesh
                                s_transform = _np2.copy(s_link.t_world)
                                s_col = getattr(s_link, 'color', '#888888') or '#888888'
                                self.main_window.canvas.add_joint_ghost(
                                    s_link.name,
                                    mesh=s_mesh, transform=s_transform,
                                    color=s_col
                                )
                    
                    self._last_ghost_angle[name] = _cur_angle
            except Exception:
                pass

            # Show Speed Overlay on 3D Canvas
            self.main_window.show_speed_overlay()
            
            self.main_window.canvas.plotter.render()
            
            # Send command to hardware with current speed
            if hasattr(self.main_window, 'serial_mgr') and self.main_window.serial_mgr.is_connected:
                joint_id = name
                self.main_window.serial_mgr.send_command(joint_id, float(value), speed=float(self.main_window.current_speed))
            
            # Update Matrices if visible
            if self.stack.currentIndex() == 1:
                self.refresh_matrices()

    def update_motion_speed(self, val):
        self.motion_speed = val

    def toggle_lock_live_point(self):
        """Lock or unlock the current live point coordinates as a fixed world target."""
        if self.live_point_locked:
            # Unlock - remove fixed point marker and return to live tracking
            self.live_point_locked = False
            self.locked_live_point = None
            self.lock_lp_btn.setText("🔓 Lock")
            self.lock_lp_btn.setStyleSheet("""
                QPushButton {
                    background-color: #f5f5f5;
                    color: #D32F2F;
                    border: 1px solid #D32F2F;
                    border-radius: 4px;
                    font-size: 10px;
                    font-weight: bold;
                }
                QPushButton:hover { background-color: #ffebee; }
                QPushButton:pressed { background-color: #D32F2F; color: white; }
            """)
            # Remove fixed point marker from canvas
            if hasattr(self.main_window.canvas, 'plotter'):
                try:
                    if "fixed_live_point_marker" in self.main_window.canvas.plotter.renderer.actors:
                        self.main_window.canvas.plotter.remove_actor("fixed_live_point_marker")
                    self.main_window.canvas.plotter.render()
                except:
                    pass
            self.main_window.log("🔓 Live Point UNLOCKED - now tracking current robot position")
            self.main_window.show_toast("Live Point Unlocked", "info")
        else:
            # Lock - capture current live point as FIXED WORLD TARGET
            current_x = self.live_x.value()
            current_y = self.live_y.value()
            current_z = self.live_z.value()
            
            self.locked_live_point = (current_x, current_y, current_z)
            self.live_point_locked = True
            self.lock_lp_btn.setText("🔒 Lock")
            self.lock_lp_btn.setStyleSheet("""
                QPushButton {
                    background-color: #D32F2F;
                    color: white;
                    border: 1px solid #D32F2F;
                    border-radius: 4px;
                    font-size: 10px;
                    font-weight: bold;
                }
                QPushButton:hover { background-color: #c62828; }
                QPushButton:pressed { background-color: #b71c1c; }
            """)
            self.main_window.log(f"🔒 Live Point LOCKED (FIXED WORLD TARGET) at: X={current_x:.2f}, Y={current_y:.2f}, Z={current_z:.2f} cm")
            self.main_window.show_toast(f"🎯 Target Locked: ({current_x:.1f}, {current_y:.1f}, {current_z:.1f}) cm", "success")
            
            # Visualize FIXED TARGET POINT in 3D view as a large red sphere that NEVER MOVES
            try:
                import pyvista as pv
                ratio = self.main_window.canvas.grid_units_per_cm
                pos_world = np.array([current_x, current_y, current_z]) * ratio
                
                # Remove old marker if it exists
                try:
                    if "fixed_live_point_marker" in self.main_window.canvas.plotter.renderer.actors:
                        self.main_window.canvas.plotter.remove_actor("fixed_live_point_marker")
                except:
                    pass
                
                # Add LARGE target sphere at FIXED world position
                sphere = pv.Sphere(radius=2.0 * ratio, center=pos_world)
                
                # Add as static target marker (bright red with white outline)
                self.main_window.canvas.plotter.add_mesh(
                    sphere,
                    color="#FF0000",           # Bright red
                    opacity=0.5,              # Semi-transparent so robot can pass through
                    name="fixed_live_point_marker",
                    show_edges=True,
                    edge_color="white",
                    edge_width=3,
                    line_width=3
                )
                
                # Also add a crosshair at the target point for reference
                crosshair_size = 3.0 * ratio
                lines = [
                    [pos_world - np.array([crosshair_size, 0, 0]), pos_world + np.array([crosshair_size, 0, 0])],
                    [pos_world - np.array([0, crosshair_size, 0]), pos_world + np.array([0, crosshair_size, 0])],
                    [pos_world - np.array([0, 0, crosshair_size]), pos_world + np.array([0, 0, crosshair_size])]
                ]
                for line in lines:
                    line_mesh = pv.Line(line[0], line[1])
                    self.main_window.canvas.plotter.add_mesh(line_mesh, color="white", line_width=2)
                
                self.main_window.canvas.plotter.render()
                self.main_window.log("✅ Target visualization added - Large red sphere shows locked target position")
            except Exception as e:
                self.main_window.log(f"⚠️ Could not visualize target point: {e}")

    def restore_fixed_target_marker(self):
        """Restore the fixed target marker if it was accidentally removed during canvas updates."""
        if not self.live_point_locked or self.locked_live_point is None:
            return
        
        try:
            # Check if marker still exists
            if "fixed_live_point_marker" in self.main_window.canvas.plotter.renderer.actors:
                return  # Already exists
            
            # Restore the marker
            import pyvista as pv
            ratio = self.main_window.canvas.grid_units_per_cm
            current_x, current_y, current_z = self.locked_live_point
            pos_world = np.array([current_x, current_y, current_z]) * ratio
            
            # Re-add target sphere
            sphere = pv.Sphere(radius=2.0 * ratio, center=pos_world)
            self.main_window.canvas.plotter.add_mesh(
                sphere,
                color="#FF0000",
                opacity=0.5,
                name="fixed_live_point_marker",
                show_edges=True,
                edge_color="white",
                edge_width=3
            )
            
            # Re-add crosshair
            crosshair_size = 3.0 * ratio
            lines = [
                [pos_world - np.array([crosshair_size, 0, 0]), pos_world + np.array([crosshair_size, 0, 0])],
                [pos_world - np.array([0, crosshair_size, 0]), pos_world + np.array([0, crosshair_size, 0])],
                [pos_world - np.array([0, 0, crosshair_size]), pos_world + np.array([0, 0, crosshair_size])]
            ]
            for line in lines:
                line_mesh = pv.Line(line[0], line[1])
                self.main_window.canvas.plotter.add_mesh(line_mesh, color="white", line_width=2)
            
            self.main_window.canvas.plotter.render()
            self.main_window.log("🔄 Fixed target marker restored")
        except Exception as e:
            self.main_window.log(f"⚠️ Could not restore target marker: {e}")

