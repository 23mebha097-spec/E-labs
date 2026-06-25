from PyQt5 import QtWidgets, QtCore, QtGui
import numpy as np
import json
import os
import re
from core.path_planner import WorkspacePlan, PathTrajectory, PathPlanner
from core.path_executor import PathExecutor, ExecutionState

class ImportDropZone(QtWidgets.QLabel):
    fileDropped = QtCore.pyqtSignal(str)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setText("Drag & Drop File Here\n(SVG, DXF, CSV, JSON)")
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setStyleSheet("""
            QLabel {
                border: 2px dashed #90caf9;
                border-radius: 8px;
                background-color: #f5f5f5;
                color: #1e88e5;
                font-weight: bold;
                padding: 20px;
            }
            QLabel:hover {
                background-color: #e1f5fe;
                border-color: #1e88e5;
            }
        """)
        self.setAcceptDrops(True)
        
    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.acceptProposedAction()
            self.setStyleSheet("""
                QLabel {
                    border: 2px dashed #1e88e5;
                    border-radius: 8px;
                    background-color: #e1f5fe;
                    color: #0d47a1;
                    font-weight: bold;
                    padding: 20px;
                }
            """)
            
    def dragLeaveEvent(self, event):
        self.setStyleSheet("""
            QLabel {
                border: 2px dashed #90caf9;
                border-radius: 8px;
                background-color: #f5f5f5;
                color: #1e88e5;
                font-weight: bold;
                padding: 20px;
            }
        """)

    def dropEvent(self, event):
        for url in event.mimeData().urls():
            filepath = url.toLocalFile()
            if filepath:
                self.fileDropped.emit(filepath)
                break
        self.dragLeaveEvent(None)


class SpeedProfileWidget(QtWidgets.QWidget):
    """
    A premium custom painted canvas widget that plots speed and acceleration curves.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.timestamps = np.array([])
        self.velocities = np.array([])
        self.accelerations = np.array([])
        self.setMinimumHeight(150)
        
    def set_data(self, t, v, a):
        self.timestamps = np.array(t, dtype=float)
        self.velocities = np.array(v, dtype=float)
        self.accelerations = np.array(a, dtype=float)
        self.update()
        
    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        
        # Dark premium background matching theme
        painter.fillRect(self.rect(), QtGui.QColor("#1e1e1e"))
        
        w = self.width()
        h = self.height()
        margin = 15
        
        # Grid line drawing
        painter.setPen(QtGui.QPen(QtGui.QColor("#333333"), 1, QtCore.Qt.DashLine))
        for i in range(1, 5):
            y_line = margin + i * (h - 2 * margin) // 5
            painter.drawLine(margin, y_line, w - margin, y_line)
            
        if len(self.timestamps) < 2:
            # Draw empty placeholder text
            painter.setPen(QtGui.QColor("#777777"))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, "No trajectory generated")
            return
            
        t_min, t_max = self.timestamps[0], self.timestamps[-1]
        v_mags = np.linalg.norm(self.velocities, axis=1) if self.velocities.ndim > 1 else self.velocities
        a_mags = np.linalg.norm(self.accelerations, axis=1) if self.accelerations.ndim > 1 else self.accelerations
        
        v_max = np.max(v_mags) if len(v_mags) > 0 else 1.0
        a_max = np.max(a_mags) if len(a_mags) > 0 else 1.0
        
        if v_max <= 1e-4: v_max = 1.0
        if a_max <= 1e-4: a_max = 1.0
        if t_max - t_min <= 1e-4: t_max = t_min + 1.0
        
        # Function to map t, val to widget coords
        def get_pt(t_val, val, val_max):
            x = margin + int((t_val - t_min) / (t_max - t_min) * (w - 2 * margin))
            y = h - margin - int(val / val_max * (h - 2 * margin))
            return QtCore.QPoint(x, y)
            
        # Draw Velocity Curve (Vibrant Royal Blue / Cyan)
        v_path = QtGui.QPainterPath()
        v_path.moveTo(get_pt(self.timestamps[0], v_mags[0], v_max))
        for t, v_val in zip(self.timestamps[1:], v_mags[1:]):
            v_path.lineTo(get_pt(t, v_val, v_max))
            
        painter.setPen(QtGui.QPen(QtGui.QColor("#00b0ff"), 2, QtCore.Qt.SolidLine))
        painter.drawPath(v_path)
        
        # Draw Acceleration Curve (Amber / Coral Orange)
        a_path = QtGui.QPainterPath()
        a_path.moveTo(get_pt(self.timestamps[0], a_mags[0], a_max))
        for t, a_val in zip(self.timestamps[1:], a_mags[1:]):
            a_path.lineTo(get_pt(t, a_val, a_max))
            
        painter.setPen(QtGui.QPen(QtGui.QColor("#ff9100"), 1.5, QtCore.Qt.SolidLine))
        painter.drawPath(a_path)
        
        # Draw labels
        font = painter.font()
        font.setPointSize(8)
        painter.setFont(font)
        painter.setPen(QtGui.QColor("#00b0ff"))
        painter.drawText(margin, margin + 12, f"V_max: {v_max:.1f} cm/s")
        painter.setPen(QtGui.QColor("#ff9100"))
        painter.drawText(w - margin - 100, margin + 12, f"A_max: {a_max:.1f} cm/s²")


class PathPlanningPanel(QtWidgets.QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.mw = main_window
        self.workspace_plan = None
        self.current_trajectory = None
        self.executor = PathExecutor()
        self.control_points = np.array([], dtype=float)
        
        # Undo/Redo States Stack
        self._undo_stack = []
        self._redo_stack = []
        
        # View Bookmarks
        self.bookmarks = []
        
        self.exec_timer = QtCore.QTimer(self)
        self.exec_timer.setInterval(33)
        self.exec_timer.timeout.connect(self.execution_tick)

        self._reach_timer = QtCore.QTimer(self)
        self._reach_timer.setSingleShot(True)
        self._reach_timer.timeout.connect(self._reachability_tick)
        self._reach_state = None
        self._reach_batch_size = 5

        self._vis_redraw_timer = QtCore.QTimer(self)
        self._vis_redraw_timer.setSingleShot(True)
        self._vis_redraw_timer.setInterval(100)
        self._vis_redraw_timer.timeout.connect(self._redraw_trajectory_preview)
        
        self.init_ui()
        self.load_session()
        
    def init_ui(self):
        # Register references in window so canvas can access it
        self.mw.path_tab = self
        
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setContentsMargins(4, 4, 4, 4)
        
        # Tab Container
        self.tab_widget = QtWidgets.QTabWidget()
        self.tab_widget.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #cccccc;
                border-radius: 4px;
                background-color: #fafafa;
            }
            QTabBar::tab {
                background: #e0e0e0;
                border: 1px solid #cccccc;
                border-bottom-color: none;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                padding: 6px 10px;
                font-weight: bold;
                font-size: 11px;
                color: #555555;
            }
            QTabBar::tab:selected {
                background: #fafafa;
                border-color: #cccccc;
                border-bottom-color: #fafafa;
                color: #1976d2;
            }
        """)
        main_layout.addWidget(self.tab_widget)
        
        # Connect tab change to redraw waypoint markers if path editor is open
        self.tab_widget.currentChanged.connect(self.on_tab_changed)

        # ----------------------------------------------------
        # TAB 1: WORKSPACE PLAN
        # ----------------------------------------------------
        ws_tab = QtWidgets.QWidget()
        ws_lay = QtWidgets.QVBoxLayout(ws_tab)
        ws_lay.setContentsMargins(10, 10, 10, 10)
        ws_lay.setSpacing(12)
        
        setup_group = QtWidgets.QGroupBox("Workspace Configuration")
        setup_group.setStyleSheet(self._group_box_style())
        setup_grid = QtWidgets.QGridLayout(setup_group)
        setup_grid.setSpacing(8)
        
        self.ws_width = self._make_spin_box(10.0, 1000.0, 200.0, "cm")
        setup_grid.addWidget(QtWidgets.QLabel("Width:"), 0, 0)
        setup_grid.addWidget(self.ws_width, 0, 1)
        
        self.ws_height = self._make_spin_box(10.0, 1000.0, 200.0, "cm")
        setup_grid.addWidget(QtWidgets.QLabel("Height:"), 0, 2)
        setup_grid.addWidget(self.ws_height, 0, 3)
        
        self.ws_grid_size = self._make_spin_box(1.0, 100.0, 10.0, "cm")
        setup_grid.addWidget(QtWidgets.QLabel("Grid Step:"), 1, 0)
        setup_grid.addWidget(self.ws_grid_size, 1, 1)
        
        self.ws_margin = self._make_spin_box(0.0, 50.0, 5.0, "cm")
        setup_grid.addWidget(QtWidgets.QLabel("Margin:"), 1, 2)
        setup_grid.addWidget(self.ws_margin, 1, 3)
        ws_lay.addWidget(setup_group)
        
        origin_group = QtWidgets.QGroupBox("Workspace Origin")
        origin_group.setStyleSheet(self._group_box_style())
        origin_grid = QtWidgets.QGridLayout(origin_group)
        self.ws_ox = self._make_spin_box(-500.0, 500.0, 50.0, "X")
        self.ws_oy = self._make_spin_box(-500.0, 500.0, 0.0, "Y")
        self.ws_oz = self._make_spin_box(-500.0, 500.0, 0.0, "Z")
        origin_grid.addWidget(QtWidgets.QLabel("Base Coordinate Offsets (cm):"), 0, 0, 1, 3)
        origin_grid.addWidget(self.ws_ox, 1, 0)
        origin_grid.addWidget(self.ws_oy, 1, 1)
        origin_grid.addWidget(self.ws_oz, 1, 2)
        ws_lay.addWidget(origin_group)
        
        self.snap_checkbox = QtWidgets.QCheckBox("Enable Grid Snapping")
        self.snap_checkbox.setChecked(True)
        self.snap_checkbox.stateChanged.connect(self.save_session)
        ws_lay.addWidget(self.snap_checkbox)
        
        btn_lay = QtWidgets.QHBoxLayout()
        self.init_ws_btn = QtWidgets.QPushButton("Initialize Workspace")
        self.init_ws_btn.setStyleSheet(self._primary_btn_style())
        self.init_ws_btn.clicked.connect(self.initialize_workspace)
        
        self.toggle_ws_btn = QtWidgets.QPushButton("Show Workspace Plane")
        self.toggle_ws_btn.setStyleSheet(self._secondary_btn_style())
        self.toggle_ws_btn.clicked.connect(self.toggle_workspace_plane_visibility)
        
        self.clear_ws_btn = QtWidgets.QPushButton("Clear Setup")
        self.clear_ws_btn.setStyleSheet(self._secondary_btn_style())
        self.clear_ws_btn.clicked.connect(self.clear_workspace)
        
        btn_lay.addWidget(self.init_ws_btn)
        btn_lay.addWidget(self.toggle_ws_btn)
        btn_lay.addWidget(self.clear_ws_btn)
        ws_lay.addLayout(btn_lay)
        ws_lay.addStretch()
        self.tab_widget.addTab(ws_tab, "Workspace")

        # ----------------------------------------------------
        # TAB 2: PATH GENERATOR
        # ----------------------------------------------------
        gen_tab = QtWidgets.QWidget()
        gen_lay = QtWidgets.QVBoxLayout(gen_tab)
        gen_lay.setContentsMargins(10, 10, 10, 10)
        gen_lay.setSpacing(12)
        
        shape_group = QtWidgets.QGroupBox("Generate Preset Shape")
        shape_group.setStyleSheet(self._group_box_style())
        shape_layout = QtWidgets.QVBoxLayout(shape_group)
        
        sel_lay = QtWidgets.QHBoxLayout()
        sel_lay.addWidget(QtWidgets.QLabel("Preset Shape:"))
        self.shape_combo = QtWidgets.QComboBox()
        self.shape_combo.addItems(["Square", "Wave"])
        self.shape_combo.setStyleSheet(self._combo_style())
        self.shape_combo.currentIndexChanged.connect(self.on_shape_changed)
        sel_lay.addWidget(self.shape_combo)
        shape_layout.addLayout(sel_lay)
        
        self.shape_stack = QtWidgets.QStackedWidget()
        
        # Square parameters
        s_widget = QtWidgets.QWidget()
        s_grid = QtWidgets.QGridLayout(s_widget)
        s_grid.setContentsMargins(0, 5, 0, 5)
        self.square_w = self._make_spin_box(1.0, 500.0, 80.0, "cm")
        self.square_h = self._make_spin_box(1.0, 500.0, 80.0, "cm")
        self.square_cx = self._make_spin_box(-500.0, 500.0, 0.0, "cm")
        self.square_cy = self._make_spin_box(-500.0, 500.0, 0.0, "cm")
        self.square_z = self._make_spin_box(-500.0, 500.0, 10.0, "cm")
        self.square_pts = self._make_spin_box_int(4, 1000, 120, "points")
        s_grid.addWidget(QtWidgets.QLabel("Width:"), 0, 0)
        s_grid.addWidget(self.square_w, 0, 1)
        s_grid.addWidget(QtWidgets.QLabel("Height:"), 1, 0)
        s_grid.addWidget(self.square_h, 1, 1)
        s_grid.addWidget(QtWidgets.QLabel("Center X:"), 2, 0)
        s_grid.addWidget(self.square_cx, 2, 1)
        s_grid.addWidget(QtWidgets.QLabel("Center Y:"), 3, 0)
        s_grid.addWidget(self.square_cy, 3, 1)
        s_grid.addWidget(QtWidgets.QLabel("Z Height:"), 4, 0)
        s_grid.addWidget(self.square_z, 4, 1)
        s_grid.addWidget(QtWidgets.QLabel("Resolution:"), 5, 0)
        s_grid.addWidget(self.square_pts, 5, 1)
        self.shape_stack.addWidget(s_widget)
        
        # Wave parameters
        w_widget = QtWidgets.QWidget()
        w_grid = QtWidgets.QGridLayout(w_widget)
        w_grid.setContentsMargins(0, 5, 0, 5)
        self.wave_start_x = self._make_spin_box(-500.0, 500.0, -80.0, "cm")
        self.wave_start_y = self._make_spin_box(-500.0, 500.0, 0.0, "cm")
        self.wave_end_x = self._make_spin_box(-500.0, 500.0, 80.0, "cm")
        self.wave_end_y = self._make_spin_box(-500.0, 500.0, 0.0, "cm")
        self.wave_amp = self._make_spin_box(0.0, 200.0, 15.0, "cm")
        self.wave_periods = self._make_spin_box(0.1, 20.0, 3.0, "cycles")
        self.wave_z = self._make_spin_box(-500.0, 500.0, 10.0, "cm")
        self.wave_pts = self._make_spin_box_int(4, 1000, 120, "points")
        w_grid.addWidget(QtWidgets.QLabel("Start X:"), 0, 0)
        w_grid.addWidget(self.wave_start_x, 0, 1)
        w_grid.addWidget(QtWidgets.QLabel("Start Y:"), 1, 0)
        w_grid.addWidget(self.wave_start_y, 1, 1)
        w_grid.addWidget(QtWidgets.QLabel("End X:"), 2, 0)
        w_grid.addWidget(self.wave_end_x, 2, 1)
        w_grid.addWidget(QtWidgets.QLabel("End Y:"), 3, 0)
        w_grid.addWidget(self.wave_end_y, 3, 1)
        w_grid.addWidget(QtWidgets.QLabel("Amplitude:"), 4, 0)
        w_grid.addWidget(self.wave_amp, 4, 1)
        w_grid.addWidget(QtWidgets.QLabel("Periods:"), 5, 0)
        w_grid.addWidget(self.wave_periods, 5, 1)
        w_grid.addWidget(QtWidgets.QLabel("Z Height:"), 6, 0)
        w_grid.addWidget(self.wave_z, 6, 1)
        w_grid.addWidget(QtWidgets.QLabel("Resolution:"), 7, 0)
        w_grid.addWidget(self.wave_pts, 7, 1)
        self.shape_stack.addWidget(w_widget)
        
        shape_layout.addWidget(self.shape_stack)
        gen_lay.addWidget(shape_group)
        
        limits_group = QtWidgets.QGroupBox("Trajectory Kinematics Limits")
        limits_group.setStyleSheet(self._group_box_style())
        lim_lay = QtWidgets.QGridLayout(limits_group)
        self.vel_limit = self._make_spin_box(1.0, 100.0, 15.0, "cm/s")
        self.accel_limit = self._make_spin_box(1.0, 100.0, 10.0, "cm/s²")
        lim_lay.addWidget(QtWidgets.QLabel("Max Velocity:"), 0, 0)
        lim_lay.addWidget(self.vel_limit, 0, 1)
        lim_lay.addWidget(QtWidgets.QLabel("Max Acceleration:"), 1, 0)
        lim_lay.addWidget(self.accel_limit, 1, 1)
        gen_lay.addWidget(limits_group)
        
        gen_btn_lay = QtWidgets.QHBoxLayout()
        self.gen_path_btn = QtWidgets.QPushButton("Generate Trajectory")
        self.gen_path_btn.setStyleSheet(self._primary_btn_style())
        self.gen_path_btn.clicked.connect(self.generate_path)
        
        self.clear_path_btn = QtWidgets.QPushButton("Clear Trajectory")
        self.clear_path_btn.setStyleSheet(self._secondary_btn_style())
        self.clear_path_btn.clicked.connect(self.clear_path)
        
        gen_btn_lay.addWidget(self.gen_path_btn)
        gen_btn_lay.addWidget(self.clear_path_btn)
        gen_lay.addLayout(gen_btn_lay)
        gen_lay.addStretch()
        self.tab_widget.addTab(gen_tab, "Generator")

        # ----------------------------------------------------
        # TAB 3: PATH EDITOR
        # ----------------------------------------------------
        edit_tab = QtWidgets.QWidget()
        edit_lay = QtWidgets.QVBoxLayout(edit_tab)
        edit_lay.setContentsMargins(10, 10, 10, 10)
        edit_lay.setSpacing(10)
        
        ctrl_box = QtWidgets.QHBoxLayout()
        self.chk_interactive_points = QtWidgets.QCheckBox("3D Waypoint Markers")
        self.chk_interactive_points.setChecked(True)
        self.chk_interactive_points.stateChanged.connect(self.on_interactive_points_toggled)
        ctrl_box.addWidget(self.chk_interactive_points)
        
        self.btn_undo = QtWidgets.QPushButton("Undo")
        self.btn_undo.setStyleSheet(self._secondary_btn_style())
        self.btn_undo.setEnabled(False)
        self.btn_undo.clicked.connect(self.trigger_undo)
        
        self.btn_redo = QtWidgets.QPushButton("Redo")
        self.btn_redo.setStyleSheet(self._secondary_btn_style())
        self.btn_redo.setEnabled(False)
        self.btn_redo.clicked.connect(self.trigger_redo)
        
        ctrl_box.addWidget(self.btn_undo)
        ctrl_box.addWidget(self.btn_redo)
        edit_lay.addLayout(ctrl_box)
        
        # Waypoint coordinate table
        self.table_waypoints = QtWidgets.QTableWidget()
        self.table_waypoints.setColumnCount(4)
        self.table_waypoints.setHorizontalHeaderLabels(["ID", "X (cm)", "Y (cm)", "Z (cm)"])
        self.table_waypoints.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table_waypoints.itemChanged.connect(self.on_table_item_changed)
        self.table_waypoints.itemSelectionChanged.connect(self.on_table_selection_changed)
        edit_lay.addWidget(self.table_waypoints)
        
        tbl_btn_lay = QtWidgets.QHBoxLayout()
        self.btn_add_pt = QtWidgets.QPushButton("+ Add")
        self.btn_add_pt.setStyleSheet(self._secondary_btn_style())
        self.btn_add_pt.clicked.connect(self.add_table_point)
        
        self.btn_del_pt = QtWidgets.QPushButton("- Delete")
        self.btn_del_pt.setStyleSheet(self._secondary_btn_style())
        self.btn_del_pt.clicked.connect(self.delete_table_point)
        
        self.btn_move_up = QtWidgets.QPushButton("▲ Up")
        self.btn_move_up.setStyleSheet(self._secondary_btn_style())
        self.btn_move_up.clicked.connect(self.move_point_up)
        
        self.btn_move_down = QtWidgets.QPushButton("▼ Down")
        self.btn_move_down.setStyleSheet(self._secondary_btn_style())
        self.btn_move_down.clicked.connect(self.move_point_down)
        
        tbl_btn_lay.addWidget(self.btn_add_pt)
        tbl_btn_lay.addWidget(self.btn_del_pt)
        tbl_btn_lay.addWidget(self.btn_move_up)
        tbl_btn_lay.addWidget(self.btn_move_down)
        edit_lay.addLayout(tbl_btn_lay)
        
        self.tab_widget.addTab(edit_tab, "Editor")

        # ----------------------------------------------------
        # TAB 4: TRAJECTORY
        # ----------------------------------------------------
        traj_tab = QtWidgets.QWidget()
        traj_lay = QtWidgets.QVBoxLayout(traj_tab)
        traj_lay.setContentsMargins(10, 10, 10, 10)
        traj_lay.setSpacing(12)
        
        # Info readout
        info_group = QtWidgets.QGroupBox("Trajectory Readout Metrics")
        info_group.setStyleSheet(self._group_box_style())
        info_grid = QtWidgets.QGridLayout(info_group)
        info_grid.setSpacing(8)
        
        info_grid.addWidget(QtWidgets.QLabel("Total Points:"), 0, 0)
        self.lbl_points = QtWidgets.QLabel("0")
        self.lbl_points.setStyleSheet("font-weight: bold; color: #1976d2;")
        info_grid.addWidget(self.lbl_points, 0, 1)
        
        info_grid.addWidget(QtWidgets.QLabel("Path Length:"), 1, 0)
        self.lbl_length = QtWidgets.QLabel("0.00 cm")
        self.lbl_length.setStyleSheet("font-weight: bold; color: #1976d2;")
        info_grid.addWidget(self.lbl_length, 1, 1)
        
        info_grid.addWidget(QtWidgets.QLabel("Duration:"), 2, 0)
        self.lbl_time = QtWidgets.QLabel("0.00 s")
        self.lbl_time.setStyleSheet("font-weight: bold; color: #1976d2;")
        info_grid.addWidget(self.lbl_time, 2, 1)
        
        info_grid.addWidget(QtWidgets.QLabel("IK Feasibility:"), 3, 0)
        self.lbl_reachable = QtWidgets.QLabel("0 / 0 (0.0%)")
        self.lbl_reachable.setStyleSheet("font-weight: bold; color: #2e7d32;")
        info_grid.addWidget(self.lbl_reachable, 3, 1)
        traj_lay.addWidget(info_group)
        
        # Graph plot area
        graph_group = QtWidgets.QGroupBox("Velocity / Acceleration Profiles")
        graph_group.setStyleSheet(self._group_box_style())
        graph_lay = QtWidgets.QVBoxLayout(graph_group)
        self.speed_graph = SpeedProfileWidget()
        graph_lay.addWidget(self.speed_graph)
        traj_lay.addWidget(graph_group)
        
        traj_lay.addStretch()
        self.tab_widget.addTab(traj_tab, "Trajectory")

        # ----------------------------------------------------
        # TAB 5: PLAYBACK
        # ----------------------------------------------------
        play_tab = QtWidgets.QWidget()
        play_lay = QtWidgets.QVBoxLayout(play_tab)
        play_lay.setContentsMargins(10, 10, 10, 10)
        play_lay.setSpacing(12)
        
        # Main simulation transport buttons
        trans_group = QtWidgets.QGroupBox("Transport Simulation Panel")
        trans_group.setStyleSheet(self._group_box_style())
        trans_lay = QtWidgets.QVBoxLayout(trans_group)
        
        btn_lay = QtWidgets.QHBoxLayout()
        self.play_btn = QtWidgets.QPushButton("Play")
        self.play_btn.setFixedHeight(30)
        self.play_btn.setStyleSheet("background-color: #4caf50; color: white; border: none; border-radius: 4px; font-weight: bold;")
        self.play_btn.setEnabled(False)
        self.play_btn.clicked.connect(self.play_path)
        
        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.pause_btn.setFixedHeight(30)
        self.pause_btn.setStyleSheet("background-color: #ffc107; color: white; border: none; border-radius: 4px; font-weight: bold;")
        self.pause_btn.setEnabled(False)
        self.pause_btn.clicked.connect(self.pause_path)
        
        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.stop_btn.setFixedHeight(30)
        self.stop_btn.setStyleSheet("background-color: #f44336; color: white; border: none; border-radius: 4px; font-weight: bold;")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.stop_path)
        
        btn_lay.addWidget(self.play_btn)
        btn_lay.addWidget(self.pause_btn)
        btn_lay.addWidget(self.stop_btn)
        trans_lay.addLayout(btn_lay)
        
        # Step Forward & Reset Button
        step_lay = QtWidgets.QHBoxLayout()
        self.reset_btn = QtWidgets.QPushButton("Reset")
        self.reset_btn.setStyleSheet(self._secondary_btn_style())
        self.reset_btn.setEnabled(False)
        self.reset_btn.clicked.connect(self.reset_path)
        
        self.step_btn = QtWidgets.QPushButton("Step Step")
        self.step_btn.setStyleSheet(self._secondary_btn_style())
        self.step_btn.setEnabled(False)
        self.step_btn.clicked.connect(self.step_path)
        
        step_lay.addWidget(self.reset_btn)
        step_lay.addWidget(self.step_btn)
        trans_lay.addLayout(step_lay)
        
        # Direction controller
        dir_lay = QtWidgets.QHBoxLayout()
        dir_lay.addWidget(QtWidgets.QLabel("Execution Direction:"))
        self.dir_combo = QtWidgets.QComboBox()
        self.dir_combo.addItems(["Forward Play", "Reverse Play"])
        self.dir_combo.setStyleSheet(self._combo_style())
        self.dir_combo.currentIndexChanged.connect(self.on_direction_changed)
        dir_lay.addWidget(self.dir_combo)
        trans_lay.addLayout(dir_lay)
        
        play_lay.addWidget(trans_group)
        
        # Dynamic Scrubber / Speeds
        scrub_group = QtWidgets.QGroupBox("Timeline & Speeds")
        scrub_group.setStyleSheet(self._group_box_style())
        scrub_grid = QtWidgets.QGridLayout(scrub_group)
        
        scrub_grid.addWidget(QtWidgets.QLabel("Timeline Scrub:"), 0, 0)
        self.timeline_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.timeline_slider.setRange(0, 1000)
        self.timeline_slider.setValue(0)
        self.timeline_slider.setEnabled(False)
        self.timeline_slider.valueChanged.connect(self.on_timeline_scrubbed)
        scrub_grid.addWidget(self.timeline_slider, 0, 1)
        
        scrub_grid.addWidget(QtWidgets.QLabel("Sim Speed:"), 1, 0)
        self.speed_lbl = QtWidgets.QLabel("100 %")
        self.speed_lbl.setStyleSheet("font-weight: bold;")
        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setRange(1, 500)
        self.speed_slider.setValue(100)
        self.speed_slider.setEnabled(False)
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        
        spd_lay = QtWidgets.QHBoxLayout()
        spd_lay.addWidget(self.speed_slider)
        spd_lay.addWidget(self.speed_lbl)
        scrub_grid.addLayout(spd_lay, 1, 1)
        
        scrub_grid.addWidget(QtWidgets.QLabel("Progress:"), 2, 0)
        self.progress_bar = QtWidgets.QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #cccccc;
                border-radius: 4px;
                text-align: center;
                height: 16px;
            }
            QProgressBar::chunk {
                background-color: #4caf50;
            }
        """)
        scrub_grid.addWidget(self.progress_bar, 2, 1)
        play_lay.addWidget(scrub_group)
        
        # View bookmarks & alignments
        cam_group = QtWidgets.QGroupBox("Viewport & Camera Bookmarks")
        cam_group.setStyleSheet(self._group_box_style())
        cam_grid = QtWidgets.QGridLayout(cam_group)
        
        self.btn_v_iso = QtWidgets.QPushButton("ISO View")
        self.btn_v_iso.setStyleSheet(self._secondary_btn_style())
        self.btn_v_iso.clicked.connect(lambda: self.mw.canvas.view_isometric())
        
        self.btn_v_top = QtWidgets.QPushButton("Top View")
        self.btn_v_top.setStyleSheet(self._secondary_btn_style())
        self.btn_v_top.clicked.connect(lambda: self.mw.canvas.view_top())
        
        self.btn_v_front = QtWidgets.QPushButton("Front View")
        self.btn_v_front.setStyleSheet(self._secondary_btn_style())
        self.btn_v_front.clicked.connect(lambda: self.mw.canvas.view_front())
        
        self.btn_v_side = QtWidgets.QPushButton("Side View")
        self.btn_v_side.setStyleSheet(self._secondary_btn_style())
        self.btn_v_side.clicked.connect(lambda: self.mw.canvas.view_side())
        
        self.btn_v_focus = QtWidgets.QPushButton("🔍 Focus Trajectory")
        self.btn_v_focus.setStyleSheet(self._secondary_btn_style())
        self.btn_v_focus.clicked.connect(self.focus_camera_on_trajectory)
        
        cam_grid.addWidget(self.btn_v_iso, 0, 0)
        cam_grid.addWidget(self.btn_v_top, 0, 1)
        cam_grid.addWidget(self.btn_v_front, 1, 0)
        cam_grid.addWidget(self.btn_v_side, 1, 1)
        cam_grid.addWidget(self.btn_v_focus, 2, 0, 1, 2)
        
        # List of bookmarks
        self.lst_bookmarks = QtWidgets.QListWidget()
        self.lst_bookmarks.setFixedHeight(80)
        self.lst_bookmarks.itemDoubleClicked.connect(self.load_selected_bookmark)
        cam_grid.addWidget(self.lst_bookmarks, 3, 0, 1, 2)
        
        self.btn_add_bookmark = QtWidgets.QPushButton("Save Current View")
        self.btn_add_bookmark.setStyleSheet(self._secondary_btn_style())
        self.btn_add_bookmark.clicked.connect(self.add_camera_bookmark)
        cam_grid.addWidget(self.btn_add_bookmark, 4, 0, 1, 2)
        
        play_lay.addWidget(cam_group)
        play_lay.addStretch()
        self.tab_widget.addTab(play_tab, "Playback")

        # ----------------------------------------------------
        # TAB 6: VISUALIZATION
        # ----------------------------------------------------
        vis_tab = QtWidgets.QWidget()
        vis_lay = QtWidgets.QVBoxLayout(vis_tab)
        vis_lay.setContentsMargins(10, 10, 10, 10)
        vis_lay.setSpacing(12)
        
        mode_group = QtWidgets.QGroupBox("Trajectory Visual Heatmaps")
        mode_group.setStyleSheet(self._group_box_style())
        mode_lay = QtWidgets.QVBoxLayout(mode_group)
        
        self.vis_combo = QtWidgets.QComboBox()
        self.vis_combo.addItems(["Geometry", "Reachability", "Velocity Heatmap", "Acceleration Heatmap", "Curvature Heatmap"])
        self.vis_combo.setStyleSheet(self._combo_style())
        self.vis_combo.currentIndexChanged.connect(self.on_vis_mode_changed)
        mode_lay.addWidget(self.vis_combo)
        vis_lay.addWidget(mode_group)
        
        overlays_group = QtWidgets.QGroupBox("Vector Overlay Toggles")
        overlays_group.setStyleSheet(self._group_box_style())
        over_lay = QtWidgets.QVBoxLayout(overlays_group)
        
        self.chk_show_axes = QtWidgets.QCheckBox("Show Tool Orientation Axes (RGB)")
        self.chk_show_axes.setChecked(False)
        self.chk_show_axes.stateChanged.connect(self.on_vis_overlays_changed)
        over_lay.addWidget(self.chk_show_axes)
        
        self.chk_show_arrows = QtWidgets.QCheckBox("Show Toolpath Direction Arrows")
        self.chk_show_arrows.setChecked(False)
        self.chk_show_arrows.stateChanged.connect(self.on_vis_overlays_changed)
        over_lay.addWidget(self.chk_show_arrows)
        
        self.chk_show_trail = QtWidgets.QCheckBox("Enable Real-time TCP Trail")
        self.chk_show_trail.setChecked(True)
        over_lay.addWidget(self.chk_show_trail)
        
        vis_lay.addWidget(overlays_group)
        vis_lay.addStretch()
        self.tab_widget.addTab(vis_tab, "Visualization")

        # ----------------------------------------------------
        # TAB 7: IMPORT / EXPORT
        # ----------------------------------------------------
        io_tab = QtWidgets.QWidget()
        io_lay = QtWidgets.QVBoxLayout(io_tab)
        io_lay.setContentsMargins(10, 10, 10, 10)
        io_lay.setSpacing(12)
        
        # Scaling parameters for importing coordinates
        scale_group = QtWidgets.QGroupBox("Import Scaling & Coordinate Offset")
        scale_group.setStyleSheet(self._group_box_style())
        scale_grid = QtWidgets.QGridLayout(scale_group)
        
        self.imp_scale = self._make_spin_box(0.01, 100.0, 1.0, "ratio")
        scale_grid.addWidget(QtWidgets.QLabel("Scale factor:"), 0, 0)
        scale_grid.addWidget(self.imp_scale, 0, 1)
        
        self.imp_ox = self._make_spin_box(-500.0, 500.0, 0.0, "cm")
        self.imp_oy = self._make_spin_box(-500.0, 500.0, 0.0, "cm")
        self.imp_oz = self._make_spin_box(-500.0, 500.0, 10.0, "cm")
        scale_grid.addWidget(QtWidgets.QLabel("Offset (X, Y, Z):"), 1, 0)
        scale_grid.addWidget(self.imp_ox, 1, 1)
        scale_grid.addWidget(self.imp_oy, 2, 1)
        scale_grid.addWidget(self.imp_oz, 3, 1)
        io_lay.addWidget(scale_group)
        
        # Drag & Drop Zone
        self.drop_zone = ImportDropZone()
        self.drop_zone.fileDropped.connect(self.import_path_file)
        io_lay.addWidget(self.drop_zone)
        
        import_btn = QtWidgets.QPushButton("Browse File to Import...")
        import_btn.setStyleSheet(self._secondary_btn_style())
        import_btn.clicked.connect(self.browse_and_import)
        io_lay.addWidget(import_btn)
        
        # Export Actions
        export_group = QtWidgets.QGroupBox("Industrial Path Code Exporters")
        export_group.setStyleSheet(self._group_box_style())
        exp_lay = QtWidgets.QVBoxLayout(export_group)
        
        self.btn_exp_json = QtWidgets.QPushButton("Export JSON Waypoint Coordinates")
        self.btn_exp_json.setStyleSheet(self._secondary_btn_style())
        self.btn_exp_json.clicked.connect(self.export_waypoints_json)
        exp_lay.addWidget(self.btn_exp_json)
        
        self.btn_exp_gcode = QtWidgets.QPushButton("Export Linear G-Code Toolpath (.nc)")
        self.btn_exp_gcode.setStyleSheet(self._secondary_btn_style())
        self.btn_exp_gcode.clicked.connect(self.export_trajectory_gcode)
        exp_lay.addWidget(self.btn_exp_gcode)
        
        self.btn_exp_joints = QtWidgets.QPushButton("Export CSV Joint-Space Profile")
        self.btn_exp_joints.setStyleSheet(self._secondary_btn_style())
        self.btn_exp_joints.clicked.connect(self.export_trajectory_joints)
        exp_lay.addWidget(self.btn_exp_joints)
        
        io_lay.addWidget(export_group)
        io_lay.addStretch()
        self.tab_widget.addTab(io_tab, "Import/Export")

        # ----------------------------------------------------
        # TAB 8: DIAGNOSTICS
        # ----------------------------------------------------
        diag_tab = QtWidgets.QWidget()
        diag_lay = QtWidgets.QVBoxLayout(diag_tab)
        diag_lay.setContentsMargins(10, 10, 10, 10)
        diag_lay.setSpacing(12)
        
        read_group = QtWidgets.QGroupBox("Real-time Solver Diagnostics")
        read_group.setStyleSheet(self._group_box_style())
        read_grid = QtWidgets.QGridLayout(read_group)
        
        read_grid.addWidget(QtWidgets.QLabel("Viewport Rate:"), 0, 0)
        self.diag_lbl_fps = QtWidgets.QLabel("30 Hz")
        self.diag_lbl_fps.setStyleSheet("font-weight: bold; color: #1976d2;")
        read_grid.addWidget(self.diag_lbl_fps, 0, 1)
        
        read_grid.addWidget(QtWidgets.QLabel("Avg IK Latency:"), 1, 0)
        self.diag_lbl_ik = QtWidgets.QLabel("1.4 ms")
        self.diag_lbl_ik.setStyleSheet("font-weight: bold; color: #1976d2;")
        read_grid.addWidget(self.diag_lbl_ik, 1, 1)
        
        read_grid.addWidget(QtWidgets.QLabel("Path Resolution:"), 2, 0)
        self.diag_lbl_points = QtWidgets.QLabel("0 pts")
        self.diag_lbl_points.setStyleSheet("font-weight: bold; color: #1976d2;")
        read_grid.addWidget(self.diag_lbl_points, 2, 1)
        
        read_grid.addWidget(QtWidgets.QLabel("IK Success Ratio:"), 3, 0)
        self.diag_lbl_reach = QtWidgets.QLabel("0.0%")
        self.diag_lbl_reach.setStyleSheet("font-weight: bold; color: #2e7d32;")
        read_grid.addWidget(self.diag_lbl_reach, 3, 1)
        
        read_grid.addWidget(QtWidgets.QLabel("Singular Configuration:"), 4, 0)
        self.diag_lbl_singularity = QtWidgets.QLabel("Safe")
        self.diag_lbl_singularity.setStyleSheet("font-weight: bold; color: #2e7d32;")
        read_grid.addWidget(self.diag_lbl_singularity, 4, 1)
        diag_lay.addWidget(read_group)
        
        # Log view Console
        log_group = QtWidgets.QGroupBox("Diagnostic Messages Console")
        log_group.setStyleSheet(self._group_box_style())
        log_lay = QtWidgets.QVBoxLayout(log_group)
        self.diag_console = QtWidgets.QPlainTextEdit()
        self.diag_console.setReadOnly(True)
        self.diag_console.setStyleSheet("""
            QPlainTextEdit {
                background-color: #1e1e1e;
                color: #a9b7c6;
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 11px;
                border: 1px solid #333333;
                border-radius: 4px;
            }
        """)
        log_lay.addWidget(self.diag_console)
        diag_lay.addWidget(log_group)
        
        self.tab_widget.addTab(diag_tab, "Diagnostics")

    # ----------------------------------------------------
    # Event Handlers & Callbacks
    # ----------------------------------------------------
    def on_tab_changed(self, index):
        if self.workspace_plan is not None:
            if index == 2: # Path Editor tab
                self.mw.canvas.draw_waypoint_markers(self.control_points)
            else:
                self.mw.canvas.clear_waypoint_markers()

    def on_shape_changed(self, index):
        self.shape_stack.setCurrentIndex(index)

    def on_direction_changed(self, index):
        # Index 0 is Forward, 1 is Reverse
        direction = -1.0 if index == 1 else 1.0
        self.executor.set_direction(direction)
        self.log_diagnostic(f"Playback direction set to: {'Reverse' if index == 1 else 'Forward'}")

    def on_timeline_scrubbed(self, value):
        if not self.current_trajectory:
            return
        
        # Map slider value (0 to 1000) to actual elapsed time range
        t_max = self.current_trajectory.timestamps[-1]
        t_target = (value / 1000.0) * t_max
        
        # Seek executor to time
        self.executor.seek(t_target)
        
        # Temporarily pause updates if slider action is manual but execute tick to update robot state
        self.move_robot_to_trajectory_point(t_target)
        
        # Update progress bar
        self.progress_bar.setValue(int(round(self.executor.get_progress())))

    def on_interactive_points_toggled(self, state):
        if self.workspace_plan is not None:
            if state == QtCore.Qt.Checked:
                self.mw.canvas.draw_waypoint_markers(self.control_points)
            else:
                self.mw.canvas.clear_waypoint_markers()

    def on_vis_mode_changed(self, index):
        self._schedule_trajectory_redraw()

    def on_vis_overlays_changed(self, state):
        self._schedule_trajectory_redraw()

    def focus_camera_on_trajectory(self):
        if self.current_trajectory is not None:
            self.mw.canvas.auto_focus_trajectory(self.current_trajectory)
            self.log_diagnostic("Focused 3D camera on trajectory.")
        else:
            self.mw.log("⚠️ No trajectory active to focus.")

    def add_camera_bookmark(self):
        camera = self.mw.canvas.plotter.camera
        pos = camera.GetPosition()
        focal = camera.GetFocalPoint()
        up = camera.GetViewUp()
        
        name, ok = QtWidgets.QInputDialog.getText(self, "Add Camera Bookmark", "Bookmark Label Name:")
        if ok and name.strip():
            bookmark = {
                "name": name.strip(),
                "position": list(pos),
                "focal_point": list(focal),
                "view_up": list(up)
            }
            self.bookmarks.append(bookmark)
            self.lst_bookmarks.addItem(name.strip())
            self.log_diagnostic(f"Camera bookmark created: {name.strip()}")

    def load_selected_bookmark(self):
        row = self.lst_bookmarks.currentRow()
        if row >= 0 and row < len(self.bookmarks):
            bm = self.bookmarks[row]
            camera = self.mw.canvas.plotter.camera
            camera.SetPosition(bm["position"][0], bm["position"][1], bm["position"][2])
            camera.SetFocalPoint(bm["focal_point"][0], bm["focal_point"][1], bm["focal_point"][2])
            camera.SetViewUp(bm["view_up"][0], bm["view_up"][1], bm["view_up"][2])
            self.mw.canvas.plotter.render()
            self.log_diagnostic(f"Loaded viewport view: {bm['name']}")

    # ----------------------------------------------------
    # Workspace Setup Commands
    # ----------------------------------------------------
    def initialize_workspace(self):
        w = self.ws_width.value()
        h = self.ws_height.value()
        grid = self.ws_grid_size.value()
        margin = self.ws_margin.value()
        ox = self.ws_ox.value()
        oy = self.ws_oy.value()
        oz = self.ws_oz.value()

        origin = np.array([ox, oy, oz], dtype=float)
        self.workspace_plan = WorkspacePlan(width=w, height=h, grid_size=grid, safe_margin=margin, origin=origin)
        self.mw.canvas.current_workspace_plan = self.workspace_plan

        self.mw.canvas.show_workspace_planner(self.workspace_plan)
        self.mw.log(f"✅ Workspace initialized: size {w:.1f}x{h:.1f} cm, grid {grid:.1f} cm, origin ({ox:.1f}, {oy:.1f}, {oz:.1f})")
        self.log_diagnostic(f"Initialized workspace plan: {w}x{h}cm grid size={grid} origin=({ox},{oy},{oz})")

        # Keep toggle button text in sync
        vis = getattr(self.mw.canvas, "workspace_visible", True)
        if vis:
            self.toggle_ws_btn.setText("Hide Workspace Plane")
        else:
            self.toggle_ws_btn.setText("Show Workspace Plane")

        # Re-sync control points bounding box inside new workspace
        if len(self.control_points) > 0:
            self.generate_path_from_control_points()
        else:
            # Default control points: small rectangle in the center of workspace
            half_w = (w - 2 * margin) / 4.0
            half_h = (h - 2 * margin) / 4.0
            self.control_points = np.array([
                [-half_w, -half_h, 10.0],
                [half_w, -half_h, 10.0],
                [half_w, half_h, 10.0],
                [-half_w, half_h, 10.0],
                [-half_w, -half_h, 10.0]
            ], dtype=float)
            self.push_undo_state()
            self.update_waypoints_table()
            self.generate_path_from_control_points()
        self.save_session()

    def toggle_workspace_plane_visibility(self):
        canvas = self.mw.canvas
        if self.workspace_plan is None:
            self.initialize_workspace()
            return

        current_vis = getattr(canvas, "workspace_visible", True)
        new_vis = not current_vis
        canvas.set_workspace_planner_visibility(new_vis)
        
        if new_vis:
            self.toggle_ws_btn.setText("Hide Workspace Plane")
            self.mw.log("👁️ Workspace plane visible.")
        else:
            self.toggle_ws_btn.setText("Show Workspace Plane")
            self.mw.log("🙈 Workspace plane hidden.")

    def clear_workspace(self):
        self._cancel_reachability_job()
        self.stop_path()
        self.workspace_plan = None
        self.current_trajectory = None
        self.control_points = np.array([], dtype=float)
        self._undo_stack.clear()
        self._redo_stack.clear()
        self.btn_undo.setEnabled(False)
        self.btn_redo.setEnabled(False)
        
        self.executor.load_trajectory(None)
        self.mw.canvas.clear_workspace_planner()
        self.mw.canvas.clear_trajectory_preview()
        self.mw.canvas.clear_waypoint_markers()
        
        self.toggle_ws_btn.setText("Show Workspace Plane")
        
        self.lbl_points.setText("0")
        self.lbl_length.setText("0.00 cm")
        self.lbl_time.setText("0.00 s")
        self.lbl_reachable.setText("0 / 0 (0.0%)")
        self.update_waypoints_table()
        self.update_execution_ui()
        self.mw.log("🧹 Workspace setup cleared.")
        self.log_diagnostic("Workspace plan cleared.")
        self.save_session()

    # ----------------------------------------------------
    # Shape Generation Logic
    # ----------------------------------------------------
    def generate_path(self):
        if self.workspace_plan is None:
            self.mw.log("❌ Error: Initialize the Workspace Plan first.")
            QtWidgets.QMessageBox.warning(self, "Plan Setup Needed", "Please configure and initialize the workspace plan first.")
            return

        shape = self.shape_combo.currentText()
        planner = PathPlanner(self.workspace_plan)

        if shape == "Square":
            sw = self.square_w.value()
            sh = self.square_h.value()
            cx = self.square_cx.value()
            cy = self.square_cy.value()
            z = self.square_z.value()
            pts = self.square_pts.value()
            raw_pts = planner.generate_square(center_x=cx, center_y=cy, width=sw, height=sh, z_height=z, num_points=pts)
        else: # Wave
            sx = self.wave_start_x.value()
            sy = self.wave_start_y.value()
            ex = self.wave_end_x.value()
            ey = self.wave_end_y.value()
            amp = self.wave_amp.value()
            periods = self.wave_periods.value()
            z = self.wave_z.value()
            pts = self.wave_pts.value()
            raw_pts = planner.generate_wave(start_x=sx, start_y=sy, end_x=ex, end_y=ey, amplitude=amp, periods=periods, z_height=z, num_points=pts)

        self.control_points = np.array(raw_pts, dtype=float)
        self.push_undo_state()
        self.update_waypoints_table()
        self.generate_path_from_control_points()
        self.mw.log(f"✅ Generated shape preset: {shape}")
        self.log_diagnostic(f"Generated shape path: {shape} with {len(raw_pts)} vertices.")
        self.save_session()

    def clear_path(self):
        self._cancel_reachability_job()
        self.stop_path()
        self.current_trajectory = None
        self.control_points = np.array([], dtype=float)
        self._undo_stack.clear()
        self._redo_stack.clear()
        self.btn_undo.setEnabled(False)
        self.btn_redo.setEnabled(False)
        
        self.executor.load_trajectory(None)
        self.mw.canvas.clear_trajectory_preview()
        self.mw.canvas.clear_waypoint_markers()
        
        self.lbl_points.setText("0")
        self.lbl_length.setText("0.00 cm")
        self.lbl_time.setText("0.00 s")
        self.lbl_reachable.setText("0 / 0 (0.0%)")
        self.update_waypoints_table()
        self.update_execution_ui()
        self.mw.log("🧹 Trajectory cleared.")
        self.log_diagnostic("Active trajectory cleared.")
        self.save_session()

    def _cancel_reachability_job(self):
        if self._reach_timer.isActive():
            self._reach_timer.stop()
        self._reach_state = None
        self._set_path_controls_enabled(True)

    def _reachability_in_progress(self):
        return self._reach_state is not None

    def _set_path_controls_enabled(self, enabled):
        for widget in (
            self.gen_path_btn,
            self.init_ws_btn,
            self.btn_add_pt,
            self.btn_del_pt,
            self.btn_move_up,
            self.btn_move_down,
            self.btn_exp_joints,
        ):
            widget.setEnabled(enabled)

    def _schedule_trajectory_redraw(self):
        if self.current_trajectory is None:
            return
        self._vis_redraw_timer.start()

    def _redraw_trajectory_preview(self):
        if self.current_trajectory is None:
            return
        vis_mode = self.vis_combo.currentText()
        if self._reachability_in_progress() and vis_mode == "Reachability":
            vis_mode = "Geometry"
        show_ax = self.chk_show_axes.isChecked()
        show_arr = self.chk_show_arrows.isChecked()
        self.mw.canvas.draw_trajectory_preview(
            self.current_trajectory,
            vis_mode=vis_mode,
            show_axes=show_ax,
            show_arrows=show_arr,
        )
        if self.tab_widget.currentIndex() == 2 and self.chk_interactive_points.isChecked():
            self.mw.canvas.draw_waypoint_markers(self.control_points)

    def _apply_trajectory_preview(self, trajectory, update_statistics=True):
        self.current_trajectory = trajectory
        self.executor.load_trajectory(trajectory)
        vel_mags = np.linalg.norm(trajectory.velocities, axis=1)
        acc_mags = np.linalg.norm(trajectory.accelerations, axis=1)
        self.speed_graph.set_data(trajectory.timestamps, vel_mags, acc_mags)
        self._redraw_trajectory_preview()
        if update_statistics:
            self.update_stats()

    def _start_reachability_job(self, trajectory):
        self._cancel_reachability_job()
        n_pts = len(trajectory.points)
        trajectory.reachable_flags = [False] * n_pts
        self._reach_state = {
            "trajectory": trajectory,
            "planner": PathPlanner(self.workspace_plan),
            "tcp_link": self._get_active_tcp_link(),
            "index": 0,
            "total": n_pts,
        }
        self._set_path_controls_enabled(False)
        self.lbl_reachable.setText(f"Checking... 0 / {n_pts}")
        self.lbl_reachable.setStyleSheet("font-weight: bold; color: #1976d2;")
        self._reach_timer.start(0)

    def _reachability_tick(self):
        state = self._reach_state
        if state is None:
            return

        trajectory = state["trajectory"]
        if trajectory is not self.current_trajectory:
            self._cancel_reachability_job()
            return

        start = state["index"]
        end = min(start + self._reach_batch_size, state["total"])
        state["planner"].check_reachability_range(
            self.mw.robot,
            state["tcp_link"],
            trajectory,
            start,
            end,
            position_tolerance=0.5,
            orientation_weight=0.0,
        )
        state["index"] = end
        self.lbl_reachable.setText(f"Checking... {end} / {state['total']}")
        QtWidgets.QApplication.processEvents()

        if end < state["total"]:
            self._reach_timer.start(0)
            return

        self._reach_state = None
        self._set_path_controls_enabled(True)
        self._redraw_trajectory_preview()
        self.update_stats()

    def generate_path_from_control_points(self, fast_drag=False, skip_reachability=False):
        if self.workspace_plan is None or len(self.control_points) < 2:
            return
            
        planner = PathPlanner(self.workspace_plan)
        
        # During fast_drag, use fewer points for instantaneous spline calculation
        num_pts = 40 if fast_drag else 120
        interpolated_pts = planner.interpolate_path(self.control_points, method="cubic", num_points=num_pts)
        
        if fast_drag:
            trajectory = PathTrajectory(interpolated_pts)
            self.current_trajectory = trajectory
            self.mw.canvas.draw_trajectory_preview(trajectory, vis_mode="Geometry", show_axes=False, show_arrows=False)
            return
            
        # Cruise parameters
        max_v = self.vel_limit.value()
        max_a = self.accel_limit.value()
        trajectory = planner.apply_velocity_profile(interpolated_pts, max_vel=max_v, max_accel=max_a)
        trajectory.reachable_flags = [False] * len(trajectory.points)
        self._apply_trajectory_preview(trajectory, update_statistics=False)

        if not skip_reachability:
            self._start_reachability_job(trajectory)
        else:
            self.update_stats()

    # ----------------------------------------------------
    # Interactive Table Editor Operations
    # ----------------------------------------------------
    def get_waypoint_coords(self, idx):
        if idx >= 0 and idx < len(self.control_points):
            return self.control_points[idx].copy()
        return None

    def update_waypoint_coords(self, idx, pt, fast_drag=False):
        if idx < 0 or idx >= len(self.control_points):
            return
        self.control_points[idx] = pt
        if not fast_drag:
            self.table_waypoints.blockSignals(True)
            self.table_waypoints.setItem(idx, 1, QtWidgets.QTableWidgetItem(f"{pt[0]:.2f}"))
            self.table_waypoints.setItem(idx, 2, QtWidgets.QTableWidgetItem(f"{pt[1]:.2f}"))
            self.table_waypoints.setItem(idx, 3, QtWidgets.QTableWidgetItem(f"{pt[2]:.2f}"))
            self.table_waypoints.blockSignals(False)
        self.generate_path_from_control_points(fast_drag=fast_drag)

    def on_waypoint_drag_finished(self):
        self.update_waypoints_table()
        self.generate_path_from_control_points(fast_drag=False)
        self.push_undo_state()
        self.save_session()

    def update_waypoints_table(self):
        self.table_waypoints.blockSignals(True)
        self.table_waypoints.setRowCount(len(self.control_points))
        for i, pt in enumerate(self.control_points):
            self.table_waypoints.setItem(i, 0, QtWidgets.QTableWidgetItem(f"WP_{i}"))
            self.table_waypoints.setItem(i, 1, QtWidgets.QTableWidgetItem(f"{pt[0]:.2f}"))
            self.table_waypoints.setItem(i, 2, QtWidgets.QTableWidgetItem(f"{pt[1]:.2f}"))
            self.table_waypoints.setItem(i, 3, QtWidgets.QTableWidgetItem(f"{pt[2]:.2f}"))
        self.table_waypoints.blockSignals(False)
        
        if self.chk_interactive_points.isChecked() and self.workspace_plan is not None:
            self.mw.canvas.draw_waypoint_markers(self.control_points)

    def on_table_item_changed(self, item):
        row = item.row()
        col = item.column()
        if col < 1 or col > 3:
            return
        try:
            val = float(item.text())
            if self.snap_checkbox.isChecked():
                grid = self.workspace_plan.grid_size if self.workspace_plan else 1.0
                val = round(val / grid) * grid
                item.setText(f"{val:.2f}")
                
            self.control_points[row, col - 1] = val
            self.push_undo_state()
            self.generate_path_from_control_points()
            self.save_session()
        except ValueError:
            pt = self.control_points[row]
            item.setText(f"{pt[col - 1]:.2f}")

    def on_table_selection_changed(self):
        row = self.table_waypoints.currentRow()
        if row < 0 or row >= len(self.control_points):
            return
        if hasattr(self.mw.canvas, '_waypoint_actors'):
            for name, actor in self.mw.canvas._waypoint_actors.items():
                idx = int(name.split("_")[1])
                if idx == row:
                    actor.GetProperty().SetColor([0.9, 0.1, 0.4]) # Highlight pink
                else:
                    actor.GetProperty().SetColor([30/255.0, 136/255.0, 229/255.0])
            self.mw.canvas.plotter.render()

    def add_table_point(self):
        if self.workspace_plan is None:
            return
        new_pt = np.array([0.0, 0.0, 10.0], dtype=float)
        if len(self.control_points) > 0:
            new_pt = self.control_points[-1].copy() + np.array([10.0, 0.0, 0.0])
            
        if len(self.control_points) == 0:
            self.control_points = np.array([new_pt])
        else:
            self.control_points = np.vstack([self.control_points, new_pt])
            
        self.push_undo_state()
        self.update_waypoints_table()
        self.generate_path_from_control_points()
        self.save_session()

    def delete_table_point(self):
        row = self.table_waypoints.currentRow()
        if row < 0 or row >= len(self.control_points):
            return
        self.control_points = np.delete(self.control_points, row, axis=0)
        self.push_undo_state()
        self.update_waypoints_table()
        self.generate_path_from_control_points()
        self.save_session()

    def move_point_up(self):
        row = self.table_waypoints.currentRow()
        if row <= 0 or row >= len(self.control_points):
            return
        # Swap rows
        self.control_points[[row, row - 1]] = self.control_points[[row - 1, row]]
        self.push_undo_state()
        self.update_waypoints_table()
        self.table_waypoints.setCurrentCell(row - 1, 0)
        self.generate_path_from_control_points()
        self.save_session()

    def move_point_down(self):
        row = self.table_waypoints.currentRow()
        if row < 0 or row >= len(self.control_points) - 1:
            return
        self.control_points[[row, row + 1]] = self.control_points[[row + 1, row]]
        self.push_undo_state()
        self.update_waypoints_table()
        self.table_waypoints.setCurrentCell(row + 1, 0)
        self.generate_path_from_control_points()
        self.save_session()

    def push_undo_state(self):
        # Limit stack size to 50
        if len(self._undo_stack) >= 50:
            self._undo_stack.pop(0)
        self._undo_stack.append(np.copy(self.control_points))
        self._redo_stack.clear() # Clear redo stack on new action
        self.btn_undo.setEnabled(True)
        self.btn_redo.setEnabled(False)

    def trigger_undo(self):
        if len(self._undo_stack) > 1:
            # Current state is at top of stack, pop it to redo
            curr = self._undo_stack.pop()
            self._redo_stack.append(curr)
            
            # The previous state is now at top
            prev = self._undo_stack[-1]
            self.control_points = np.copy(prev)
            
            self.update_waypoints_table()
            self.generate_path_from_control_points()
            
            self.btn_redo.setEnabled(True)
            self.mw.log("↩️ Undo action completed.")
        elif len(self._undo_stack) == 1:
            # Only one state, popping it means emptying
            curr = self._undo_stack.pop()
            self._redo_stack.append(curr)
            self.control_points = np.array([], dtype=float)
            
            self.update_waypoints_table()
            self.generate_path_from_control_points()
            
            self.btn_redo.setEnabled(True)
            self.mw.log("↩️ Undo action completed (path cleared).")
            
        self.btn_undo.setEnabled(len(self._undo_stack) > 0)

    def trigger_redo(self):
        if len(self._redo_stack) > 0:
            state = self._redo_stack.pop()
            self._undo_stack.append(state)
            self.control_points = np.copy(state)
            
            self.update_waypoints_table()
            self.generate_path_from_control_points()
            
            self.btn_undo.setEnabled(True)
            self.mw.log("↪️ Redo action completed.")
        self.btn_redo.setEnabled(len(self._redo_stack) > 0)

    # ----------------------------------------------------
    # Playback Control System
    # ----------------------------------------------------
    def play_path(self):
        self.executor.play()
        self.update_execution_ui()
        self.exec_timer.start(33)
        self.mw.log("▶️ Trajectory simulation execution started.")

    def pause_path(self):
        self.executor.pause()
        self.update_execution_ui()
        self.exec_timer.stop()
        self.mw.log("⏸️ Trajectory simulation execution paused.")

    def stop_path(self):
        self.executor.stop()
        self.update_execution_ui()
        self.exec_timer.stop()
        self.reset_robot_to_start()
        self.mw.canvas.clear_live_tcp_marker()
        self.mw.log("⏹️ Trajectory simulation execution stopped.")

    def reset_path(self):
        self.executor.reset()
        self.update_execution_ui()
        self.exec_timer.stop()
        self.reset_robot_to_start()
        self.mw.canvas.clear_live_tcp_marker()
        self.mw.log("🔄 Trajectory simulation reset.")

    def step_path(self):
        # Allow running a single step forward / backward (approx 0.1s step size)
        if self.executor.state not in [ExecutionState.RUNNING, ExecutionState.ERROR, ExecutionState.COMPLETED]:
            self.executor.state = ExecutionState.RUNNING
            self.execution_tick(dt=0.1)
            if self.executor.state == ExecutionState.RUNNING:
                self.executor.state = ExecutionState.PAUSED
            self.update_execution_ui()

    def on_speed_changed(self, value):
        self.executor.set_speed(value)
        self.speed_lbl.setText(f"{value} %")

    def reset_robot_to_start(self):
        if not self.current_trajectory or len(self.current_trajectory.points) == 0:
            return
        self.move_robot_to_trajectory_point(0)

    def move_robot_to_trajectory_point(self, index_or_time):
        if not self.current_trajectory:
            return
        if isinstance(index_or_time, int):
            pt = self.current_trajectory.points[index_or_time]
            rot = self.current_trajectory.orientations[index_or_time]
        else:
            sample = self.current_trajectory.sample_at_time(index_or_time)
            if not sample: return
            pt, _, rot, _, _, _ = sample

        base_world = self.mw.robot.base_link.t_world if self.mw.robot.base_link else np.eye(4)
        r_base_world = base_world[:3, :3]
        pt_world = self.workspace_plan.convert_local_to_world(pt, base_world)
        r_world = r_base_world @ rot

        target_pose = np.eye(4, dtype=float)
        target_pose[:3, :3] = r_world
        target_pose[:3, 3] = pt_world

        tcp_link_obj = self._get_active_tcp_link()
        if tcp_link_obj is not None and hasattr(self.mw, "get_link_tool_point"):
            self.mw.get_link_tool_point(tcp_link_obj)
        success, _ = self.mw.robot.inverse_kinematics_pose(
            target_tcp_pose=target_pose,
            tcp_link=tcp_link_obj,
            max_iters=150,
            position_tolerance=0.5,
            orientation_weight=0.0
        )
        if success:
            self.mw.robot.update_kinematics()
            self.mw.canvas.update_transforms(self.mw.robot)
            self.mw.update_live_ui(render=True)

    def update_execution_ui(self):
        state = self.executor.state
        has_traj = self.current_trajectory is not None
        
        self.play_btn.setEnabled(has_traj and state in [ExecutionState.READY, ExecutionState.PAUSED, ExecutionState.STOPPED, ExecutionState.COMPLETED])
        self.pause_btn.setEnabled(has_traj and state == ExecutionState.RUNNING)
        self.stop_btn.setEnabled(has_traj and state in [ExecutionState.RUNNING, ExecutionState.PAUSED])
        self.reset_btn.setEnabled(has_traj and state in [ExecutionState.PAUSED, ExecutionState.COMPLETED, ExecutionState.ERROR])
        self.step_btn.setEnabled(has_traj and state in [ExecutionState.READY, ExecutionState.PAUSED, ExecutionState.STOPPED])
        self.speed_slider.setEnabled(has_traj)
        self.timeline_slider.setEnabled(has_traj)
        
        if state == ExecutionState.RUNNING:
            self.play_btn.setText("Running...")
            self.play_btn.setStyleSheet("background-color: #1b5e20; color: white; border: none; border-radius: 4px; font-weight: bold;")
        else:
            self.play_btn.setText("Play")
            self.play_btn.setStyleSheet("background-color: #4caf50; color: white; border: none; border-radius: 4px; font-weight: bold;")

        self.progress_bar.setValue(int(round(self.executor.get_progress())))

    def execution_tick(self, dt=None):
        if dt is None:
            dt = 0.033
            
        sample = self.executor.tick(dt)
        if sample is None:
            self.update_execution_ui()
            if self.executor.state in [ExecutionState.STOPPED, ExecutionState.COMPLETED, ExecutionState.ERROR]:
                self.exec_timer.stop()
                if self.executor.state == ExecutionState.COMPLETED:
                    self.mw.log("✅ Trajectory simulation execution completed.")
            return

        pt, norm, rot, vel, acc, reachable = sample

        if not reachable:
            self.executor.state = ExecutionState.ERROR
            self.exec_timer.stop()
            self.update_execution_ui()
            self.mw.log("❌ Execution halted: trajectory point is unreachable by kinematics solver!")
            self.log_diagnostic("Execution stopped: Unreachable trajectory point.")
            return

        base_world = self.mw.robot.base_link.t_world if self.mw.robot.base_link else np.eye(4)
        r_base_world = base_world[:3, :3]
        pt_world = self.workspace_plan.convert_local_to_world(pt, base_world)
        r_world = r_base_world @ rot

        target_pose = np.eye(4, dtype=float)
        target_pose[:3, :3] = r_world
        target_pose[:3, 3] = pt_world

        tcp_link_obj = self._get_active_tcp_link()
        if tcp_link_obj is not None and hasattr(self.mw, "get_link_tool_point"):
            self.mw.get_link_tool_point(tcp_link_obj)
        old_values = {name: joint.current_value for name, joint in self.mw.robot.joints.items()}
        
        # Calculate latency
        t_start = QtCore.QDeadlineTimer.current().deadlineNS()
        success, _ = self.mw.robot.inverse_kinematics_pose(
            target_tcp_pose=target_pose,
            tcp_link=tcp_link_obj,
            max_iters=150,
            position_tolerance=0.5,
            orientation_weight=0.0
        )
        t_end = QtCore.QDeadlineTimer.current().deadlineNS()
        latency_ms = (t_end - t_start) / 1e6
        self.diag_lbl_ik.setText(f"{latency_ms:.2f} ms")

        if success:
            self.mw.robot.update_kinematics()
            self.mw.canvas.update_transforms(self.mw.robot)
            
            # Live TCP markers & trails
            self.mw.canvas.update_live_tcp_marker(pt_world)
            if self.chk_show_trail.isChecked():
                self.mw.canvas.update_live_tcp_trail(pt_world)
            
            # Trajectory progress lines coloring
            self.mw.canvas.update_trajectory_progress(self.executor.elapsed_time, self.current_trajectory)
            
            # Sync timeline slider
            if self.current_trajectory:
                t_max = self.current_trajectory.timestamps[-1]
                t_curr = self.executor.elapsed_time
                val = int((t_curr / t_max) * 1000.0) if t_max > 0 else 0
                self.timeline_slider.blockSignals(True)
                self.timeline_slider.setValue(val)
                self.timeline_slider.blockSignals(False)
                
            self.mw.update_live_ui(render=True)
        else:
            for name, val in old_values.items():
                self.mw.robot.joints[name].current_value = val
            self.mw.robot.update_kinematics()
            self.mw.canvas.update_transforms(self.mw.robot)
            
            self.executor.state = ExecutionState.ERROR
            self.exec_timer.stop()
            self.mw.log("❌ Execution halted: IK solver failed to converge at runtime!")
            self.log_diagnostic("Execution aborted: IK solver converged failure.")
            
        self.update_execution_ui()

    def update_display(self):
        """Lifecycle hook called when tab is swapped to this panel."""
        if self.workspace_plan is not None:
            self.mw.canvas.show_workspace_planner(self.workspace_plan)
        if self.current_trajectory is not None:
            self._redraw_trajectory_preview()
        elif len(self.control_points) >= 2 and not self._reachability_in_progress():
            self.generate_path_from_control_points(skip_reachability=False)

    def _get_active_tcp_link(self):
        if hasattr(self.mw, "_get_preferred_tcp_link"):
            tcp = self.mw._get_preferred_tcp_link()
            if tcp: return tcp
        return "link2"

    # ----------------------------------------------------
    # Import / Export Implementation
    # ----------------------------------------------------
    def browse_and_import(self):
        filepath, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Import Toolpath", "", "Vector Paths (*.svg *.dxf *.csv *.json)"
        )
        if filepath:
            self.import_path_file(filepath)

    def import_path_file(self, filepath):
        pts = []
        try:
            ext = os.path.splitext(filepath)[1].lower()
            if ext == ".json":
                with open(filepath, "r") as f:
                    data = json.load(f)
                if isinstance(data, list):
                    for item in data:
                        if isinstance(item, list) and len(item) >= 2:
                            z = item[2] if len(item) >= 3 else 10.0
                            pts.append([float(item[0]), float(item[1]), float(z)])
                elif isinstance(data, dict) and "points" in data:
                    for item in data["points"]:
                        if len(item) >= 2:
                            z = item[2] if len(item) >= 3 else 10.0
                            pts.append([float(item[0]), float(item[1]), float(z)])
            elif ext == ".csv":
                with open(filepath, "r") as f:
                    import csv
                    reader = csv.reader(f)
                    for row in reader:
                        if len(row) >= 2:
                            try:
                                x = float(row[0])
                                y = float(row[1])
                                z = float(row[2]) if len(row) >= 3 else 10.0
                                pts.append([x, y, z])
                            except ValueError:
                                continue
            elif ext == ".svg":
                import xml.etree.ElementTree as ET
                tree = ET.parse(filepath)
                root = tree.getroot()
                
                for line in root.findall('.//{http://www.w3.org/2000/svg}line'):
                    pts.append([float(line.attrib.get('x1', 0)), float(line.attrib.get('y1', 0)), 10.0])
                    pts.append([float(line.attrib.get('x2', 0)), float(line.attrib.get('y2', 0)), 10.0])
                
                for path in root.findall('.//{http://www.w3.org/2000/svg}path'):
                    d = path.attrib.get('d', '')
                    coords = re.findall(r"[-+]?\d*\.\d+|\d+", d)
                    floats = [float(c) for c in coords]
                    for idx in range(0, len(floats) - 1, 2):
                        pts.append([floats[idx], floats[idx+1], 10.0])
            elif ext == ".dxf":
                with open(filepath, "r") as f:
                    lines = [l.strip() for l in f.readlines()]
                i = 0
                while i < len(lines):
                    if lines[i] == "AcDbLine":
                        x1 = y1 = z1 = x2 = y2 = z2 = 0.0
                        while i < len(lines) and lines[i] != "AcDbEntity":
                            if lines[i] == "10": x1 = float(lines[i+1])
                            elif lines[i] == "20": y1 = float(lines[i+1])
                            elif lines[i] == "30": z1 = float(lines[i+1])
                            elif lines[i] == "11": x2 = float(lines[i+1])
                            elif lines[i] == "21": y2 = float(lines[i+1])
                            elif lines[i] == "31": z2 = float(lines[i+1])
                            i += 1
                        pts.append([x1, y1, z1])
                        pts.append([x2, y2, z2])
                    i += 1
            else:
                self.mw.log(f"❌ Unsupported file extension: {ext}")
                return
                
            if len(pts) > 0:
                scale = self.imp_scale.value()
                off_x = self.imp_ox.value()
                off_y = self.imp_oy.value()
                off_z = self.imp_oz.value()
                
                final_pts = []
                for p in pts:
                    final_pts.append([
                        p[0] * scale + off_x,
                        p[1] * scale + off_y,
                        p[2] * scale + off_z
                    ])
                self.control_points = np.array(final_pts)
                self.push_undo_state()
                self.update_waypoints_table()
                self.generate_path_from_control_points()
                self.mw.log(f"✅ Imported {len(self.control_points)} waypoints from file.")
                self.log_diagnostic(f"Loaded import file coordinates: {len(self.control_points)} waypoints.")
            else:
                self.mw.log("⚠️ No points parsed from import file.")
        except Exception as e:
            self.mw.log(f"❌ Failed to parse imported file: {e}")

    def export_waypoints_json(self):
        if len(self.control_points) == 0:
            return
        filepath, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export Waypoints", "", "JSON Waypoints (*.json)")
        if filepath:
            try:
                data = {"points": self.control_points.tolist()}
                with open(filepath, "w") as f:
                    json.dump(data, f, indent=4)
                self.mw.log(f"✅ Waypoints exported to: {filepath}")
            except Exception as e:
                self.mw.log(f"❌ Waypoint export failed: {e}")

    def export_trajectory_gcode(self):
        if not self.current_trajectory:
            return
        filepath, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export G-Code Toolpath", "", "NC Toolpath (*.nc *.gcode)")
        if filepath:
            try:
                with open(filepath, "w") as f:
                    f.write("; E-Labs Path-Planning G-code Export\n")
                    f.write(f"; Points: {len(self.current_trajectory.points)}\n")
                    f.write("G90 ; Absolute positioning\n")
                    f.write("G21 ; Dimensions in mm\n")
                    for i, pt in enumerate(self.current_trajectory.points):
                        # Convert cm workspace coordinates to mm G-code space
                        v = np.linalg.norm(self.current_trajectory.velocities[i]) * 10.0 # mm/s
                        f.write(f"G1 X{pt[0]*10.0:.3f} Y{pt[1]*10.0:.3f} Z{pt[2]*10.0:.3f} F{v*60.0:.1f}\n")
                self.mw.log(f"✅ G-code toolpath exported: {filepath}")
            except Exception as e:
                self.mw.log(f"❌ G-code export failed: {e}")

    def export_trajectory_joints(self):
        if not self.current_trajectory:
            return
        if self._reachability_in_progress():
            QtWidgets.QMessageBox.information(
                self,
                "Reachability In Progress",
                "Please wait for reachability validation to finish before exporting joint angles.",
            )
            return
        filepath, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export Joint Angles", "", "Joint Profiles (*.csv)")
        if filepath:
            self._start_joint_export(filepath)

    def _start_joint_export(self, filepath):
        import csv

        robot = self.mw.robot
        trajectory = self.current_trajectory
        tcp_link_obj = self._get_active_tcp_link()
        original_joints = {name: j.current_value for name, j in robot.joints.items()}
        base_world = robot.base_link.t_world if robot.base_link else np.eye(4)
        r_base_world = base_world[:3, :3]

        progress = QtWidgets.QProgressDialog(
            "Computing joint angles for export...",
            "Cancel",
            0,
            len(trajectory.points),
            self,
        )
        progress.setWindowModality(QtCore.Qt.WindowModal)
        progress.setMinimumDuration(0)

        try:
            with open(filepath, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["Timestamp (s)"] + list(robot.joints.keys()))

                for idx, pt in enumerate(trajectory.points):
                    if progress.wasCanceled():
                        self.mw.log("⚠️ Joint export cancelled.")
                        return

                    progress.setValue(idx)
                    QtWidgets.QApplication.processEvents()

                    t = trajectory.timestamps[idx]
                    rot = trajectory.orientations[idx]
                    pt_world = self.workspace_plan.convert_local_to_world(pt, base_world)
                    r_world = r_base_world @ rot

                    target_pose = np.eye(4, dtype=float)
                    target_pose[:3, :3] = r_world
                    target_pose[:3, 3] = pt_world

                    success, _ = robot.inverse_kinematics_pose(
                        target_tcp_pose=target_pose,
                        tcp_link=tcp_link_obj,
                        max_iters=150,
                        position_tolerance=0.5,
                        orientation_weight=0.0,
                    )
                    if success:
                        robot.update_kinematics()
                        writer.writerow([t] + [robot.joints[name].current_value for name in robot.joints])

                progress.setValue(len(trajectory.points))
            self.mw.log(f"✅ Joint trajectory CSV profile exported: {filepath}")
        except Exception as e:
            self.mw.log(f"❌ Joint profile export failed: {e}")
        finally:
            progress.close()
            for name, val in original_joints.items():
                robot.joints[name].current_value = val
            robot.update_kinematics()
            self.mw.canvas.update_transforms(robot)
            self.mw.update_live_ui(render=True)

    # ----------------------------------------------------
    # Diagnostics & Autosave Session Loader
    # ----------------------------------------------------
    def log_diagnostic(self, msg):
        self.diag_console.appendPlainText(msg)
        
    def update_stats(self):
        if not self.current_trajectory:
            return
        if self._reachability_in_progress():
            state = self._reach_state
            self.lbl_reachable.setText(f"Checking... {state['index']} / {state['total']}")
            self.lbl_reachable.setStyleSheet("font-weight: bold; color: #1976d2;")
            return
        total = len(self.current_trajectory.points)
        reachable = sum(self.current_trajectory.reachable_flags)
        pct = (reachable / total * 100.0) if total > 0 else 0.0
        
        self.lbl_points.setText(f"{total}")
        self.lbl_length.setText(f"{self.current_trajectory.get_total_length():.2f} cm")
        self.lbl_time.setText(f"{self.current_trajectory.timestamps[-1]:.2f} s")
        
        reach_text = f"{reachable} / {total} ({pct:.1f}%)"
        self.lbl_reachable.setText(reach_text)
        if pct >= 99.9:
            self.lbl_reachable.setStyleSheet("font-weight: bold; color: #2e7d32;")
        else:
            self.lbl_reachable.setStyleSheet("font-weight: bold; color: #f44336;")
            
        # Diagnostics tab readouts
        self.diag_lbl_points.setText(f"{total} pts")
        self.diag_lbl_reach.setText(f"{pct:.1f}%")
        
        if pct < 100.0:
            self.diag_lbl_singularity.setText("⚠️ Singularity Alert")
            self.diag_lbl_singularity.setStyleSheet("font-weight: bold; color: #ff9100;")
            self.log_diagnostic("⚠️ Diagnostic WARNING: Path contains unreachable or near-singular waypoint locations.")
        else:
            self.diag_lbl_singularity.setText("Safe")
            self.diag_lbl_singularity.setStyleSheet("font-weight: bold; color: #2e7d32;")

    def load_session(self):
        session_path = "ccd_session.json"
        if os.path.exists(session_path):
            try:
                with open(session_path, "r") as f:
                    data = json.load(f)
                
                self.ws_width.setValue(data.get("width", 200.0))
                self.ws_height.setValue(data.get("height", 200.0))
                self.ws_grid_size.setValue(data.get("grid_size", 10.0))
                self.ws_margin.setValue(data.get("margin", 5.0))
                self.ws_ox.setValue(data.get("origin_x", 50.0))
                self.ws_oy.setValue(data.get("origin_y", 0.0))
                self.ws_oz.setValue(data.get("origin_z", 0.0))
                self.snap_checkbox.setChecked(data.get("snap", True))
                
                if "control_points" in data and len(data["control_points"]) > 0:
                    self.control_points = np.array(data["control_points"], dtype=float)
                    
                # Restore the workspace controls without drawing the plane.
                # The visualization is created by Initialize Workspace or Make Robo.
                ox = self.ws_ox.value()
                oy = self.ws_oy.value()
                oz = self.ws_oz.value()
                origin = np.array([ox, oy, oz], dtype=float)
                self.workspace_plan = WorkspacePlan(
                    width=self.ws_width.value(),
                    height=self.ws_height.value(),
                    grid_size=self.ws_grid_size.value(),
                    safe_margin=self.ws_margin.value(),
                    origin=origin
                )
                self.mw.canvas.current_workspace_plan = None
                self.toggle_ws_btn.setText("Show Workspace Plane")
                
                self.push_undo_state()
                self.update_waypoints_table()
                
                self.log_diagnostic("Loaded autosaved workspace settings. Workspace plane is hidden until initialized or Make Robo is pressed.")
            except Exception as e:
                self.log_diagnostic(f"Failed to load autosaved session: {e}")

    def save_session(self):
        session_path = "ccd_session.json"
        try:
            data = {
                "width": self.ws_width.value(),
                "height": self.ws_height.value(),
                "grid_size": self.ws_grid_size.value(),
                "margin": self.ws_margin.value(),
                "origin_x": self.ws_ox.value(),
                "origin_y": self.ws_oy.value(),
                "origin_z": self.ws_oz.value(),
                "snap": self.snap_checkbox.isChecked(),
                "control_points": self.control_points.tolist() if len(self.control_points) > 0 else []
            }
            with open(session_path, "w") as f:
                json.dump(data, f, indent=4)
        except Exception as e:
            self.log_diagnostic(f"Autosave session failed: {e}")

    # ----------------------------------------------------
    # Groupbox and button style properties
    # ----------------------------------------------------
    def _group_box_style(self):
        return """
            QGroupBox {
                border: 2px solid #e0e0e0;
                border-radius: 8px;
                margin-top: 10px;
                font-weight: bold;
                color: #1976d2;
                padding-top: 15px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 0 5px;
                left: 10px;
            }
        """

    def _primary_btn_style(self):
        return """
            QPushButton {
                background-color: #1976d2;
                color: white;
                border: none;
                border-radius: 6px;
                font-size: 12px;
                font-weight: bold;
                padding: 5px;
            }
            QPushButton:hover {
                background-color: #1565c0;
            }
            QPushButton:pressed {
                background-color: #0d47a1;
            }
        """

    def _secondary_btn_style(self):
        return """
            QPushButton {
                background-color: white;
                color: #424242;
                border: 1px solid #cccccc;
                border-radius: 6px;
                font-size: 11px;
                font-weight: bold;
                padding: 4px;
            }
            QPushButton:hover {
                background-color: #f5f5f5;
                border-color: #999999;
            }
            QPushButton:pressed {
                background-color: #e0e0e0;
            }
        """

    def _combo_style(self):
        return """
            QComboBox {
                border: 1px solid #cccccc;
                border-radius: 4px;
                padding: 3px;
                background-color: white;
                font-size: 11px;
            }
        """

    def _make_spin_box(self, lo, hi, val, suffix=""):
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setValue(val)
        spin.setSuffix(f" {suffix}" if suffix else "")
        spin.setDecimals(1)
        spin.setStyleSheet("""
            QDoubleSpinBox {
                border: 1px solid #cccccc;
                border-radius: 4px;
                padding: 3px;
                background-color: white;
            }
        """)
        return spin

    def _make_spin_box_int(self, lo, hi, val, suffix=""):
        spin = QtWidgets.QSpinBox()
        spin.setRange(lo, hi)
        spin.setValue(val)
        spin.setSuffix(f" {suffix}" if suffix else "")
        spin.setStyleSheet("""
            QSpinBox {
                border: 1px solid #cccccc;
                border-radius: 4px;
                padding: 3px;
                background-color: white;
            }
        """)
        return spin
