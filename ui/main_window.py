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
        self.last_project_dir = os.getcwd()
        self._init_navigation_mixin()
        self.init_ui()
        self.apply_styles()
        self._setup_live_point_refresh()
        
        # Connect signals
        self.log_signal.connect(self.log)
        
        # Center the window and fix geometry warnings
        self.center_on_screen()

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
        
        self.save_btn = QtWidgets.QPushButton("Save")
        self.save_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_DialogSaveButton))
        self.save_btn.setStyleSheet(btn_file_style)
        self.save_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.save_btn.clicked.connect(self.save_project)
        top_layout.addWidget(self.save_btn)
        
        self.open_btn = QtWidgets.QPushButton("Open")
        self.open_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_DialogOpenButton))
        self.open_btn.setStyleSheet(btn_file_style)
        self.open_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.open_btn.clicked.connect(self.load_project)
        top_layout.addWidget(self.open_btn)
        
        top_layout.addStretch()
        
        self.main_layout.addWidget(top_bar)
        
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
        
        # --- Home Position Button (next to isometric) ---
        self.home_btn = QtWidgets.QPushButton(self.canvas)
        self.home_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_DirHomeIcon))
        self.home_btn.setToolTip("Reset Robot to Home Position (0°)")
        self.home_btn.setFixedSize(38, 38)
        self.home_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.home_btn.setStyleSheet("""
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
        self.home_btn.clicked.connect(self.reset_to_home)
        
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
            self.home_btn.move(self.canvas.width() - 204, 24)
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
        self.log("✅ Home Position Restored.")
        
        # Show a friendly toast if method exists
        if hasattr(self, 'show_toast'):
            self.show_toast("Home Position Reset", "success")

    def _toggle_live_point_marker(self):
        """Toggle the red live-point dot on the 3D canvas."""
        visible = self.canvas.toggle_live_point_marker()
        self.live_point_btn.setChecked(visible)
        self.show_toast(
            "Live Point visible" if visible else "Live Point hidden",
            "info",
        )

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
            return # No geometry found
            
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
            return
            
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
            
            # Dynamically rewire Live Point to the peak of the robot
            self._auto_detect_topmost_tcp()
            
            # Update kinematics again to ensure TCP calculations are fresh
            self.robot.update_kinematics()
            
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
            workspace = None
            structure = None
            if joint_count and tcp_link is not None:
                self.custom_tcp_name = tcp_link.name
                # High-density workspace sampling (5000 samples for accuracy)
                workspace = self.robot.compute_workspace(tcp_link, max_samples=5000)
                if workspace.get("ok"):
                    ratio = getattr(self.canvas, "grid_units_per_cm", 1.0) or 1.0
                    bounds_min_cm = workspace["bounds_min"] / ratio
                    bounds_max_cm = workspace["bounds_max"] / ratio
                    radius_cm = workspace["radius_max"] / ratio
                    directional = workspace.get("directional_reach", {})
                    structure = self.robot.compute_structure_dynamics(
                        tcp_link=tcp_link,
                        workspace_report=workspace,
                        length_scale=ratio,
                    )
                    self.canvas.show_workspace_cloud(workspace["points"])
                    self.log(
                        "Workspace computed: "
                        f"{workspace['sample_count']} sampled configurations for TCP '{workspace['tcp_link']}'."
                    )
                    self.log("Workspace model is now available as IK seed memory for faster, smarter solves.")
                    self.log(
                        "Workspace bounds (cm): "
                        f"X[{bounds_min_cm[0]:.2f}, {bounds_max_cm[0]:.2f}] "
                        f"Y[{bounds_min_cm[1]:.2f}, {bounds_max_cm[1]:.2f}] "
                        f"Z[{bounds_min_cm[2]:.2f}, {bounds_max_cm[2]:.2f}] "
                        f"| max reach={radius_cm:.2f} cm"
                    )
                    if directional.get("ok"):
                        cardinal = directional.get("cardinal_reach", {})
                        self.log(
                            "Directional max reach (cm): "
                            f"+X={cardinal.get('+X', 0.0) / ratio:.2f}, "
                            f"-X={cardinal.get('-X', 0.0) / ratio:.2f}, "
                            f"+Y={cardinal.get('+Y', 0.0) / ratio:.2f}, "
                            f"-Y={cardinal.get('-Y', 0.0) / ratio:.2f}, "
                            f"+Z={cardinal.get('+Z', 0.0) / ratio:.2f}, "
                            f"-Z={cardinal.get('-Z', 0.0) / ratio:.2f}"
                        )
                        best_dir = directional.get("best_direction", {})
                        worst_dir = directional.get("worst_direction", {})
                        self.log(
                            "All-angle directional reach model ready: "
                            f"max={directional.get('max_directional_reach', 0.0) / ratio:.2f} cm "
                            f"at az={best_dir.get('azimuth_deg', 0.0):.1f} deg, el={best_dir.get('elevation_deg', 0.0):.1f} deg; "
                            f"min={directional.get('min_directional_reach', 0.0) / ratio:.2f} cm "
                            f"at az={worst_dir.get('azimuth_deg', 0.0):.1f} deg, el={worst_dir.get('elevation_deg', 0.0):.1f} deg."
                        )
                    if structure and structure.get("ok"):
                        total_com = structure["total_com_cm"]
                        balance_offset = structure["base_balance_offset_cm"]
                        self.log(
                            "Structure balance: "
                            f"total mass={structure['total_mass']:.3f} kg, "
                            f"COM=({total_com[0]:.2f}, {total_com[1]:.2f}, {total_com[2]:.2f}) cm, "
                            f"base offset=({balance_offset[0]:.2f}, {balance_offset[1]:.2f}, {balance_offset[2]:.2f}) cm."
                        )
                        sampled_loads = structure.get("sampled_joint_loads", {})
                        if sampled_loads:
                            self.log("Joint load analysis (static gravity torque, N*cm):")
                            for joint_name, load in sampled_loads.items():
                                self.log(
                                    f"  {joint_name}: "
                                    f"max axis={load['max_abs_axis_torque_ncm']:.2f}, "
                                    f"mean axis={load['mean_abs_axis_torque_ncm']:.2f}, "
                                    f"max support={load['max_resultant_torque_ncm']:.2f}, "
                                    f"max bending={load['max_bending_torque_ncm']:.2f}"
                                )
                else:
                    self.canvas.show_workspace_cloud(None)
                    self.log(f"Workspace computation skipped: {workspace.get('reason', 'unknown')}.")
            else:
                self.canvas.show_workspace_cloud(None)

            if joint_count:
                self.log(f"Robot model ready: {joint_count} joint(s) linked from base '{self.robot.base_link.name}'.")
                if workspace and workspace.get("ok"):
                    self.show_toast("Assembly Finalized + Workspace + Dynamics Computed", "success")
            else:
                self.log(f"Assembly refreshed with base '{self.robot.base_link.name}', but no joints are defined yet.")
            return True
        except Exception as exc:
            self.log(f"MAKE ROBO ERROR: {exc}")
            self.show_toast("Unable to finalize assembly", "error")
            return False

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
