from PyQt5 import QtWidgets, QtCore, QtGui
from graphics.canvas import RobotCanvas
from core.robot import Robot
from ui.panels.align_panel import AlignPanel
from ui.panels.joint_panel import JointPanel
from ui.panels.experiment_panel import ExperimentPanel
from ui.panels.program_panel import ProgramPanel
from ui.panels.gripper_panel import GripperPanel
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
        self.init_ui()
        self.apply_styles()
        
        # Connect signals
        self.log_signal.connect(self.log)
        
        # Center the window and fix geometry warnings
        self.center_on_screen()

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
        self.simulation_tab = self.gripper_tab  # Alias for Torotron compatibility
        
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

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
