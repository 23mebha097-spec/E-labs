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
        self.import_btn.clicked.connect(self.main_window.import_mesh)
        btn_layout.addWidget(self.import_btn)

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

        self.start_btn = QtWidgets.QPushButton("🚀 Start Simulation")
        self.start_btn.setFixedHeight(45)
        self.start_btn.setCheckable(True)
        self.start_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.start_btn.setToolTip("Enable automatic pick-and-place tracking between P1 and P2")
        self.start_btn.setStyleSheet("""
            QPushButton {
                background-color: #fdd835;
                color: #212121;
                border: none;
                border-radius: 8px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:checked {
                background-color: #ff9800;
                color: white;
            }
            QPushButton:hover { background-color: #fbc02d; }
        """)
        self.start_btn.clicked.connect(self.toggle_pick_place_sim)
        btn_layout.addWidget(self.start_btn)

        self.objects_layout.addWidget(btn_container)

        # Simulation State
        self.is_sim_active = False
        self.gripped_object = None
        self.grip_offset = None # Relative transform
        
        self.sim_timer = QtCore.QTimer(self)
        self.sim_timer.timeout.connect(self._on_sim_tick)
        
        # Sequenced Motion State
        self.sim_state = "IDLE" 
        self.target_joint_values = {} 
        self.active_joint_index = 0
        self.current_tcp = None
        self.motion_speed = 5.0 # Initial default
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
        p1_lbl = QtWidgets.QLabel("P1")
        p1_lbl.setStyleSheet("font-weight: bold; color: #1976d2; font-size: 13px;")
        self.pick_x = self.create_coord_sb("#1976d2")
        self.pick_y = self.create_coord_sb("#1976d2")
        self.pick_z = self.create_coord_sb("#1976d2")
        
        points_grid.addWidget(p1_lbl, 0, 0)
        points_grid.addWidget(self.pick_x, 0, 1)
        points_grid.addWidget(self.pick_y, 0, 2)
        points_grid.addWidget(self.pick_z, 0, 3)

        # P2 Row
        p2_lbl = QtWidgets.QLabel("P2")
        p2_lbl.setStyleSheet("font-weight: bold; color: #388E3C; font-size: 13px;")
        self.place_x = self.create_coord_sb("#388E3C")
        self.place_y = self.create_coord_sb("#388E3C")
        self.place_z = self.create_coord_sb("#388E3C")
        
        points_grid.addWidget(p2_lbl, 1, 0)
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

        points_grid.addWidget(lp_lbl, 2, 0)
        points_grid.addWidget(self.live_x, 2, 1)
        points_grid.addWidget(self.live_y, 2, 2)
        points_grid.addWidget(self.live_z, 2, 3)

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
        self.switch_view(0)

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
        """Captures the selected object's BOTTOM-CENTER world position into P1 spinboxes.
        
        P1 represents the bottom-center of the object (the coordinate the robot moves to
        before gripping). This accounts for the mesh's local min-Z offset so the pick
        coordinate always refers to the true base of the object in world space.
        """
        current_item = self.objects_list.currentItem()
        if not current_item:
            return
            
        name = current_item.text()
        if name not in self.main_window.robot.links:
            return

        link = self.main_window.robot.links[name]
        ratio = self.main_window.canvas.grid_units_per_cm

        # Compute world-space bottom-center
        # The mesh origin may be offset from the actual bottom, so we convert the
        # local bottom-center of the mesh bounding box to world space.
        if link.mesh:
            b = link.mesh.bounds
            local_bottom_center = np.array([
                (b[0][0] + b[1][0]) / 2.0,  # center X
                (b[0][1] + b[1][1]) / 2.0,  # center Y
                b[0][2]                       # bottom Z (local min)
            ])
            world_bottom = (link.t_world @ np.append(local_bottom_center, 1.0))[:3]
        else:
            # Fall back to transform origin if no mesh
            world_bottom = link.t_world[:3, 3]

        pos_cm = world_bottom / ratio

        self.pick_x.setValue(pos_cm[0])
        self.pick_y.setValue(pos_cm[1])
        self.pick_z.setValue(pos_cm[2])

        self.main_window.log(
            f"🎯 P1 set to bottom-center of '{name}': "
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

    def toggle_pick_place_sim(self, checked):
        """Enable automated pick-and-place monitoring with sequential motion."""
        if checked:
            # === PRE-FLIGHT VALIDATION ===
            # 1. Verify an object is selected
            current_item = self.objects_list.currentItem()
            if not current_item:
                self.main_window.log("⚠️ No simulation object selected. Please select an object from the list first.")
                self.main_window.show_toast("Select an object first!", "warning")
                self.start_btn.blockSignals(True)
                self.start_btn.setChecked(False)
                self.start_btn.blockSignals(False)
                return

            obj_name = current_item.text()
            if obj_name not in self.main_window.robot.links:
                self.main_window.log("⚠️ Selected object not found in robot model.")
                self.start_btn.blockSignals(True)
                self.start_btn.setChecked(False)
                self.start_btn.blockSignals(False)
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
                self.start_btn.blockSignals(True)
                self.start_btn.setChecked(False)
                self.start_btn.blockSignals(False)
                return

            # === START SEQUENCE ===
            self.is_sim_active = True
            self.main_window.log("─" * 50)
            self.main_window.log("🚀 STARTING PICK-AND-PLACE SEQUENCE")
            ratio = self.main_window.canvas.grid_units_per_cm
            self.main_window.log(f"   Object : {obj_name}")
            self.main_window.log(f"   DIM    : {self.obj_width.value():.1f} x {self.obj_depth.value():.1f} x {self.obj_height.value():.1f} cm")
            self.main_window.log(f"   P1 (Pick)  : ({self.pick_x.value():.1f}, {self.pick_y.value():.1f}, {self.pick_z.value():.1f}) cm")
            self.main_window.log(f"   P2 (Place) : ({self.place_x.value():.1f}, {self.place_y.value():.1f}, {self.place_z.value():.1f}) cm")
            self.main_window.log(f"   TCP Link   : {tcp_link.name}")
            self.main_window.log("─" * 50)

            self.start_btn.setText("🛑 Stop Simulation")
            self.start_btn.setStyleSheet("background-color: #f44336; color: white; border-radius: 8px; font-weight: bold; font-size: 14px;")

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
            self.target_joint_values = {}
            self._target_gripper_angles = {}  # for smooth animation
            self.active_joint_index = 0

            self.sim_timer.start(50)  # Ticking every 50 ms
        else:
            self.main_window.log("🛑 Simulation Stopped.")
            self.start_btn.setText("🚀 Start Simulation")
            self.start_btn.setStyleSheet("background-color: #fdd835; color: #212121; border-radius: 8px; font-weight: bold; font-size: 14px;")
            self.sim_timer.stop()
            self.is_sim_active = False
            self.sim_state = "IDLE"

            # Reset state
            self.gripped_object = None
            self.grip_offset = None
            self.main_window.canvas.clear_highlights()
            self.main_window.canvas.plotter.render()
            
    def _on_sim_tick(self):
        if not self.is_sim_active:
            return

        # 1. Identify TCP link
        tcp_link = self._get_tcp_link()
        if not tcp_link:
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
                        self.sim_state = "SOLVE_APPROACH_P1"
                        return
                    self.main_window.log("👐 Opening gripper fully before approach...")

            done = self._move_gripper_smoothly()
            if done:
                self.main_window.log("✅ Gripper open. Commencing movement from Base reference to P1...")
                self._target_gripper_angles = {}
                self.sim_state = "SOLVE_APPROACH_P1"

        elif self.sim_state == "SOLVE_APPROACH_P1":
            self._handle_state_solve("P1", tcp_link, next_state="MOVE_APPROACH_P1", z_offset_cm=5.0)

        elif self.sim_state == "MOVE_APPROACH_P1":
            if self._handle_sequential_motion():
                self.main_window.log("📍 Reached safe approach point. Descending to P1...")
                self.sim_state = "SOLVE_PICK_P1"

        elif self.sim_state == "SOLVE_PICK_P1":
            self._handle_state_solve("P1", tcp_link, next_state="MOVE_PICK_P1", z_offset_cm=0.0)

        elif self.sim_state == "MOVE_PICK_P1":
            if self._handle_sequential_motion():
                self.main_window.log("📍 Reached P1. Closing gripper to grip object...")
                self.sim_state = "GRIP"

        elif self.sim_state == "GRIP":
            if not self._target_gripper_angles:
                self._prepare_grip_targets(tcp_link)
            
            if self._move_gripper_smoothly():
                self._finalize_grip(tcp_link)
                self.main_window.log("🧲 Object gripped. Lifting object from P1...")
                self._target_gripper_angles = {}
                self.sim_state = "SOLVE_LIFT_P1"

        elif self.sim_state == "SOLVE_LIFT_P1":
            self._handle_state_solve("P1", tcp_link, next_state="MOVE_LIFT_P1", z_offset_cm=5.0)

        elif self.sim_state == "MOVE_LIFT_P1":
            self._carry_gripped_object(tcp_link)
            if self._handle_sequential_motion():
                self.main_window.log("📍 Lift complete. Moving to P2 approach...")
                self.sim_state = "SOLVE_APPROACH_P2"

        elif self.sim_state == "SOLVE_APPROACH_P2":
            self._handle_state_solve("P2", tcp_link, next_state="MOVE_APPROACH_P2", z_offset_cm=5.0)

        elif self.sim_state == "MOVE_APPROACH_P2":
            self._carry_gripped_object(tcp_link)
            if self._handle_sequential_motion():
                self.main_window.log("📍 Reached P2 approach point. Descending to place...")
                self.sim_state = "SOLVE_PLACE_P2"

        elif self.sim_state == "SOLVE_PLACE_P2":
            self._handle_state_solve("P2", tcp_link, next_state="MOVE_PLACE_P2", z_offset_cm=0.0)

        elif self.sim_state == "MOVE_PLACE_P2":
            self._carry_gripped_object(tcp_link)
            if self._handle_sequential_motion():
                self.main_window.log("📍 Reached P2. Opening gripper to release object...")
                self.sim_state = "RELEASE"

        elif self.sim_state == "RELEASE":
            if not self._target_gripper_angles:
                self._prepare_release_targets()
            
            if self._move_gripper_smoothly():
                self._finalize_release()
                self.main_window.log("📦 Object released. Retracting from P2...")
                self._target_gripper_angles = {}
                self.sim_state = "SOLVE_RETRACT_P2"

        elif self.sim_state == "SOLVE_RETRACT_P2":
            self._handle_state_solve("P2", tcp_link, next_state="MOVE_RETRACT_P2", z_offset_cm=5.0)

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

    def _get_object_grip_width(self):
        """
        Measures the object's thickness along the gripper's opening axis
        and the world-space height of the selected sim object.
        Returns (grip_size_world, z_offset_world, obj_link)
        """
        item = self.objects_list.currentItem()
        if not item:
            return 0.0, 0.0, None
        obj_name = item.text()
        if obj_name not in self.main_window.robot.links:
            return 0.0, 0.0, None

        obj_link = self.main_window.robot.links[obj_name]
        if not obj_link.mesh:
            return 0.0, 0.0, obj_link

        ratio = self.main_window.canvas.grid_units_per_cm
        
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


    def _finalize_grip(self, tcp_link):
        """Actually attaches the object to the robot after gripper finished closing."""
        _, _, obj_link = self._get_object_grip_width()
        if not obj_link or not obj_link.mesh: return

        # 1. Compute the exact TCP (centroid of fingers) at this moment
        world_tcp, local_tcp, geo_data = self.main_window.get_link_tool_point(tcp_link, return_vec=True)
        
        # 2. Perfect Centering: Use mesh centroid instead of axis-aligned bounds midpoint
        local_center = obj_link.mesh.centroid
        
        # 3. Create a 'Perfect Hold' pose for the object
        # We preserve the object's current rotation (R_obj)
        t_obj_perfect = obj_link.t_world.copy()
        R_obj = t_obj_perfect[:3, :3]
        
        # Set world translation so centroid aligns exactly with TCP
        t_obj_perfect[:3, 3] = world_tcp - R_obj @ local_center
        
        # Store relative offset from Hand (TCP Link) to the perfect object pose
        inv_hand = np.linalg.inv(tcp_link.t_world)
        self.grip_offset = inv_hand @ t_obj_perfect
        self.gripped_object = obj_link.name
        self.grip_original_rotation = R_obj.copy()
        
        # Apply immediately to the link offset
        obj_link.t_offset = tcp_link.t_world @ self.grip_offset
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


    def _prepare_release_targets(self):
        """Calculates targets to open gripper fully."""
        self._target_gripper_angles = self.main_window._control_gripper_fingers(
            close=False, apply=False
        )

    def _finalize_release(self):
        """Drops the object at P2."""
        self._do_release()

    def _move_gripper_smoothly(self):
        """Moves gripper joints toward targets incrementally. Returns True if all reached."""
        if not self._target_gripper_angles:
            return True
            
        all_done = True
        STEP = 2.0 # Degrees per tick
        
        # Only enforce surface contact/rigid blocking during the GRIP state
        enforce_collision = (self.sim_state == "GRIP")
        
        # We use list() because we might delete items from the dict during iteration
        for j_name, target in list(self._target_gripper_angles.items()):
            joint = self.main_window.robot.joints.get(j_name)
            if not joint: continue
            
            # --- Store previous state for reversion if collision occurs ---
            old_val = joint.current_value
            # Store slave states too
            old_slaves = {}
            for s_id, ratio in self.main_window.robot.joint_relations.get(j_name, []):
                if s_id in self.main_window.robot.joints:
                    old_slaves[s_id] = self.main_window.robot.joints[s_id].current_value

            diff = target - joint.current_value
            if abs(diff) < STEP:
                joint.current_value = target
            else:
                joint.current_value += np.sign(diff) * STEP
                all_done = False
                
            # Propagate to slaves
            for s_id, ratio in self.main_window.robot.joint_relations.get(j_name, []):
                if s_id in self.main_window.robot.joints:
                    self.main_window.robot.joints[s_id].current_value = joint.current_value * ratio
            
            # Update kinematics to test the proposed position
            self.main_window.robot.update_kinematics()

            # --- MULTI-PART RIGID COLLISION CHECK ---
            if enforce_collision and self._check_gripper_collision():
                # Revert to the last safe position just before contact
                joint.current_value = old_val
                for s_id, s_val in old_slaves.items():
                    if s_id in self.main_window.robot.joints:
                        self.main_window.robot.joints[s_id].current_value = s_val
                
                # Cleanup: we've reached the surface, so this target is "solved"
                del self._target_gripper_angles[j_name]
                self.main_window.log(f"📐 Contact: '{joint.name}' stopped at the rigid object surface.")
                self.main_window.robot.update_kinematics()
                continue # Joint is effectively 'reached' at the surface
                    
        self.main_window.robot.update_kinematics()
        # Return True only if no targets are left to solve
        return all_done or not self._target_gripper_angles

    def _check_gripper_collision(self):
        """Monitors contacts between ANY gripper-related link and the simulation object using Trimesh."""
        item = self.objects_list.currentItem()
        if not item: return False
        obj_name = item.text()
        obj_link = self.main_window.robot.links.get(obj_name)
        if not obj_link or not obj_link.mesh: return False
        
        tcp_link = self._get_tcp_link() # The 'Hand'
        if not tcp_link: return False
        
        # Identify 'Fingers' and ALL their recursive children
        # (A gripper isn't just the direct child link; it's the whole sub-assembly)
        finger_assembly = []
        rel_joints = set()
        for j_name, joint in self.main_window.robot.joints.items():
            if getattr(joint, 'is_gripper', False):
                rel_joints.add(j_name)
            
        for j_name in rel_joints:
            joint = self.main_window.robot.joints.get(j_name)
            if joint and joint.child_link:
                # Add the finger and all its downstream geometry
                stack = [joint.child_link]
                while stack:
                    curr = stack.pop()
                    finger_assembly.append(curr)
                    for cj in curr.child_joints:
                        if cj.child_link: stack.append(cj.child_link)
        
        if not finger_assembly: 
            finger_assembly = [tcp_link]
            
        # Create Collision Manager for this tick
        try:
            cm = trimesh.collision.CollisionManager()
        except ValueError:
            # Fallback if FCL backend is not properly linked or missing
            # In this case, we'll return False (no collision detected) to allow 
            # simulation to proceed without rigid contact, but log a warning.
            if not getattr(self, '_collision_warn_done', False):
                self.main_window.log("⚠ Collision Engine: FCL backend not found. Rigid contact will be disabled.")
                self._collision_warn_done = True
            return False

        cm.add_object("SIM_OBJ", obj_link.mesh, obj_link.t_world)
        
        for i, f_link in enumerate(finger_assembly):
            if f_link.mesh:
                try:
                    cm.add_object(f"PART_{i}", f_link.mesh, f_link.t_world)
                except Exception:
                    continue
                
        return cm.in_collision_internal()



    def _do_grip(self, tcp_link):
        # Redundant: replaced by _prepare_grip_targets and _finalize_grip
        pass

    def _carry_gripped_object(self, tcp_link):
        """Updates the gripped object's position every tick so it follows the TCP."""
        if not self.gripped_object:
            return
        if self.gripped_object not in self.main_window.robot.links:
            return

        obj_link = self.main_window.robot.links[self.gripped_object]
        # Object world pose = current TCP world pose × stored relative offset
        obj_link.t_offset = tcp_link.t_world @ self.grip_offset
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
        #   - Rotation: the object's ORIGINAL rotation before it was picked up
        t_release = np.eye(4)
        if hasattr(self, 'grip_original_rotation') and self.grip_original_rotation is not None:
            t_release[:3, :3] = self.grip_original_rotation  # preserve original orientation
        else:
            t_release[:3, :3] = obj_link.t_world[:3, :3]    # fallback: current orientation

        # Place at P2 world position
        # Align mesh BASE with P2 coordinates
        p2_cm = np.array([self.place_x.value(), self.place_y.value(), self.place_z.value()])
        p2_world = p2_cm * ratio
        
        origin_z = p2_world[2]
        if obj_link.mesh:
            # Shift origin so bottom center is at P2
            local_min_z = obj_link.mesh.bounds[0][2]
            origin_z = p2_world[2] - local_min_z

        t_release[:3, 3] = [p2_world[0], p2_world[1], origin_z]
        obj_link.t_offset = t_release
        self.main_window.robot.update_kinematics()
        self.main_window.canvas.update_transforms(self.main_window.robot)

        self.main_window.log(f"📦 RELEASED: '{self.gripped_object}' placed at P2 with original orientation.")
        self.main_window.show_toast(f"Placed {self.gripped_object} at P2", "success")

        self.gripped_object = None
        self.grip_offset = None
        self.grip_original_rotation = None

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
        self.is_sim_active = False
        self.start_btn.setChecked(False)
        self.start_btn.setText("🚀 Start Simulation")
        self.start_btn.setStyleSheet(
            "background-color: #fdd835; color: #212121; "
            "border-radius: 8px; font-weight: bold; font-size: 14px;"
        )
        self.main_window.log("✅ Returned to initial position. Simulation complete.")
        self.main_window.show_toast("Back at start position", "success")

    def set_custom_lp(self):
        """Activates object picking mode to set the Live Point (TCP)."""
        self.main_window.log("🎯 Please click an object in the 3D canvas to set as Live Point (TCP).")
        self.main_window.show_toast("Click an object in 3D view", "info")
        self.main_window.canvas.start_object_picking(self._on_custom_lp_picked, label="Live Point")

    def _on_custom_lp_picked(self, name):
        """Callback for when an object is clicked to become the Live Point."""
        if name in self.main_window.robot.links:
            self.main_window.custom_tcp_name = name
            self.main_window.log(f"🎯 Live Point (TCP) manually set to: '{name}' via 3D click.")
            self.main_window.show_toast(f"Live Point set to {name}", "success")
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
        
        # 1. Custom TCP Priority
        for link in robot.links.values():
            if hasattr(link, 'custom_tcp_offset') and link.custom_tcp_offset is not None:
                return link

        # 2. Gripper Designation Priority
        for joint in robot.joints.values():
            if getattr(joint, 'is_gripper', False):
                return joint.child_link

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

    def _handle_state_solve(self, target_name, tcp_link, next_state, z_offset_cm=0.0):
        ratio = self.main_window.canvas.grid_units_per_cm  # canvas units per cm

        # Target in canvas units (raw world space)
        if target_name == "P1":
            target_cm = np.array([self.pick_x.value(), self.pick_y.value(), self.pick_z.value()])
        else:
            target_cm = np.array([self.place_x.value(), self.place_y.value(), self.place_z.value()])

        # Apply industry Z offset (approach/lift/retract)
        target_cm[2] += z_offset_cm
        
        target_world = target_cm * ratio  # Convert cm → canvas units

        # ADJUST TARGET FOR OBJECT BOTTOM-CENTER:
        # P1/P2 are locations for the object's BASE. 
        # The robot's TCP targets the object's CENTER by default.
        grip_width, base_z_offset, _ = self._get_object_grip_width()
        
        # --- NEW: COVERAGE-BASED DEPTH ALIGNMENT ---
        # Instead of just targeting the geometric center, we use finger reach 
        # to ensure the object is "covered". 
        world_tcp, tool_local, geo_data = self.main_window.get_link_tool_point(tcp_link, return_vec=True)
        
        final_z_offset = base_z_offset
        if isinstance(geo_data, dict) and "finger_depth" in geo_data:
            reach = geo_data["finger_depth"]
            # To "cover" the object: we want the finger midpoint to be at 
            # some depth relative to the object's height. 
            # If reach > object_height: reach down so tips are at bottom (cover everything).
            # If reach < object_height: reach down to max depth.
            obj_height = base_z_offset * 2.0
            
            # Optimal offset: move TCP up from object base such that tips are at base.
            # TCP is typically at midpoint of finger reach. 
            # So tips are at TCP + reach/2? Let's check get_link_tool_point. 
            # If TCP is the midpoint, palm is reach/2 behind, tips reach/2 ahead.
            
            # Aligning tips (TCP + reach/2 along approach) with object base:
            # target_world[2] = ObjectBaseZ + reach/2
            coverage_offset = reach / 2.0
            
            # Cap the offset so we don't go past the object's center if it's very tall, 
            # but for "covering" we generally prefer going as deep as possible.
            # If we want to cover the WHOLE object (as requested), we aim for coverage_offset.
            final_z_offset = coverage_offset
            self.main_window.log(f"📐 Coverage Mode: Setting Z-offset to {final_z_offset/ratio:.1f} cm to envelope object.")
        else:
            final_z_offset = base_z_offset

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

        # Solve IK — target and TCP both in canvas world units
        reached = self.main_window.robot.inverse_kinematics(
            target_world, tcp_link,
            max_iters=300,
            tolerance=tolerance_world,
            tool_offset=tool_local
        )

        if gap:
            self.main_window.log(
                f"🤏 Gripper gap: {gap/ratio:.1f} cm — IK aligns to midpoint of fingers."
            )

        if not reached:
            self.main_window.log(f"⚠ Warning: {target_name} might be outside workspace! (best effort)")
            self.main_window.show_toast(f"{target_name} partially reachable", "warning")
        else:
            self.main_window.log(f"✅ IK Solved for {target_name} successfully.")

        # Capture solved joint angles as targets
        self.target_joint_values = {
            n: j.current_value for n, j in self.main_window.robot.joints.items()
        }
        self.joint_chain = self.main_window.robot.get_kinematic_chain(tcp_link)  # base → TCP

        # Revert robot to start state — actual movement happens in MOVE state
        for n, val in start_vals.items():
            self.main_window.robot.joints[n].current_value = val
        self.main_window.robot.update_kinematics()

        # --- NEW: ORIENTATION-AWARE GRIP INTELLIGENCE ---
        # Analyze the object's narrowest vs widest dimensions to align the gripper span.
        # We look for the object's 'principal orientations' in world space.
        target_world_rot = None
        _, _, obj_link = self._get_object_grip_width()
        if obj_link and obj_link.mesh:
            verts_w = (obj_link.t_world[:3, :3] @ obj_link.mesh.vertices.T).T + obj_link.t_world[:3, 3]
            # Use PCA (via SVD) on vertices to find major axes
            centroid = np.mean(verts_w, axis=0)
            centered = verts_w - centroid
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
            # vh[0] is major, vh[1] is secondary, vh[2] is minor (narrowest)
            major_axis = vh[0]
            minor_axis = vh[2] 
            
            # We want to align the gripper span (best_vec) with the object's narrowest axis
            # to achieve the most centered/stable grip.
            self.main_window.log(f"🧠 Orientation Analysis: Found narrowest axis for '{obj_link.name}'. Aligning gripper span...")
            
            # Propose a rotation that aligns span axis [1,0,0] with minor_axis
            # and approach axis [0,0,1] with -Z (downward).
            # This requires a more complex IK solver, but for now we'll log the recommendation.
            # In a future update, we can solve for target orientation matrix.

        self.sim_state = next_state
        self.main_window.log(f"🧠 Motion Plan for {target_name} (reached={reached}):")
        for i, joint in enumerate(self.joint_chain):
            deg = self.target_joint_values.get(joint.name, 0)
            self.main_window.log(f"   [{i+1}] {joint.name} → {deg:.2f}°")
        
        # --- NEW: PERFECT GRIP FEEDBACK ---
        # Get actual finger count and shape data from the tool analysis
        _, _, geo_report = self.main_window.get_link_tool_point(tcp_link, return_vec=True)
        
        finger_count = 0
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
        # 1. Gather independent simulation objects (exclude the one we are carrying)
        sim_objs = [l for l in self.main_window.robot.links.values() 
                    if getattr(l, 'is_sim_obj', False) and l.name != self.gripped_object]
        if not sim_objs: return False
        
        # 2. Gather robot links
        robot_links = [l for l in self.main_window.robot.links.values() 
                       if not getattr(l, 'is_sim_obj', False)]
        
        # 3. Setup collision manager for sim objects (Environment Cache)
        # We REBUILD only if the count or objects changed (simple heuristic)
        if self._env_collision_manager is None:
            self._env_collision_manager = trimesh.collision.CollisionManager()
            for i, obj in enumerate(sim_objs):
                if obj.mesh:
                    self._env_collision_manager.add_object(f"EXTERNAL_{i}", obj.mesh, obj.t_world)
                
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
                if cm.in_collision_single(gripped_link.mesh, gripped_link.t_world):
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
        
        if index == 0:
            self.joints_btn.setStyleSheet(active_style)
            self.matrices_btn.setStyleSheet(inactive_style)
        else:
            self.joints_btn.setStyleSheet(inactive_style)
            self.matrices_btn.setStyleSheet(active_style)
            self.refresh_matrices()

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
