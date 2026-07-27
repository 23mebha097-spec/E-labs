from PyQt5 import QtWidgets, QtCore, QtGui
import numpy as np


def build_alignment_transform(parent_center, parent_normal, child_center, child_normal, flip=False):
    """Build the rigid transform that snaps a child component onto a parent face."""
    p_center = np.asarray(parent_center, dtype=float)
    p_normal = np.asarray(parent_normal, dtype=float)
    c_center = np.asarray(child_center, dtype=float)
    c_normal = np.asarray(child_normal, dtype=float)

    if np.linalg.norm(p_normal) < 1e-9 or np.linalg.norm(c_normal) < 1e-9:
        raise ValueError("Parent and child normals must be non-zero.")

    target_normal = -p_normal if not flip else p_normal
    rotation_mat = get_rotation_between_vectors(c_normal, target_normal)

    r_mat = np.eye(4)
    r_mat[:3, :3] = rotation_mat
    t_to_origin = np.eye(4)
    t_to_origin[:3, 3] = -c_center
    t_back = np.eye(4)
    t_back[:3, 3] = p_center

    return t_back @ r_mat @ t_to_origin


def get_rotation_between_vectors(v_from, v_to):
    v_from = np.asarray(v_from, dtype=float)
    v_to = np.asarray(v_to, dtype=float)

    v_from = v_from / np.linalg.norm(v_from)
    v_to = v_to / np.linalg.norm(v_to)

    v = np.cross(v_from, v_to)
    c = np.dot(v_from, v_to)
    s = np.linalg.norm(v)

    if s < 1e-6:
        if c > 0:
            return np.eye(3)
        ortho = np.array([1, 0, 0]) if abs(v_from[0]) < 0.9 else np.array([0, 1, 0])
        axis = np.cross(v_from, ortho)
        axis /= np.linalg.norm(axis)
        return 2 * np.outer(axis, axis) - np.eye(3)

    v_skew = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + v_skew + (v_skew @ v_skew) * ((1 - c) / (s**2))


class TypeOnlyDoubleSpinBox(QtWidgets.QDoubleSpinBox):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
    def stepBy(self, steps): pass
    def wheelEvent(self, event): event.ignore()

class AlignPanel(QtWidgets.QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.mw = main_window
        
        self.parent_pick_data = None # (center, normal)
        self.child_pick_data = None  # (center, normal)
        self.temp_offset = np.eye(4)
        
        self.init_ui()

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)
        
        # 1. Section Header
        header = QtWidgets.QLabel("ASSEMBLY ALIGNMENT")
        header.setFont(QtGui.QFont("Segoe UI", 14, QtGui.QFont.Bold))
        header.setStyleSheet("color: #1976d2; margin-bottom: 6px;")
        layout.addWidget(header)

        self.info_label = QtWidgets.QLabel(
            "Pick a parent face and a child face to align a component against the assembly."
        )
        self.info_label.setWordWrap(True)
        self.info_label.setStyleSheet("font-size: 12px; color: #546e7a; margin-bottom: 4px;")
        layout.addWidget(self.info_label)

        layout.addSpacing(6)

        # 2. Parent Picking
        step1_lbl = QtWidgets.QLabel("Step 1: Pick Parent Face")
        step1_lbl.setStyleSheet("font-size: 14px; font-weight: bold; color: #424242; padding: 4px 0;")
        layout.addWidget(step1_lbl)
        self.parent_btn = QtWidgets.QPushButton("Select Parent Face")
        self.parent_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.parent_btn.setStyleSheet("font-size: 14px; padding: 10px;")
        self.parent_btn.clicked.connect(self.pick_parent_face)
        layout.addWidget(self.parent_btn)
        
        self.parent_label = QtWidgets.QLabel("Parent: None selected")
        self.parent_label.setStyleSheet("font-size: 13px; color: #757575; padding: 2px 4px;")
        layout.addWidget(self.parent_label)

        layout.addSpacing(8)

        # 3. Child Picking
        step2_lbl = QtWidgets.QLabel("Step 2: Pick Child Face")
        step2_lbl.setStyleSheet("font-size: 14px; font-weight: bold; color: #424242; padding: 4px 0;")
        layout.addWidget(step2_lbl)
        self.child_btn = QtWidgets.QPushButton("Select Child Face")
        self.child_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.child_btn.setStyleSheet("font-size: 14px; padding: 10px;")
        self.child_btn.clicked.connect(self.pick_child_face)
        layout.addWidget(self.child_btn)
        
        self.child_label = QtWidgets.QLabel("Child: None selected")
        self.child_label.setStyleSheet("font-size: 13px; color: #757575; padding: 2px 4px;")
        layout.addWidget(self.child_label)

        layout.addSpacing(8)

        self.clear_btn = QtWidgets.QPushButton("Clear Selections")
        self.clear_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.clear_btn.setStyleSheet("font-size: 13px; padding: 8px;")
        self.clear_btn.clicked.connect(self.reset_panel)
        layout.addWidget(self.clear_btn)

        layout.addSpacing(8)
        
        # Flip Option
        self.flip_check = QtWidgets.QCheckBox("Flip Direction")
        self.flip_check.setStyleSheet("font-size: 14px; padding: 4px;")
        self.flip_check.stateChanged.connect(self.apply_alignment)
        layout.addWidget(self.flip_check)

        layout.addSpacing(8)

        # 4. Action Buttons
        button_layout = QtWidgets.QHBoxLayout()
        self.align_btn = QtWidgets.QPushButton("Align Components")
        self.align_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.align_btn.setStyleSheet("""
            QPushButton {
                background-color: #1976d2;
                color: white;
                font-weight: bold;
                font-size: 14px;
                padding: 10px;
                border: none;
                border-radius: 8px;
            }
            QPushButton:hover { background-color: #1565c0; }
        """)
        self.align_btn.clicked.connect(self.apply_alignment)
        button_layout.addWidget(self.align_btn)
        layout.addLayout(button_layout)

        layout.addSpacing(16)

        # 5. Fine Tuning
        layout.addWidget(QtWidgets.QLabel("FINE ROTATION (Deg)"))
        self.sliders = {}
        self.spins = {}
        for axis in ['Roll', 'Pitch', 'Yaw']:
            slider_layout = QtWidgets.QHBoxLayout()
            
            label = QtWidgets.QLabel(axis[:1])
            label.setFixedWidth(15)
            slider_layout.addWidget(label)
            
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(-180, 180)
            slider.setValue(0)
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
            
            spin = TypeOnlyDoubleSpinBox()
            spin.setRange(-180, 180)
            spin.setValue(0)
            spin.setFixedWidth(70)
            spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
            spin.setStyleSheet("""
                QDoubleSpinBox {
                    background: white;
                    color: #1976d2;
                    border: 1px solid #1976d2;
                    border-radius: 3px;
                    padding: 2px;
                    font-weight: bold;
                }
            """)

            # Bidirectional sync
            slider.valueChanged.connect(lambda val, s=spin: s.setValue(val))
            spin.valueChanged.connect(lambda val, sl=slider: sl.setValue(int(val)))
            
            # Trigger updates
            slider.valueChanged.connect(self.update_preview)
            
            slider_layout.addWidget(slider)
            slider_layout.addWidget(spin)
            layout.addLayout(slider_layout)
            
            self.sliders[axis] = slider
            self.spins[axis] = spin

        layout.addSpacing(20)

        self.save_btn = QtWidgets.QPushButton("Save & Lock")
        self.save_btn.clicked.connect(self.save_alignment)
        layout.addWidget(self.save_btn)

        layout.addSpacing(10)

        # 6. History Management (Undo/Redo)
        history_layout = QtWidgets.QHBoxLayout()
        self.undo_btn = QtWidgets.QPushButton("Undo")
        self.undo_btn.clicked.connect(self.undo_action)
        
        self.redo_btn = QtWidgets.QPushButton("Redo")
        self.redo_btn.clicked.connect(self.redo_action)
        
        history_layout.addWidget(self.undo_btn)
        history_layout.addWidget(self.redo_btn)
        layout.addLayout(history_layout)
        
        layout.addStretch()

        # History Stacks
        self.undo_stack = []
        self.redo_stack = []

    def push_history(self):
        """Saves everything: robot pose, selected faces, and UI settings."""
        state = {
            'robot': {name: link.t_offset.copy() for name, link in self.mw.robot.links.items()},
            'ui': {
                'parent_pick': self.parent_pick_data.copy() if self.parent_pick_data else None,
                'child_pick': self.child_pick_data.copy() if self.child_pick_data else None,
                'temp_offset': self.temp_offset.copy(),
                'flip': self.flip_check.isChecked(),
                'rotations': {axis: s.value() for axis, s in self.spins.items()}
            }
        }
        self.undo_stack.append(state)
        if len(self.undo_stack) > 30: self.undo_stack.pop(0)
        self.redo_stack.clear()

    def restore_state(self, state):
        """Deep restoration of both geometry and interface."""
        # 1. Restore Labels
        ui = state['ui']
        self.parent_pick_data = ui['parent_pick']
        self.child_pick_data = ui['child_pick']
        self.temp_offset = ui['temp_offset']
        
        p_name = self.parent_pick_data['name'] if self.parent_pick_data else "None selected"
        c_name = self.child_pick_data['name'] if self.child_pick_data else "None selected"
        self.parent_label.setText(f"Parent: {p_name}")
        self.child_label.setText(f"Child: {c_name}")
        
        # 2. Restore Checkbox (Signal blocked to avoid auto-align loop)
        self.flip_check.blockSignals(True)
        self.flip_check.setChecked(ui['flip'])
        self.flip_check.blockSignals(False)
        
        # 3. Restore Sliders & Spins
        for axis, val in ui['rotations'].items():
            self.spins[axis].blockSignals(True)
            self.spins[axis].setValue(val)
            self.spins[axis].blockSignals(False)
            self.sliders[axis].blockSignals(True)
            self.sliders[axis].setValue(int(val))
            self.sliders[axis].blockSignals(False)

        # 4. Restore Robot
        for name, offset in state['robot'].items():
            if name in self.mw.robot.links:
                self.mw.robot.links[name].t_offset = offset
        
        self.mw.robot.update_kinematics()
        self.mw.canvas.update_transforms(self.mw.robot)
        self.mw.canvas.plotter.render()

    def reset_panel(self):
        """Clears current picking state and UI labels."""
        self.parent_pick_data = None
        self.child_pick_data = None
        self.parent_label.setText("Parent: None selected")
        self.child_label.setText("Child: None selected")
        self.info_label.setText("Selections cleared. Pick a parent face and a child face to continue aligning.")
        # Block signals to avoid triggering apply_alignment during reset
        self.flip_check.blockSignals(True)
        self.flip_check.setChecked(False)
        self.flip_check.blockSignals(False)
        for s in self.spins.values(): s.setValue(0)
        self.temp_offset = np.eye(4)
        self.alignment_point = None
        self.alignment_normal = None
        self.original_child_transform = None
        self.undo_stack.clear()
        self.redo_stack.clear()
        self.mw.canvas.clear_highlights()
        self.mw.robot.update_kinematics()
        self.mw.canvas.update_transforms(self.mw.robot)
        self.mw.log("Align Panel Reset.")

    def undo_action(self):
        if not self.undo_stack:
            self.mw.log("Nothing to undo.")
            return
        
        # Save current for redo
        self.redo_stack.append({
            'robot': {name: link.t_offset.copy() for name, link in self.mw.robot.links.items()},
            'ui': {
                'parent_pick': self.parent_pick_data.copy() if self.parent_pick_data else None,
                'child_pick': self.child_pick_data.copy() if self.child_pick_data else None,
                'temp_offset': self.temp_offset.copy(),
                'flip': self.flip_check.isChecked(),
                'rotations': {axis: s.value() for axis, s in self.spins.items()}
            }
        })
        
        state = self.undo_stack.pop()
        self.restore_state(state)
        self.mw.log("Undo step successful.")

    def redo_action(self):
        if not self.redo_stack:
            self.mw.log("Nothing to redo.")
            return
            
        # Save current for undo
        self.undo_stack.append({
            'robot': {name: link.t_offset.copy() for name, link in self.mw.robot.links.items()},
            'ui': {
                'parent_pick': self.parent_pick_data.copy() if self.parent_pick_data else None,
                'child_pick': self.child_pick_data.copy() if self.child_pick_data else None,
                'temp_offset': self.temp_offset.copy(),
                'flip': self.flip_check.isChecked(),
                'rotations': {axis: s.value() for axis, s in self.spins.items()}
            }
        })
        
        state = self.redo_stack.pop()
        self.restore_state(state)
        self.mw.log("Redo step successful.")

    def refresh_links(self):
        pass # No longer needed with 3D picking

    def pick_parent_face(self):
        # Using red highlight as requested
        self.mw.canvas.start_face_picking(self.on_parent_face_picked, color="red")

    def on_parent_face_picked(self, name, center, normal):
        self.push_history()
        self.parent_pick_data = {"name": name, "center": center, "normal": normal}
        self.parent_label.setText(f"Parent: {name}")
        self.mw.log(f"Parent Face Picked on {name}")

    def pick_child_face(self):
        # Using red highlight as requested
        self.mw.canvas.start_face_picking(self.on_child_face_picked, color="red")

    def on_child_face_picked(self, name, center, normal):
        # --- BASE PROTECTION RULE: The Base and Jointed parts cannot move for alignment ---
        if name in self.mw.robot.links:
            link = self.mw.robot.links[name]
            if link.is_base:
                self.mw.log(f"⚠️ Locked: '{name}' is the Base and cannot be moved for alignment.")
                QtWidgets.QMessageBox.warning(self, "Locked", f"The Base component '{name}' is fixed and cannot be moved to align with another part.")
                return
            if link.parent_joint:
                self.mw.log(f"⚠️ Locked: '{name}' is jointed and cannot be moved freely for alignment.")
                QtWidgets.QMessageBox.warning(self, "Locked", f"'{name}' is already jointed. Remove the joint before re-aligning.")
                return

        self.push_history()
        self.child_pick_data = {"name": name, "center": center, "normal": normal}
        self.child_label.setText(f"Child: {name}")
        self.mw.log(f"Child Face Picked on {name}")

    def apply_alignment(self):
        if not self.parent_pick_data or not self.child_pick_data:
            self.mw.log("Error: Select BOTH parent and child faces first.")
            return
            
        self.push_history()
        child_name = self.child_pick_data['name']
        if self.parent_pick_data['name'] == child_name:
            self.mw.log("Error: Parent and Child cannot be the same object.")
            return

        p_center = self.parent_pick_data['center']
        p_normal = self.parent_pick_data['normal']
        c_center = self.child_pick_data['center']
        c_normal = self.child_pick_data['normal']

        m_align = build_alignment_transform(
            parent_center=p_center,
            parent_normal=p_normal,
            child_center=c_center,
            child_normal=c_normal,
            flip=self.flip_check.isChecked(),
        )
        
        # 3. APPLY TO CURRENT CHILD WORLD POSE
        # W_new = M_align @ W_current
        current_world = self.mw.canvas.actors[child_name].user_matrix
        self.temp_offset = m_align @ current_world
        
        self.update_preview()

    def compute_local_basis(self, normal):
        """Constructs an orthonormal basis where Z axis is aligned with 'normal'."""
        z = normal / (np.linalg.norm(normal) + 1e-9)
        
        # Pick arbitrary helper vector not parallel to Z
        if abs(z[2]) < 0.9: 
            helper = np.array([0, 0, 1])
        else:
            helper = np.array([1, 0, 0])
            
        y = np.cross(z, helper)
        y /= np.linalg.norm(y)
        
        x = np.cross(y, z)
        x /= np.linalg.norm(x)
        
        # Basis Matrix: Columns are X, Y, Z (Local vectors in Global Coords)
        basis = np.eye(4)
        basis[:3, 0] = x
        basis[:3, 1] = y
        basis[:3, 2] = z
        return basis

    def update_preview(self):
        if not self.child_pick_data or not self.parent_pick_data: return
        
        child_name = self.child_pick_data['name']
        child_link = self.mw.robot.links[child_name]
        
        # 1. Get User Input Rotations
        rx = np.radians(self.spins['Roll'].value())  # Around X (Local)
        ry = np.radians(self.spins['Pitch'].value()) # Around Y (Local)
        rz = np.radians(self.spins['Yaw'].value())   # Around Z (Local = Normal)
        
        # 2. Construct Rotation in Local Frame
        # Rz is "Clocking" (Face-to-Face spin) - Most important!
        Rx = np.array([[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]])
        Ry = np.array([[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]])
        Rz = np.array([[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]])
        local_rot_3x3 = Rz @ Ry @ Rx
        
        local_rot_4x4 = np.eye(4)
        local_rot_4x4[:3, :3] = local_rot_3x3

        # 3. Convert Local Rotation to Global Rotation
        # R_global = Basis @ R_local @ Basis_Inverse
        p_normal = self.parent_pick_data['normal']
        
        # --- IMPROVED BASIS CONSTRUCTION ---
        # Instead of arbitrary World axes, we want the basis X/Y to align 
        # with the CHILD OBJECT's geometry as much as possible.
        # This makes "Pitch" and "Roll" feel relative to the object's shape/edges.
        
        # Extract the rotation part of the current alignment (temp_offset)
        # temp_offset is the 4x4 matrix placing the child on the parent
        current_aligned_rotation = self.temp_offset[:3, :3]
        
        # Get the Child's "Local X" direction in World Space
        child_local_x = current_aligned_rotation @ np.array([1, 0, 0])
        
        # Verify it's not parallel to the Normal (Z)
        z_axis = p_normal / np.linalg.norm(p_normal)
        dot = np.abs(np.dot(child_local_x, z_axis))
        
        if dot > 0.9:
            # If Child X is roughly parallel to Normal, use Child Y
            ref_vector = current_aligned_rotation @ np.array([0, 1, 0])
        else:
            ref_vector = child_local_x
            
        # Project ref_vector onto the plane perpendicular to Normal to get "aligned X"
        # v_proj = v - (v . n) * n
        x_axis = ref_vector - np.dot(ref_vector, z_axis) * z_axis
        x_axis /= np.linalg.norm(x_axis)
        
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        
        # Construct Basis: [X_child_proj, Y_child_proj, Normal]
        basis = np.eye(4)
        basis[:3, 0] = x_axis
        basis[:3, 1] = y_axis
        basis[:3, 2] = z_axis
        
        inv_basis = np.linalg.inv(basis)
        global_rot_aligned = basis @ local_rot_4x4 @ inv_basis
        
        # 4. Apply Rotation AT THE CONTACT POINT (p_center)
        p_center = self.parent_pick_data['center']
        t_to_origin = np.eye(4); t_to_origin[:3, 3] = -p_center
        t_back = np.eye(4); t_back[:3, 3] = p_center
        
        # Combine: Move to Origin -> Rotate (Aligned) -> Move Back -> Apply Initial Snap Offset
        final_world_mat = t_back @ global_rot_aligned @ t_to_origin @ self.temp_offset
        
        # 5. Hierarchy Update: Apply to robot model so children follow
        if child_link.parent_joint:
            parent_world = child_link.parent_joint.parent_link.t_world
            inv_p = np.linalg.inv(parent_world)
            inv_j = np.linalg.inv(child_link.parent_joint.get_matrix())
            child_link.t_offset = inv_p @ final_world_mat @ inv_j
        else:
            child_link.t_offset = final_world_mat
            
        # Re-calculate EVERYTHING (whole assembly moves together)
        self.mw.robot.update_kinematics()
        self.mw.canvas.update_transforms(self.mw.robot)

    def save_alignment(self):
        if not self.child_pick_data or not self.parent_pick_data: return
        
        # Capture state for undo history
        self.push_history()
        
        child_name = self.child_pick_data['name']
        child_link = self.mw.robot.links[child_name]
        
        # 1. Capture the final position (with degrees) before we clear anything
        final_world = self.mw.canvas.actors[child_name].user_matrix
        
        # CACHE DATA LOCALLY BEFORE CLEARING
        # We need this for the Joint creation logic below!
        parent_cache = self.parent_pick_data.copy()
        child_cache = self.child_pick_data.copy()
        
        # STORE ALIGNMENT POINT FOR JOINT CREATION
        parent_center = np.array(parent_cache['center'])
        self.alignment_point = parent_center  # The actual contact point
        self.alignment_normal = np.array(parent_cache['normal'])
        
        # Store in project-wide cache for Joint Panel retrieval
        self.mw.alignment_cache[(parent_cache['name'], child_name)] = self.alignment_point.copy()
        
        self.mw.log(f"Stored contact point: {self.alignment_point}")
        
        # 2. CLEAR PICK DATA FIRST
        self.parent_pick_data = None
        self.child_pick_data = None
        self.parent_label.setText("Parent: None selected")
        self.child_label.setText("Child: None selected")
        
        # 3. Finalize the Robot Model. If no motion joint is created later,
        # the aligned child still follows the selected parent as a rigid part.
        self.ensure_rigid_alignment_joint(
            parent_cache['name'],
            child_name,
            final_world,
            contact_point=self.alignment_point,
        )
        self.mw.canvas.update_transforms(self.mw.robot)

        # 4. Lock and Log
        self.mw.log(f"ALIGNMENT LOCKED: {child_name} position updated.")
        self.mw.canvas.clear_highlights()
        
        # Proactively start Joint creation in Joint Tab
        if hasattr(self.mw, 'joint_tab'):
            self.mw.joint_tab.parent_object = parent_cache['name']
            self.mw.joint_tab.child_object = child_name
            self.mw.joint_tab.alignment_point = self.alignment_point.copy()
            
            # Switch to Joint Tab (Index 2)
            self.mw.switch_panel(1)
            
            # Trigger 'create_joint' logic
            self.mw.joint_tab.create_joint()
            
        # Now safe to reset UI
        for s in self.sliders.values(): s.setValue(0)
        self.mw.robot.update_kinematics()
        self.mw.canvas.update_transforms(self.mw.robot)

    def ensure_rigid_alignment_joint(self, parent_name, child_name, child_world_transform, contact_point=None):
        """Attach an aligned child with a fixed joint until the user creates a motion joint."""
        robot = getattr(self.mw, "robot", None)
        if robot is None:
            return None
        joint = robot.ensure_fixed_joint(
            parent_name,
            child_name,
            child_world_transform=child_world_transform,
            origin_world=contact_point,
        )
        robot.update_kinematics()
        return joint
