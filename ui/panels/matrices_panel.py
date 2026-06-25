from PyQt5 import QtWidgets, QtGui, QtCore
import numpy as np

from core.torotron_dh import resolve_torotron_dh, compute_forward_kinematics, compute_joint_matrix

class MatricesPanel(QtWidgets.QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.mw = main_window
        self.sliders = {} # Store slider widgets for each joint
        self.init_ui()

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        
        # Top: Matrix Display
        header_matrices = QtWidgets.QLabel("TRANSFORM MATRICES")
        header_matrices.setStyleSheet("color: #1976d2; font-size: 14px; font-weight: bold; padding: 5px;")
        layout.addWidget(header_matrices)

        self.refresh_btn = QtWidgets.QPushButton("Update Matrices")
        self.refresh_btn.clicked.connect(self.update_display)
        layout.addWidget(self.refresh_btn)
        
        self.text_area = QtWidgets.QTextEdit()
        self.text_area.setReadOnly(True)
        self.text_area.setFont(QtGui.QFont("Consolas", 10))
        self.text_area.setStyleSheet("background-color: white; color: #1565c0; border: 1px solid #e0e0e0;")
        layout.addWidget(self.text_area)

        # Bottom: Joint Control Sliders
        header_sliders = QtWidgets.QLabel("Joint Rotation Controls")
        header_sliders.setStyleSheet("color: #1976d2; font-size: 15px; font-weight: bold; margin-top: 15px; padding: 5px;")
        layout.addWidget(header_sliders)

        self.scroll_area = QtWidgets.QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setStyleSheet("background-color: white; border: none;")
        
        self.slider_container = QtWidgets.QWidget()
        self.slider_layout = QtWidgets.QVBoxLayout(self.slider_container)
        self.slider_layout.setAlignment(QtCore.Qt.AlignTop)
        self.scroll_area.setWidget(self.slider_container)
        
        layout.addWidget(self.scroll_area)

    def refresh_sliders(self):
        """Clears and rebuilds sliders based on confirmed joints"""
        # Clear existing
        while self.slider_layout.count():
            item = self.slider_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        
        self.sliders = {}
        
        joint_data = self.mw.joint_tab.joints
        if not joint_data:
            empty_msg = QtWidgets.QLabel("No joints created yet.")
            empty_msg.setStyleSheet("color: #9e9e9e; font-style: italic; padding: 10px;")
            self.slider_layout.addWidget(empty_msg)
            return

        for child_name, data in joint_data.items():
            # Hide slave joints - their movement is driven by the master
            joint_id = data.get('joint_id', child_name)
            is_slave = False
            for master, slaves in self.mw.robot.joint_relations.items():
                if any(s_id == joint_id for s_id, r in slaves):
                    is_slave = True
                    break
            
            if is_slave:
                continue

            # Container for each joint's control
            group = QtWidgets.QFrame()
            group.setStyleSheet("background-color: transparent; border-radius: 5px; margin-bottom: 5px;")
            glay = QtWidgets.QVBoxLayout(group)
            glay.setContentsMargins(10, 8, 10, 8)
            
            # Label: Custom Name Only
            custom_name = data.get('custom_name', f"{data['parent']} \u2192 {child_name}")
            lbl = QtWidgets.QLabel(f"{custom_name} ({['X','Y','Z'][data['axis']]})")
            lbl.setStyleSheet("color: #1976d2; font-weight: bold; font-size: 13px;")
            glay.addWidget(lbl)
            
            # Slider Row
            row = QtWidgets.QHBoxLayout()
            
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(int(data['min'] * 10), int(data['max'] * 10))
            slider.setValue(int(data.get('current_angle', 0.0) * 10))
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
            
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(data['min'], data['max'])
            spin.setValue(data.get('current_angle', 0.0))
            spin.setFixedWidth(70)
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
            
            # Connect
            slider.valueChanged.connect(lambda v, c=child_name, s=spin: self.on_slider_move(c, v/10.0, s))
            spin.valueChanged.connect(lambda v, c=child_name, sl=slider: self.on_spin_move(c, v, sl))
            
            row.addWidget(slider)
            row.addWidget(spin)
            glay.addLayout(row)
            
            self.slider_layout.addWidget(group)
            self.sliders[child_name] = {'slider': slider, 'spin': spin}

    def on_slider_move(self, child_name, value, spinbox):
        spinbox.blockSignals(True)
        spinbox.setValue(value)
        spinbox.blockSignals(False)
        self.apply_rotation(child_name, value)
        
        # Sync the Joint Panel slider if it's currently showing this joint
        if hasattr(self.mw.joint_tab, 'active_joint_control') and self.mw.joint_tab.active_joint_control == child_name:
            self.mw.joint_tab.joint_control_slider.blockSignals(True)
            self.mw.joint_tab.joint_control_slider.setValue(int(value * 10))
            self.mw.joint_tab.joint_control_slider.blockSignals(False)
            self.mw.joint_tab.joint_control_spinbox.blockSignals(True)
            self.mw.joint_tab.joint_control_spinbox.setValue(value)
            self.mw.joint_tab.joint_control_spinbox.blockSignals(False)

    def sync_slider(self, child_name, value):
        """External call to update a slider value without triggering events"""
        if child_name in self.sliders:
            data = self.sliders[child_name]
            data['slider'].blockSignals(True)
            data['slider'].setValue(int(value * 10))
            data['slider'].blockSignals(False)
            data['spin'].blockSignals(True)
            data['spin'].setValue(value)
            data['spin'].blockSignals(False)
        # Always refresh matrices, including updates coming from slave/coupled joints.
        self.update_display()

    def _is_slave_joint(self, joint_id):
        for _, slaves in self.mw.robot.joint_relations.items():
            if any(s_id == joint_id for s_id, _ in slaves):
                return True
        return False

    def _display_value(self, value):
        # Clamp tiny floating artifacts such as 6.123e-17 for cleaner UI output.
        return 0.0 if abs(value) < 1e-9 else value

    def _clean_display_matrix(self, mat):
        out = np.array(mat, dtype=float, copy=True)
        out[np.abs(out) < 1e-9] = 0.0
        out[3, :] = np.array([0.0, 0.0, 0.0, 1.0])
        return out

    def _validate_homogeneous_matrix(self, mat):
        R = mat[:3, :3]
        is_orthonormal = np.allclose(R.T @ R, np.eye(3), atol=1e-6)
        det_r = np.linalg.det(R)
        has_valid_det = abs(det_r - 1.0) <= 1e-6
        valid_last_row = np.allclose(mat[3, :], np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-9)
        return is_orthonormal and has_valid_det and valid_last_row, det_r

    def computeJointMatrix(self, theta_deg, d, a, alpha_deg, joint_type="revolute", q_value=0.0):
        return compute_joint_matrix(theta_deg, d, a, alpha_deg, joint_type, q_value)

    def computeForwardKinematics(self, dh_rows):
        return compute_forward_kinematics(dh_rows)

    def _joint_meta_by_id(self):
        meta = {}
        created_joints = getattr(self.mw.joint_tab, 'joints', {})
        for child_name, data in created_joints.items():
            joint_id = data.get('joint_id', child_name)
            meta[joint_id] = data
        return meta

    def _ordered_robot_joints(self, robot):
        roots = [link for link in robot.links.values() if link.parent_joint is None]
        if robot.base_link and robot.base_link in roots:
            roots.remove(robot.base_link)
            roots.insert(0, robot.base_link)

        ordered = []
        visited_links = set()
        queue = list(roots)
        while queue:
            parent = queue.pop(0)
            if parent.name in visited_links:
                continue
            visited_links.add(parent.name)

            for joint in parent.child_joints:
                ordered.append(joint)
                queue.append(joint.child_link)
        return ordered

    def on_spin_move(self, child_name, value, slider):
        slider.blockSignals(True)
        slider.setValue(int(value * 10))
        slider.blockSignals(False)
        self.apply_rotation(child_name, value)

    def apply_rotation(self, child_name, angle):
        """Apply rotation using the JointPanel's unified logic"""
        if child_name not in self.mw.joint_tab.joints:
            return
            
        # Call the JointPanel's logic to handle the actual 3D rotation
        # This ensures the object rotates exactly the same way as in the Joint tab
        self.mw.joint_tab.apply_joint_rotation(child_name, angle)
        
        # Refresh matrix display
        self.update_display()

    def update_display(self):
        self.text_area.clear()
        robot = self.mw.robot
        if not robot.joints:
            self.text_area.setHtml("<p style='color:#9e9e9e; font-style:italic; padding: 20px;'>No active joints created yet.</p>")
            return

        joint_meta = self._joint_meta_by_id()
        ordered_joints = self._ordered_robot_joints(robot)

        dh_rows = []
        ratio = getattr(self.mw.canvas, "grid_units_per_cm", 1.0)
        for idx, joint in enumerate(ordered_joints, start=1):
            joint_id = joint.name
            meta = joint_meta.get(joint_id, {})
            inferred = resolve_torotron_dh(joint, meta, ratio)

            current_value = float(meta.get("current_angle", joint.current_value))
            q_value = current_value if inferred["joint_type"] != "prismatic" else float(meta.get("current_offset", 0.0))

            custom_name = meta.get("custom_name", joint_id)
            title = f"J{idx} : {custom_name}"
            if idx == 1:
                title += " (Base)"

            dh_rows.append({
                "joint_name": joint_id,
                "title": title,
                "joint_type": inferred["joint_type"],
                "theta0_deg": inferred["theta0_deg"],
                "d": inferred["d"],
                "a": inferred["a"],
                "alpha_deg": inferred["alpha_deg"],
                "q_value": q_value,
            })

        fk_results = self.computeForwardKinematics(dh_rows)

        html = """
        <style>
            .container { padding: 15px; background-color: #ffffff; }

            /* ── Per-joint Cumulative T0 Matrix ── */
            .matrix-box {
                border: 1px solid #e2e8f0;
                border-radius: 12px;
                margin-bottom: 24px;
                overflow: hidden;
                box-shadow: 0 4px 6px -1px rgb(0 0 0 / 0.07), 0 2px 4px -2px rgb(0 0 0 / 0.07);
            }
            .box-header {
                background-color: #2563eb;
                color: #ffffff;
                font-family: 'Segoe UI', sans-serif;
                font-size: 16px;
                font-weight: 700;
                padding: 11px 18px;
                letter-spacing: 1px;
            }
            .sub-head {
                padding: 8px 12px;
                background-color: #f8fafc;
                color: #334155;
                font-family: 'Segoe UI', sans-serif;
                font-size: 12px;
                font-weight: 700;
                border-top: 1px solid #e2e8f0;
                border-bottom: 1px solid #e2e8f0;
            }
            .matrix-grid {
                border-collapse: collapse;
                width: 100%;
            }
            .matrix-grid th {
                background-color: #f1f5f9;
                color: #475569;
                font-family: 'Segoe UI', sans-serif;
                font-size: 14px;
                font-weight: 800;
                text-transform: uppercase;
                padding: 10px 8px;
                text-align: center;
                border-bottom: 2px solid #e2e8f0;
                letter-spacing: 1.2px;
            }
            .matrix-grid td {
                background-color: #ffffff;
                border: none;
                padding: 11px 8px;
                text-align: center;
                font-family: 'Consolas', monospace;
                font-size: 16px;
                color: #0f172a;
                font-weight: 600;
            }
            .matrix-grid tr:nth-child(even) td { background-color: #f8fafc; }
            .matrix-grid td.t-col { color: #2563eb; font-weight: 800; }
            .warn {
                color: #b91c1c;
                font-size: 12px;
                font-weight: 700;
                padding: 8px 12px;
                background: #fef2f2;
                border-top: 1px solid #fee2e2;
            }
        </style>
        <div class="container">
        """

        # ── Per-joint Cumulative T0 Transform Matrices ──
        for idx, result in enumerate(fk_results, start=1):
            cum_ok, cum_det = self._validate_homogeneous_matrix(result["cumulative"])

            # Pivot Point: read directly from the 3D engine's world-space joint origin.
            joint_obj = ordered_joints[idx - 1]
            origin_world = (joint_obj.parent_link.t_world @ np.append(
                np.array(joint_obj.origin, dtype=float), 1.0))[:3]
            px = origin_world[0] / ratio
            py = origin_world[1] / ratio
            pz = origin_world[2] / ratio

            html += f'<div class="matrix-box">'
            html += f'  <div class="box-header">{result["title"]}</div>'
            html += (
                f"<div class='sub-head'>Cumulative T0{idx}"
                f"<span style='font-weight:normal; color:#64748b; font-size:11px; margin-left:10px;'>"
                f"Pivot Point: ({px:.1f}, {py:.1f}, {pz:.1f}) cm"
                f"</span></div>"
            )
            html += self.format_matrix_html(result["cumulative"])

            if not cum_ok:
                html += (
                    f"<div class='warn'>Validation warning: "
                    f"det(T0{idx}.R)={cum_det:.6f}</div>"
                )
            html += '</div>'

        html += "</div>"
        self.text_area.setHtml(html)

        # Sync with Result Tab (Summary View)
        if hasattr(self.mw, 'experiment_tab') and hasattr(self.mw.experiment_tab, 'result_tab'):
            self.mw.experiment_tab.result_tab.update_display(fk_results)

    def format_matrix_html(self, mat):
        mat_display = self._clean_display_matrix(mat)
        
        col_labels = ['X', 'Y', 'Z', 'T']
        
        table = '<table class="matrix-grid">'
        table += '<tr>'
        for lbl in col_labels:
            table += f'<th>{lbl}</th>'
        table += '</tr>'
        for row in mat_display:
            table += '<tr>'
            for c_idx, val in enumerate(row):
                cls = ' class="t-col"' if c_idx == 3 else ''
                table += f'<td{cls}>{val:8.1f}</td>'
            table += '</tr>'
        table += '</table>'
        return table
