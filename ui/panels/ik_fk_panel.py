from PyQt5 import QtWidgets, QtCore, QtGui
import numpy as np

class KeyboardOnlyDoubleSpinBox(QtWidgets.QDoubleSpinBox):
    """SpinBox that only accepts keyboard input — no scroll wheel or step buttons."""
    def stepBy(self, steps): pass          # Disable Up/Down arrow key stepping
    def wheelEvent(self, event): event.ignore()  # Disable mouse scroll


class IKFKPanel(QtWidgets.QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.mw = main_window
        self._joint_order = []
        
        # Animation state
        self._anim_timer = QtCore.QTimer()
        self._anim_timer.timeout.connect(self._on_anim_tick)
        self._target_angles = []
        self._current_angles = []
        
        self.init_ui()

    def init_ui(self):
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(10)

        content_scroll = QtWidgets.QScrollArea()
        content_scroll.setWidgetResizable(True)
        content_scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")
        root.addWidget(content_scroll, 3)

        content = QtWidgets.QWidget()
        content_layout = QtWidgets.QVBoxLayout(content)
        content_layout.setAlignment(QtCore.Qt.AlignTop)
        content_layout.setSpacing(12)
        content_scroll.setWidget(content)

        title = QtWidgets.QLabel("IK and FK")
        title.setStyleSheet("color: #1565c0; font-size: 24px; font-weight: 700; padding: 4px 6px;")
        content_layout.addWidget(title)

        subtitle = QtWidgets.QLabel("Inverse Kinematics and Forward Kinematics using the Standard DH convention as specified in Lab Manual BE04041011 (L.D. College of Engineering).")
        subtitle.setWordWrap(True)
        subtitle.setStyleSheet("color: #546e7a; font-size: 14px; font-weight: 500; line-height: 1.4; padding: 0 6px 8px 6px;")
        content_layout.addWidget(subtitle)


        self.ik_group = QtWidgets.QFrame()
        self.ik_group.setStyleSheet(
            "QFrame { background: #ffffff; border: 1px solid #dbe6ee; border-radius: 10px; }"
        )
        ik_layout = QtWidgets.QVBoxLayout(self.ik_group)
        ik_layout.setContentsMargins(12, 12, 12, 12)
        ik_layout.setSpacing(10)

        ik_header = QtWidgets.QLabel("Inverse Kinematics")
        ik_header.setStyleSheet("color: #1976d2; font-size: 19px; font-weight: bold;")
        ik_layout.addWidget(ik_header)

        target_row = QtWidgets.QHBoxLayout()
        self.ik_x = self._make_num_input(-99999, 99999, 0.0, 3)
        self.ik_y = self._make_num_input(-99999, 99999, 0.0, 3)
        self.ik_z = self._make_num_input(-99999, 99999, 0.0, 3)
        target_row.addWidget(self._labeled_widget("Target X", self.ik_x))
        target_row.addWidget(self._labeled_widget("Target Y", self.ik_y))
        target_row.addWidget(self._labeled_widget("Target Z", self.ik_z))
        ik_layout.addLayout(target_row)

        solver_row = QtWidgets.QHBoxLayout()
        self.solve_ik_btn = QtWidgets.QPushButton("Solve IK")
        self.solve_ik_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.solve_ik_btn.setStyleSheet(self._primary_btn_style())
        self.solve_ik_btn.clicked.connect(self.solve_ik)
        solver_row.addStretch()
        solver_row.addWidget(self.solve_ik_btn)

        self.solve_smart_btn = QtWidgets.QPushButton("🚀 Smart Solve (CCD)")
        self.solve_smart_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.solve_smart_btn.setStyleSheet("""
            QPushButton {
                background-color: #43a047;
                color: white;
                border: none;
                border-radius: 8px;
                padding: 8px 14px;
                font-size: 14px;
                font-weight: 700;
            }
            QPushButton:hover { background-color: #2e7d32; }
        """)
        self.solve_smart_btn.clicked.connect(self.solve_ik_smart)
        solver_row.addWidget(self.solve_smart_btn)
        ik_layout.addLayout(solver_row)

        content_layout.addWidget(self.ik_group)

        self.fk_group = QtWidgets.QFrame()
        self.fk_group.setStyleSheet(
            "QFrame { background: #ffffff; border: 1px solid #dbe6ee; border-radius: 10px; }"
        )
        fk_layout = QtWidgets.QVBoxLayout(self.fk_group)
        fk_layout.setContentsMargins(12, 12, 12, 12)
        fk_layout.setSpacing(10)

        fk_header = QtWidgets.QLabel("Forward Kinematics (Standard DH)")
        fk_header.setStyleSheet("color: #1976d2; font-size: 19px; font-weight: bold;")
        fk_layout.addWidget(fk_header)

        tool_row = QtWidgets.QHBoxLayout()
        
        self.run_fk_btn = QtWidgets.QPushButton("🔄 Compute FK")
        self.run_fk_btn.setStyleSheet(self._primary_btn_style())
        self.run_fk_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.run_fk_btn.clicked.connect(self.update_display)
        tool_row.addWidget(self.run_fk_btn)

        self.clear_dh_btn = QtWidgets.QPushButton("🗑️ Clear")
        self.clear_dh_btn.setStyleSheet(self._ghost_btn_style())
        self.clear_dh_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.clear_dh_btn.clicked.connect(self.clear_dh_table)
        tool_row.addWidget(self.clear_dh_btn)

        tool_row.addStretch()
        fk_layout.addLayout(tool_row)

        self.dh_table = QtWidgets.QTableWidget(0, 2)
        self.dh_table.setHorizontalHeaderLabels(["Joint", "theta (deg)"])
        self.dh_table.verticalHeader().setVisible(False)
        self.dh_table.setAlternatingRowColors(True)
        self.dh_table.setStyleSheet(
            """
            QTableWidget {
                background: #ffffff;
                color: #102a43;
                border: 1px solid #dbe6ee;
                border-radius: 8px;
                font-size: 14px;
                gridline-color: #e3edf5;
            }
            QHeaderView::section {
                background: #f0f6fb;
                color: #1e3a5f;
                font-size: 14px;
                font-weight: 700;
                border: none;
                border-bottom: 1px solid #dbe6ee;
                padding: 8px;
            }
            """
        )
        self.dh_table.horizontalHeader().setStretchLastSection(False)
        self.dh_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        fk_layout.addWidget(self.dh_table)

        self.result_view = QtWidgets.QTextEdit()
        self.result_view.setReadOnly(True)
        self.result_view.setMinimumHeight(220)
        self.result_view.setStyleSheet(
            """
            QTextEdit {
                background: #ffffff;
                color: #0f172a;
                border: 1px solid #dbe6ee;
                border-radius: 8px;
                font-size: 14px;
                padding: 10px;
            }
            """
        )
        fk_layout.addWidget(self.result_view)

        dh_mats_row = QtWidgets.QHBoxLayout()
        dh_mats_label = QtWidgets.QLabel("DH Matrices for Joints")
        dh_mats_label.setStyleSheet("color: #1976d2; font-size: 17px; font-weight: 700;")
        dh_mats_row.addWidget(dh_mats_label)
        dh_mats_row.addStretch()

        self.compute_dh_mats_btn = QtWidgets.QPushButton("Compute DH Matrices")
        self.compute_dh_mats_btn.setStyleSheet(self._ghost_btn_style())
        self.compute_dh_mats_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.compute_dh_mats_btn.clicked.connect(self.compute_dh_matrices)
        dh_mats_row.addWidget(self.compute_dh_mats_btn)
        fk_layout.addLayout(dh_mats_row)

        self.dh_matrices_view = QtWidgets.QTextEdit()
        self.dh_matrices_view.setReadOnly(True)
        self.dh_matrices_view.setMinimumHeight(220)
        self.dh_matrices_view.setStyleSheet(
            """
            QTextEdit {
                background: #ffffff;
                color: #0f172a;
                border: 1px solid #dbe6ee;
                border-radius: 8px;
                font-size: 14px;
                padding: 10px;
            }
            """
        )
        fk_layout.addWidget(self.dh_matrices_view)

        content_layout.addWidget(self.fk_group)

        self.rebuild_dh_table()
        self.update_display()
        self.compute_dh_matrices()

    def _labeled_widget(self, text, widget):
        frame = QtWidgets.QFrame()
        frame.setStyleSheet("QFrame { background: transparent; border: none; }")
        lay = QtWidgets.QVBoxLayout(frame)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(4)
        lbl = QtWidgets.QLabel(text)
        lbl.setStyleSheet("color: #455a64; font-size: 14px; font-weight: 600;")
        lay.addWidget(lbl)
        lay.addWidget(widget)
        return frame

    def _make_num_input(self, lo, hi, val, decimals):
        spin = KeyboardOnlyDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setDecimals(decimals)
        spin.setValue(val)
        spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        spin.setStyleSheet(self._spin_style())
        return spin

    def _spin_style(self):
        return (
            "QDoubleSpinBox, QSpinBox {"
            " background: #ffffff;"
            " color: #1565c0;"
            " border: 1px solid #90caf9;"
            " border-radius: 6px;"
            " padding: 6px 8px;"
            " font-size: 15px;"
            " font-weight: 700;"
            "}"
        )

    def _primary_btn_style(self):
        return (
            "QPushButton {"
            " background-color: #1976d2;"
            " color: white;"
            " border: none;"
            " border-radius: 8px;"
            " padding: 8px 14px;"
            " font-size: 14px;"
            " font-weight: 700;"
            "}"
            "QPushButton:hover { background-color: #1565c0; }"
        )

    def _ghost_btn_style(self):
        return (
            "QPushButton {"
            " background-color: #ffffff;"
            " color: #1976d2;"
            " border: 1px solid #90caf9;"
            " border-radius: 8px;"
            " padding: 8px 14px;"
            " font-size: 14px;"
            " font-weight: 700;"
            "}"
            "QPushButton:hover { background-color: #e3f2fd; }"
        )

    def _active_joint_child_names(self):
        joint_tab = getattr(self.mw, "joint_tab", None)
        if joint_tab is None:
            return []

        joint_data = getattr(joint_tab, "joints", {})
        result = []
        for child_name, data in joint_data.items():
            joint_id = data.get("joint_id", child_name)
            is_slave = False
            for _, slaves in self.mw.robot.joint_relations.items():
                if any(s_id == joint_id for s_id, _ in slaves):
                    is_slave = True
                    break
            if not is_slave:
                result.append(child_name)
        return result

    def rebuild_dh_table(self):
        joint_tab = getattr(self.mw, "joint_tab", None)
        joint_data = getattr(joint_tab, "joints", {}) if joint_tab is not None else {}

        prev = {}
        for r in range(self.dh_table.rowCount()):
            j_name = self.dh_table.item(r, 0).text() if self.dh_table.item(r, 0) else f"J{r+1}"
            prev[j_name] = self._table_num(r, 1, 0.0)

        self._joint_order = self._active_joint_child_names()
        if not self._joint_order:
            self._joint_order = [f"J{i+1}" for i in range(max(1, self.dh_table.rowCount()))]

        self.dh_table.setRowCount(len(self._joint_order))
        for idx, child_name in enumerate(self._joint_order):
            display = child_name
            if child_name in joint_data:
                display = joint_data[child_name].get("custom_name", child_name)

            name_item = QtWidgets.QTableWidgetItem(display)
            name_item.setFlags(name_item.flags() & ~QtCore.Qt.ItemIsEditable)
            self.dh_table.setItem(idx, 0, name_item)

            theta = prev.get(display, 0.0)
            self.dh_table.setItem(idx, 1, QtWidgets.QTableWidgetItem(f"{theta:.4f}"))

        self.load_joint_angles_into_dh()

    def _table_num(self, row, col, default):
        item = self.dh_table.item(row, col)
        if not item:
            return default
        try:
            return float(item.text())
        except Exception:
            return default

    def load_joint_angles_into_dh(self):
        joint_tab = getattr(self.mw, "joint_tab", None)
        if joint_tab is None:
            return

        if not self._joint_order:
            return

        for idx, child_name in enumerate(self._joint_order):
            if child_name not in joint_tab.joints:
                continue
            theta = joint_tab.joints[child_name].get("current_angle", 0.0)
            self.dh_table.setItem(idx, 1, QtWidgets.QTableWidgetItem(f"{theta:.4f}"))

    def _dh_rows(self):
        rows = []
        for r in range(self.dh_table.rowCount()):
            theta_deg = self._table_num(r, 1, 0.0)
            rows.append(theta_deg)
        return rows

    def _a_matrix(self, theta_rad, d, a, alpha_rad):
        """
        Transformation matrix A_i from frame i-1 to frame i.
        Basis: BE04041011 Lab Manual (Standard DH convention).
        Formula: A_i = RotZ(theta_i) * TransZ(d_i) * TransX(a_i-1) * RotX(alpha_i-1)
        
        Resulting Matrix Structure (Perfected as per PDF page 34):
        [ cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a*cos(θ) ]
        [ sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a*sin(θ) ]
        [   0        sin(α)          cos(α)         d     ]
        [   0          0              0             1     ]
        """
        ct, st = np.cos(theta_rad), np.sin(theta_rad)
        ca, sa = np.cos(alpha_rad), np.sin(alpha_rad)
        return np.array(
            [
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0.0, sa, ca, d],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

    def _fk_with_thetas_rad(self, thetas_rad):
        T = np.eye(4)
        chain = []
        for theta in thetas_rad:
            A = self._a_matrix(theta, 0.0, 0.0, 0.0)
            T = T @ A
            chain.append(T.copy())
        return T, chain

    def update_display(self):
        rows = self._dh_rows()
        if not rows:
            self.result_view.setHtml("<p style='color:#78909c;'>No DH rows configured.</p>")
            self.compute_dh_matrices()
            return

        theta_rad = np.array([np.radians(theta) for theta in rows], dtype=float)
        T, chain = self._fk_with_thetas_rad(theta_rad)

        html = """
        <style>
            .box { border: 1px solid #dbe6ee; border-radius: 10px; margin: 10px 0; overflow: hidden; }
            .head { background: #1976d2; color: #fff; font-weight: 700; font-size: 16px; padding: 10px 12px; }
            .matrix { width: 100%; border-collapse: collapse; }
            .matrix td, .matrix th { text-align: center; padding: 8px; font-family: Consolas, monospace; font-size: 14px; }
            .matrix th { background: #f2f7fb; color: #334e68; }
            .matrix tr:nth-child(even) td { background: #fafcfe; }
        </style>
        """

        for i, Ti in enumerate(chain):
            html += f"<div class='box'><div class='head'>T0{i+1}</div>{self._matrix_html(Ti)}</div>"

        html += f"<div class='box'><div class='head'>End-Effector Pose</div>{self._matrix_html(T)}</div>"
        p = T[:3, 3]
        html += (
            f"<p style='font-size:15px;color:#0f172a;'><b>Position:</b> "
            f"X={p[0]:.4f}, Y={p[1]:.4f}, Z={p[2]:.4f}</p>"
        )

        self.result_view.setHtml(html)
        self.compute_dh_matrices()

    def _matrix_html(self, mat):
        labels = ["X", "Y", "Z", "T"]
        s = "<table class='matrix'><tr>"
        for lbl in labels:
            s += f"<th>{lbl}</th>"
        s += "</tr>"
        for row in mat:
            s += "<tr>"
            for val in row:
                s += f"<td>{val: .5f}</td>"
            s += "</tr>"
        s += "</table>"
        return s

    def compute_dh_matrices(self):
        rows = self._dh_rows()
        if not rows:
            self.dh_matrices_view.setHtml("<p style='color:#78909c;'>No joint DH rows configured.</p>")
            return

        html = """
        <style>
            .box { border: 1px solid #dbe6ee; border-radius: 10px; margin: 10px 0; overflow: hidden; }
            .head { background: #1976d2; color: #fff; font-weight: 700; font-size: 16px; padding: 10px 12px; }
            .matrix { width: 100%; border-collapse: collapse; }
            .matrix td, .matrix th { text-align: center; padding: 8px; font-family: Consolas, monospace; font-size: 14px; }
            .matrix th { background: #f2f7fb; color: #334e68; }
            .matrix tr:nth-child(even) td { background: #fafcfe; }
        </style>
        """

        for idx, theta_deg in enumerate(rows):
            theta_rad = np.radians(theta_deg)
            Ai = self._a_matrix(theta_rad, 0.0, 0.0, 0.0)

            joint_name = f"J{idx + 1}"
            if idx < self.dh_table.rowCount() and self.dh_table.item(idx, 0):
                joint_name = self.dh_table.item(idx, 0).text()

            html += f"<div class='box'><div class='head'>A{idx+1} ({joint_name})</div>{self._matrix_html(Ai)}</div>"

        self.dh_matrices_view.setHtml(html)

    def solve_ik(self):
        """Solves IK using the Jacobian-based numerical method (DH-centric)."""
        rows = self._dh_rows()
        n = len(rows)
        if n == 0:
            QtWidgets.QMessageBox.warning(self, "IK", "Please configure DH rows first.")
            return

        target = np.array([self.ik_x.value(), self.ik_y.value(), self.ik_z.value()], dtype=float)
        tol = 0.1
        max_iters = 350

        theta = np.array([np.radians(v) for v in rows], dtype=float)
        lam = 1e-2
        eps = 1e-5

        def pos_of(th):
            joint_tab = getattr(self.mw, "joint_tab", None)
            if not joint_tab or not joint_tab.joints:
                T, _ = self._fk_with_thetas_rad(th)
                return T[:3, 3]
                
            old_angles = {n: j.current_value for n, j in self.mw.robot.joints.items()}
            
            for i, child_name in enumerate(self._joint_order):
                if child_name in self.mw.robot.joints:
                    val_deg = np.degrees(th[i])
                    self.mw.robot.joints[child_name].current_value = val_deg
                    if child_name in self.mw.robot.joint_relations:
                        for s_id, ratio in self.mw.robot.joint_relations[child_name]:
                            if s_id in self.mw.robot.joints:
                                self.mw.robot.joints[s_id].current_value = val_deg * ratio

            self.mw.robot.update_kinematics()
            
            links = list(self.mw.robot.links.values())
            if not links:
                pos = np.zeros(3)
            else:
                pos = links[-1].t_world[:3, 3]
                
            for n, val in old_angles.items():
                self.mw.robot.joints[n].current_value = val
            self.mw.robot.update_kinematics()
            
            return pos

        converged = False
        for _ in range(max_iters):
            p = pos_of(theta)
            err = target - p
            if np.linalg.norm(err) <= tol:
                converged = True
                break

            J = np.zeros((3, n), dtype=float)
            for i in range(n):
                tp = theta.copy(); tm = theta.copy()
                tp[i] += eps; tm[i] -= eps
                J[:, i] = (pos_of(tp) - pos_of(tm)) / (2.0 * eps)

            jj_t = J @ J.T
            step = J.T @ np.linalg.solve(jj_t + (lam * lam) * np.eye(3), err)
            step = np.clip(step, -0.2, 0.2)
            theta += step

        theta_deg = np.degrees(theta)
        self._start_ik_animation(theta_deg)
        
        status = "converged" if converged else "best effort"
        self.mw.log(f"DH IK: Solve {status} (Target: {target}).")

    def solve_ik_smart(self):
        """Solves IK using the Robot core's robust CCD solver (3D model-centric)."""
        joint_tab = getattr(self.mw, "joint_tab", None)
        if not joint_tab or not joint_tab.joints:
             QtWidgets.QMessageBox.warning(self, "IK", "No joints created in the robot model yet.")
             return

        # Find TCP link (usually the last child in the model)
        links = list(self.mw.robot.links.values())
        if not links: return
        tcp_link = links[-1] 
        
        target = [self.ik_x.value(), self.ik_y.value(), self.ik_z.value()]
        
        # Snapshot current angles
        old_angles = {n: j.current_value for n, j in self.mw.robot.joints.items()}
        
        # Use robust CCD solver
        success = self.mw.robot.inverse_kinematics(target, tcp_link, max_iters=300, tolerance=0.5)
        
        if success:
            new_angles_deg = [self.mw.robot.joints[jn].current_value for jn in self._joint_order]
            # Reset to animate properly
            for n, val in old_angles.items():
                self.mw.robot.joints[n].current_value = val
            self.mw.robot.update_kinematics()
            
            self._start_ik_animation(new_angles_deg)
            self.mw.log(f"Smart IK: Target {target} reached.")
        else:
            self.mw.log(f"Smart IK: Failed to reach target {target}.")
            QtWidgets.QMessageBox.information(self, "IK", "Target might be out of reachable workspace.")

    def _start_ik_animation(self, target_deg_list):
        self._target_angles = list(target_deg_list)
        self._current_angles = []
        for i, child_name in enumerate(self._joint_order):
            joint_tab = getattr(self.mw, "joint_tab", None)
            curr = 0.0
            if joint_tab and child_name in joint_tab.joints:
                curr = joint_tab.joints[child_name].get("current_angle", 0.0)
            self._current_angles.append(curr)
        
        self._anim_timer.start(30)

    def _on_anim_tick(self):
        done = True
        step_size = 2.0 # degrees per tick
        
        for i, child_name in enumerate(self._joint_order):
            curr = self._current_angles[i]
            target = self._target_angles[i]
            
            if abs(target - curr) < step_size:
                self._current_angles[i] = target
            else:
                self._current_angles[i] += np.sign(target - curr) * step_size
                done = False
                
            self.apply_rotation(child_name, self._current_angles[i])
            self.sync_slider(child_name, self._current_angles[i])

        if done:
            self._anim_timer.stop()
            self.update_display()
            self.mw.show_toast("Target Reached", "success")

    def refresh_sliders(self):
        # Joint rotation controls were removed from this panel by UI request.
        # Keep this method for compatibility with existing callers.
        return

    def on_slider_move(self, child_name, value, spinbox):
        spinbox.blockSignals(True)
        spinbox.setValue(value)
        spinbox.blockSignals(False)
        self.apply_rotation(child_name, value)

    def on_spin_move(self, child_name, value, slider):
        slider.blockSignals(True)
        slider.setValue(int(value * 10))
        slider.blockSignals(False)
        self.apply_rotation(child_name, value)

    def apply_rotation(self, child_name, angle):
        joint_tab = getattr(self.mw, "joint_tab", None)
        if joint_tab is None or child_name not in joint_tab.joints:
            return
        joint_tab.apply_joint_rotation(child_name, angle)
        if child_name in self._joint_order:
            row = self._joint_order.index(child_name)
            self.dh_table.setItem(row, 1, QtWidgets.QTableWidgetItem(f"{angle:.4f}"))
        self.update_display()

    def sync_slider(self, child_name, value):
        if child_name in self._joint_order:
            row = self._joint_order.index(child_name)
            self.dh_table.setItem(row, 1, QtWidgets.QTableWidgetItem(f"{value:.4f}"))
        self.update_display()
    def clear_dh_table(self):
        for r in range(self.dh_table.rowCount()):
            for c in range(1, 5):
                self.dh_table.setItem(r, c, QtWidgets.QTableWidgetItem("0.0000"))
        self.update_display()

