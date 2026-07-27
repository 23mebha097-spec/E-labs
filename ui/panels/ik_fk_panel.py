from PyQt5 import QtWidgets, QtCore
import numpy as np

class KeyboardOnlyDoubleSpinBox(QtWidgets.QDoubleSpinBox):
    def stepBy(self, steps):
        pass

    def wheelEvent(self, event):
        event.ignore()


class IKFKPanel(QtWidgets.QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.mw = main_window
        self._fk_rows = []
        self._fk_target_by_joint = {}
        self._anim_timer = self.mw._anim_timer
        self.init_ui()

    def init_ui(self):
        main_root = QtWidgets.QVBoxLayout(self)
        main_root.setContentsMargins(0, 0, 0, 0)
        
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")
        main_root.addWidget(scroll)

        content = QtWidgets.QWidget()
        content.setStyleSheet("background-color: transparent;")
        root = QtWidgets.QVBoxLayout(content)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(20)
        scroll.setWidget(content)

        # --- SECTION: INVERSE KINEMATICS ---
        ik_section = QtWidgets.QVBoxLayout()
        ik_section.setSpacing(15)
        root.addLayout(ik_section)

        # Main horizontal container for IK Controls and Info
        ik_main_h = QtWidgets.QHBoxLayout()
        ik_main_h.setSpacing(15)
        ik_section.addLayout(ik_main_h)

        # Left Column: IK Controls
        ik_controls = QtWidgets.QVBoxLayout()
        ik_controls.setSpacing(12)
        ik_main_h.addLayout(ik_controls)
        ik_main_h.addStretch()

        ik_title = QtWidgets.QLabel("Inverse Kinematics")
        ik_title.setStyleSheet("color: #1976d2; font-size: 20px; font-weight: bold;")
        ik_controls.addWidget(ik_title)

        # IK Input Group (x=, y=, z=)
        input_container = QtWidgets.QFrame()
        input_container.setStyleSheet("""
            QFrame {
                background-color: white;
                border: 2px solid #e0e0e0;
                border-radius: 12px;
            }
        """)
        input_layout = QtWidgets.QHBoxLayout(input_container)
        input_layout.setContentsMargins(5, 5, 5, 5)
        input_layout.setSpacing(0)

        self.ik_x = self._make_num_input_minimal(-99999, 99999, 0.0)
        self.ik_y = self._make_num_input_minimal(-99999, 99999, 0.0)
        self.ik_z = self._make_num_input_minimal(-99999, 99999, 0.0)

        input_layout.addWidget(self._labeled_widget_minimal("x=", self.ik_x))
        input_layout.addWidget(self._v_line())
        input_layout.addWidget(self._labeled_widget_minimal("y=", self.ik_y))
        input_layout.addWidget(self._v_line())
        input_layout.addWidget(self._labeled_widget_minimal("z=", self.ik_z))
        ik_controls.addWidget(input_container)

        # Solve IK Button
        self.solve_ik_btn = QtWidgets.QPushButton("solve ik")
        self.solve_ik_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.solve_ik_btn.setFixedHeight(50)
        self.solve_ik_btn.setStyleSheet(self._btn_style("#424242"))
        self.solve_ik_btn.clicked.connect(self.solve_ik)
        ik_controls.addWidget(self.solve_ik_btn)

        # --- SECTION: FORWARD KINEMATICS ---
        fk_section = QtWidgets.QVBoxLayout()
        fk_section.setSpacing(12)
        root.addLayout(fk_section)

        fk_title = QtWidgets.QLabel("Forward Kinematics")
        fk_title.setStyleSheet("color: #2e7d32; font-size: 20px; font-weight: bold; margin-top: 10px;")
        fk_section.addWidget(fk_title)

        # Joint Table
        self.dh_table = QtWidgets.QTableWidget(0, 3)
        self.dh_table.setHorizontalHeaderLabels(["Joint", "current", "target"])
        self.dh_table.verticalHeader().setVisible(False)
        self.dh_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.dh_table.setFixedHeight(200)
        self.dh_table.setStyleSheet("""
            QTableWidget {
                background-color: white;
                border: 2px solid #e0e0e0;
                border-radius: 12px;
                gridline-color: #f0f0f0;
                font-size: 14px;
            }
            QHeaderView::section {
                background-color: #f5f5f5;
                padding: 8px;
                border: none;
                border-bottom: 2px solid #e0e0e0;
                font-weight: bold;
                color: #616161;
            }
        """)
        fk_section.addWidget(self.dh_table)

        # Solve FK Button
        self.run_fk_btn = QtWidgets.QPushButton("solve fk")
        self.run_fk_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.run_fk_btn.setFixedHeight(50)
        self.run_fk_btn.setStyleSheet(self._btn_style("#2e7d32"))
        self.run_fk_btn.clicked.connect(self.solve_fk)
        fk_section.addWidget(self.run_fk_btn)

        # (Removed) Key Point Button

        root.addStretch()
        self.rebuild_dh_table()

    def _btn_style(self, color):
        return f"""
            QPushButton {{
                background-color: white;
                color: {color};
                border: 2px solid #e0e0e0;
                border-radius: 12px;
                font-size: 18px;
                font-weight: 500;
            }}
            QPushButton:hover {{
                background-color: #f5f5f5;
                border-color: #bdbdbd;
            }}
            QPushButton:pressed {{
                background-color: #eeeeee;
                padding-top: 2px;
            }}
            QPushButton:disabled {{
                color: #bdbdbd;
                background-color: #fafafa;
            }}
        """

    def _make_num_input_minimal(self, lo, hi, val):
        spin = KeyboardOnlyDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setDecimals(2)
        spin.setValue(val)
        spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        spin.setStyleSheet("""
            QDoubleSpinBox {
                background: transparent;
                color: #424242;
                border: none;
                padding: 8px;
                font-size: 16px;
                font-weight: 500;
            }
        """)
        return spin

    def _labeled_widget_minimal(self, text, widget):
        frame = QtWidgets.QWidget()
        lay = QtWidgets.QHBoxLayout(frame)
        lay.setContentsMargins(10, 0, 10, 0)
        lay.setSpacing(2)
        lbl = QtWidgets.QLabel(text)
        lbl.setStyleSheet("color: #424242; font-size: 16px; font-weight: 500;")
        lay.addWidget(lbl)
        lay.addWidget(widget)
        return frame

    def _v_line(self):
        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.VLine)
        line.setFrameShadow(QtWidgets.QFrame.Plain)
        line.setStyleSheet("color: #e0e0e0; background-color: #e0e0e0;")
        line.setFixedWidth(2)
        return line


    def _make_num_input(self, lo, hi, val, decimals):
        spin = KeyboardOnlyDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setDecimals(decimals)
        spin.setValue(val)
        spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        spin.setStyleSheet(
            "QDoubleSpinBox { background:#ffffff; color:#1565c0; border:1px solid #90caf9; border-radius:6px; "
            "padding:6px 8px; font-size:15px; font-weight:700; }"
        )
        return spin

    def _labeled_widget(self, text, widget):
        frame = QtWidgets.QFrame()
        lay = QtWidgets.QHBoxLayout(frame)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(4)
        lbl = QtWidgets.QLabel(text)
        lbl.setStyleSheet("color: #455a64; font-size: 14px; font-weight: 600;")
        lay.addWidget(lbl)
        lay.addWidget(widget)
        return frame

    def _primary_btn_style(self):
        return (
            "QPushButton { background-color:#1976d2; color:white; border:none; border-radius:8px; padding:8px 14px; font-size:14px; font-weight:700; }"
            "QPushButton:hover { background-color:#1565c0; }"
        )

    def _get_tcp_link(self):
        robot = getattr(self.mw, "robot", None)
        if robot is None:
            return None

        def chain_len(link):
            if link is None:
                return -1
            return len(robot.get_kinematic_chain(link))

        committed_tcp = getattr(self.mw, "locked_live_point_link_name", None)
        if committed_tcp and committed_tcp in robot.links:
            link = robot.links[committed_tcp]
            if hasattr(self.mw, '_resolve_rigid_tcp_link'):
                link = self.mw._resolve_rigid_tcp_link(link)
            return link

        custom_tcp = getattr(self.mw, "custom_tcp_name", None)
        if custom_tcp and custom_tcp in robot.links:
            link = robot.links[custom_tcp]
            if hasattr(self.mw, '_resolve_rigid_tcp_link'):
                link = self.mw._resolve_rigid_tcp_link(link)
            return link

        tcp_candidates = [
            link for link in robot.links.values()
            if getattr(link, "custom_tcp_offset", None) is not None
        ]
        if tcp_candidates:
            return max(tcp_candidates, key=chain_len)

        gripper_candidates = [
            joint.child_link for joint in robot.joints.values()
            if getattr(joint, "is_gripper", False) and joint.child_link is not None
        ]
        if gripper_candidates:
            return max(gripper_candidates, key=chain_len)

        leaf_candidates = [
            link for link in robot.links.values()
            if link.parent_joint is not None and not link.child_joints
        ]
        if leaf_candidates:
            return max(leaf_candidates, key=chain_len)

        non_base = [link for link in robot.links.values() if not getattr(link, "is_base", False)]
        if non_base:
            return max(non_base, key=chain_len)

        links = list(robot.links.values())
        return max(links, key=chain_len) if links else None

    def _visible_master_joints(self):
        robot = getattr(self.mw, "robot", None)
        if robot is None:
            return []

        joint_tab = getattr(self.mw, "joint_tab", None)
        joint_data = getattr(joint_tab, "joints", {}) if joint_tab is not None else {}
        ordered = []
        seen = set()

        for child_name, data in joint_data.items():
            joint_id = data.get("joint_id", child_name)
            joint = robot.joints.get(joint_id)
            if joint is None:
                continue
            is_slave = any(
                any(slave_id == joint_id for slave_id, _ in slaves)
                for _, slaves in robot.joint_relations.items()
            )
            if is_slave or joint_id in seen:
                continue
            ordered.append((child_name, joint))
            seen.add(joint_id)

        for joint_id, joint in robot.joints.items():
            if joint_id in seen:
                continue
            is_slave = any(
                any(slave_id == joint_id for slave_id, _ in slaves)
                for _, slaves in robot.joint_relations.items()
            )
            if is_slave:
                continue
            child_name = joint.child_link.name if joint.child_link is not None else joint_id
            ordered.append((child_name, joint))
            seen.add(joint_id)

        return ordered

    def rebuild_dh_table(self):
        visible_joints = self._visible_master_joints()
        self._fk_rows = []
        self.dh_table.setRowCount(len(visible_joints))
        
        for idx, (child_name, joint) in enumerate(visible_joints):
            # Name
            name_item = QtWidgets.QTableWidgetItem(child_name)
            name_item.setFlags(name_item.flags() & ~QtCore.Qt.ItemIsEditable)
            name_item.setTextAlignment(QtCore.Qt.AlignCenter)
            self.dh_table.setItem(idx, 0, name_item)

            # Current
            curr_item = QtWidgets.QTableWidgetItem(f"{float(joint.current_value):.2f}")
            curr_item.setFlags(curr_item.flags() & ~QtCore.Qt.ItemIsEditable)
            curr_item.setTextAlignment(QtCore.Qt.AlignCenter)
            self.dh_table.setItem(idx, 1, curr_item)

            # Target
            target_val = self._fk_target_by_joint.get(joint.name, float(joint.current_value))
            target_item = QtWidgets.QTableWidgetItem(f"{target_val:.2f}")
            target_item.setTextAlignment(QtCore.Qt.AlignCenter)
            self.dh_table.setItem(idx, 2, target_item)

            self._fk_rows.append({
                "joint_id": joint.name,
                "child_name": child_name
            })

        self.run_fk_btn.setEnabled(len(visible_joints) > 0)

    def _table_num(self, row, col, default):
        item = self.dh_table.item(row, col)
        if not item: return default
        try:
            return float(item.text())
        except:
            return default

    def _sync_fk_current_angle(self, joint_id, angle_deg):
        for row_idx, row_data in enumerate(self._fk_rows):
            if row_data.get("joint_id") == joint_id:
                curr_item = self.dh_table.item(row_idx, 1)
                if curr_item:
                    curr_item.setText(f"{float(angle_deg):.2f}")
                break
    def update_display(self):
        self.rebuild_dh_table()

    def solve_ik(self):
        """Solves IK based on target XYZ, marks it with a black dot, and animates."""
        if self.mw._anim_timer.isActive():
            self.mw.log("⏳ IK Solve requested while animation is active. Ignoring.")
            return

        tx, ty, tz = self.ik_x.value(), self.ik_y.value(), self.ik_z.value()
        target_cm = np.array([tx, ty, tz])
        
        # --- Visual Feedback: Small Black Dot at Target ---
        import pyvista as pv
        dot = pv.Sphere(radius=0.5, center=target_cm)
        self.mw.canvas.plotter.add_mesh(dot, color="black", name="ik_target_dot", pickable=False)
        self.mw.canvas.plotter.render()

        tcp_link = self._get_preferred_tcp_link()
        if not tcp_link:
            QtWidgets.QMessageBox.warning(self, "IK Error", "No suitable TCP (End Effector) found.")
            return

        self.mw.log(f"🎯 Solving IK for target: ({tx:.1f}, {ty:.1f}, {tz:.1f})")
        
        # Call the centralized IK mover in the main window
        self.mw._move_tcp_to_xyz(tx, ty, tz, tcp_link)

    def _get_preferred_tcp_link(self):
        links = list(self.mw.robot.links.values())
        if not links: return None

        def chain_len(link):
            return len(self.mw.robot.get_kinematic_chain(link))

        committed_tcp_name = getattr(self.mw, "locked_live_point_link_name", None)
        if committed_tcp_name and committed_tcp_name in self.mw.robot.links:
            link = self.mw.robot.links[committed_tcp_name]
            if hasattr(self.mw, '_resolve_rigid_tcp_link'):
                link = self.mw._resolve_rigid_tcp_link(link)
            return link

        # 1. Custom TCP
        custom_tcp_name = getattr(self.mw, "custom_tcp_name", None)
        if custom_tcp_name and custom_tcp_name in self.mw.robot.links:
            link = self.mw.robot.links[custom_tcp_name]
            if hasattr(self.mw, '_resolve_rigid_tcp_link'):
                link = self.mw._resolve_rigid_tcp_link(link)
            return link

        tcp_candidates = [l for l in links if getattr(l, "custom_tcp_offset", None) is not None]
        if tcp_candidates:
            link = max(tcp_candidates, key=chain_len)
            if hasattr(self.mw, '_resolve_rigid_tcp_link'):
                link = self.mw._resolve_rigid_tcp_link(link)
            return link

        # 2. Gripper
        gripper_candidates = [j.parent_link for j in self.mw.robot.joints.values() if getattr(j, "is_gripper", False) and j.parent_link is not None]
        if gripper_candidates: return max(gripper_candidates, key=chain_len)

        # 3. Leaf nodes
        leaf_candidates = [l for l in links if not l.child_joints]
        if leaf_candidates: return max(leaf_candidates, key=chain_len)

        return max(links, key=chain_len)

    def solve_fk(self):
        if self.mw._anim_timer.isActive() or not self._fk_rows:
            return

        joint_ids, child_names, targets = [], [], []
        for idx, row in enumerate(self._fk_rows):
            joint = self.mw.robot.joints.get(row["joint_id"])
            if not joint: continue
            
            val = self._table_num(idx, 2, joint.current_value)
            val = float(np.clip(val, joint.min_limit, joint.max_limit))
            self._fk_target_by_joint[joint.name] = val
            
            joint_ids.append(joint.name)
            child_names.append(row["child_name"])
            targets.append(val)

        if joint_ids:
            self.mw._start_joint_animation(joint_ids, child_names, targets)

    # key_to_ik_point removed — no key button in UI anymore

    def refresh_sliders(self):
        self.rebuild_dh_table()

    def sync_slider(self, child_name, value):
        joint_tab = getattr(self.mw, "joint_tab", None)
        joint_id = child_name
        if joint_tab and child_name in joint_tab.joints:
            joint_id = joint_tab.joints[child_name].get("joint_id", child_name)
        self._sync_fk_current_angle(joint_id, value)

