from PyQt5 import QtWidgets
import os
import numpy as np

from core.robot import Robot
from core.import_units import get_engine_internal_unit


class ProjectMixin:
    """Methods for saving and loading robot project files (.trn)."""

    def _project_dialog_dir(self):
        candidate = getattr(self, "last_project_dir", "") or os.getcwd()
        if os.path.isfile(candidate):
            candidate = os.path.dirname(candidate)
        if not candidate or not os.path.isdir(candidate):
            candidate = os.getcwd()
        return candidate

    def _detect_legacy_project_scale(self, robot_data, temp_dir):
        """Detect older project files saved in meters before internal-mm normalization."""
        links = robot_data.get("links", [])
        if not links:
            return 1.0, "no links"

        # Modern projects include import metadata/preferences from the newer loader.
        has_modern_metadata = any(link.get("import_metadata") for link in links) or bool(
            robot_data.get("ui_state", {}).get("import_preferences")
        )
        if has_modern_metadata:
            return 1.0, "modern metadata present"

        try:
            import trimesh
        except Exception:
            return 1.0, "trimesh unavailable for detection"

        max_extent = 0.0
        max_translation = 0.0
        inspected = 0
        for link in links:
            mesh_rel_path = link.get("mesh_file")
            if not mesh_rel_path:
                continue
            mesh_path = os.path.join(temp_dir, mesh_rel_path)
            if not os.path.exists(mesh_path):
                continue
            mesh = trimesh.load(mesh_path)
            if isinstance(mesh, trimesh.Scene):
                mesh = mesh.to_mesh()
            if hasattr(mesh, "bounds"):
                bounds = np.array(mesh.bounds, dtype=float)
                if bounds.shape == (2, 3):
                    extent = float(np.max(bounds[1] - bounds[0]))
                    max_extent = max(max_extent, extent)
            t_offset = np.array(link.get("t_offset", np.eye(4)), dtype=float)
            if t_offset.shape == (4, 4):
                max_translation = max(max_translation, float(np.max(np.abs(t_offset[:3, 3]))))
            inspected += 1

        if not inspected:
            return 1.0, "no mesh data inspected"

        # Older projects stored robot geometry in meters; typical robot dimensions and
        # offsets therefore fall below ~2.0. In current internal-mm projects they are
        # normally tens to hundreds.
        if max_extent <= 2.0 and max_translation <= 2.0:
            return 1000.0, "legacy meter-based project detected"

        return 1.0, "already aligned with current internal units"

    def _scale_legacy_project_data(self, robot_data, scale_factor):
        if abs(scale_factor - 1.0) < 1e-12:
            return

        for link in robot_data.get("links", []):
            t_offset = np.array(link.get("t_offset", np.eye(4)), dtype=float)
            if t_offset.shape == (4, 4):
                t_offset[:3, 3] *= scale_factor
                link["t_offset"] = t_offset.tolist()

            if link.get("custom_tcp_offset") is not None:
                link["custom_tcp_offset"] = (np.array(link["custom_tcp_offset"], dtype=float) * scale_factor).tolist()

            if link.get("pick_pos") is not None:
                link["pick_pos"] = (np.array(link["pick_pos"], dtype=float) * scale_factor).tolist()

            if link.get("place_pos") is not None:
                link["place_pos"] = (np.array(link["place_pos"], dtype=float) * scale_factor).tolist()

        for joint in robot_data.get("joints", []):
            if joint.get("origin") is not None:
                joint["origin"] = (np.array(joint["origin"], dtype=float) * scale_factor).tolist()

        ui_state = robot_data.get("ui_state", {})
        if ui_state.get("alignment_point") is not None:
            ui_state["alignment_point"] = (np.array(ui_state["alignment_point"], dtype=float) * scale_factor).tolist()
        if isinstance(ui_state.get("alignment_cache"), dict):
            for key, value in list(ui_state["alignment_cache"].items()):
                ui_state["alignment_cache"][key] = (np.array(value, dtype=float) * scale_factor).tolist()
        if isinstance(ui_state.get("joint_panel_joints"), dict):
            for _, data in ui_state["joint_panel_joints"].items():
                if data.get("alignment_point") is not None:
                    data["alignment_point"] = (np.array(data["alignment_point"], dtype=float) * scale_factor).tolist()
                if data.get("custom_tcp_offset") is not None:
                    data["custom_tcp_offset"] = (np.array(data["custom_tcp_offset"], dtype=float) * scale_factor).tolist()

    def save_project(self):
        """Saves current robot configuration into a .trn zip file."""
        import json
        import zipfile
        import io
        import tempfile
        import shutil

        default_filename = "project.trn"
        if hasattr(self, "current_session_index") and self.current_session_index >= 0:
            sess = self.robot_sessions[self.current_session_index]
            if sess.get("project_file_path"):
                default_filename = os.path.basename(sess["project_file_path"])
            elif hasattr(self, "session_tab_bar"):
                tab_title = self.session_tab_bar.tabText(self.current_session_index)
                if tab_title != "ToRoTrOn" and not tab_title.startswith("Robo "):
                    default_filename = f"{tab_title}.trn"

        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save Project",
            os.path.join(self._project_dialog_dir(), default_filename),
            "ToRoTRoN Project (*.trn);;Zip Archive (*.zip);;All Files (*)"
        )
        if not file_path:
            return
            
        if not file_path.lower().endswith(('.trn', '.zip')):
            file_path += '.trn'
        self.last_project_dir = os.path.dirname(file_path)

        try:
            # Create a temporary directory to gather files
            with tempfile.TemporaryDirectory() as temp_dir:
                mesh_dir = os.path.join(temp_dir, "meshes")
                os.makedirs(mesh_dir)

                robot_data = {
                    "links": [],
                    "joints": [],
                    "ui_state": {
                        "joint_panel_joints": {},
                        "program_code": "",
                        "live_sync": False,
                        "alignment_point": None,
                        "alignment_normal": None,
                        "alignment_cache": {},
                        "current_speed": 50,
                        "camera_position": None,
                        "import_preferences": {},
                        "last_project_dir": getattr(self, "last_project_dir", os.getcwd()),
                    },
                    "joint_relations": {}
                }

                # 1. Gather Links
                for name, link in self.robot.links.items():
                    mesh_filename = f"{name}.stl"
                    mesh_path = os.path.join(mesh_dir, mesh_filename)
                    
                    # Export mesh
                    link.mesh.export(mesh_path, file_type='stl')
                    
                    robot_data["links"].append({
                        "name": link.name,
                        "mesh_file": f"meshes/{mesh_filename}",
                        "color": link.color,
                        "is_base": link.is_base,
                        "t_offset": link.t_offset.tolist(),
                        "custom_tcp_offset": None if getattr(link, "custom_tcp_offset", None) is None else list(link.custom_tcp_offset),
                        "custom_tcp_rpy_deg": list(getattr(link, "custom_tcp_rpy_deg", [0.0, 0.0, 0.0])),
                        "is_sim_obj": getattr(link, "is_sim_obj", False),
                        "pick_pos": list(getattr(link, "pick_pos", [0.0, 0.0, 0.0])),
                        "place_pos": list(getattr(link, "place_pos", [0.0, 0.0, 0.0])),
                        "import_metadata": getattr(link, "import_metadata", {}),
                    })

                # 2. Gather Joints (Robot Core)
                saved_home = getattr(self.robot, "home_joint_values", None)
                for name, joint in self.robot.joints.items():
                    robot_data["joints"].append({
                        "name": joint.name,
                        "parent_link": joint.parent_link.name,
                        "child_link": joint.child_link.name,
                        "joint_type": joint.joint_type,
                        "origin": joint.origin.tolist(),
                        "axis": joint.axis.tolist(),
                        "min_limit": joint.min_limit,
                        "max_limit": joint.max_limit,
                        "current_value": joint.current_value
                    })
                if saved_home:
                    robot_data["ui_state"]["home_joint_values"] = {
                        str(name): float(value) for name, value in saved_home.items()
                    }

                # 2b. Joint Relations
                for master_id, slaves in self.robot.joint_relations.items():
                    robot_data["joint_relations"][master_id] = slaves

                # 3. Gather UI State
                # Joint Panel UI Data
                if hasattr(self, 'joint_tab'):
                    for child_name, data in self.joint_tab.joints.items():
                        clean_data = data.copy()
                        if 'alignment_point' in clean_data and isinstance(clean_data['alignment_point'], np.ndarray):
                            clean_data['alignment_point'] = clean_data['alignment_point'].tolist()
                        robot_data["ui_state"]["joint_panel_joints"][child_name] = clean_data

                # Program Tab Code
                if hasattr(self, 'experiment_tab') and hasattr(self.experiment_tab, 'program_tab'):
                    robot_data["ui_state"]["program_code"] = self.experiment_tab.program_tab.code_edit.toPlainText()

                # Align Panel Stored Point (for continuing joint creation)
                if hasattr(self, 'align_tab'):
                    if hasattr(self.align_tab, 'alignment_point') and self.align_tab.alignment_point is not None:
                        robot_data["ui_state"]["alignment_point"] = self.align_tab.alignment_point.tolist()
                    if hasattr(self.align_tab, 'alignment_normal') and self.align_tab.alignment_normal is not None:
                        robot_data["ui_state"]["alignment_normal"] = self.align_tab.alignment_normal.tolist()

                # Alignment Cache (from MainWindow)
                if hasattr(self, 'alignment_cache'):
                    # Convert {(p, c): point} to {"p,c": point} for JSON
                    serializable_cache = {}
                    for (p, c), pt in self.alignment_cache.items():
                        serializable_cache[f"{p}|||{c}"] = pt.tolist()
                    robot_data["ui_state"]["alignment_cache"] = serializable_cache

                # Speed
                if hasattr(self, 'current_speed'):
                    robot_data["ui_state"]["current_speed"] = self.current_speed

                # Camera Position
                if hasattr(self, 'canvas'):
                    robot_data["ui_state"]["camera_position"] = [list(p) for p in self.canvas.plotter.camera_position]

                if hasattr(self, "import_preferences"):
                    robot_data["ui_state"]["import_preferences"] = dict(self.import_preferences)

                # 4. Write JSON
                json_path = os.path.join(temp_dir, "robot.json")
                with open(json_path, 'w') as f:
                    json.dump(robot_data, f, indent=4)

                # 4. ZIP everything up
                with zipfile.ZipFile(file_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
                    for root, dirs, files in os.walk(temp_dir):
                        for file in files:
                            abs_file = os.path.join(root, file)
                            rel_file = os.path.relpath(abs_file, temp_dir)
                            zipf.write(abs_file, rel_file)

            self.log(f"Project saved to: {file_path}")
            if hasattr(self, "current_session_index") and self.current_session_index >= 0:
                session = self.robot_sessions[self.current_session_index]
                session["robot"] = self.robot
                session["project_file_path"] = file_path
                title, _ = os.path.splitext(os.path.basename(file_path))
                session["title"] = title
                if hasattr(self, "_capture_current_robot_session"):
                    self._capture_current_robot_session()
                if hasattr(self, "session_tab_bar"):
                    self.session_tab_bar.setTabText(self.current_session_index, title)
            QtWidgets.QMessageBox.information(self, "Success", "Project saved successfully.")

        except Exception as e:
            self.log(f"SAVE ERROR: {str(e)}")
            QtWidgets.QMessageBox.critical(self, "Save Error", f"Could not save project: {str(e)}")

    def load_project_from_path(self, file_path, show_dialogs=True, auto_finalize=True):
        """Loads a robot configuration from a .trn zip file path."""
        import json
        import zipfile
        import tempfile
        import shutil
        import trimesh

        if not file_path:
            return False
        if not os.path.exists(file_path):
            msg = f"Could not load project because the file does not exist: {file_path}"
            self.log(msg)
            if show_dialogs:
                QtWidgets.QMessageBox.critical(self, "Load Error", msg)
            return False
        self.last_project_dir = os.path.dirname(file_path)

        try:
            # 1. Clear Current Robot
            self.robot = Robot()
            self.canvas.clear_highlights()
            # Remove all actors from canvas
            actor_names = list(self.canvas.actors.keys())
            for name in actor_names:
                self.canvas.remove_actor(name)
            self.canvas.fixed_actors.clear()
            self.links_list.clear()
            self.alignment_cache = {}

            # Reset UI Panels
            if hasattr(self, 'joint_tab'):
                self.joint_tab.reset_joint_ui()
            if hasattr(self, 'align_tab'):
                self.align_tab.reset_panel()
            if hasattr(self, "import_preferences"):
                self.import_preferences = {
                    "last_stl_unit": "mm",
                    "last_up_axis": "preserve",
                }

            # 2. Extract ZIP to temp folder
            with tempfile.TemporaryDirectory() as temp_dir:
                with zipfile.ZipFile(file_path, 'r') as zipf:
                    zipf.extractall(temp_dir)

                # 3. Read JSON
                json_path = os.path.join(temp_dir, "robot.json")
                if not os.path.exists(json_path):
                    raise Exception("Invalid project file: robot.json missing")

                with open(json_path, 'r') as f:
                    robot_data = json.load(f)

                legacy_scale_factor, legacy_reason = self._detect_legacy_project_scale(robot_data, temp_dir)
                if abs(legacy_scale_factor - 1.0) > 1e-12:
                    self.log(
                        f"Legacy project normalization: scaling saved geometry and transforms by "
                        f"{legacy_scale_factor:.1f} for {get_engine_internal_unit()} engine units "
                        f"({legacy_reason})."
                    )
                    self._scale_legacy_project_data(robot_data, legacy_scale_factor)

                # 4. Load Links
                for l_data in robot_data["links"]:
                    QtWidgets.QApplication.processEvents()
                    name = l_data["name"]
                    mesh_rel_path = l_data["mesh_file"]
                    mesh_path = os.path.join(temp_dir, mesh_rel_path)
                    
                    if not os.path.exists(mesh_path):
                        self.log(f"WARNING: Mesh file missing for {name}")
                        continue

                    raw_mesh = trimesh.load(mesh_path)
                    if isinstance(raw_mesh, trimesh.Scene):
                        mesh = raw_mesh.to_mesh()
                    else:
                        mesh = raw_mesh

                    if abs(legacy_scale_factor - 1.0) > 1e-12:
                        mesh.apply_scale(legacy_scale_factor)
                        
                    link = self.robot.add_link(name, mesh)
                    link.color = l_data.get("color", "lightgray")
                    link.is_base = l_data.get("is_base", False)
                    link.t_offset = np.array(l_data["t_offset"])
                    if l_data.get("custom_tcp_offset") is not None:
                        link.custom_tcp_offset = np.array(l_data["custom_tcp_offset"], dtype=float)
                    link.custom_tcp_rpy_deg = l_data.get("custom_tcp_rpy_deg", [0.0, 0.0, 0.0])
                    link.is_sim_obj = l_data.get("is_sim_obj", False)
                    link.pick_pos = l_data.get("pick_pos", [0.0, 0.0, 0.0])
                    link.place_pos = l_data.get("place_pos", [0.0, 0.0, 0.0])
                    link.import_metadata = l_data.get("import_metadata", {})
                    
                    if link.is_base:
                        self.robot.base_link = link
                        self.canvas.fixed_actors.add(name)

                    # Add to UI and Canvas
                    self.add_link_item(name)
                    self.canvas.update_link_mesh(name, mesh, link.t_offset, color=link.color)

                # 5. Load Joints (Robot Core)
                for j_data in robot_data["joints"]:
                    QtWidgets.QApplication.processEvents()
                    name = j_data["name"]
                    parent_name = j_data["parent_link"]
                    child_name = j_data["child_link"]
                    
                    if parent_name in self.robot.links and child_name in self.robot.links:
                        joint = self.robot.add_joint(name, parent_name, child_name)
                        joint.joint_type = j_data.get("joint_type", "revolute")
                        joint.origin = np.array(j_data["origin"])
                        joint.axis = np.array(j_data["axis"])
                        joint.min_limit = j_data.get("min_limit", -180.0)
                        joint.max_limit = j_data.get("max_limit", 180.0)
                        joint.current_value = j_data.get("current_value", 0.0)

                # 5b. Load Joint Relations
                self.robot.joint_relations = robot_data.get("joint_relations", {})
                self.robot.home_joint_values = {
                    j_data["name"]: float(j_data.get("current_value", 0.0))
                    for j_data in robot_data.get("joints", [])
                }

                # 6. Load UI State
                ui_state = robot_data.get("ui_state", {})
                saved_home = ui_state.get("home_joint_values")
                if isinstance(saved_home, dict):
                    self.robot.home_joint_values.update({
                        str(name): float(value) for name, value in saved_home.items()
                        if name in self.robot.joints
                    })
                
                # Restore Joint Panel Data
                if hasattr(self, 'joint_tab'):
                    self.joint_tab.joints = ui_state.get("joint_panel_joints", {})
                    # Convert alignment points back to numpy
                    for child_name, data in self.joint_tab.joints.items():
                        if 'alignment_point' in data and data['alignment_point'] is not None:
                            data['alignment_point'] = np.array(data['alignment_point'])
                    
                    self.joint_tab.refresh_joints_history()
                    self.joint_tab.refresh_links()

                # Restore Program Tab
                if hasattr(self, 'experiment_tab') and hasattr(self.experiment_tab, 'program_tab'):
                    self.experiment_tab.program_tab.code_edit.setPlainText(ui_state.get("program_code", ""))

                # Restore Align Panel alignment data
                if hasattr(self, 'align_tab'):
                    ap = ui_state.get("alignment_point")
                    if ap: self.align_tab.alignment_point = np.array(ap)
                    an = ui_state.get("alignment_normal")
                    if an: self.align_tab.alignment_normal = np.array(an)
                
                # Restore Alignment Cache
                cache_data = ui_state.get("alignment_cache", {})
                for key, pt in cache_data.items():
                    if "|||" in key:
                        p, c = key.split("|||")
                        self.alignment_cache[(p, c)] = np.array(pt)

                # Restore the UI Joint panels and re-render visual joints.
                # During silent startup loading, skip the eager rebuild because it
                # refreshes several heavy panels for every joint and can make the
                # app look hung before the first paint.
                if hasattr(self, 'joint_tab'):
                    # Clear out arrows first
                    arrow_names = [a for a in self.canvas.actors.keys() if a.startswith("joint_axis_")]
                    for aname in arrow_names:
                        self.canvas.remove_actor(aname)

                    should_rebuild_joint_controls = show_dialogs or auto_finalize
                    if should_rebuild_joint_controls:
                        # Rebuild 3D arrows for all joints by syncing the UI state.
                        for child_name, data in self.joint_tab.joints.items():
                            self.joint_tab.show_joint_control(child_name)

                    self.joint_tab.active_joint_control = None # Unselect

                # Restore Speed
                if "current_speed" in ui_state:
                    self.current_speed = ui_state["current_speed"]
                    if hasattr(self, 'speed_slider'):
                        self.speed_slider.blockSignals(True)
                        self.speed_slider.setValue(self.current_speed)
                        self.speed_slider.blockSignals(False)
                    if hasattr(self, 'speed_spin'):
                        self.speed_spin.blockSignals(True)
                        self.speed_spin.setValue(self.current_speed)
                        self.speed_spin.blockSignals(False)
                
                # Restore Camera
                if "camera_position" in ui_state and ui_state["camera_position"]:
                    try:
                        self.canvas.plotter.camera_position = [tuple(p) for p in ui_state["camera_position"]]
                    except:
                        self.canvas.plotter.reset_camera()
                else:
                    self.canvas.plotter.reset_camera()

                prefs = ui_state.get("import_preferences")
                if isinstance(prefs, dict):
                    self.import_preferences.update(prefs)

                last_project_dir = ui_state.get("last_project_dir")
                if last_project_dir and os.path.isdir(last_project_dir):
                    self.last_project_dir = last_project_dir

                zero_joint_values = {name: 0.0 for name in self.robot.joints.keys()}
                self.robot.home_joint_values = dict(zero_joint_values)
                self.robot.reset_to_home(home_angle=0.0, home_joint_values=zero_joint_values)

            # 7. Final Update
            self.robot.update_kinematics()
            self.canvas.update_transforms(self.robot)
            self.update_link_colors()
            if hasattr(self, "update_live_ui"):
                self.update_live_ui(render=False)
            
            # Refresh Matrices Panel
            if hasattr(self, 'matrices_tab'):
                self.matrices_tab.refresh_sliders()
                self.matrices_tab.update_display()

            # Smart Camera Reset: use robot world-space bounds so loaded parts
            # appear even when actor bounds/cached camera state are stale.
            self.canvas.focus_on_robot(self.robot)

            if auto_finalize and hasattr(self, "make_robot"):
                self.make_robot()
            
            self.log(f"Project loaded from: {os.path.basename(file_path)}")
            if hasattr(self, "current_session_index") and self.current_session_index >= 0:
                session = self.robot_sessions[self.current_session_index]
                session["robot"] = self.robot
                session["project_file_path"] = file_path
                title, _ = os.path.splitext(os.path.basename(file_path))
                if self.current_session_index == 0 and title.lower() == "default_robot":
                    title = "ToRoTrOn"
                session["title"] = title
                if hasattr(self, "_capture_current_robot_session"):
                    self._capture_current_robot_session()
                if hasattr(self, "session_tab_bar"):
                    self.session_tab_bar.setTabText(self.current_session_index, title)
            if show_dialogs:
                QtWidgets.QMessageBox.information(self, "Success", f"Project '{os.path.basename(file_path)}' loaded successfully.")
            return True

        except Exception as e:
            self.log(f"LOAD ERROR: {str(e)}")
            if show_dialogs:
                QtWidgets.QMessageBox.critical(self, "Load Error", f"Could not load project: {str(e)}")
            return False

    def load_project(self):
        """Loads a robot configuration from a .trn zip file."""
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Open Project",
            self._project_dialog_dir(),
            "Project Files (*.trn *.zip);;ToRoTRoN Project (*.trn);;Zip Archive (*.zip);;All Files (*)"
        )
        if not file_path:
            return False
        return self.load_project_from_path(file_path, show_dialogs=True, auto_finalize=True)

    def new_project(self):
        """Clears the workspace and initializes a new empty robot project."""
        reply = QtWidgets.QMessageBox.question(
            self,
            "New Project",
            "Are you sure you want to clear the current project and start a new robot assembly?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No
        )
        if reply == QtWidgets.QMessageBox.Yes:
            self.robot = Robot()
            self.canvas.clear_highlights()
            # Remove all actors from canvas
            actor_names = list(self.canvas.actors.keys())
            for name in actor_names:
                self.canvas.remove_actor(name)
            self.canvas.fixed_actors.clear()
            self.links_list.clear()
            self.alignment_cache = {}

            # Reset UI Panels
            if hasattr(self, 'joint_tab'):
                self.joint_tab.reset_joint_ui()
            if hasattr(self, 'align_tab'):
                self.align_tab.reset_panel()
            if hasattr(self, "import_preferences"):
                self.import_preferences = {
                    "last_stl_unit": "mm",
                    "last_up_axis": "preserve",
                }
            self.canvas.plotter.reset_camera()
            self.canvas.plotter.render()

            if hasattr(self, "experiment_btn"):
                self.experiment_btn.setChecked(False)
            if hasattr(self, "experiment_container"):
                self.experiment_container.setVisible(False)
            if hasattr(self, "assembly_btn"):
                self.assembly_btn.setChecked(True)
            if hasattr(self, "left_container"):
                self.left_container.setVisible(True)
            if hasattr(self, "_set_main_splitter_layout"):
                self._set_main_splitter_layout(show_assembly=True, show_experiment=False)
            if hasattr(self, "switch_panel"):
                self.switch_panel(0)

            self.log("Add Robo: new empty robot assembly initialized.")
            if hasattr(self, "current_session_index") and self.current_session_index >= 0:
                self.robot_sessions[self.current_session_index]["project_file_path"] = None
                if hasattr(self, "session_tab_bar"):
                    default_name = "ToRoTrOn" if self.current_session_index == 0 else f"Robo {self.current_session_index + 1}"
                    self.session_tab_bar.setTabText(self.current_session_index, default_name)
            if hasattr(self, "show_toast"):
                self.show_toast("New project created", "success")





