from PyQt5 import QtWidgets, QtGui, QtCore
import numpy as np
import os
import random

from core.import_units import (
    SUPPORTED_LENGTH_UNITS,
    UP_AXIS_OPTIONS,
    detect_file_unit,
    get_engine_internal_unit,
    get_engine_units_per_cm,
    rotation_matrix_for_up_axis,
    unit_scale_to_internal,
)


class ImportOptionsDialog(QtWidgets.QDialog):
    def __init__(self, parent, file_name, detected_unit=None, detected_source="", default_unit="mm", default_up_axis="preserve"):
        super().__init__(parent)
        self.setWindowTitle("Import Units")
        self.setModal(True)
        self.resize(420, 220)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setSpacing(12)

        header = QtWidgets.QLabel(f"Import options for {file_name}")
        header.setStyleSheet("font-size: 15px; font-weight: bold; color: #1976d2;")
        layout.addWidget(header)

        detected_text = detected_unit or "Not detected"
        source_text = detected_source or "manual selection required"
        info = QtWidgets.QLabel(
            f"Detected unit: {detected_text}\nSource: {source_text}\nEngine internal unit: {get_engine_internal_unit()} (1 cm = {get_engine_units_per_cm():.0f} {get_engine_internal_unit()})"
        )
        info.setWordWrap(True)
        layout.addWidget(info)

        form = QtWidgets.QFormLayout()

        self.unit_combo = QtWidgets.QComboBox()
        self.unit_combo.addItems(list(SUPPORTED_LENGTH_UNITS))
        self.unit_combo.setCurrentText(detected_unit or default_unit)
        form.addRow("CAD unit", self.unit_combo)

        self.axis_combo = QtWidgets.QComboBox()
        self.axis_combo.addItems(list(UP_AXIS_OPTIONS))
        self.axis_combo.setCurrentText(default_up_axis if default_up_axis in UP_AXIS_OPTIONS else "preserve")
        form.addRow("Source up-axis", self.axis_combo)

        layout.addLayout(form)

        button_box = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

    def selected_options(self):
        return {
            "unit": self.unit_combo.currentText(),
            "up_axis": self.axis_combo.currentText(),
        }


class LinksMixin:
    """Methods for managing robot links: import, select, base, remove, color."""

    def setup_links_tab(self):
        layout = QtWidgets.QVBoxLayout(self.links_tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)
        
        # Section header
        header = QtWidgets.QLabel("COMPONENTS")
        header.setStyleSheet("color: #1976d2; font-size: 16px; font-weight: bold; padding: 4px 0;")
        layout.addWidget(header)
        
        import_btn = QtWidgets.QPushButton("Import STEP / STL")
        import_btn.setCursor(QtCore.Qt.PointingHandCursor)
        import_btn.setStyleSheet("""
            QPushButton {
                background-color: #1976d2;
                color: white;
                font-weight: bold;
                font-size: 14px;
                padding: 12px;
                border: none;
                border-radius: 8px;
            }
            QPushButton:hover { background-color: #1565c0; }
        """)
        import_btn.clicked.connect(self.import_mesh)
        layout.addWidget(import_btn)
        
        self.links_list = QtWidgets.QListWidget()
        self.links_list.setStyleSheet("""
            QListWidget {
                background-color: white;
                border: 1px solid #e0e0e0;
                border-radius: 6px;
                font-size: 14px;
            }
            QListWidget::item {
                padding: 6px 4px;
                border-bottom: 1px solid #f0f0f0;
            }
            QListWidget::item:selected {
                background-color: #e3f2fd;
            }
        """)
        self.links_list.itemClicked.connect(self.on_link_selected)
        layout.addWidget(self.links_list)
        
        btn_layout = QtWidgets.QHBoxLayout()
        btn_layout.setSpacing(8)
        self.set_base_btn = QtWidgets.QPushButton("Set as Base")
        self.set_base_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.remove_btn = QtWidgets.QPushButton("Remove")
        self.remove_btn.setCursor(QtCore.Qt.PointingHandCursor)
        
        self.set_base_btn.clicked.connect(self.set_as_base)
        self.remove_btn.clicked.connect(self.remove_link)
        
        btn_layout.addWidget(self.set_base_btn)
        btn_layout.addWidget(self.remove_btn)
        layout.addLayout(btn_layout)

        self.color_btn = QtWidgets.QPushButton("Change Color")
        self.color_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.color_btn.clicked.connect(self.change_color)
        layout.addWidget(self.color_btn)

        layout.addStretch()

    def on_link_selected(self, item):
        name = item.text()
        
        # Allow all objects to be selected, including jointed ones
        self.canvas.select_actor(name)
        
        # Update button text based on whether selection is the base
        if self.robot.base_link and name == self.robot.base_link.name:
            self.set_base_btn.setText("Deselect as Base")
        else:
            self.set_base_btn.setText("Set as Base")

    def set_as_base(self):
        item = self.links_list.currentItem()
        if not item:
            return
            
        name = item.text()
        if name not in self.robot.links:
            return
            
        link = self.robot.links[name]
        
        # --- COMPLIANCE CHECK: Only un-constrained objects can become the Base ---
        if self.robot.base_link != link:
            is_aligned = False
            if hasattr(self, 'alignment_cache'):
                for (p, c), pt in self.alignment_cache.items():
                    if c == name:
                        is_aligned = True; break
            
            if link.parent_joint:
                self.log(f"⚠️ Locked: '{name}' is jointed. Components with a parent joint cannot be set as the Base.")
                QtWidgets.QMessageBox.warning(self, "Locked", f"'{name}' is part of a joint. Remove the joint before making it the Base.")
                return
            if is_aligned:
                self.log(f"⚠️ Locked: '{name}' is aligned. Undo alignment before making it the Base.")
                QtWidgets.QMessageBox.warning(self, "Locked", f"'{name}' is aligned to another component. Reset alignment first.")
                return
        
        # TOGGLE LOGIC: If it's already the base, unset it
        if self.robot.base_link == link:
            self.robot.base_link = None
            link.is_base = False
            self.canvas.fixed_actors.clear()
            self.log(f"BASE UNSET: {name}. Link is now floating.")
            self.set_base_btn.setText("Set as Base")
        else:
            # 1. Calculate offset to center the mesh at (0,0,0)
            centroid = link.mesh.centroid
            
            # Create a translation matrix that moves the mesh's centroid to (0,0,0)
            t_center = np.eye(4)
            t_center[:3, 3] = -centroid
            
            # 2. Update Link Properties
            if self.robot.base_link:
                self.robot.base_link.is_base = False
                
            link.is_base = True
            link.t_offset = t_center
            
            # Base is defined at World Origin
            self.robot.base_link = link
            
            # LOCK in 3D Canvas (so it cannot be dragged)
            self.canvas.fixed_actors.clear()
            self.canvas.fixed_actors.add(name)
            self.set_base_btn.setText("Deselect as Base")
            self.log(f"BASE SET: {name}")
            self.log(f"Moved centroid {centroid} to (0,0,0)")
            self.canvas.plotter.reset_camera()
        
        # 3. Update Robot
        self.robot.update_kinematics()
        self.canvas.update_transforms(self.robot)
        
        # 4. Focus Camera
        self.update_link_colors()

    def go_to_joint_tab(self):
        item = self.links_list.currentItem()
        if not item:
            return
        
        name = item.text()
        # Switch to Joint Tab (Index 2)
        self.switch_panel(2)
        
        # Refresh links first to ensure combo boxes are up to date
        self.joint_tab.refresh_links()
        
        # Pre-select this link as the Child Link
        self.joint_tab.select_child_link(name)
        
        self.log(f"Switched to Joint creation for: {name}")

    def remove_link(self):
        item = self.links_list.currentItem()
        if not item:
            return
        
        name = item.text()
        
        # 1. Remove from Robot Model (Core)
        self.robot.remove_link(name)
        
        # 2. Cleanup and Sync Graphics state
        self.canvas.fixed_actors.clear()
        if self.robot.base_link:
            self.canvas.fixed_actors.add(self.robot.base_link.name)
        
        # Remove from Scene (Graphics)
        self.canvas.remove_actor(name)
        
        # 3. Remove from UI List
        row = self.links_list.row(item)
        self.links_list.takeItem(row)
        
        self.log(f"Removed link: {name}")
        
        # Refresh kinematics just in case
        self.robot.update_kinematics()
        self.canvas.update_transforms(self.robot)
        self.update_link_colors()

    def update_link_colors(self):
        """Updates the icons in the link list to show Base (Red) vs Normal/Joint (Green)."""
        root = self.robot.base_link
        
        # Create helper to make colored icons
        def make_icon(color_str):
            pixmap = QtGui.QPixmap(20, 20)
            pixmap.fill(QtGui.QColor(color_str))
            return QtGui.QIcon(pixmap)
            
        red_icon = make_icon("#d32f2f")   # Base Red
        green_icon = make_icon("#388e3c") # Joint Green
        
        for i in range(self.links_list.count()):
            item = self.links_list.item(i)
            name = item.text()
            
            if name in self.robot.links:
                link = self.robot.links[name]
                if link == root:
                    item.setIcon(red_icon)
                    item.setToolTip("Base Link (Fixed/Locked)")
                else:
                    item.setIcon(green_icon)
                    item.setToolTip("Joint/Child Link")

    def change_color(self):
        item = self.links_list.currentItem()
        if not item:
            return
            
        name = item.text()
        if name not in self.robot.links:
            return
            
        link = self.robot.links[name]
        initial_color = QtGui.QColor(link.color)
        
        color = QtWidgets.QColorDialog.getColor(initial_color, self, f"Select Color for {name}")
        if color.isValid():
            hex_color = color.name()
            link.color = hex_color
            if name in self.canvas.actors:
                self.canvas.set_actor_color(name, hex_color)
            self.update_link_colors()
            self.log(f"Changed color of {name} to {hex_color}")

    def apply_manual_scale(self):
        """Manually scales the selected link mesh."""
        item = self.links_list.currentItem()
        if not item:
            return
            
        name = item.text()
        if name not in self.robot.links:
            return
            
        scale = self.scale_spin.value()
        if scale == 1.0:
            return
            
        link = self.robot.links[name]
        try:
            link.mesh.apply_scale(scale)
            # Re-apply transform to refresh visual
            self.canvas.update_link_mesh(name, link.mesh, link.t_world, color=link.color)
            self.log(f"Scaled {name} by {scale}x")
            # Reset spinbox
            self.scale_spin.setValue(1.0)
        except Exception as e:
            self.log(f"Scale Error: {e}")

    def _ask_import_options(self, file_path, detected_unit, detection_source):
        suffix = os.path.splitext(file_path)[1].lower()
        prefs = getattr(self, "import_preferences", {})
        default_unit = prefs.get("last_stl_unit", "mm")
        default_up_axis = prefs.get("last_up_axis", "preserve")

        # STL has no dependable unit metadata, so always confirm it.
        must_confirm = suffix == ".stl" or detected_unit is None
        if not must_confirm:
            return {
                "unit": detected_unit,
                "up_axis": default_up_axis,
            }

        dialog = ImportOptionsDialog(
            self,
            os.path.basename(file_path),
            detected_unit=detected_unit,
            detected_source=detection_source,
            default_unit=detected_unit or default_unit,
            default_up_axis=default_up_axis,
        )
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return None

        selected = dialog.selected_options()
        if suffix == ".stl":
            prefs["last_stl_unit"] = selected["unit"]
        prefs["last_up_axis"] = selected["up_axis"]
        self.import_preferences = prefs
        return selected

    def _finalize_loaded_mesh(self, file_path, loaded):
        if isinstance(loaded, tuple):
            loaded = loaded[0]

        if hasattr(loaded, "geometry") and not hasattr(loaded, "vertices"):
            self.log("Detected assembly/scene. Merging meshes...")
            mesh = loaded.to_mesh()
        else:
            mesh = loaded

        if not hasattr(mesh, "vertices") or len(mesh.vertices) == 0:
            raise ValueError("Imported mesh has 0 vertices")

        return mesh.copy()

    def _prepare_imported_mesh(self, file_path, mesh):
        raw_bounds = np.array(mesh.bounds, dtype=float)
        raw_size = raw_bounds[1] - raw_bounds[0]
        detected_unit, detection_source = detect_file_unit(file_path, mesh)
        import_options = self._ask_import_options(file_path, detected_unit, detection_source)
        if import_options is None:
            return None

        source_unit = import_options["unit"]
        up_axis = import_options["up_axis"]
        scale_to_internal = unit_scale_to_internal(source_unit)

        prepared_mesh = mesh.copy()

        if abs(scale_to_internal - 1.0) > 1e-12:
            prepared_mesh.apply_scale(scale_to_internal)

        axis_rotation = rotation_matrix_for_up_axis(up_axis)
        if not np.allclose(axis_rotation, np.eye(4)):
            prepared_mesh.apply_transform(axis_rotation)

        final_bounds = np.array(prepared_mesh.bounds, dtype=float)
        final_size = final_bounds[1] - final_bounds[0]

        debug = {
            "source_unit": source_unit,
            "detected_unit": detected_unit,
            "detection_source": detection_source,
            "scale_to_internal": scale_to_internal,
            "up_axis": up_axis,
            "raw_bounds": raw_bounds,
            "raw_size": raw_size,
            "final_bounds": final_bounds,
            "final_size": final_size,
            "engine_unit": get_engine_internal_unit(),
        }
        return prepared_mesh, debug

    def _log_import_debug(self, name, debug):
        final_cm = debug["final_size"] / get_engine_units_per_cm()
        axis_label = "preserve native axes" if debug["up_axis"] == "preserve" else f"{debug['up_axis']}-up to z-up"

        self.log(
            f"[Import] {name}: source unit={debug['source_unit']} "
            f"(detected={debug['detected_unit'] or 'manual'}, via {debug['detection_source']})"
        )
        self.log(
            f"[Import] Scale factor to {debug['engine_unit']}: {debug['scale_to_internal']:.6f} "
            f"applied uniformly on X/Y/Z"
        )
        self.log(f"[Import] Up-axis mapping: {axis_label}")
        self.log(
            f"[Import] Bounding box before scaling: "
            f"{debug['raw_size'][0]:.3f} x {debug['raw_size'][1]:.3f} x {debug['raw_size'][2]:.3f} {debug['source_unit']}"
        )
        self.log(
            f"[Import] Bounding box after scaling: "
            f"{debug['final_size'][0]:.3f} x {debug['final_size'][1]:.3f} x {debug['final_size'][2]:.3f} mm "
            f"({final_cm[0]:.3f} x {final_cm[1]:.3f} x {final_cm[2]:.3f} cm)"
        )

    def measure_selected_link(self):
        item = self.links_list.currentItem()
        name = item.text() if item else getattr(self.canvas, "selected_name", None)
        if not name:
            self.show_toast("Select a link first", "warning")
            return
        measurement = self.canvas.measure_actor_bounds(name)
        if not measurement:
            self.show_toast("Unable to measure selection", "error")
            return

        dims_internal = measurement["dims_internal"]
        dims_cm = measurement["dims_cm"]
        self.log(
            f"[Measure] {name}: "
            f"{dims_internal[0]:.3f} x {dims_internal[1]:.3f} x {dims_internal[2]:.3f} {measurement['internal_unit']} "
            f"= {dims_cm[0]:.3f} x {dims_cm[1]:.3f} x {dims_cm[2]:.3f} cm"
        )
        self.show_toast(f"Measured {name}", "success")

    def add_reference_cube(self):
        import trimesh

        mesh = trimesh.creation.box(extents=[100.0, 100.0, 100.0])
        name = "reference_cube_100mm"
        base_name = name
        counter = 1
        while name in self.robot.links:
            name = f"{base_name}_{counter}"
            counter += 1

        link = self.robot.add_link(name, mesh)
        link.color = "#1976d2"
        link.import_metadata = {
            "source_unit": "mm",
            "detected_unit": "mm",
            "detection_source": "generated reference cube",
            "scale_to_internal": 1.0,
            "up_axis": "z",
            "engine_unit": get_engine_internal_unit(),
            "source_path": None,
            "raw_bounds": [[-50.0, -50.0, -50.0], [50.0, 50.0, 50.0]],
            "final_bounds": [[-50.0, -50.0, -50.0], [50.0, 50.0, 50.0]],
            "raw_size": [100.0, 100.0, 100.0],
            "final_size": [100.0, 100.0, 100.0],
        }

        self.add_link_item(name)

        t_import = np.eye(4)
        t_import[:3, 3] = [50.0 * get_engine_units_per_cm(), 50.0 * get_engine_units_per_cm(), 50.0 * get_engine_units_per_cm()]
        link.t_offset = t_import

        self.canvas.update_link_mesh(name, mesh, t_import, color=link.color)
        actor = self.canvas.actors[name]
        self.canvas.ensure_grid_fits_bounds(actor.GetBounds())
        self.canvas.select_actor(name)
        for i in range(self.links_list.count()):
            item = self.links_list.item(i)
            if item and item.text() == name:
                self.links_list.setCurrentItem(item)
                break
        self.canvas.focus_on_actor(name)
        self.update_link_colors()

        self.log("[Validation] Added reference cube with exact size 100.000 x 100.000 x 100.000 mm (10.000 cm)")
        self.measure_selected_link()

    def import_mesh(self):
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Import Mesh", "", "3D Files (*.stl *.step *.stp *.obj)"
        )
        if file_path:
            self.log(f"Importing: {os.path.basename(file_path)}")
            import trimesh
            try:
                loaded = trimesh.load(file_path)
                mesh = self._finalize_loaded_mesh(file_path, loaded)
                prepared = self._prepare_imported_mesh(file_path, mesh)
                if prepared is None:
                    self.log("Import cancelled.")
                    return
                mesh, import_debug = prepared

                # Assign a random distinct color
                colors = ["#e74c3c", "#3498db", "#2ecc71", "#f1c40f", "#9b59b6", "#1abc9c", "#e67e22", "#95a5a6"]
                link_color = random.choice(colors)

                name = os.path.basename(file_path).split('.')[0]
                
                # Handle unique naming
                base_name = name
                counter = 1
                while name in self.robot.links:
                    name = f"{base_name}_{counter}"
                    counter += 1
                
                link = self.robot.add_link(name, mesh)
                link.color = link_color
                link.import_metadata = {
                    "source_unit": import_debug["source_unit"],
                    "detected_unit": import_debug["detected_unit"],
                    "detection_source": import_debug["detection_source"],
                    "scale_to_internal": import_debug["scale_to_internal"],
                    "up_axis": import_debug["up_axis"],
                    "engine_unit": import_debug["engine_unit"],
                    "source_path": file_path,
                    "raw_size": import_debug["raw_size"].tolist(),
                    "final_size": import_debug["final_size"].tolist(),
                    "raw_bounds": import_debug["raw_bounds"].tolist(),
                    "final_bounds": import_debug["final_bounds"].tolist(),
                }
                
                # Tag as Simulation Object if imported in simulation mode
                if hasattr(self, 'sim_toggle_btn') and self.sim_toggle_btn.isChecked():
                    link.is_sim_obj = True
                
                # Use new helper to add row with 'Eye' button
                self.add_link_item(name)
                
                # Default spawn position: (50, 50, 50) cm
                ratio = get_engine_units_per_cm()
                t_import = np.eye(4)
                t_import[:3, 3] = [50.0 * ratio, 50.0 * ratio, 50.0 * ratio]
                link.t_offset = t_import
                
                self.canvas.update_link_mesh(name, mesh, t_import, color=link.color)
                
                # SELF-ADJUSTING GRAPH: 
                # If component is larger than grid, expand the grid automatically
                actor = self.canvas.actors[name]
                self.canvas.ensure_grid_fits_bounds(actor.GetBounds())
                
                self.log(f"Successfully loaded: {name}")
                self._log_import_debug(name, import_debug)
                
                # Auto-select and focus
                self.canvas.select_actor(name)
                for i in range(self.links_list.count()):
                    item_i = self.links_list.item(i)
                    if item_i and item_i.text() == name:
                        self.links_list.setCurrentItem(item_i)
                        break
                self.canvas.focus_on_actor(name)
                
                self.update_link_colors()
                
                # Refresh Simulation Objects list if needed
                if getattr(link, 'is_sim_obj', False):
                    self.refresh_sim_objects_list()
                    
                    # --- AUTO-SELECT and AUTO-POPULATE DIM on import ---
                    # Find and select the newly-imported item in the sim objects list
                    sim_list = self.simulation_tab.objects_list
                    for i in range(sim_list.count()):
                        item_i = sim_list.item(i)
                        if item_i and item_i.text() == name:
                            sim_list.setCurrentItem(item_i)
                            break
                    
                    # Populate DIM fields and object info immediately
                    self.simulation_tab.refresh_object_info(name)
                    
                    # --- INDUSTRIAL READINESS: Auto-capture P1 and set P2 ---
                    # 1. Capture current bottom-center as P1
                    self.simulation_tab.capture_object_to_p1()
                    
                    # 2. Set default P2 (e.g., 20cm away in Y axis)
                    p1_y = self.simulation_tab.pick_y.value()
                    self.simulation_tab.place_x.setValue(self.simulation_tab.pick_x.value())
                    self.simulation_tab.place_y.setValue(p1_y + 20.0) # Move 20cm north
                    self.simulation_tab.place_z.setValue(0.0) # Place at floor
                    
                    self.log(f"📐 System Ready: Dimensions and P1/P2 auto-populated for '{name}'.")
                    self.show_toast(f"Robot ready to pick {name}", "success")
                
            except ImportError as ie:
                self.log(f"MISSING DEPENDENCY: {str(ie)}")
                QtWidgets.QMessageBox.critical(self, "Import Error", 
                    f"To load STEP files, you need extra libraries.\n\nError: {str(ie)}\n\n"
                    "I am currently trying to install 'cascadio' for you. "
                    "Please restart the app once the installation finishes.")
            except Exception as e:
                self.log(f"Error: {str(e)}")

    def add_link_item(self, name):
        """Helper to add an item to the list with a focus button."""
        item = QtWidgets.QListWidgetItem(self.links_list)
        item.setText(name)
        
        # Create custom widget for the row
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(widget)
        layout.setContentsMargins(12, 8, 10, 8)
        
        # Label with Name
        name_label = QtWidgets.QLabel(name)
        name_label.setStyleSheet("border: none; font-size: 16px; font-weight: bold; color: #212121;")
        layout.addWidget(name_label)
        layout.addStretch()
        
        # Focus Button — uses Qt standard icon (always visible on Windows)
        focus_btn = QtWidgets.QPushButton()
        focus_btn.setIcon(widget.style().standardIcon(QtWidgets.QStyle.SP_FileDialogContentsView))
        focus_btn.setIconSize(QtCore.QSize(20, 20))
        focus_btn.setToolTip(f"Focus on {name}")
        focus_btn.setAccessibleName(f"Focus {name}")
        focus_btn.setFixedSize(38, 38)
        focus_btn.setCursor(QtCore.Qt.PointingHandCursor)
        focus_btn.setStyleSheet("""
            QPushButton {
                background-color: white;
                border: 2px solid #e0e0e0;
                border-radius: 19px;
                padding: 4px;
            }
            QPushButton:hover {
                background-color: white;
                border-color: #1976d2;
            }
        """)
        focus_btn.clicked.connect(lambda: self.canvas.focus_on_actor(name))
        layout.addWidget(focus_btn)
        
        # Set taller row height
        item.setSizeHint(QtCore.QSize(0, 52))
        
        # Apply to list
        self.links_list.addItem(item)
        self.links_list.setItemWidget(item, widget)
