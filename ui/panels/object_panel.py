from PyQt5 import QtWidgets, QtCore, QtGui


class ObjectPanel(QtWidgets.QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.mw = main_window
        self.selected_object_type = None
        self.init_ui()

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(0)

        self.surface = QtWidgets.QFrame()
        self.surface.setStyleSheet("""
            QFrame {
                background: #ffffff;
                border: 2px solid #2d2d2d;
                border-radius: 30px;
            }
            QLabel {
                color: #222222;
                background: transparent;
            }
        """)
        surface_layout = QtWidgets.QVBoxLayout(self.surface)
        surface_layout.setContentsMargins(18, 16, 18, 18)
        surface_layout.setSpacing(0)

        normal_text = """
            color: #222222;
            font-family: 'Segoe UI', Arial, sans-serif;
            font-size: 18px;
            font-weight: normal;
        """

        self.root_stack = QtWidgets.QStackedWidget()
        surface_layout.addWidget(self.root_stack)

        self.selection_page = self._build_selection_page(normal_text)
        self.root_stack.addWidget(self.selection_page)

        self.detail_page = self._build_detail_page(normal_text)
        self.root_stack.addWidget(self.detail_page)

        self.root_stack.setCurrentWidget(self.selection_page)
        layout.addWidget(self.surface)
        self.refresh_object_list()

    def update_display(self):
        self._sync_selection_preview()
        self.refresh_object_list()

    def refresh_sliders(self):
        pass

    def refresh_object_list(self):
        object_lists = [
            widget
            for widget in (
                getattr(self, "main_objects_list", None),
                getattr(self, "objects_list", None),
            )
            if widget is not None
        ]
        if not object_lists:
            return

        current_name = getattr(self, "_selected_scene_object_name", None)
        if not current_name:
            for object_list in object_lists:
                current_item = object_list.currentItem()
                if current_item is None:
                    continue
                current_name = current_item.data(QtCore.Qt.UserRole)
                if not isinstance(current_name, str):
                    current_name = None
                if current_name:
                    break

        robot = getattr(self.mw, "robot", None)
        for object_list in object_lists:
            object_list.blockSignals(True)
        try:
            for object_list in object_lists:
                object_list.clear()
            if robot is None:
                return

            display_index = 1
            for name, link in robot.links.items():
                if not getattr(link, "is_sim_obj", False):
                    continue
                label = self._object_list_label(display_index, name, link)
                for object_list in object_lists:
                    item = QtWidgets.QListWidgetItem(label)
                    item.setData(QtCore.Qt.UserRole, name)
                    item.setToolTip(f"Scene object: {name}")
                    object_list.addItem(item)
                display_index += 1

            if current_name:
                for object_list in object_lists:
                    self._select_object_in_list(object_list, current_name)
        finally:
            for object_list in object_lists:
                object_list.blockSignals(False)
        self._update_action_button_state()

    @staticmethod
    def _object_list_label(display_index, name, link):
        metadata = getattr(link, "import_metadata", {})
        object_type = ""
        if isinstance(metadata, dict):
            object_type = str(metadata.get("object_type", "")).strip().lower()
        if not object_type:
            lowered_name = str(name).lower()
            if "cube" in lowered_name:
                object_type = "cube"
            elif "cylinder" in lowered_name:
                object_type = "cylinder"
            else:
                object_type = "object"
        return f"{display_index} - {object_type} ({name})"

    @staticmethod
    def _select_object_in_list(object_list, name):
        for index in range(object_list.count()):
            item = object_list.item(index)
            if item.data(QtCore.Qt.UserRole) == name:
                object_list.setCurrentItem(item)
                return

    def _build_selection_page(self, handwritten):
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        self.title_label = QtWidgets.QLabel("Object")
        self.title_label.setStyleSheet(handwritten + "font-size: 24px;")
        layout.addWidget(self.title_label)

        self.select_label = QtWidgets.QLabel("select object")
        self.select_label.setStyleSheet(handwritten + "margin-top: 8px;")
        self.select_label.setWordWrap(True)
        layout.addWidget(self.select_label)

        layout.addSpacing(10)
        self._add_divider(layout)

        chooser = QtWidgets.QWidget()
        chooser_layout = QtWidgets.QVBoxLayout(chooser)
        chooser_layout.setContentsMargins(0, 18, 0, 18)
        chooser_layout.setSpacing(12)

        self.object_type_group = QtWidgets.QButtonGroup(self)
        self.object_type_group.setExclusive(True)

        self.cube_radio = QtWidgets.QRadioButton("1 - cube")
        self.cylinder_radio = QtWidgets.QRadioButton("2 - cylinder")
        for radio in (self.cube_radio, self.cylinder_radio):
            radio.setStyleSheet(handwritten + "font-size: 19px;")
            chooser_layout.addWidget(radio)
            self.object_type_group.addButton(radio)

        self.object_type_group.buttonClicked.connect(self._sync_selection_preview)

        self.selection_hint = QtWidgets.QLabel(
            "Choose a shape, then press OK to open its specification menu."
        )
        self.selection_hint.setWordWrap(True)
        self.selection_hint.setStyleSheet(handwritten + "font-size: 16px;")
        chooser_layout.addWidget(self.selection_hint)

        layout.addWidget(chooser)

        button_row = QtWidgets.QHBoxLayout()
        button_row.addStretch()
        self.selection_ok_btn = QtWidgets.QPushButton("OK")
        self.selection_ok_btn.setFixedSize(90, 34)
        self.selection_ok_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.selection_ok_btn.setStyleSheet("""
            QPushButton {
                background: #2d2d2d;
                color: white;
                border: none;
                border-radius: 8px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover { background: #444444; }
            QPushButton:pressed { background: #1f1f1f; }
            QPushButton:disabled { background: #bdbdbd; color: #eeeeee; }
        """)
        self.selection_ok_btn.clicked.connect(self.open_selected_object_menu)
        button_row.addWidget(self.selection_ok_btn)
        layout.addLayout(button_row)

        layout.addSpacing(8)
        self._add_divider(layout)

        self.main_objects_label = QtWidgets.QLabel("Objects in Scene")
        self.main_objects_label.setStyleSheet(
            handwritten + "font-size: 19px; margin-top: 10px;"
        )
        layout.addWidget(self.main_objects_label)

        self.main_objects_list = self._create_scene_objects_list(minimum_height=170)
        self.main_objects_list.itemClicked.connect(self.on_object_item_clicked)
        layout.addWidget(self.main_objects_list)

        layout.addStretch()
        self._sync_selection_preview()
        return page

    def _build_detail_page(self, handwritten):
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        header_row = QtWidgets.QHBoxLayout()
        header_row.setContentsMargins(0, 0, 0, 0)
        header_row.setSpacing(10)

        self.detail_title = QtWidgets.QLabel("Object Specification")
        self.detail_title.setStyleSheet(handwritten + "font-size: 24px;")
        header_row.addWidget(self.detail_title)
        header_row.addStretch()

        self.back_btn = QtWidgets.QPushButton("Back")
        self.back_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.back_btn.setFixedSize(84, 32)
        self.back_btn.setStyleSheet("""
            QPushButton {
                background: #f5f5f5;
                color: #222222;
                border: 1px solid #888888;
                border-radius: 6px;
                font-weight: bold;
            }
            QPushButton:hover { background: #eeeeee; }
            QPushButton:pressed { background: #e0e0e0; }
        """)
        self.back_btn.clicked.connect(self.show_selection_menu)
        header_row.addWidget(self.back_btn)
        layout.addLayout(header_row)

        self.detail_hint = QtWidgets.QLabel("")
        self.detail_hint.setStyleSheet(handwritten + "margin-top: 8px;")
        self.detail_hint.setWordWrap(True)
        layout.addWidget(self.detail_hint)

        layout.addSpacing(20)
        self._add_divider(layout)

        self.spec_stack = QtWidgets.QStackedWidget()
        layout.addWidget(self.spec_stack)

        self.cube_section = self._build_cube_section(handwritten)
        self.spec_stack.addWidget(self.cube_section)

        self.cylinder_section = self._build_cylinder_section(handwritten)
        self.spec_stack.addWidget(self.cylinder_section)

        self._add_divider(layout)

        self.import_section = self._build_import_section(handwritten)
        layout.addWidget(self.import_section)

        action_row = QtWidgets.QHBoxLayout()
        action_row.addStretch()
        self.create_btn = QtWidgets.QPushButton("Import Shape")
        self.create_btn.setFixedHeight(36)
        self.create_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.create_btn.setStyleSheet("""
            QPushButton {
                background: #1976d2;
                color: white;
                border: none;
                border-radius: 8px;
                padding: 6px 14px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover { background: #1565c0; }
            QPushButton:pressed { background: #0d47a1; }
        """)
        self.create_btn.clicked.connect(self.import_shape_to_scene)
        action_row.addWidget(self.create_btn)
        layout.addLayout(action_row)

        self.objects_label = QtWidgets.QLabel("Objects")
        self.objects_label.setStyleSheet(handwritten + "font-size: 18px; margin-top: 10px;")
        layout.addWidget(self.objects_label)

        self.objects_list = self._create_scene_objects_list(minimum_height=150)
        self.objects_list.itemClicked.connect(self.on_object_item_clicked)
        layout.addWidget(self.objects_list)

        action_row = QtWidgets.QHBoxLayout()
        action_row.addStretch()
        self.delete_btn = QtWidgets.QPushButton("Delete Object")
        self.delete_btn.setFixedHeight(34)
        self.delete_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.delete_btn.setStyleSheet("""
            QPushButton {
                background: #d32f2f;
                color: white;
                border: none;
                border-radius: 8px;
                padding: 6px 14px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover { background: #b71c1c; }
            QPushButton:pressed { background: #8e0000; }
            QPushButton:disabled { background: #bdbdbd; color: #eeeeee; }
        """)
        self.delete_btn.clicked.connect(self.delete_selected_object)
        action_row.addWidget(self.delete_btn)

        self.color_btn = QtWidgets.QPushButton("Change Colour")
        self.color_btn.setFixedHeight(34)
        self.color_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.color_btn.setStyleSheet("""
            QPushButton {
                background: #2e7d32;
                color: white;
                border: none;
                border-radius: 8px;
                padding: 6px 14px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover { background: #256428; }
            QPushButton:pressed { background: #1b5e20; }
            QPushButton:disabled { background: #bdbdbd; color: #eeeeee; }
        """)
        self.color_btn.clicked.connect(self.change_selected_object_color)
        action_row.addWidget(self.color_btn)
        layout.addLayout(action_row)

        layout.addStretch()
        return page

    def _create_scene_objects_list(self, minimum_height):
        object_list = QtWidgets.QListWidget()
        object_list.setMinimumHeight(minimum_height)
        object_list.setStyleSheet("""
            QListWidget {
                background: #f7fafc;
                border: 1px solid #888888;
                border-radius: 8px;
                padding: 4px;
                font-size: 14px;
                color: #222222;
            }
            QListWidget::item {
                padding: 8px;
                border-bottom: 1px solid #e0e0e0;
            }
            QListWidget::item:selected {
                background: #dbeeff;
                color: #1976d2;
            }
        """)
        return object_list

    def _add_divider(self, layout):
        line = QtWidgets.QFrame()
        line.setFixedHeight(2)
        line.setStyleSheet("background: #2d2d2d; border: none;")
        layout.addWidget(line)

    def _mk_value_label(self, text, style):
        label = QtWidgets.QLabel(text)
        label.setStyleSheet(style)
        return label

    def _mk_text_input(self, placeholder):
        edit = QtWidgets.QLineEdit()
        edit.setPlaceholderText(placeholder)
        edit.setFixedHeight(30)
        edit.setValidator(QtGui.QDoubleValidator(-999999.0, 999999.0, 3, edit))
        edit.setStyleSheet("""
            QLineEdit {
                background: #ffffff;
                color: #222222;
                border: 1px solid #888888;
                border-radius: 6px;
                padding: 4px 8px;
                font-family: 'Segoe UI', Arial, sans-serif;
                font-size: 16px;
                font-weight: normal;
            }
        """)
        return edit

    def _sync_selection_preview(self, *_args):
        selection = self.get_selected_object_type()
        if selection == "cube":
            self.select_label.setText("cube selected")
        elif selection == "cylinder":
            self.select_label.setText("cylinder selected")
        else:
            self.select_label.setText("select object")

        self.selection_ok_btn.setEnabled(selection is not None)

    def get_selected_object_type(self):
        if getattr(self, "cube_radio", None) and self.cube_radio.isChecked():
            return "cube"
        if getattr(self, "cylinder_radio", None) and self.cylinder_radio.isChecked():
            return "cylinder"
        return None

    def open_selected_object_menu(self):
        selection = self.get_selected_object_type()
        if selection is None:
            if hasattr(self.mw, "show_toast"):
                self.mw.show_toast("Select cube or cylinder first", "warning")
            return

        self.selected_object_type = selection
        self.detail_title.setText(f"{selection.title()} Specification")
        self.detail_hint.setText(
            "Fill in the dimensions and import coordinates for the selected object in mm."
        )

        if selection == "cube":
            self.spec_stack.setCurrentWidget(self.cube_section)
            self.select_label.setText("cube specification menu")
        else:
            self.spec_stack.setCurrentWidget(self.cylinder_section)
            self.select_label.setText("cylinder specification menu")

        self.root_stack.setCurrentWidget(self.detail_page)

    def show_selection_menu(self):
        self.root_stack.setCurrentWidget(self.selection_page)
        self._sync_selection_preview()

    def get_object_specification(self):
        """Return the currently visible object specification values."""
        if self.selected_object_type == "cube":
            return {
                "type": "cube",
                "length": self.cube_length.text().strip(),
                "breadth": self.cube_breadth.text().strip(),
                "height": self.cube_height.text().strip(),
                "import": self.get_import_coordinates(),
            }
        if self.selected_object_type == "cylinder":
            return {
                "type": "cylinder",
                "diameter": self.cylinder_dia.text().strip(),
                "height": self.cylinder_height.text().strip(),
                "import": self.get_import_coordinates(),
            }
        return {"type": None, "import": self.get_import_coordinates()}

    def get_import_coordinates(self):
        return {
            "x": self.import_x.text().strip(),
            "y": self.import_y.text().strip(),
            "z": self.import_z.text().strip(),
        }

    def import_shape_to_scene(self):
        spec = self.get_object_specification()
        if hasattr(self.mw, "create_object_from_panel"):
            self.mw.create_object_from_panel(spec)
        elif hasattr(self.mw, "show_toast"):
            self.mw.show_toast("Object import is not available in this build", "error")

    def on_object_item_clicked(self, item):
        name = item.data(QtCore.Qt.UserRole)
        if not isinstance(name, str):
            name = item.text().strip()
        robot = getattr(self.mw, "robot", None)
        canvas = getattr(self.mw, "canvas", None)
        if robot is None or canvas is None or name not in robot.links:
            return

        self._selected_scene_object_name = name
        for object_list in (
            getattr(self, "main_objects_list", None),
            getattr(self, "objects_list", None),
        ):
            if object_list is None or object_list is item.listWidget():
                continue
            object_list.blockSignals(True)
            try:
                self._select_object_in_list(object_list, name)
            finally:
                object_list.blockSignals(False)

        canvas.select_actor(name)
        self._update_action_button_state(name)

        sim_tab = getattr(self.mw, "simulation_tab", None)
        if sim_tab is not None and hasattr(sim_tab, "objects_list"):
            matches = sim_tab.objects_list.findItems(name, QtCore.Qt.MatchExactly)
            if matches:
                sim_tab.objects_list.setCurrentItem(matches[0])
            sim_tab.refresh_object_info(name)

    def _selected_object_name(self):
        tracked_name = getattr(self, "_selected_scene_object_name", None)
        robot = getattr(self.mw, "robot", None)
        if tracked_name and robot is not None and tracked_name in robot.links:
            return tracked_name
        item = self.objects_list.currentItem() if hasattr(self, "objects_list") else None
        if item is None:
            return None
        name = item.data(QtCore.Qt.UserRole)
        if isinstance(name, str):
            return name
        return item.text().strip()

    def _update_action_button_state(self, name=None):
        if not hasattr(self, "delete_btn") or not hasattr(self, "color_btn"):
            return
        name = name or self._selected_object_name()
        robot = getattr(self.mw, "robot", None)
        link = robot.links.get(name) if robot is not None and name in robot.links else None
        enabled = bool(link is not None and getattr(link, "is_sim_obj", False))
        self.delete_btn.setEnabled(enabled)
        self.color_btn.setEnabled(enabled)

    def change_selected_object_color(self):
        name = self._selected_object_name()
        if name is None:
            if hasattr(self.mw, "show_toast"):
                self.mw.show_toast("Select an object first", "warning")
            return

        robot = getattr(self.mw, "robot", None)
        canvas = getattr(self.mw, "canvas", None)
        if robot is None or canvas is None or name not in robot.links:
            return

        link = robot.links[name]
        initial = QtGui.QColor(getattr(link, "color", "#7B1FA2"))
        color = QtWidgets.QColorDialog.getColor(initial, self, f"Select Colour for {name}")
        if not color.isValid():
            return

        link.color = color.name()
        if name in canvas.actors:
            canvas.set_actor_color(name, link.color)
        if hasattr(self.mw, "update_link_colors"):
            self.mw.update_link_colors()
        self.refresh_object_list()
        if hasattr(self.mw, "show_toast"):
            self.mw.show_toast(f"Changed colour for {name}", "success")

    def delete_selected_object(self):
        name = self._selected_object_name()
        if name is None:
            if hasattr(self.mw, "show_toast"):
                self.mw.show_toast("Select an object first", "warning")
            return

        robot = getattr(self.mw, "robot", None)
        canvas = getattr(self.mw, "canvas", None)
        if robot is None or canvas is None or name not in robot.links:
            return

        link = robot.links[name]
        if not getattr(link, "is_sim_obj", False):
            if hasattr(self.mw, "show_toast"):
                self.mw.show_toast("Only simulation objects can be deleted here", "warning")
            return

        reply = QtWidgets.QMessageBox.question(
            self,
            "Delete Object",
            f"Delete '{name}'?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if reply != QtWidgets.QMessageBox.Yes:
            return

        if robot.base_link is link:
            robot.base_link = None
            link.is_base = False
        if name in canvas.fixed_actors:
            canvas.fixed_actors.discard(name)

        robot.remove_link(name)
        canvas.remove_actor(name)
        if getattr(self, "_selected_scene_object_name", None) == name:
            self._selected_scene_object_name = None

        if hasattr(self.mw, "refresh_link_hierarchy"):
            self.mw.refresh_link_hierarchy()
        if hasattr(self.mw, "update_link_colors"):
            self.mw.update_link_colors()
        if hasattr(self.mw, "refresh_sim_objects_list"):
            self.mw.refresh_sim_objects_list()
        sim_tab = getattr(self.mw, "simulation_tab", None)
        if sim_tab is not None and hasattr(sim_tab, "objects_list"):
            current_item = sim_tab.objects_list.currentItem()
            if current_item is None or current_item.text() == name:
                next_item = sim_tab.objects_list.item(0) if sim_tab.objects_list.count() else None
                if next_item is not None:
                    sim_tab.objects_list.setCurrentItem(next_item)
                    sim_tab.refresh_object_info(next_item.text())
                else:
                    sim_tab.dim_label.setText("Dimensions: ---")
                    sim_tab.pos_label.setText("Current Pos: ---")
                    if hasattr(sim_tab, "obj_width"):
                        sim_tab.obj_width.setValue(0.0)
                    if hasattr(sim_tab, "obj_depth"):
                        sim_tab.obj_depth.setValue(0.0)
                    if hasattr(sim_tab, "obj_height"):
                        sim_tab.obj_height.setValue(0.0)

        self.refresh_object_list()
        self._update_action_button_state()
        if hasattr(self.mw, "show_toast"):
            self.mw.show_toast(f"Deleted {name}", "success")

    def _build_cube_section(self, handwritten):
        section = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(section)
        layout.setContentsMargins(0, 18, 0, 18)
        layout.setSpacing(12)

        header = QtWidgets.QLabel("1 - cube (mm)")
        header.setStyleSheet(handwritten)
        layout.addWidget(header)

        self.cube_length = self._mk_text_input("l mm")
        self.cube_breadth = self._mk_text_input("b mm")
        self.cube_height = self._mk_text_input("h mm")
        for field, prefix in (
            (self.cube_length, "l mm ="),
            (self.cube_breadth, "b mm ="),
            (self.cube_height, "h mm ="),
        ):
            row = QtWidgets.QHBoxLayout()
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(10)
            label = QtWidgets.QLabel(prefix)
            label.setStyleSheet(handwritten + "font-size: 17px;")
            row.addWidget(label)
            row.addWidget(field, 1)
            layout.addLayout(row)

        return section

    def _build_cylinder_section(self, handwritten):
        section = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(section)
        layout.setContentsMargins(0, 18, 0, 18)
        layout.setSpacing(12)

        header = QtWidgets.QLabel("2 - cylinder (mm)")
        header.setStyleSheet(handwritten)
        layout.addWidget(header)

        self.cylinder_dia = self._mk_text_input("dia mm")
        self.cylinder_height = self._mk_text_input("height mm")

        for field, prefix in (
            (self.cylinder_dia, "Dia mm ="),
            (self.cylinder_height, "Height mm ="),
        ):
            row = QtWidgets.QHBoxLayout()
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(10)
            label = QtWidgets.QLabel(prefix)
            label.setStyleSheet(handwritten + "font-size: 17px;")
            row.addWidget(label)
            row.addWidget(field, 1)
            layout.addLayout(row)

        return section

    def _build_import_section(self, handwritten):
        section = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(section)
        layout.setContentsMargins(0, 18, 0, 0)
        layout.setSpacing(12)

        import_label = QtWidgets.QLabel("Import (mm)")
        import_label.setStyleSheet(handwritten + "font-size: 17px;")
        layout.addWidget(import_label)

        self.import_x = self._mk_text_input("x mm")
        self.import_y = self._mk_text_input("y mm")
        self.import_z = self._mk_text_input("z mm")

        for label_text, field in (("x mm:", self.import_x), ("y mm:", self.import_y), ("z mm:", self.import_z)):
            row = QtWidgets.QVBoxLayout()
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(4)
            label = QtWidgets.QLabel(label_text)
            label.setStyleSheet(handwritten + "font-size: 17px;")
            row.addWidget(label)
            row.addWidget(field)
            layout.addLayout(row, 1)

        layout.addStretch()
        return section
