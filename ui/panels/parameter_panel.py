from PyQt5 import QtWidgets, QtCore, QtGui

class ParameterPanel(QtWidgets.QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.mw = main_window
        self.current_link_name = None
        self.inputs = {}
        self.init_ui()

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)
        
        header = QtWidgets.QLabel("PARAMETERS")
        header.setFont(QtGui.QFont("Segoe UI", 14, QtGui.QFont.Bold))
        header.setStyleSheet("color: #1976d2; margin-bottom: 6px;")
        layout.addWidget(header)
        
        self.count_label = QtWidgets.QLabel("Total: 0 components")
        self.count_label.setStyleSheet("color: #757575; font-size: 12px; font-weight: bold;")
        layout.addWidget(self.count_label)
        
        info_label = QtWidgets.QLabel("Imported Components:")
        info_label.setStyleSheet("color: #424242; font-weight: bold; margin-top: 10px;")
        layout.addWidget(info_label)
        
        self.link_list = QtWidgets.QListWidget()
        self.link_list.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.link_list.setStyleSheet("""
            QListWidget {
                background-color: white;
                border: 1px solid #e0e0e0;
                border-radius: 6px;
                font-size: 14px;
                outline: none;
            }
            QListWidget::item {
                background-color: #f8f9fa;
                color: #333;
                font-weight: bold;
                padding: 12px;
                margin-bottom: 2px;
                border-left: 4px solid transparent;
            }
            QListWidget::item:selected {
                background-color: #e3f2fd;
                color: #1565c0;
                border-left: 4px solid #1976d2;
            }
            QListWidget::item:hover {
                background-color: #f1f8fe;
            }
        """)
        self.link_list.itemClicked.connect(self.on_item_clicked)
        layout.addWidget(self.link_list)
        
        # --- PARAMETERS EDITOR ---
        self.editor_scroll = QtWidgets.QScrollArea()
        self.editor_scroll.setWidgetResizable(True)
        self.editor_scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")
        
        self.editor_widget = QtWidgets.QWidget()
        self.editor_layout = QtWidgets.QVBoxLayout(self.editor_widget)
        self.editor_layout.setContentsMargins(0, 15, 0, 0)
        self.editor_layout.setSpacing(15)
        
        # 1. Mass Group
        self.mass_group = self.create_group("1. Mass (m)", ["m"])
        self.editor_layout.addWidget(self.mass_group)
        
        # 2-4. CoG Group
        self.cog_group = self.create_group("2-4. Center of Gravity (CoG)", ["x", "y", "z"])
        self.editor_layout.addWidget(self.cog_group)
        
        # 5-10. Inertia Group
        self.inertia_group = self.create_group("5-10. Inertia Tensor", ["Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz"])
        self.editor_layout.addWidget(self.inertia_group)
        
        # --- AUTO COMPUTE BUTTON ---
        self.auto_compute_btn = QtWidgets.QPushButton("Auto-Calculate from Mesh")
        self.auto_compute_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.auto_compute_btn.setStyleSheet("""
            QPushButton {
                background-color: #1976d2;
                color: white;
                font-weight: bold;
                border-radius: 6px;
                padding: 10px;
                margin-top: 10px;
            }
            QPushButton:hover { background-color: #1565c0; }
        """)
        self.auto_compute_btn.clicked.connect(self.compute_from_mesh)
        self.editor_layout.addWidget(self.auto_compute_btn)
        
        self.editor_layout.addStretch()
        self.editor_scroll.setWidget(self.editor_widget)
        self.editor_scroll.hide()
        layout.addWidget(self.editor_scroll)
        
        layout.addStretch()

    def create_group(self, title, fields):
        group = QtWidgets.QGroupBox(title)
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size: 13px;
                color: #1976d2;
                border: 1px solid #e0e0e0;
                border-radius: 8px;
                margin-top: 12px;
                padding-top: 15px;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
        """)
        g_layout = QtWidgets.QVBoxLayout(group)
        
        for f in fields:
            row = QtWidgets.QHBoxLayout()
            label = QtWidgets.QLabel(f"{f}:")
            label.setFixedWidth(40)
            label.setStyleSheet("color: #616161; font-weight: normal;")
            
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(-10000, 10000)
            spin.setDecimals(4)
            spin.setSingleStep(0.01)
            spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
            spin.setCursor(QtCore.Qt.IBeamCursor)
            
            # Disable mouse wheel scrolling
            spin.wheelEvent = lambda event: event.ignore()
            
            spin.setStyleSheet("""
                QDoubleSpinBox {
                    background: white;
                    border: 1px solid #ddd;
                    border-radius: 4px;
                    padding: 4px;
                    color: #1976d2;
                    font-weight: bold;
                }
            """)
            spin.valueChanged.connect(self.save_parameters)
            
            row.addWidget(label)
            row.addWidget(spin)
            g_layout.addLayout(row)
            self.inputs[f] = spin
            
        return group

    def showEvent(self, event):
        super().showEvent(event)
        self.refresh_links()

    def on_item_clicked(self, item):
        self.current_link_name = item.text()
        self.load_parameters(self.current_link_name)
        self.editor_scroll.show()

    def compute_from_mesh(self):
        if not self.current_link_name or self.current_link_name not in self.mw.robot.links: return
        link = self.mw.robot.links[self.current_link_name]
        
        # Call the auto-calculation logic in the core model
        link.compute_physics_from_mesh()
        
        # Reload the UI with new values
        self.load_parameters(self.current_link_name)
        self.mw.log(f"Auto-calculated physics for '{self.current_link_name}'")

    def load_parameters(self, name):
        if name not in self.mw.robot.links: return
        link = self.mw.robot.links[name]
        
        for spin in self.inputs.values(): spin.blockSignals(True)
        self.inputs["m"].setValue(link.mass)
        for i, axis in enumerate(["x", "y", "z"]): self.inputs[axis].setValue(link.com[i])
        for key in ["ixx", "iyy", "izz", "ixy", "ixz", "iyz"]:
            self.inputs[key.capitalize() if len(key)==3 else key].setValue(link.inertia[key])
        for spin in self.inputs.values(): spin.blockSignals(False)

    def save_parameters(self):
        if not self.current_link_name or self.current_link_name not in self.mw.robot.links: return
        link = self.mw.robot.links[self.current_link_name]
        link.mass = self.inputs["m"].value()
        link.com = [self.inputs["x"].value(), self.inputs["y"].value(), self.inputs["z"].value()]
        link.inertia = {k.lower(): self.inputs[k.capitalize() if len(k)==3 else k].value() 
                        for k in ["ixx", "iyy", "izz", "ixy", "ixz", "iyz"]}

    def refresh_links(self):
        self.link_list.clear()
        def make_icon(color_str):
            pixmap = QtGui.QPixmap(20, 20); pixmap.fill(QtGui.QColor(color_str))
            return QtGui.QIcon(pixmap)
        red_icon = make_icon("#d32f2f"); green_icon = make_icon("#388e3c")

        if hasattr(self.mw, 'robot'):
            count = len(self.mw.robot.links)
            self.count_label.setText(f"Total: {count} components")
            for name, link in self.mw.robot.links.items():
                item = QtWidgets.QListWidgetItem(name)
                item.setIcon(red_icon if link.is_base else green_icon)
                self.link_list.addItem(item)
        
        if not self.link_list.currentItem(): self.editor_scroll.hide()
