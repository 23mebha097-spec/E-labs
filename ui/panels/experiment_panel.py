from PyQt5 import QtWidgets, QtCore

from ui.panels.matrices_panel import MatricesPanel
from ui.panels.ik_fk_panel import IKFKPanel
from ui.panels.result_panel import ResultPanel
from ui.panels.object_panel import ObjectPanel
from ui.panels.program_panel import ProgramPanel

class ExperimentPanel(QtWidgets.QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.mw = main_window
        self.init_ui()

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.setStyleSheet(
            """
            QTabWidget::pane {
                border: none;
                background: #f7fafc;
            }
            QTabBar::tab {
                background: #e8f1f8;
                color: #1e3a5f;
                padding: 10px 16px;
                margin: 4px 2px 0 2px;
                border-top-left-radius: 8px;
                border-top-right-radius: 8px;
                font-size: 14px;
                font-weight: 700;
            }
            QTabBar::tab:selected {
                background: #1976d2;
                color: white;
            }
            QTabBar::tab:hover:!selected {
                background: #d8e8f7;
            }
            """
        )

        self.matrices_tab = MatricesPanel(self.mw)
        self.ik_fk_tab = IKFKPanel(self.mw)
        self.result_tab = ResultPanel(self.mw)
        self.object_tab = ObjectPanel(self.mw)
        self.program_tab = ProgramPanel(self.mw)

        self.tabs.addTab(self.matrices_tab, "Matrices")
        self.tabs.addTab(self.ik_fk_tab, "IK and FK")
        self.tabs.addTab(self.result_tab, "Result")
        self.tabs.addTab(self.object_tab, "Object")
        self.tabs.addTab(self.program_tab, "Code")

        layout.addWidget(self.tabs)

        self.tabs.currentChanged.connect(self.on_tab_changed)

    def _safe_call(self, panel, method_name, *args):
        method = getattr(panel, method_name, None)
        if method is None:
            return None
        try:
            return method(*args)
        except Exception as exc:
            panel_name = panel.__class__.__name__
            if hasattr(self.mw, "log"):
                self.mw.log(f"Experiment panel warning: {panel_name}.{method_name} failed: {exc}")
            return None

    def on_tab_changed(self, index):
        widget = self.tabs.widget(index)
        self._safe_call(widget, "update_display")

    def refresh_sliders(self):
        self._safe_call(self.matrices_tab, "refresh_sliders")
        self._safe_call(self.ik_fk_tab, "refresh_sliders")
        self._safe_call(self.ik_fk_tab, "rebuild_dh_table")
        self._safe_call(self.result_tab, "update_display")
        self._safe_call(self.object_tab, "refresh_sliders")

    def update_display(self):
        self._safe_call(self.matrices_tab, "update_display")
        self._safe_call(self.ik_fk_tab, "update_display")
        self._safe_call(self.result_tab, "update_display")
        self._safe_call(self.object_tab, "update_display")

    def sync_slider(self, child_name, value):
        self._safe_call(self.matrices_tab, "sync_slider", child_name, value)
        self._safe_call(self.ik_fk_tab, "sync_slider", child_name, value)
