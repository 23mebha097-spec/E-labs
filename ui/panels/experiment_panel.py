from PyQt5 import QtWidgets, QtCore

from ui.panels.matrices_panel import MatricesPanel
from ui.panels.ik_fk_panel import IKFKPanel
from ui.panels.result_panel import ResultPanel
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
        self.program_tab = ProgramPanel(self.mw)

        self.tabs.addTab(self.matrices_tab, "Matrices")
        self.tabs.addTab(self.ik_fk_tab, "IK and FK")
        self.tabs.addTab(self.result_tab, "Result")
        self.tabs.addTab(self.program_tab, "Code")

        layout.addWidget(self.tabs)

    def refresh_sliders(self):
        self.matrices_tab.refresh_sliders()
        self.ik_fk_tab.refresh_sliders()
        self.ik_fk_tab.rebuild_dh_table()

    def update_display(self):
        self.matrices_tab.update_display()
        self.ik_fk_tab.update_display()

    def sync_slider(self, child_name, value):
        self.matrices_tab.sync_slider(child_name, value)
        self.ik_fk_tab.sync_slider(child_name, value)
