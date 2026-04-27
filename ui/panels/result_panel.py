from PyQt5 import QtWidgets, QtCore
import numpy as np

class ResultPanel(QtWidgets.QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.mw = main_window
        self.init_ui()

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        title = QtWidgets.QLabel("Computation Results")
        title.setStyleSheet("color: #1565c0; font-size: 24px; font-weight: 700; padding: 4px 6px;")
        layout.addWidget(title)

        self.result_view = QtWidgets.QTextEdit()
        self.result_view.setReadOnly(True)
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
        layout.addWidget(self.result_view)

    def update_display(self, chain=None, T=None):
        if chain is None or T is None:
            self.result_view.setHtml(
                "<div style='margin-top: 50px; text-align: center;'>"
                "<p style='color:#b0bec5; font-size: 18px; font-style: italic;'>No computation data available.</p>"
                "<p style='color:#cfd8dc; font-size: 14px;'>Run Forward Kinematics in the 'IK and FK' tab to see results here.</p>"
                "</div>"
            )
            return

        html = """
        <style>
            body { font-family: 'Segoe UI', sans-serif; background-color: #ffffff; }
            .box { 
                border: 1px solid #e1e8ed; 
                border-radius: 12px; 
                margin: 20px 0; 
                background: #ffffff;
                box-shadow: 0 2px 4px rgba(0,0,0,0.05);
            }
            .head { 
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #1976d2, stop:1 #1565c0); 
                color: #ffffff; 
                font-weight: 700; 
                font-size: 16px; 
                padding: 12px 16px; 
                border-top-left-radius: 11px;
                border-top-right-radius: 11px;
            }
            .matrix-container { padding: 10px; }
            .matrix { width: 100%; border-collapse: collapse; margin: 5px 0; }
            .matrix td, .matrix th { text-align: center; padding: 10px; font-family: 'Consolas', 'Courier New', monospace; font-size: 14px; }
            .matrix th { background: #f8fbff; color: #546e7a; font-weight: 800; border-bottom: 2px solid #e3f2fd; }
            .matrix tr:nth-child(even) td { background: #fafcfe; }
            .matrix td { color: #263238; border-bottom: 1px solid #f0f4f8; }
            .matrix td:last-child { color: #1976d2; font-weight: 700; }
            
            .summary-box { 
                background: #f1f8ff; 
                padding: 20px; 
                border-radius: 12px; 
                margin-top: 25px; 
                border: 1px solid #bbdefb;
            }
            .summary-title { margin-top:0; color:#1565c0; font-size: 18px; font-weight: 700; }
            .pos-item { font-size: 20px; color:#0d47a1; font-family: 'Consolas', monospace; margin: 5px 0; }
            .pos-label { color: #546e7a; font-size: 14px; font-weight: normal; width: 30px; display: inline-block; }
        </style>
        """

        # Final End-Effector Pose
        html += (
            f"<div class='box' style='border: 2px solid #1976d2;'>"
            f"<div class='head' style='background:#0d47a1;'>End-Effector Pose Matrix (T_Total)</div>"
            f"<div class='matrix-container'>{self._matrix_html(T)}</div>"
            f"</div>"
        )
        
        # Position Summary
        p = T[:3, 3]
        html += (
            f"<div class='summary-box'>"
            f"<div class='summary-title'>End-Effector Position</div>"
            f"<div class='pos-item'><span class='pos-label'>X:</span> {p[0]:.4f}</div>"
            f"<div class='pos-item'><span class='pos-label'>Y:</span> {p[1]:.4f}</div>"
            f"<div class='pos-item'><span class='pos-label'>Z:</span> {p[2]:.4f}</div>"
            f"</div>"
        )

        self.result_view.setHtml(html)

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
