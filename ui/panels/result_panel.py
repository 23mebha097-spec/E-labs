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

    def update_display(self, fk_results=None):
        fk_results = fk_results or []

        # Use the globally accurate live point if available, otherwise fall back to DH calculation
        p = getattr(self.mw, "current_live_point_cm", None)
        if p is None and fk_results:
            # Fallback to DH-based end-effector position
            ee_result = fk_results[-1]
            p = ee_result["cumulative"][:3, 3]

        # Calculate the 4x4 Transform Matrix for the live point / TCP
        T_matrix = None
        if hasattr(self.mw, "robot") and hasattr(self.mw, "_get_preferred_tcp_link"):
            tcp_link = getattr(self.mw, "_get_preferred_tcp_link")()
            if tcp_link:
                T_raw = self.mw.robot.get_tcp_world_pose(tcp_link).copy()
                ratio = getattr(self.mw.canvas, "grid_units_per_cm", 1.0) or 1.0
                T_raw[:3, 3] /= ratio
                T_matrix = T_raw

        if T_matrix is None and fk_results:
            ee_result = fk_results[-1]
            T_matrix = ee_result["cumulative"]

        if p is None and T_matrix is not None:
            p = T_matrix[:3, 3]

        if T_matrix is None:
            self.result_view.setHtml(
                "<div style='margin-top: 50px; text-align: center;'>"
                "<p style='color:#b0bec5; font-size: 18px; font-style: italic;'>No computation data available.</p>"
                "<p style='color:#cfd8dc; font-size: 14px;'>Adjust joint rotations or run IK/FK to see results here.</p>"
                "</div>"
            )
            return

        html = """
        <style>
            body { font-family: 'Segoe UI', sans-serif; background-color: #ffffff; }
            .section-box { 
                border: 1px solid #e2e8f0; 
                border-radius: 12px; 
                margin-bottom: 25px; 
                background: #ffffff;
                box-shadow: 0 4px 6px -1px rgba(0,0,0,0.05);
                overflow: hidden;
            }
            .section-head { 
                background: #1e3a8a; 
                color: #ffffff; 
                font-weight: 700; 
                font-size: 16px; 
                padding: 12px 16px; 
            }
            .pos-container { 
                padding: 24px; 
                background: #f8fafc;
                display: flex;
                justify-content: space-around;
                align-items: center;
            }
            .pos-item { 
                font-size: 28px; 
                color: #1e40af; 
                font-family: 'Consolas', monospace; 
                font-weight: 700;
            }
            .pos-label { 
                color: #94a3b8; 
                font-size: 14px; 
                font-weight: 600; 
                text-transform: uppercase;
                margin-right: 8px;
            }
            
            .matrix-table {
                width: 100%;
                border-collapse: collapse;
                background: #f8fafc;
            }
            .matrix-table td {
                padding: 12px;
                text-align: center;
                font-family: 'Consolas', monospace;
                font-size: 16px;
                color: #1e293b;
                border: 1px solid #e2e8f0;
            }
            .matrix-table tr:nth-child(even) td { background: #f1f5f9; }
            
            .dh-table { 
                width: 100%; 
                border-collapse: collapse; 
            }
            .dh-table th { 
                background: #f1f5f9; 
                color: #475569; 
                font-weight: 800; 
                padding: 12px;
                border-bottom: 2px solid #e2e8f0;
                text-align: center;
                font-size: 13px;
                text-transform: uppercase;
            }
            .dh-table td { 
                padding: 12px; 
                text-align: center; 
                border-bottom: 1px solid #f1f5f9;
                font-family: 'Consolas', monospace;
                font-size: 15px;
                color: #1e293b;
            }
            .dh-table tr:nth-child(even) td { background: #fafafa; }
            .dh-table td.joint-name { 
                text-align: left; 
                font-weight: 700; 
                color: #1e3a8a;
                background: #eff6ff !important;
            }
        </style>
        """

        # 1. Live Point Position Section
        html += (
            f"<div class='section-box'>"
            f"<div class='section-head'>Live Point Position</div>"
            f"<div class='pos-container'>"
            f"<span class='pos-item'><span class='pos-label'>X:</span>{p[0]:.1f}</span>"
            f"<span class='pos-item'><span class='pos-label'>Y:</span>{p[1]:.1f}</span>"
            f"<span class='pos-item'><span class='pos-label'>Z:</span>{p[2]:.1f}</span>"
            f"</div>"
            f"</div>"
        )
        
        # 2. Transform Matrix Section
        html += (
            f"<div class='section-box'>"
            f"<div class='section-head'>Final Live Point Matrix (4x4)</div>"
            f"<table class='matrix-table'>"
        )
        for i in range(4):
            html += "<tr>"
            for j in range(4):
                val = T_matrix[i, j]
                # Format translation differently (with 3 decimal places) vs rotation (with 4)
                if j == 3 and i < 3:
                    html += f"<td>{val:8.1f}</td>"
                else:
                    html += f"<td>{val:8.1f}</td>"
            html += "</tr>"
        html += "</table></div>"

        # 3. DH Matrix (Parameters Table) Section
        html += (
            f"<div class='section-box'>"
            f"<div class='section-head'>DH Matrix (Parameters)</div>"
            f"<table class='dh-table'>"
            f"<tr><th>Joint</th><th>&theta; (deg)</th><th>d (cm)</th><th>a (cm)</th><th>&alpha; (deg)</th></tr>"
        )

        if fk_results:
            for res in fk_results:
                is_prismatic = res["joint_type"] == "prismatic"
                theta = res["theta0_deg"] + (0 if is_prismatic else res["q_value"])
                d = res["d"] + (res["q_value"] if is_prismatic else 0)

                html += (
                    f"<tr>"
                    f"<td class='joint-name'>{res['title']}</td>"
                    f"<td>{theta:.2f}</td>"
                    f"<td>{d:.2f}</td>"
                    f"<td>{res['a']:.2f}</td>"
                    f"<td>{res['alpha_deg']:.2f}</td>"
                    f"</tr>"
                )
        else:
            html += (
                f"<tr>"
                f"<td class='joint-name' colspan='5' style='text-align:center; padding:20px;'>No FK data available</td>"
                f"</tr>"
            )

        html += "</table></div>"

        self.result_view.setHtml(html)
