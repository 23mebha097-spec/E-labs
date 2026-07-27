import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5 import QtWidgets
import numpy as np

from ui.panels.experiment_panel import ExperimentPanel
from ui.panels.matrices_panel import MatricesPanel
from ui.panels.result_panel import ResultPanel


def test_experiment_safe_call_logs_panel_errors():
    messages = []
    panel = SimpleNamespace(mw=SimpleNamespace(log=messages.append))
    broken_panel = SimpleNamespace(
        __class__=SimpleNamespace(__name__="BrokenPanel"),
        update_display=lambda: (_ for _ in ()).throw(RuntimeError("boom")),
    )

    ExperimentPanel._safe_call(panel, broken_panel, "update_display")

    assert messages
    assert "update_display failed: boom" in messages[0]


def test_matrices_slider_refresh_accepts_string_axis_metadata():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    mw = SimpleNamespace(
        robot=SimpleNamespace(joint_relations={}),
        joint_tab=SimpleNamespace(
            joints={
                "link_2": {
                    "parent": "link_1",
                    "axis": "Y",
                    "min": -90,
                    "max": 90,
                    "current_angle": 15,
                    "custom_name": "Shoulder",
                }
            }
        ),
    )
    panel = MatricesPanel(mw)

    panel.refresh_sliders()

    assert "link_2" in panel.sliders


def test_result_panel_uses_tcp_matrix_when_live_point_is_pending():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    tcp_link = SimpleNamespace()
    tcp_pose = np.eye(4)
    tcp_pose[:3, 3] = [10.0, 20.0, 30.0]
    mw = SimpleNamespace(
        current_live_point_cm=None,
        canvas=SimpleNamespace(grid_units_per_cm=10.0),
        _get_preferred_tcp_link=lambda: tcp_link,
        robot=SimpleNamespace(get_tcp_world_pose=lambda _: tcp_pose),
    )
    panel = ResultPanel(mw)

    panel.update_display()

    text = panel.result_view.toPlainText()
    assert "X:1.0" in text
    assert "Y:2.0" in text
    assert "Z:3.0" in text
