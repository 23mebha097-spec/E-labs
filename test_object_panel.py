import os
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5 import QtCore, QtWidgets

from core.robot import Robot
from ui.panels.object_panel import ObjectPanel


def test_main_object_menu_lists_and_selects_scene_objects():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    robot = Robot()
    robot.add_link("robot_base")
    cube = robot.add_link("ob_1")
    cylinder = robot.add_link("ob_2")
    cube.is_sim_obj = True
    cylinder.is_sim_obj = True
    cube.import_metadata = {"object_type": "cube"}
    cylinder.import_metadata = {"object_type": "cylinder"}

    selected_names = []
    mw = SimpleNamespace(
        robot=robot,
        canvas=SimpleNamespace(
            select_actor=selected_names.append,
        ),
        simulation_tab=None,
    )
    panel = ObjectPanel(mw)

    assert panel.main_objects_list.count() == 2
    assert panel.objects_list.count() == 2
    assert panel.main_objects_list.item(0).text() == "1 - cube (ob_1)"
    assert panel.main_objects_list.item(1).text() == "2 - cylinder (ob_2)"
    assert panel.main_objects_list.item(0).data(QtCore.Qt.UserRole) == "ob_1"

    panel.on_object_item_clicked(panel.main_objects_list.item(1))

    assert selected_names == ["ob_2"]
    assert panel._selected_object_name() == "ob_2"
    assert (
        panel.objects_list.currentItem().data(QtCore.Qt.UserRole)
        == "ob_2"
    )
