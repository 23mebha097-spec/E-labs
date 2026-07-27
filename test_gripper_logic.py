#!/usr/bin/env python
"""Test gripper panel button handlers."""

import os
import sys
from types import SimpleNamespace

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)


from core.gripper_contact import GripperContactAnalyzer
from core.pick_place import PickPlaceExecutor
from core.robot import Robot
from ui.panels.gripper_panel import GripperPanel


def test_gripper_tool_ui_visibility():
    from PyQt5 import QtWidgets

    robot = Robot()
    analyzer = GripperContactAnalyzer(robot)
    executor = PickPlaceExecutor(robot, canvas=None, contact_analyzer=analyzer)
    assert executor is not None

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    mw = SimpleNamespace(
        robot=robot,
        log=lambda message: None,
        show_toast=lambda *args, **kwargs: None,
        active_gripper_joint_names=[],
        active_gripper_joint_name=None,
        joint_tab=None,
        experiment_tab=None,
        simulation_tab=None,
    )

    panel = GripperPanel(mw)
    panel.show()
    app.processEvents()

    panel.tool_combo.setCurrentText("Gripper Tool")
    panel.on_select_tool_ok()
    app.processEvents()

    assert panel.selection_group.isVisible(), "Gripper Tool should expose the joint selection workflow"
    assert panel.gripper_compile_btn.isVisible(), "Compile button should be visible for Gripper Tool"
    assert panel.control_group.isVisible(), "The shared manual gripper control should be visible"
    assert panel.stroke_slider.minimum() == 0
    assert panel.stroke_slider.maximum() == 100


def test_gripper_joint_list_contains_all_joint_names():
    from PyQt5 import QtWidgets

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    robot = Robot()
    robot.joint_relations = {"J1": [("J2", 0.5)]}
    robot.joints = {
        "J1": SimpleNamespace(
            parent_link=SimpleNamespace(name="base"),
            child_link=SimpleNamespace(name="link_1"),
            is_gripper=False,
            min_limit=0.0,
            max_limit=90.0,
            current_value=0.0,
        ),
        "J2": SimpleNamespace(
            parent_link=SimpleNamespace(name="link_1"),
            child_link=SimpleNamespace(name="link_2"),
            is_gripper=False,
            min_limit=0.0,
            max_limit=90.0,
            current_value=0.0,
        ),
        "J3": SimpleNamespace(
            parent_link=SimpleNamespace(name="link_2"),
            child_link=SimpleNamespace(name="link_3"),
            is_gripper=False,
            min_limit=0.0,
            max_limit=90.0,
            current_value=0.0,
        ),
    }

    mw = SimpleNamespace(robot=robot, log=lambda message: None, show_toast=lambda *args, **kwargs: None)
    panel = GripperPanel(mw)
    panel.refresh_joints()

    displayed_names = [panel.joints_list.item(index).text() for index in range(panel.joints_list.count())]
    assert displayed_names == ["J1", "J2", "J3"], "the joint selector should list only independent joint names"
    assert not any(entry.startswith("Single ") for entry in displayed_names)


def test_gripper_slider_moves_selected_joint():
    from PyQt5 import QtWidgets

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    robot = Robot()
    link = SimpleNamespace(name="link_1", child_joints=[])
    robot.joints = {
        "J1": SimpleNamespace(
            parent_link=SimpleNamespace(name="base"),
            child_link=link,
            is_gripper=True,
            min_limit=0.0,
            max_limit=90.0,
            current_value=0.0,
        )
    }
    robot.joint_relations = {}
    robot.update_kinematics = lambda: None

    mw = SimpleNamespace(
        robot=robot,
        canvas=SimpleNamespace(update_transforms=lambda robot: None),
        log=lambda message: None,
        show_toast=lambda *args, **kwargs: None,
        serial_mgr=SimpleNamespace(is_connected=False),
    )

    panel = GripperPanel(mw)
    panel.show()
    app.processEvents()
    panel.tool_combo.setCurrentText("Gripper Tool")
    panel.on_select_tool_ok()
    panel.refresh_joints()
    panel.joints_list.setCurrentRow(0)
    item = panel.joints_list.currentItem()
    panel.on_joint_selected(item)
    panel.gripper_max_input.setValue(45)

    assert robot.joints["J1"].current_value > 0.0, "typing the opening angle should rotate the selected joint"


def test_gripper_opening_inputs_use_degree_fields():
    from PyQt5 import QtWidgets

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    robot = Robot()
    robot.joints = {
        "J1": SimpleNamespace(
            parent_link=SimpleNamespace(name="base"),
            child_link=SimpleNamespace(name="link_1", child_joints=[]),
            is_gripper=True,
            min_limit=0.0,
            max_limit=90.0,
            current_value=0.0,
        )
    }
    robot.joint_relations = {}
    robot.update_kinematics = lambda: None

    mw = SimpleNamespace(
        robot=robot,
        canvas=SimpleNamespace(update_transforms=lambda robot: None),
        log=lambda message: None,
        show_toast=lambda *args, **kwargs: None,
        serial_mgr=SimpleNamespace(is_connected=False),
    )

    panel = GripperPanel(mw)
    panel.show()
    app.processEvents()
    panel.tool_combo.setCurrentText("Gripper Tool")
    panel.on_select_tool_ok()
    app.processEvents()

    assert hasattr(panel, "gripper_min_input"), "Gripper Tool should expose a typed Min degree input"
    assert hasattr(panel, "gripper_max_input"), "Gripper Tool should expose a typed Max degree input"
    assert not hasattr(panel, "gripper_min_slider"), "The old slider-only input should be replaced by typed degree fields"
    assert not hasattr(panel, "gripper_max_slider"), "The old slider-only input should be replaced by typed degree fields"


def test_gripper_endpoint_calculation_uses_both_jaws_and_supports_mirrored_motion():
    import numpy as np
    from PyQt5 import QtWidgets

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    jaw_1 = SimpleNamespace(name="jaw_1", child_joints=[], t_world=np.eye(4))
    jaw_2 = SimpleNamespace(name="jaw_2", child_joints=[], t_world=np.eye(4))
    joint_1 = SimpleNamespace(
        parent_link=SimpleNamespace(name="hand"), child_link=jaw_1,
        is_gripper=True, min_limit=0.0, max_limit=90.0, current_value=0.0,
    )
    joint_2 = SimpleNamespace(
        parent_link=SimpleNamespace(name="hand"), child_link=jaw_2,
        is_gripper=True, min_limit=0.0, max_limit=90.0, current_value=0.0,
    )
    robot = SimpleNamespace(joints={"j6": joint_1, "j7": joint_2}, joint_relations={})

    def update_kinematics():
        jaw_1.t_world = np.eye(4)
        jaw_2.t_world = np.eye(4)
        jaw_1.t_world[0, 3] = -2.0 + joint_1.current_value / 60.0
        jaw_2.t_world[0, 3] = 0.5 + joint_2.current_value / 60.0

    robot.update_kinematics = update_kinematics
    mw = SimpleNamespace(
        robot=robot,
        log=lambda message: None,
        show_toast=lambda *args, **kwargs: None,
    )
    panel = GripperPanel(mw)
    panel.gripper_min_input.setValue(0)
    panel.gripper_max_input.setValue(90)
    panel._gripper_face_selection_data = {
        "j6": {"local_center": np.zeros(3)},
        "j7": {"local_center": np.zeros(3)},
    }

    endpoints = panel._calculate_gripper_endpoint_angles(["j6", "j7"])

    assert endpoints == {
        "j6": {"closed": 90.0, "open": 0.0},
        "j7": {"closed": 0.0, "open": 90.0},
    }
    assert joint_1.current_value == 0.0
    assert joint_2.current_value == 0.0


def test_selected_jaw_midpoint_is_bound_to_the_rigid_gripper_flange():
    import numpy as np
    from PyQt5 import QtWidgets

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    robot = Robot()
    robot.add_link("base")
    hand = robot.add_link("hand")
    jaw_1 = robot.add_link("jaw_1")
    jaw_2 = robot.add_link("jaw_2")
    robot.base_link = robot.links["base"]
    robot.add_joint("arm", "base", "hand")
    joint_1 = robot.add_joint("grip_1", "hand", "jaw_1")
    joint_2 = robot.add_joint("grip_2", "hand", "jaw_2")
    joint_1.is_gripper = True
    joint_2.is_gripper = True
    jaw_1.t_offset[0, 3] = -2.0
    jaw_2.t_offset[0, 3] = 2.0
    robot.update_kinematics()

    mw = SimpleNamespace(
        robot=robot,
        custom_tcp_name=None,
        log=lambda message: None,
        show_toast=lambda *args, **kwargs: None,
        _resolve_rigid_tcp_link=lambda link: link.parent_joint.parent_link,
    )
    panel = GripperPanel(mw)
    panel._gripper_face_selection_data = {
        "grip_1": {"local_center": np.zeros(3)},
        "grip_2": {"local_center": np.zeros(3)},
    }

    tcp_link = panel._bind_gripper_live_point_to_flange(["grip_1", "grip_2"])

    assert tcp_link is hand
    assert mw.custom_tcp_name == "hand"
    np.testing.assert_allclose(hand.custom_tcp_offset, [0.0, 0.0, 0.0])


def test_saved_gripper_jaw_centroid_restores_the_live_tcp():
    import numpy as np

    from ui.mixins.navigation_mixin import NavigationMixin

    robot = Robot()
    base = robot.add_link("base")
    hand = robot.add_link("hand")
    jaw_1 = robot.add_link("jaw_1")
    jaw_2 = robot.add_link("jaw_2")
    robot.base_link = base
    robot.add_joint("arm", "base", "hand")
    robot.add_joint("grip_1", "hand", "jaw_1")
    robot.add_joint("grip_2", "hand", "jaw_2")
    jaw_1.t_offset[0, 3] = 0.0
    jaw_2.t_offset[0, 3] = 0.0
    robot.update_kinematics()

    midpoint_local = np.array([3.0, 0.0, 0.0])
    payload = {
        "EndEffector": {
            "ToolType": "Gripper Tool",
            "TCPLink": "hand",
            "Jaws": [
                {
                    "JointID": "grip_1",
                    "FaceCenterLocal": [1.0, 0.0, 0.0],
                },
                {
                    "JointID": "grip_2",
                    "FaceCenterLocal": [5.0, 0.0, 0.0],
                },
            ],
            "BaseAlignmentFace": {
                "LinkID": "hand",
                "TCPLink": "hand",
                "FaceCenterTCPLocal": [9.0, 9.0, 9.0],
                "FaceNormalTCPLocal": [0.0, 0.0, -1.0],
                "FaceCenterLinkLocal": [9.0, 9.0, 9.0],
            },
        }
    }

    class Harness(NavigationMixin):
        def __init__(self):
            self.robot = robot
            self.gripper_tool_config = payload
            self.end_effector_tool_config = payload
            self.active_gripper_joint_names = []
            self.custom_tcp_name = None

    harness = Harness()
    restored_link = harness.ensure_saved_gripper_tcp()

    assert restored_link is hand
    assert harness.custom_tcp_name == "hand"
    np.testing.assert_allclose(hand.custom_tcp_offset, midpoint_local)
    np.testing.assert_allclose(
        robot.get_tcp_world_pose(hand)[:3, 3],
        midpoint_local,
    )


def test_compile_auto_detects_inward_contact_faces_without_manual_clicks():
    import numpy as np
    import trimesh
    from PyQt5 import QtCore, QtWidgets

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    robot = Robot()
    robot.add_link("hand")
    jaw_1 = robot.add_link("left_jaw", trimesh.creation.box(extents=[1.0, 2.0, 3.0]))
    jaw_2 = robot.add_link("right_jaw", trimesh.creation.box(extents=[1.0, 2.0, 3.0]))
    robot.base_link = robot.links["hand"]
    joint_1 = robot.add_joint("left_grip", "hand", "left_jaw")
    joint_2 = robot.add_joint("right_grip", "hand", "right_jaw")
    for joint in (joint_1, joint_2):
        joint.min_limit = 0.0
        joint.max_limit = 20.0
    jaw_1.t_offset[0, 3] = -2.0
    jaw_2.t_offset[0, 3] = 2.0
    robot.update_kinematics()

    face_pick_requests = []
    mw = SimpleNamespace(
        robot=robot,
        active_gripper_joint_names=[],
        active_gripper_joint_name=None,
        canvas=SimpleNamespace(
            update_transforms=lambda robot: None,
            start_face_picking=lambda callback, **kwargs: face_pick_requests.append(
                (callback, kwargs)
            ),
        ),
        log=lambda message: None,
        show_toast=lambda *args, **kwargs: None,
    )
    panel = GripperPanel(mw)
    panel.tool_combo.setCurrentText("Gripper Tool")
    panel.on_select_tool_ok()
    panel.refresh_joints()
    for index in range(panel.joints_list.count()):
        item = panel.joints_list.item(index)
        if item.data(QtCore.Qt.UserRole) in ("left_grip", "right_grip"):
            item.setSelected(True)

    panel.on_gripper_compile_clicked()

    left = panel._gripper_face_selection_data["left_grip"]
    right = panel._gripper_face_selection_data["right_grip"]
    assert left is not None
    assert right is not None
    assert left["world_center"][0] > jaw_1.t_world[0, 3]
    assert right["world_center"][0] < jaw_2.t_world[0, 3]
    assert left["world_normal"][0] > 0.0
    assert right["world_normal"][0] < 0.0
    assert not panel.gripper_save_btn.isEnabled()

    row_by_joint = {
        panel.face_selection_table.item(row, 0).text(): row
        for row in range(panel.face_selection_table.rowCount())
    }
    assert set(row_by_joint) == {"left_grip", "right_grip"}
    for row in row_by_joint.values():
        face_cell = panel.face_selection_table.cellWidget(row, 1)
        assert face_cell is not None
        assert face_cell.findChild(QtWidgets.QPushButton).text() == "Select Face"

    left_cell = panel.face_selection_table.cellWidget(
        row_by_joint["left_grip"], 1
    )
    left_cell.findChild(QtWidgets.QPushButton).click()
    assert panel._pending_gripper_contact_joint_name == "left_grip"
    assert len(face_pick_requests) == 1
    callback, options = face_pick_requests.pop()
    assert options["color"] == "cyan"
    callback(
        "left_jaw",
        world_center=jaw_1.t_world[:3, 3] + np.array([0.5, 0.0, 0.0]),
        world_normal=[1.0, 0.0, 0.0],
    )
    assert panel._pending_gripper_contact_joint_name is None
    assert (
        panel._gripper_face_selection_data["left_grip"]["surface_name"]
        == "Manual Contact Face"
    )

    panel._on_gripper_alignment_face_selected(
        "hand",
        world_center=[1.0, 2.0, 3.0],
        world_normal=[0.0, 0.0, -1.0],
    )
    assert panel.gripper_save_btn.isEnabled()
    assert mw.custom_tcp_name == "hand"
    expected_midpoint = (
        np.asarray(panel._gripper_face_selection_data["left_grip"]["world_center"], dtype=float)
        + np.asarray(panel._gripper_face_selection_data["right_grip"]["world_center"], dtype=float)
    ) / 2.0
    np.testing.assert_allclose(
        robot.get_tcp_world_pose(robot.links["hand"])[:3, 3],
        expected_midpoint,
    )
    assert "face assigned for left_grip" in panel.gripper_face_status.text().lower()
    payload = panel._build_end_effector_payload()
    alignment = payload["EndEffector"]["BaseAlignmentFace"]
    assert alignment["LinkID"] == "hand"
    assert alignment["TCPLink"] == "hand"
    np.testing.assert_allclose(payload["EndEffector"]["LivePoint"], expected_midpoint)
    np.testing.assert_allclose(alignment["FaceCenterTCPLocal"], [0.0, 0.0, 0.0])
    np.testing.assert_allclose(
        alignment["FaceCenterLinkLocal"], expected_midpoint
    )
    np.testing.assert_allclose(alignment["FaceNormalTCPLocal"], [0.0, 0.0, -1.0])


def test_endpoint_calculation_maps_positive_range_to_negative_relation_slave():
    import numpy as np
    from PyQt5 import QtWidgets

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    left_link = SimpleNamespace(name="left_jaw", child_joints=[], t_world=np.eye(4))
    right_link = SimpleNamespace(name="right_jaw", child_joints=[], t_world=np.eye(4))
    master = SimpleNamespace(
        parent_link=SimpleNamespace(name="hand"), child_link=left_link,
        is_gripper=True, min_limit=0.0, max_limit=90.0, current_value=0.0,
    )
    slave = SimpleNamespace(
        parent_link=SimpleNamespace(name="hand"), child_link=right_link,
        is_gripper=True, min_limit=-90.0, max_limit=0.0, current_value=0.0,
    )
    robot = SimpleNamespace(
        joints={"master": master, "slave": slave},
        joint_relations={"master": [("slave", -1.0)]},
    )

    def update_kinematics():
        left_link.t_world = np.eye(4)
        right_link.t_world = np.eye(4)
        left_link.t_world[0, 3] = -2.0 + master.current_value / 45.0
        right_link.t_world[0, 3] = 2.0 + slave.current_value / 45.0

    robot.update_kinematics = update_kinematics
    mw = SimpleNamespace(
        robot=robot,
        log=lambda message: None,
        show_toast=lambda *args, **kwargs: None,
    )
    panel = GripperPanel(mw)
    panel.gripper_min_input.setValue(0)
    panel.gripper_max_input.setValue(45)
    panel._gripper_face_selection_data = {
        "master": {"local_center": np.zeros(3)},
        "slave": {"local_center": np.zeros(3)},
    }

    endpoints = panel._calculate_gripper_endpoint_angles(["master", "slave"])

    assert endpoints["master"] == {"closed": 45.0, "open": 0.0}
    assert endpoints["slave"] == {"closed": -45.0, "open": 0.0}
    assert endpoints["master"]["closed"] != endpoints["master"]["open"]
    assert endpoints["slave"]["closed"] != endpoints["slave"]["open"]


def test_pick_place_executor_geometric_grip_uses_saved_end_effector_payload():
    import numpy as np
    import trimesh
    from types import SimpleNamespace

    robot = Robot()
    robot.joints = {}
    robot.joint_relations = {}

    gripper_mesh = trimesh.creation.box(extents=[0.5, 0.5, 1.0])
    gripper_link = SimpleNamespace(name="gripper_tip", mesh=gripper_mesh, t_world=np.eye(4), child_joints=[])
    gripper_joint = SimpleNamespace(
        child_link=gripper_link,
        is_gripper=True,
        min_limit=0.0,
        max_limit=90.0,
        current_value=0.0,
    )
    robot.joints["JAW1"] = gripper_joint

    object_mesh = trimesh.creation.box(extents=[0.8, 0.8, 0.8])
    target_link = SimpleNamespace(name="object", mesh=object_mesh, t_world=np.eye(4), child_joints=[])

    payload = {
        "EndEffector": {
            "ToolType": "Gripper Tool",
            "LivePoint": [0.0, 0.0, 0.0],
            "MinOpening": 0,
            "MaxOpening": 40,
            "JawCount": 1,
            "Jaws": [
                {
                    "JointID": "JAW1",
                    "FaceID": "gripper_tip",
                    "FaceCenter": [0.0, 0.0, 0.5],
                    "FaceNormal": [0.0, 0.0, -1.0],
                }
            ],
        }
    }

    executor = PickPlaceExecutor(
        robot,
        canvas=None,
        contact_analyzer=GripperContactAnalyzer(robot),
        end_effector_config=payload,
    )

    bounds = executor.compute_object_bounds(target_link)
    assert bounds["world_min"] is not None
    assert bounds["world_max"] is not None

    grip_result = executor.geometric_grip(target_link, gripper_joint="JAW1")
    assert grip_result["success"] is True, grip_result
    assert grip_result["contact_count"] == 1
