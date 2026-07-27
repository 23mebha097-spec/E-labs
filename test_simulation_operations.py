import os
from types import SimpleNamespace

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5 import QtWidgets

from ui.panels.simulation_panel import SimulationPanel
from ui.panels.program_panel import ProgramPanel
from ui.panels.gripper_panel import GripperPanel
from ui.mixins.navigation_mixin import NavigationMixin
from core.robot import Robot


def _main_window_stub():
    return SimpleNamespace(
        robot=SimpleNamespace(links={}, joints={}, joint_relations={}),
        import_mesh=lambda: None,
        on_sim_object_clicked=lambda item: None,
        save_sim_object_coords=lambda: None,
        refresh_sim_objects_list=lambda: None,
    )


def test_internal_object_task_controller_supports_all_three_operations():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = SimulationPanel(_main_window_stub())

    assert panel.stack.currentWidget() is panel.objects_view
    assert [panel.operation_combo.itemData(i) for i in range(panel.operation_combo.count())] == [
        "pick_place",
        "welding",
        "painting",
    ]

    panel.operation_combo.setCurrentIndex(1)
    assert panel.pick_place_btn.text() == "Run Welding"
    assert panel.process_points_sb.isVisibleTo(panel)

    panel.operation_combo.setCurrentIndex(2)
    assert panel.pick_place_btn.text() == "Run Painting"
    assert panel.paint_color_combo.isVisibleTo(panel)


def test_surface_path_contains_endpoints_and_even_intermediate_points():
    path = SimulationPanel._build_surface_path([0, 0, 0], [9, 3, 6], 4)

    assert len(path) == 4
    np.testing.assert_allclose(path[0], [0, 0, 0])
    np.testing.assert_allclose(path[1], [3, 1, 2])
    np.testing.assert_allclose(path[-1], [9, 3, 6])


def test_simulation_import_flag_is_scoped_to_import_call():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    mw = _main_window_stub()
    observed = []
    mw.import_mesh = lambda: observed.append(mw._simulation_object_import_active)
    panel = SimulationPanel(mw)

    panel.import_simulation_object()

    assert observed == [True]
    assert mw._simulation_object_import_active is False


def test_welding_operation_starts_surface_state_machine_with_saved_tool():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    mw = _main_window_stub()
    mw.robot.links["ob_1"] = SimpleNamespace(name="ob_1")
    mw.welding_tool_config = {"EndEffector": {"ToolType": "Welding Tool"}}
    mw.end_effector_tool_config = mw.welding_tool_config
    mw.log = lambda message: None
    mw.show_toast = lambda *args, **kwargs: None
    panel = SimulationPanel(mw)
    panel._get_tcp_link = lambda: SimpleNamespace(name="weld_tcp")
    panel.objects_list.addItem("ob_1")
    panel.objects_list.setCurrentRow(0)
    panel.pick_x.setValue(1.0)
    panel.place_x.setValue(9.0)

    panel.run_surface_operation("welding")

    assert panel.is_sim_active is True
    assert panel.active_operation == "welding"
    assert panel.sim_state == "SOLVE_PROCESS_APPROACH"
    assert len(panel._process_path_cm) == panel.process_points_sb.value()
    panel.sim_timer.stop()


def test_code_panel_parses_all_object_operation_scripts():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = ProgramPanel(SimpleNamespace(log=lambda message: None))

    pick = panel._parse_task_script(
        "robot.operation_pick_and_place\n"
        "robot.end_effector = gripper_tool\n"
        "obj = 1\n"
        "Px, Py, Pz: 20, 10, 5\n"
    )
    weld = panel._parse_task_script(
        "robot.operation_welding\n"
        "robot.end_effector = welding_tool\n"
        "obj = 2\n"
        "P1: 1, 2, 3\n"
        "P2: 8, 2, 3\n"
        "path_points = 12\n"
    )
    paint = panel._parse_task_script(
        "robot.operation_painting\n"
        "robot.end_effector = painting_tool\n"
        "obj = 3\n"
        "paint_color = signal_blue\n"
    )

    assert pick["operation"] == "pick_and_place"
    assert pick["place_pos"] == (20, 10, 5)
    assert weld["operation"] == "welding"
    assert weld["start_pos"] == (1, 2, 3)
    assert weld["place_pos"] == (8, 2, 3)
    assert weld["path_points"] == 12
    assert paint["operation"] == "painting"
    assert paint["paint_color"] == "signal_blue"


def test_delete_end_effector_clears_saved_gripper_and_restores_tool_selection():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    robot = Robot()
    robot.add_link("base")
    robot.add_link("jaw_1")
    robot.add_link("jaw_2")
    robot.base_link = robot.links["base"]
    joint_1 = robot.add_joint("j1", "base", "jaw_1")
    joint_2 = robot.add_joint("j2", "base", "jaw_2")
    joint_1.is_gripper = True
    joint_2.is_gripper = True
    payload = {
        "EndEffector": {
            "ToolType": "Gripper Tool",
            "Jaws": [{"JointID": "j1"}, {"JointID": "j2"}],
        }
    }
    messages = []
    mw = SimpleNamespace(
        robot=robot,
        gripper_tool_config=payload,
        end_effector_tool_config=payload,
        custom_tcp_name="jaw_1",
        locked_live_point_link_name="jaw_1",
        locked_live_point_local=np.zeros(3),
        live_point_locked=False,
        locked_live_point=None,
        robot_finalized=True,
        active_gripper_joint_names=["j1", "j2"],
        active_gripper_joint_name="j1",
        simulation_tab=None,
        joint_tab=None,
        canvas=SimpleNamespace(
            clear_live_point_marker=lambda: None,
            clear_live_tcp_marker=lambda: None,
        ),
        update_live_ui=lambda **kwargs: None,
        log=messages.append,
        show_toast=lambda *args, **kwargs: None,
    )
    panel = GripperPanel(mw)
    panel._set_gripper_confirmation_mode(True)

    panel.on_delete_end_effector()

    assert mw.end_effector_tool_config is None
    assert mw.gripper_tool_config is None
    assert joint_1.is_gripper is False
    assert joint_2.is_gripper is False
    assert mw.custom_tcp_name is None
    assert mw.robot_finalized is False
    assert panel.tool_selection_status.text() == "No tool selected"
    assert panel.end_effector_summary_group.isHidden()
    assert panel.make_robo_btn.isHidden()
    assert messages and "End-effector deleted" in messages[-1]


def test_saved_independent_gripper_joints_open_and_close_their_child_jaws():
    class Harness(NavigationMixin):
        pass

    robot = Robot()
    robot.add_link("hand")
    jaw_1 = robot.add_link("jaw_1")
    jaw_2 = robot.add_link("jaw_2")
    robot.base_link = robot.links["hand"]
    joint_1 = robot.add_joint("grip_1", "hand", "jaw_1")
    joint_2 = robot.add_joint("grip_2", "hand", "jaw_2")
    for joint in (joint_1, joint_2):
        joint.min_limit = 0.0
        joint.max_limit = 90.0
        joint.is_gripper = True
    payload = {
        "EndEffector": {
            "ToolType": "Gripper Tool",
            "MinOpening": 5,
            "MaxOpening": 45,
            "Jaws": [{"JointID": "grip_1"}, {"JointID": "grip_2"}],
        }
    }
    harness = Harness()
    harness.robot = robot
    harness.gripper_tool_config = payload
    harness.end_effector_tool_config = payload
    harness.active_gripper_joint_names = ["grip_1", "grip_2"]
    harness.canvas = SimpleNamespace(update_transforms=lambda robot: None)

    open_targets = harness._control_gripper_fingers(close=False, apply=False)
    close_targets = harness._control_gripper_fingers(close=True, apply=False)
    harness._control_gripper_fingers(close=False, apply=True)

    assert open_targets == {"grip_1": 45.0, "grip_2": 45.0}
    assert close_targets == {"grip_1": 5.0, "grip_2": 5.0}
    assert joint_1.current_value == 45.0
    assert joint_2.current_value == 45.0
    assert jaw_1.parent_joint is joint_1
    assert jaw_2.parent_joint is joint_2


def test_shared_gripper_percentage_uses_each_jaws_mirrored_endpoints():
    class Harness(NavigationMixin):
        pass

    robot = Robot()
    robot.add_link("hand")
    robot.add_link("jaw_1")
    robot.add_link("jaw_2")
    robot.base_link = robot.links["hand"]
    joint_1 = robot.add_joint("j6", "hand", "jaw_1")
    joint_2 = robot.add_joint("j7", "hand", "jaw_2")
    for joint in (joint_1, joint_2):
        joint.min_limit = 0.0
        joint.max_limit = 90.0
        joint.is_gripper = True

    payload = {
        "EndEffector": {
            "ToolType": "Gripper Tool",
            "MinOpening": 0,
            "MaxOpening": 90,
            "Jaws": [
                {"JointID": "j6", "ClosedAngle": 10.0, "OpenAngle": 70.0},
                {"JointID": "j7", "ClosedAngle": 80.0, "OpenAngle": 20.0},
            ],
        }
    }
    harness = Harness()
    harness.robot = robot
    harness.gripper_tool_config = payload
    harness.end_effector_tool_config = payload
    harness.active_gripper_joint_names = ["j6", "j7"]
    harness.canvas = SimpleNamespace(update_transforms=lambda robot: None)

    midpoint_targets = harness.set_gripper_opening_percent(50.0, apply=False)
    harness.set_gripper_opening_percent(100.0)

    assert midpoint_targets == {"j6": 40.0, "j7": 50.0}
    assert joint_1.current_value == 70.0
    assert joint_2.current_value == 20.0
    assert harness.get_gripper_opening_percent() == 100.0


def test_shared_gripper_repairs_old_zero_span_relation_slave():
    class Harness(NavigationMixin):
        pass

    robot = Robot()
    robot.add_link("hand")
    robot.add_link("jaw_1")
    robot.add_link("jaw_2")
    robot.base_link = robot.links["hand"]
    master = robot.add_joint("j6", "hand", "jaw_1")
    slave = robot.add_joint("j7", "hand", "jaw_2")
    master.min_limit = 0.0
    master.max_limit = 90.0
    slave.min_limit = -90.0
    slave.max_limit = 0.0
    master.is_gripper = True
    slave.is_gripper = True
    robot.joint_relations = {"j6": [("j7", -1.0)]}

    payload = {
        "EndEffector": {
            "ToolType": "Gripper Tool",
            "MinOpening": 0.0,
            "MaxOpening": 45.0,
            "Jaws": [
                {"JointID": "j6", "ClosedAngle": 45.0, "OpenAngle": 0.0},
                {"JointID": "j7", "ClosedAngle": 0.0, "OpenAngle": 0.0},
            ],
        }
    }
    harness = Harness()
    harness.robot = robot
    harness.gripper_tool_config = payload
    harness.end_effector_tool_config = payload
    harness.active_gripper_joint_names = ["j6", "j7"]
    harness.canvas = SimpleNamespace(update_transforms=lambda robot: None)

    closed_targets = harness.set_gripper_opening_percent(0.0)
    assert closed_targets == {"j6": 45.0, "j7": -45.0}
    assert master.current_value == 45.0
    assert slave.current_value == -45.0

    open_targets = harness.set_gripper_opening_percent(100.0)
    assert open_targets == {"j6": 0.0, "j7": 0.0}
    assert master.current_value == 0.0
    assert slave.current_value == 0.0


def test_saved_jaw_faces_rebuild_the_runtime_tcp_on_the_flange():
    class Harness(NavigationMixin):
        pass

    robot = Robot()
    robot.add_link("base")
    hand = robot.add_link("hand")
    jaw_1 = robot.add_link("jaw_1")
    jaw_2 = robot.add_link("jaw_2")
    robot.base_link = robot.links["base"]
    robot.add_joint("arm", "base", "hand")
    grip_1 = robot.add_joint("grip_1", "hand", "jaw_1")
    grip_2 = robot.add_joint("grip_2", "hand", "jaw_2")
    grip_1.is_gripper = True
    grip_2.is_gripper = True
    jaw_1.t_offset[0, 3] = -2.0
    jaw_2.t_offset[0, 3] = 2.0
    robot.update_kinematics()
    payload = {
        "EndEffector": {
            "ToolType": "Gripper Tool",
            "Jaws": [
                {"JointID": "grip_1", "FaceCenterLocal": [0.0, 0.0, 0.0]},
                {"JointID": "grip_2", "FaceCenterLocal": [0.0, 0.0, 0.0]},
            ],
        }
    }
    harness = Harness()
    harness.robot = robot
    harness.gripper_tool_config = payload
    harness.end_effector_tool_config = payload
    harness.active_gripper_joint_names = ["grip_1", "grip_2"]
    harness.custom_tcp_name = None
    harness._resolve_rigid_tcp_link = lambda link: link.parent_joint.parent_link

    tcp_link = harness.ensure_saved_gripper_tcp()

    assert tcp_link is hand
    assert harness.custom_tcp_name == "hand"
    np.testing.assert_allclose(hand.custom_tcp_offset, [0.0, 0.0, 0.0])


def test_smooth_gripper_motion_does_not_deadlock_explicit_relation_jaws():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    class Harness(NavigationMixin):
        pass

    robot = Robot()
    robot.add_link("hand")
    robot.add_link("jaw_1")
    robot.add_link("jaw_2")
    robot.base_link = robot.links["hand"]
    master = robot.add_joint("jaw_master", "hand", "jaw_1")
    slave = robot.add_joint("jaw_slave", "hand", "jaw_2")
    master.min_limit = 0.0
    master.max_limit = 90.0
    slave.min_limit = -90.0
    slave.max_limit = 90.0
    robot.joint_relations = {"jaw_master": [("jaw_slave", -1.0)]}

    mw = Harness()
    mw.robot = robot
    mw.import_mesh = lambda: None
    mw.on_sim_object_clicked = lambda item: None
    mw.save_sim_object_coords = lambda: None
    mw.refresh_sim_objects_list = lambda: None
    panel = SimulationPanel(mw)
    panel.sim_state = "OPEN_GRIPPER"
    panel._target_gripper_angles = {
        "jaw_master": 45.0,
        "jaw_slave": 0.0,
    }

    completed = False
    for _ in range(30):
        completed = panel._move_gripper_smoothly()
        if completed:
            break

    assert completed is True
    assert master.current_value == 45.0
    assert slave.current_value == 0.0


def test_pick_contact_uses_selected_joint_child_jaws_on_opposing_sides():
    import trimesh

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    robot = Robot()
    robot.add_link("hand")
    jaw_1 = robot.add_link("jaw_1", trimesh.creation.box(extents=[0.4, 0.4, 0.4]))
    jaw_2 = robot.add_link("jaw_2", trimesh.creation.box(extents=[0.4, 0.4, 0.4]))
    target = robot.add_link("ob_1", trimesh.creation.box(extents=[1.0, 1.0, 1.0]))
    target.is_sim_obj = True
    robot.base_link = robot.links["hand"]
    joint_1 = robot.add_joint("grip_1", "hand", "jaw_1")
    joint_2 = robot.add_joint("grip_2", "hand", "jaw_2")
    joint_1.is_gripper = True
    joint_2.is_gripper = True
    robot.joint_relations = {"grip_1": [("grip_2", -1.0)]}
    jaw_1.t_offset[:3, 3] = [-0.6, 0.0, 0.0]
    jaw_2.t_offset[:3, 3] = [0.6, 0.0, 0.0]
    robot.update_kinematics()

    mw = _main_window_stub()
    mw.robot = robot
    mw.canvas = SimpleNamespace(grid_units_per_cm=10.0)
    mw._configured_gripper_joint_names = lambda: ["grip_1", "grip_2"]
    panel = SimulationPanel(mw)
    panel.objects_list.addItem("ob_1")
    panel.objects_list.setCurrentRow(0)

    contacts = panel._contacting_configured_gripper_joints()

    assert [link.name for link in panel._gripper_joint_child_links("grip_1")] == ["jaw_1"]
    assert [link.name for link in panel._gripper_joint_child_links("grip_2")] == ["jaw_2"]
    assert contacts == {"grip_1", "grip_2"}
    assert panel._object_is_between_jaws(target, contacts) is True


def test_pick_rejects_two_jaws_touching_the_same_object_side():
    import trimesh

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    robot = Robot()
    robot.add_link("hand")
    jaw_1 = robot.add_link("jaw_1", trimesh.creation.box(extents=[0.4, 0.4, 0.4]))
    jaw_2 = robot.add_link("jaw_2", trimesh.creation.box(extents=[0.4, 0.4, 0.4]))
    target = robot.add_link("ob_1", trimesh.creation.box(extents=[1.0, 1.0, 1.0]))
    target.is_sim_obj = True
    robot.base_link = robot.links["hand"]
    joint_1 = robot.add_joint("grip_1", "hand", "jaw_1")
    joint_2 = robot.add_joint("grip_2", "hand", "jaw_2")
    joint_1.is_gripper = True
    joint_2.is_gripper = True
    jaw_1.t_offset[:3, 3] = [-0.6, -0.2, 0.0]
    jaw_2.t_offset[:3, 3] = [-0.6, 0.2, 0.0]
    robot.update_kinematics()

    mw = _main_window_stub()
    mw.robot = robot
    mw.canvas = SimpleNamespace(grid_units_per_cm=10.0)
    mw._configured_gripper_joint_names = lambda: ["grip_1", "grip_2"]
    panel = SimulationPanel(mw)
    panel.objects_list.addItem("ob_1")
    panel.objects_list.setCurrentRow(0)

    contacts = panel._contacting_configured_gripper_joints()

    assert contacts == {"grip_1", "grip_2"}
    assert panel._object_is_between_jaws(target, contacts) is False


def test_known_cube_dimensions_are_read_from_object_metadata():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    mw = _main_window_stub()
    mw.canvas = SimpleNamespace(grid_units_per_cm=10.0)
    panel = SimulationPanel(mw)
    cube = SimpleNamespace(
        import_metadata={
            "source_type": "panel_primitive",
            "object_type": "cube",
            "final_size": [50.0, 40.0, 30.0],
        }
    )

    object_type, dimensions = panel._known_primitive_dimensions_world(cube)

    assert object_type == "cube"
    np.testing.assert_allclose(dimensions, [50.0, 40.0, 30.0])


def test_known_cylinder_uses_diameter_and_half_height_for_grip():
    import trimesh

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    cylinder = SimpleNamespace(
        name="ob_1",
        mesh=trimesh.creation.cylinder(radius=25.0, height=80.0),
        t_world=np.eye(4),
        import_metadata={
            "source_type": "panel_primitive",
            "object_type": "cylinder",
            "final_size": [50.0, 50.0, 80.0],
        },
    )
    messages = []
    mw = _main_window_stub()
    mw.robot.links["ob_1"] = cylinder
    mw.canvas = SimpleNamespace(grid_units_per_cm=10.0)
    mw.log = messages.append
    panel = SimulationPanel(mw)
    panel.objects_list.addItem("ob_1")
    panel.objects_list.setCurrentRow(0)

    grip_width, center_height, selected_link = panel._get_object_grip_width()

    assert selected_link is cylinder
    assert grip_width == pytest.approx(50.0)
    assert center_height == pytest.approx(40.0)
    assert any("known cylinder" in message.lower() for message in messages)


def test_cylinder_is_valid_between_two_opposing_contact_faces():
    import trimesh

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    robot = Robot()
    robot.add_link("hand")
    jaw_1 = robot.add_link(
        "jaw_1", trimesh.creation.box(extents=[0.4, 0.4, 0.4])
    )
    jaw_2 = robot.add_link(
        "jaw_2", trimesh.creation.box(extents=[0.4, 0.4, 0.4])
    )
    target = robot.add_link(
        "ob_1", trimesh.creation.cylinder(radius=0.5, height=1.0)
    )
    target.is_sim_obj = True
    target.import_metadata = {
        "source_type": "panel_primitive",
        "object_type": "cylinder",
        "final_size": [1.0, 1.0, 1.0],
    }
    robot.base_link = robot.links["hand"]
    joint_1 = robot.add_joint("grip_1", "hand", "jaw_1")
    joint_2 = robot.add_joint("grip_2", "hand", "jaw_2")
    joint_1.is_gripper = True
    joint_2.is_gripper = True
    jaw_1.t_offset[:3, 3] = [-0.6, 0.0, 0.0]
    jaw_2.t_offset[:3, 3] = [0.6, 0.0, 0.0]
    robot.update_kinematics()

    mw = _main_window_stub()
    mw.robot = robot
    mw.canvas = SimpleNamespace(grid_units_per_cm=10.0)
    mw._configured_gripper_joint_names = lambda: ["grip_1", "grip_2"]
    panel = SimulationPanel(mw)
    panel.objects_list.addItem("ob_1")
    panel.objects_list.setCurrentRow(0)

    contacts = panel._contacting_configured_gripper_joints()

    assert contacts == {"grip_1", "grip_2"}
    assert panel._object_is_between_jaws(target, contacts) is True


def test_selected_tool_face_orientation_aligns_with_object_base_plane():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    mw = _main_window_stub()
    mw.robot = SimpleNamespace(get_tcp_world_pose=lambda tcp_link: np.eye(4))
    mw.gripper_tool_config = {
        "EndEffector": {
            "ToolType": "Gripper Tool",
            "BaseAlignmentFace": {
                "LinkID": "gripper_palm",
                "FaceNormalTCPLocal": [1.0, 0.0, 0.0],
            },
        }
    }
    mw.log = lambda message: None
    panel = SimulationPanel(mw)
    tcp_link = SimpleNamespace(name="hand")
    obj_link = SimpleNamespace(t_world=np.eye(4))

    target_rotation = panel._build_pick_place_alignment_orientation(tcp_link, obj_link)

    aligned_normal = target_rotation @ np.array([1.0, 0.0, 0.0])
    object_base_normal = np.array([0.0, 0.0, -1.0])
    assert abs(float(np.dot(aligned_normal, object_base_normal))) > 1.0 - 1e-9
    np.testing.assert_allclose(target_rotation.T @ target_rotation, np.eye(3), atol=1e-9)
    assert np.linalg.det(target_rotation) == pytest.approx(1.0)


def test_pick_aligns_selected_tool_face_before_approaching_object():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    joint = SimpleNamespace(name="arm_1", current_value=0.0)
    current_pose = np.eye(4)
    current_pose[:3, 3] = [10.0, 20.0, 30.0]
    target_rotation = np.diag([1.0, -1.0, -1.0])
    observed = {}
    obj_link = SimpleNamespace(t_world=np.eye(4), mesh=None)

    class FakeRobot:
        joints = {"arm_1": joint}

        @staticmethod
        def get_tcp_world_pose(_tcp_link):
            return current_pose.copy()

        @staticmethod
        def inverse_kinematics_axis(
            target_position,
            _tcp_link,
            local_axis,
            target_axis,
            **kwargs,
        ):
            observed["target_position"] = np.asarray(target_position).copy()
            observed["local_axis"] = np.asarray(local_axis).copy()
            observed["target_axis"] = np.asarray(target_axis).copy()
            observed.update(kwargs)
            joint.current_value = 25.0
            solved_pose = target_rotation.copy()
            tcp_pose = np.eye(4)
            tcp_pose[:3, :3] = solved_pose
            tcp_pose[:3, 3] = target_position
            return True, {
                "position_error": 0.0,
                "axis_error": 0.0,
                "tcp_pose": tcp_pose,
            }

        @staticmethod
        def get_kinematic_chain(_tcp_link):
            return [joint]

        @staticmethod
        def update_kinematics():
            return None

    mw = _main_window_stub()
    mw.robot = FakeRobot()
    mw.canvas = SimpleNamespace(grid_units_per_cm=10.0)
    mw.gripper_tool_config = {
        "EndEffector": {
            "BaseAlignmentFace": {
                "LinkID": "gripper_palm",
                "FaceNormalTCPLocal": [0.0, 0.0, 1.0],
            }
        }
    }
    mw.get_link_tool_point = lambda *args, **kwargs: (
        np.zeros(3),
        np.zeros(3),
        0.0,
    )
    mw.log = lambda message: None
    mw.show_toast = lambda *args, **kwargs: None
    panel = SimulationPanel(mw)
    panel._pick_place_tcp_orientation = target_rotation
    panel._pick_place_original_object_rotation = np.eye(3)
    panel._get_object_grip_width = lambda: (0.0, 0.0, obj_link)
    tcp_link = SimpleNamespace(name="hand", t_world=np.eye(4))

    panel._solve_initial_gripper_alignment(tcp_link)

    np.testing.assert_allclose(observed["target_position"], [0.0, 0.0, 50.0])
    np.testing.assert_allclose(observed["local_axis"], [0.0, 0.0, 1.0])
    np.testing.assert_allclose(observed["target_axis"], [0.0, 0.0, -1.0])
    assert observed["axis_weight"] == 0.8
    assert panel.target_joint_values == {"arm_1": 25.0}
    assert joint.current_value == 0.0
    assert panel.sim_state == "MOVE_ALIGN_TOOL"


def test_pick_solver_generates_arm_targets_with_position_only_ik():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    joint = SimpleNamespace(name="arm_1", current_value=0.0, min_limit=-90.0, max_limit=90.0)
    tcp_link = SimpleNamespace(name="hand", t_world=np.eye(4))
    observed = {}

    class FakeRobot:
        joints = {"arm_1": joint}

        @staticmethod
        def get_tcp_world_pose(_tcp_link):
            return np.eye(4)

        @staticmethod
        def inverse_kinematics_pose(target_pose, _tcp_link, **kwargs):
            observed["target_pose"] = target_pose.copy()
            observed["solver"] = "pose"
            observed.update(kwargs)
            joint.current_value = 35.0
            return True, {"position_error": 0.0, "orientation_error": 0.0}

        @staticmethod
        def inverse_kinematics_axis(
            target_position,
            _tcp_link,
            local_axis,
            target_axis,
            **kwargs,
        ):
            observed["solver"] = "axis"
            observed["target_position"] = np.asarray(target_position).copy()
            observed["local_axis"] = np.asarray(local_axis).copy()
            observed["target_axis"] = np.asarray(target_axis).copy()
            observed.update(kwargs)
            joint.current_value = 40.0
            tcp_pose = np.eye(4)
            tcp_pose[:3, 3] = target_position
            return True, {
                "position_error": 0.0,
                "axis_error": 0.0,
                "tcp_pose": tcp_pose,
            }

        @staticmethod
        def get_kinematic_chain(_tcp_link):
            return [joint]

        @staticmethod
        def update_kinematics():
            return None

    messages = []
    mw = _main_window_stub()
    mw.robot = FakeRobot()
    mw.canvas = SimpleNamespace(grid_units_per_cm=10.0)
    mw.get_link_tool_point = lambda *args, **kwargs: (np.zeros(3), np.zeros(3), 0.0)
    mw.log = messages.append
    mw.show_toast = lambda *args, **kwargs: None
    panel = SimulationPanel(mw)

    panel._handle_state_solve(
        "P1",
        tcp_link,
        next_state="MOVE_APPROACH_P1",
        target_cm_override=[1.0, 2.0, 3.0],
        align_to_object=False,
    )

    np.testing.assert_allclose(observed["target_pose"][:3, 3], [10.0, 20.0, 30.0])
    assert observed["orientation_weight"] == 0.0
    assert observed["max_iters"] == 1500
    assert panel.target_joint_values["arm_1"] == 35.0
    assert joint.current_value == 0.0
    assert panel.sim_state == "MOVE_APPROACH_P1"

    alignment_rotation = np.diag([1.0, -1.0, -1.0])
    panel._pick_place_tcp_orientation = alignment_rotation
    panel._pick_place_original_object_rotation = np.eye(3)
    mw.gripper_tool_config = {
        "EndEffector": {
            "BaseAlignmentFace": {
                "LinkID": "gripper_palm",
                "FaceNormalTCPLocal": [1.0, 0.0, 0.0],
            }
        }
    }
    obj_link = SimpleNamespace(t_world=np.eye(4), mesh=None)
    panel._get_object_grip_width = lambda: (0.0, 0.0, obj_link)
    panel._handle_state_solve(
        "P1",
        tcp_link,
        next_state="MOVE_PICK_P1",
        target_cm_override=[1.0, 2.0, 3.0],
        align_to_object=True,
    )

    assert observed["solver"] == "axis"
    np.testing.assert_allclose(observed["target_position"], [10.0, 20.0, 30.0])
    np.testing.assert_allclose(observed["local_axis"], [1.0, 0.0, 0.0])
    np.testing.assert_allclose(observed["target_axis"], [0.0, 0.0, -1.0])
    assert observed["axis_weight"] == 0.8
    assert observed["axis_tolerance"] == pytest.approx(np.deg2rad(7.5))
    assert panel.target_joint_values["arm_1"] == 40.0


def test_imported_object_keeps_its_base_orientation_after_grip():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    object_link = SimpleNamespace(t_offset=np.eye(4))
    import_rotation = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ])
    object_link.import_metadata = {
        "import_world_rotation": import_rotation.tolist(),
    }
    tcp_pose = np.eye(4)
    tcp_pose[:3, :3] = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    tcp_pose[:3, 3] = [4.0, 5.0, 6.0]
    mw = _main_window_stub()
    mw.robot = SimpleNamespace(
        links={"ob_1": object_link},
        joints={},
        joint_relations={},
        update_kinematics=lambda: None,
    )
    mw.canvas = SimpleNamespace(update_transforms=lambda robot: None)
    mw.simulation_tab = SimpleNamespace(refresh_object_info=lambda name: None)
    panel = SimulationPanel(mw)
    panel.gripped_object = "ob_1"
    panel.grip_offset = np.eye(4)
    panel.grip_original_rotation = import_rotation

    panel._carry_gripped_object(SimpleNamespace(t_world=tcp_pose))

    np.testing.assert_allclose(object_link.t_offset[:3, :3], import_rotation)
    np.testing.assert_allclose(object_link.t_offset[:3, 3], [4.0, 5.0, 6.0])


def test_grasped_object_keeps_locked_rotation_even_if_other_rotation_data_changes():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    locked_rotation = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    object_link = SimpleNamespace(
        name="ob_1",
        t_offset=np.eye(4),
        import_metadata={"import_world_rotation": np.eye(3).tolist()},
    )
    mw = _main_window_stub()
    mw.robot = SimpleNamespace(
        links={"ob_1": object_link},
        joints={},
        joint_relations={},
        update_kinematics=lambda: None,
    )
    mw.canvas = SimpleNamespace(update_transforms=lambda robot: None)
    mw.simulation_tab = SimpleNamespace(refresh_object_info=lambda name: None)
    panel = SimulationPanel(mw)
    panel.gripped_object = "ob_1"
    panel.grip_original_rotation = np.eye(3)
    panel.grip_locked_rotation = locked_rotation
    panel.grip_translation_offset = np.zeros(3)
    object_link.import_metadata["import_world_rotation"] = np.diag([1.0, -1.0, -1.0]).tolist()

    tcp_pose = np.eye(4)
    tcp_pose[:3, 3] = [1.0, 2.0, 3.0]
    panel._carry_gripped_object(SimpleNamespace(t_world=tcp_pose))

    np.testing.assert_allclose(object_link.t_offset[:3, :3], locked_rotation)


def test_release_pose_anchors_the_bottom_face_without_changing_rotation():
    import trimesh

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    mesh = trimesh.creation.box(extents=[2.0, 4.0, 6.0])
    object_link = SimpleNamespace(
        name="ob_1",
        mesh=mesh,
        t_world=np.eye(4),
        import_metadata={
            "import_world_rotation": np.array([
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ]).tolist(),
        },
    )
    mw = _main_window_stub()
    mw.robot = SimpleNamespace(
        links={"ob_1": object_link},
        joints={},
        joint_relations={},
        update_kinematics=lambda: None,
    )
    mw.canvas = SimpleNamespace(update_transforms=lambda robot: None)
    panel = SimulationPanel(mw)

    pose = panel._ground_aligned_object_pose(object_link, np.array([10.0, 20.0, 30.0]))

    np.testing.assert_allclose(
        pose[:3, :3],
        np.array([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]),
    )
    np.testing.assert_allclose(pose[:3, 3], [10.0, 20.0, 33.0])
