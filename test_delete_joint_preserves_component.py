from types import MethodType, SimpleNamespace

import numpy as np

from core.robot import Robot
from ui.panels.joint_panel import JointPanel


class _DummySection:
    def __init__(self):
        self.visible = True

    def setVisible(self, value):
        self.visible = bool(value)


def _harness():
    robot = Robot()
    robot.add_link("base")
    robot.add_link("child")
    robot.base_link = robot.links["base"]
    robot.base_link.is_base = True
    joint = robot.add_joint("joint_base_child", "base", "child")
    joint.current_value = 30.0
    robot.links["child"].t_offset = np.array(
        [
            [1.0, 0.0, 0.0, 4.0],
            [0.0, 1.0, 0.0, 2.0],
            [0.0, 0.0, 1.0, 1.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    robot.update_kinematics()

    harness = SimpleNamespace(
        mw=SimpleNamespace(
            robot=robot,
            matrices_tab=SimpleNamespace(refresh_sliders=lambda: None),
            refresh_link_hierarchy=lambda: None,
            canvas=SimpleNamespace(update_transforms=lambda _robot: None),
            log=lambda *_args, **_kwargs: None,
            show_toast=lambda *_args, **_kwargs: None,
        ),
        joints={
            "child": {
                "parent": "base",
                "joint_id": "joint_base_child",
            }
        },
        active_joint_control="child",
        joint_control_section=_DummySection(),
        refresh_links=lambda: None,
        refresh_joints_history=lambda: None,
    )
    harness.delete_joint = MethodType(JointPanel.delete_joint, harness)
    return harness


def test_delete_joint_keeps_child_component_in_place():
    harness = _harness()
    before = harness.mw.robot.links["child"].t_world.copy()

    harness.delete_joint("child")

    assert "joint_base_child" not in harness.mw.robot.joints
    assert "child" in harness.mw.robot.links
    after = harness.mw.robot.links["child"].t_world
    assert np.allclose(after, before)
    assert harness.active_joint_control is None
    assert harness.joint_control_section.visible is False
