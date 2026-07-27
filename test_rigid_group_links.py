from types import MethodType, SimpleNamespace

import numpy as np

from core.robot import Robot
from ui.mixins.links_mixin import LinksMixin


class _DummyItem:
    def __init__(self, name):
        self._name = name

    def text(self):
        return self._name


class _DummyLinksList:
    def __init__(self, selected_names, current_name):
        self._selected = [_DummyItem(name) for name in selected_names]
        self._current = _DummyItem(current_name) if current_name is not None else None

    def selectedItems(self):
        return list(self._selected)

    def currentItem(self):
        return self._current


def _rigid_group_harness(selected_names, current_name):
    robot = Robot()
    for name in ("A", "B", "C"):
        robot.add_link(name)
    robot.base_link = robot.links["A"]
    robot.base_link.is_base = True

    robot.links["A"].t_world = np.eye(4)
    robot.links["B"].t_world = np.array([[1.0, 0.0, 0.0, 2.0],
                                         [0.0, 1.0, 0.0, 0.0],
                                         [0.0, 0.0, 1.0, 0.0],
                                         [0.0, 0.0, 0.0, 1.0]])
    robot.links["C"].t_world = np.array([[1.0, 0.0, 0.0, 0.0],
                                         [0.0, 1.0, 0.0, 3.0],
                                         [0.0, 0.0, 1.0, 0.0],
                                         [0.0, 0.0, 0.0, 1.0]])

    harness = SimpleNamespace(
        robot=robot,
        links_list=_DummyLinksList(selected_names, current_name),
        alignment_cache={},
        canvas=SimpleNamespace(update_transforms=lambda robot: None),
        logs=[],
        toasts=[],
    )
    harness.log = harness.logs.append
    harness.show_toast = lambda *args: harness.toasts.append(args)
    harness.update_link_colors = lambda: None
    harness.create_rigid_group_from_selection = MethodType(
        LinksMixin.create_rigid_group_from_selection,
        harness,
    )
    harness._selected_link_names = MethodType(
        LinksMixin._selected_link_names,
        harness,
    )
    harness._create_rigid_group = MethodType(
        LinksMixin._create_rigid_group,
        harness,
    )
    harness._ensure_rigid_groups_store = MethodType(
        LinksMixin._ensure_rigid_groups_store,
        harness,
    )
    harness._register_rigid_group = MethodType(
        LinksMixin._register_rigid_group,
        harness,
    )
    return harness


def test_create_rigid_group_from_selection_binds_free_components():
    harness = _rigid_group_harness(["A", "B", "C"], "A")

    harness.create_rigid_group_from_selection()

    assert "rigid__A__B" in harness.robot.joints
    assert "rigid__A__C" in harness.robot.joints
    assert harness.robot.links["B"].parent_joint.parent_link.name == "A"
    assert harness.robot.links["C"].parent_joint.parent_link.name == "A"
    assert harness.robot.joints["rigid__A__B"].joint_type == "fixed"
    assert harness.robot.joints["rigid__A__C"].joint_type == "fixed"
    assert ("A", "B") in harness.alignment_cache
    assert ("A", "C") in harness.alignment_cache


def test_create_rigid_group_skips_already_jointed_components():
    harness = _rigid_group_harness(["A", "B"], "A")
    harness.robot.add_joint("joint_A_B", "A", "B")

    harness.create_rigid_group_from_selection()

    assert "rigid__A__B" not in harness.robot.joints
    assert any("already have joints" in toast[0] for toast in harness.toasts)
