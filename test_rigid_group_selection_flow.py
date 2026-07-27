import os
from types import MethodType, SimpleNamespace

from PyQt5 import QtWidgets

from core.robot import Robot
from ui.mixins.links_mixin import LinksMixin


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


class _DummyButton:
    def __init__(self):
        self.enabled = None
        self.text = None

    def setEnabled(self, value):
        self.enabled = bool(value)

    def setText(self, value):
        self.text = value


class _DummyLabel:
    def __init__(self):
        self.text = None

    def setText(self, value):
        self.text = value

    def setWordWrap(self, _value):
        pass


class _DummyCanvas:
    def __init__(self):
        self.callback = None
        self.selected = []

    def start_actor_click_capture(self, callback):
        self.callback = callback

    def stop_actor_click_capture(self):
        self.callback = None

    def select_actor(self, name):
        self.selected.append(name)

    def update_transforms(self, _robot):
        pass


def _selection_harness():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    robot = Robot()
    for name in ("A", "B", "C"):
        robot.add_link(name)
    robot.base_link = robot.links["A"]
    robot.base_link.is_base = True

    selection_list = QtWidgets.QListWidget()

    harness = SimpleNamespace(
        robot=robot,
        canvas=_DummyCanvas(),
        alignment_cache={},
        rigid_groups=[],
        rigid_group_selection_list=selection_list,
        rigid_groups_list=QtWidgets.QListWidget(),
        rigid_group_status=_DummyLabel(),
        rigid_group_btn=_DummyButton(),
        rigid_group_ok_btn=_DummyButton(),
        rigid_group_cancel_btn=_DummyButton(),
        rigid_group_delete_btn=_DummyButton(),
        logs=[],
        toasts=[],
        joint_tab=SimpleNamespace(refresh_links=lambda: None),
        matrices_tab=SimpleNamespace(refresh_sliders=lambda: None, update_display=lambda: None),
        update_link_colors=lambda: None,
    )
    harness.log = harness.logs.append
    harness.show_toast = lambda *args: harness.toasts.append(args)
    harness.begin_rigid_group_selection = MethodType(LinksMixin.begin_rigid_group_selection, harness)
    harness.cancel_rigid_group_selection = MethodType(LinksMixin.cancel_rigid_group_selection, harness)
    harness._on_rigid_group_actor_clicked = MethodType(LinksMixin._on_rigid_group_actor_clicked, harness)
    harness.confirm_rigid_group_selection = MethodType(LinksMixin.confirm_rigid_group_selection, harness)
    harness._refresh_rigid_group_selection_list = MethodType(LinksMixin._refresh_rigid_group_selection_list, harness)
    harness._refresh_rigid_groups_list = MethodType(LinksMixin._refresh_rigid_groups_list, harness)
    harness._ensure_rigid_groups_store = MethodType(LinksMixin._ensure_rigid_groups_store, harness)
    harness._set_rigid_group_ui_state = MethodType(LinksMixin._set_rigid_group_ui_state, harness)
    harness._create_rigid_group = MethodType(LinksMixin._create_rigid_group, harness)
    harness._register_rigid_group = MethodType(LinksMixin._register_rigid_group, harness)
    harness._selected_rigid_group_record = MethodType(LinksMixin._selected_rigid_group_record, harness)
    harness.on_rigid_group_selection_changed = MethodType(LinksMixin.on_rigid_group_selection_changed, harness)
    harness.delete_selected_rigid_group_relation = MethodType(LinksMixin.delete_selected_rigid_group_relation, harness)
    return harness, app


def test_rigid_group_selection_flow_builds_fixed_group():
    harness, _app = _selection_harness()

    harness.begin_rigid_group_selection()
    assert harness.canvas.callback is not None
    assert harness.rigid_group_btn.text == "Collecting..."
    assert harness.rigid_group_ok_btn.enabled is False

    assert harness.canvas.callback("A") is True
    assert harness.canvas.callback("B") is True

    labels = [harness.rigid_group_selection_list.item(i).text() for i in range(harness.rigid_group_selection_list.count())]
    assert labels[0].startswith("1. A")
    assert labels[1].startswith("2. B")
    assert harness.rigid_group_ok_btn.enabled is True

    harness.confirm_rigid_group_selection()

    assert harness.canvas.callback is None
    assert "rigid__A__B" in harness.robot.joints
    assert harness.robot.links["B"].parent_joint.parent_link.name == "A"
    assert harness.toasts[-1][0] == "Rigid group created"


def test_delete_selected_rigid_group_relation_removes_only_joint_links():
    harness, _app = _selection_harness()

    harness.begin_rigid_group_selection()
    harness.canvas.callback("A")
    harness.canvas.callback("B")
    harness.confirm_rigid_group_selection()

    assert harness.rigid_groups_list.count() == 1
    harness.rigid_groups_list.setCurrentRow(0)
    harness.on_rigid_group_selection_changed()

    harness.delete_selected_rigid_group_relation()

    assert "rigid__A__B" not in harness.robot.joints
    assert "A" in harness.robot.links
    assert "B" in harness.robot.links
    assert harness.rigid_groups_list.count() == 0
