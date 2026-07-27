import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from types import SimpleNamespace

from PyQt5 import QtWidgets

from ui.panels.joint_panel import JointPanel


def test_axis_index_from_vector_detects_cardinal_axes():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = JointPanel(SimpleNamespace())

    assert panel._axis_index_from_vector([1.0, 0.0, 0.0]) == 0
    assert panel._axis_index_from_vector([0.0, 1.0, 0.0]) == 1
    assert panel._axis_index_from_vector([0.0, 0.0, 1.0]) == 2
    assert panel._axis_index_from_vector([0.2, 0.0, 0.9]) == 2
    assert panel._axis_index_from_vector([0.0, 0.0, 0.0]) == 2


def test_reset_joint_ui_does_not_require_legacy_axis_section():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    mw = SimpleNamespace(
        robot=SimpleNamespace(links={}),
        log=lambda *_: None,
    )
    panel = JointPanel(mw)

    panel.reset_joint_ui()


def test_canvas_face_picking_accepts_joint_panel_options():
    class CanvasStub:
        from graphics.canvas import RobotCanvas
        start_face_picking = RobotCanvas.start_face_picking

        def mw_log(self, *_):
            pass

    canvas = CanvasStub()

    canvas.start_face_picking(
        lambda *_: None,
        color="#ff9800",
        highlight_prefix="joint_face",
        center_mode="surface",
    )

    assert canvas.picking_face is True
    assert canvas.highlight_prefix == "joint_face"
    assert canvas.center_mode == "surface"


def test_quick_angle_box_jumps_slider_to_typed_angle():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    calls = []
    mw = SimpleNamespace(
        robot=SimpleNamespace(links={}),
        log=lambda *_: None,
    )
    panel = JointPanel(mw)
    panel.test_rotation = lambda value: calls.append(value)

    panel.quick_angle_spin.setValue(45.0)

    assert panel.quick_joint_slider.value() == 450
    assert panel.rotation_slider.value() == 450
    assert panel.rotation_spinbox.value() == 45.0
    assert calls[-1] == 450


def test_quick_joint_combos_hide_internal_status_labels():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    base = SimpleNamespace(name="base", is_base=True, parent_joint=None, child_joints=[])
    arm = SimpleNamespace(name="arm", is_base=False, parent_joint=None, child_joints=[])
    tool = SimpleNamespace(name="tool", is_base=False, parent_joint=None, child_joints=[])
    base_to_arm = SimpleNamespace(parent_link=base, child_link=arm)
    arm_to_tool = SimpleNamespace(parent_link=arm, child_link=tool)
    base.child_joints.append(base_to_arm)
    arm.child_joints.append(arm_to_tool)
    arm.parent_joint = base_to_arm
    tool.parent_joint = arm_to_tool
    mw = SimpleNamespace(
        robot=SimpleNamespace(
            links={
                "base": base,
                "arm": arm,
                "tool": tool,
            },
            base_link=base,
        ),
        log=lambda *_: None,
    )
    panel = JointPanel(mw)

    parent_items = [panel.parent_combo.itemText(i) for i in range(panel.parent_combo.count())]
    child_items = [panel.child_combo.itemText(i) for i in range(panel.child_combo.count())]

    assert "arm" in parent_items
    assert all("child link" not in text for text in parent_items)
    assert "base" not in child_items
    assert "arm" in child_items
    assert "tool" in child_items
    assert all("already has parent" not in text for text in child_items)


def test_quick_joint_child_combo_uses_robot_tree_rules():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    base = SimpleNamespace(name="base", is_base=True, parent_joint=None, child_joints=[])
    arm = SimpleNamespace(name="arm", is_base=False, parent_joint=None, child_joints=[])
    wrist = SimpleNamespace(name="wrist", is_base=False, parent_joint=None, child_joints=[])
    tool = SimpleNamespace(name="tool", is_base=False, parent_joint=None, child_joints=[])
    base_to_arm = SimpleNamespace(parent_link=base, child_link=arm)
    arm_to_wrist = SimpleNamespace(parent_link=arm, child_link=wrist)
    wrist_to_tool = SimpleNamespace(parent_link=wrist, child_link=tool)
    base.child_joints.append(base_to_arm)
    arm.child_joints.append(arm_to_wrist)
    wrist.child_joints.append(wrist_to_tool)
    arm.parent_joint = base_to_arm
    wrist.parent_joint = arm_to_wrist
    tool.parent_joint = wrist_to_tool

    mw = SimpleNamespace(
        robot=SimpleNamespace(
            links={"base": base, "arm": arm, "wrist": wrist, "tool": tool},
            base_link=base,
        ),
        log=lambda *_: None,
    )
    panel = JointPanel(mw)

    assert panel.validate_joint_tree_choice("base", "arm") == (True, "")
    assert panel.validate_joint_tree_choice("wrist", "arm")[0] is False
    assert panel.validate_joint_tree_choice("arm", "base")[0] is False

    panel.set_quick_combo_value(panel.parent_combo, "base")
    panel.refresh_quick_link_combos()
    child_items_for_base = [panel.child_combo.itemData(i) for i in range(panel.child_combo.count())]

    assert "base" not in child_items_for_base
    assert "arm" in child_items_for_base
    assert "wrist" in child_items_for_base
    assert "tool" in child_items_for_base

    panel.set_quick_combo_value(panel.parent_combo, "wrist")
    panel.refresh_quick_link_combos()
    child_items_for_wrist = [panel.child_combo.itemData(i) for i in range(panel.child_combo.count())]

    assert "base" not in child_items_for_wrist
    assert "arm" not in child_items_for_wrist
    assert "wrist" not in child_items_for_wrist
    assert "tool" in child_items_for_wrist
