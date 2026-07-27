import numpy as np

from core.robot import Robot


def test_parent_joint_rotation_carries_descendant_subtree():
    robot = Robot()
    base = robot.add_link("base")
    upper = robot.add_link("upper")
    forearm = robot.add_link("forearm")
    robot.base_link = base
    base.is_base = True

    upper.t_offset[:3, 3] = [10.0, 0.0, 0.0]
    forearm.t_offset[:3, 3] = [10.0, 0.0, 0.0]

    shoulder = robot.add_joint("shoulder", "base", "upper")
    elbow = robot.add_joint("elbow", "upper", "forearm")
    shoulder.axis = np.array([0.0, 0.0, 1.0])
    elbow.axis = np.array([0.0, 0.0, 1.0])

    robot.update_kinematics()
    shoulder.current_value = 90.0
    robot.update_kinematics()

    np.testing.assert_allclose(upper.t_world[:3, 3], [0.0, 10.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(forearm.t_world[:3, 3], [0.0, 20.0, 0.0], atol=1e-6)


def test_joint_origin_rotation_keeps_link_attached_to_pivot():
    robot = Robot()
    base = robot.add_link("base")
    arm = robot.add_link("arm")
    robot.base_link = base
    base.is_base = True

    arm.t_offset[:3, 3] = [10.0, 0.0, 0.0]
    joint = robot.add_joint("j1", "base", "arm")
    joint.axis = np.array([0.0, 0.0, 1.0])
    joint.origin = np.array([5.0, 0.0, 0.0])

    robot.update_kinematics()
    joint.current_value = 180.0
    robot.update_kinematics()

    np.testing.assert_allclose(arm.t_world[:3, 3], [0.0, 0.0, 0.0], atol=1e-6)


def test_fixed_joint_makes_unjointed_attachment_follow_parent():
    robot = Robot()
    base = robot.add_link("base")
    arm = robot.add_link("arm")
    bracket = robot.add_link("bracket")
    robot.base_link = base
    base.is_base = True

    arm.t_offset[:3, 3] = [10.0, 0.0, 0.0]
    bracket.t_offset[:3, 3] = [2.0, 0.0, 0.0]

    shoulder = robot.add_joint("shoulder", "base", "arm")
    rigid = robot.add_joint("rigid__arm__bracket", "arm", "bracket", joint_type="fixed")
    shoulder.axis = np.array([0.0, 0.0, 1.0])
    rigid.current_value = 90.0

    robot.update_kinematics()
    shoulder.current_value = 90.0
    robot.update_kinematics()

    np.testing.assert_allclose(arm.t_world[:3, 3], [0.0, 10.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(bracket.t_world[:3, 3], [0.0, 12.0, 0.0], atol=1e-6)


def test_real_joint_replaces_temporary_fixed_attachment():
    robot = Robot()
    base = robot.add_link("base")
    link = robot.add_link("link")
    robot.base_link = base
    base.is_base = True

    fixed = robot.add_joint("rigid__base__link", "base", "link", joint_type="fixed")
    real = robot.add_joint("j1", "base", "link", joint_type="revolute")

    assert fixed.name not in robot.joints
    assert robot.joints["j1"] is real
    assert link.parent_joint is real
    assert fixed not in base.child_joints


def test_alignment_cache_rigidizes_unjointed_parts_under_one_real_joint():
    robot = Robot()
    base = robot.add_link("base")
    arm = robot.add_link("arm")
    bracket = robot.add_link("bracket")
    robot.base_link = base
    base.is_base = True

    arm.t_offset[:3, 3] = [10.0, 0.0, 0.0]
    bracket.t_offset[:3, 3] = [12.0, 0.0, 0.0]
    shoulder = robot.add_joint("shoulder", "base", "arm")
    shoulder.axis = np.array([0.0, 0.0, 1.0])
    robot.update_kinematics()

    created = robot.rigidize_alignment_cache({("arm", "bracket"): np.array([10.0, 0.0, 0.0])})

    assert created == 1
    assert bracket.parent_joint.parent_link is arm
    assert bracket.parent_joint.joint_type == "fixed"

    shoulder.current_value = 90.0
    robot.update_kinematics()

    np.testing.assert_allclose(arm.t_world[:3, 3], [0.0, 10.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(bracket.t_world[:3, 3], [0.0, 12.0, 0.0], atol=1e-6)


def test_alignment_cache_rigidizes_nested_unjointed_subassemblies():
    robot = Robot()
    base = robot.add_link("base")
    arm = robot.add_link("arm")
    bracket = robot.add_link("bracket")
    tool = robot.add_link("tool")
    robot.base_link = base
    base.is_base = True

    arm.t_offset[:3, 3] = [10.0, 0.0, 0.0]
    bracket.t_offset[:3, 3] = [12.0, 0.0, 0.0]
    tool.t_offset[:3, 3] = [14.0, 0.0, 0.0]
    shoulder = robot.add_joint("shoulder", "base", "arm")
    shoulder.axis = np.array([0.0, 0.0, 1.0])
    robot.update_kinematics()

    created = robot.rigidize_alignment_cache({
        ("bracket", "tool"): np.array([12.0, 0.0, 0.0]),
        ("arm", "bracket"): np.array([10.0, 0.0, 0.0]),
    })

    assert created == 2
    shoulder.current_value = 90.0
    robot.update_kinematics()

    np.testing.assert_allclose(bracket.t_world[:3, 3], [0.0, 12.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(tool.t_world[:3, 3], [0.0, 14.0, 0.0], atol=1e-6)
