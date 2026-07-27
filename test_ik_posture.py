import unittest

import numpy as np

from core.robot import Robot


class IKPostureTest(unittest.TestCase):
    def test_joint_posture_penalty_prefers_neutral_pose(self):
        robot = Robot()

        base = robot.add_link("base")
        elbow = robot.add_link("elbow")
        tool = robot.add_link("tool")

        robot.base_link = base

        joint_a = robot.add_joint("joint_a", "base", "elbow")
        joint_a.min_limit = -90.0
        joint_a.max_limit = 90.0
        joint_a.current_value = 0.0

        joint_b = robot.add_joint("joint_b", "elbow", "tool")
        joint_b.min_limit = -120.0
        joint_b.max_limit = 120.0
        joint_b.current_value = 0.0

        neutral = np.array([0.0, 0.0], dtype=float)
        folded = np.array([80.0, -100.0], dtype=float)

        neutral_penalty = robot._posture_penalty([joint_a, joint_b], neutral)
        folded_penalty = robot._posture_penalty([joint_a, joint_b], folded)

        self.assertLess(neutral_penalty, folded_penalty)

    def test_axis_ik_aligns_selected_face_without_constraining_roll(self):
        robot = Robot()
        base = robot.add_link("base")
        tool = robot.add_link("tool")
        robot.base_link = base

        joint = robot.add_joint("wrist", "base", "tool")
        joint.axis = np.array([0.0, 0.0, 1.0])
        joint.min_limit = -180.0
        joint.max_limit = 180.0
        robot.update_kinematics()

        reached, info = robot.inverse_kinematics_axis(
            [0.0, 0.0, 0.0],
            tool,
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            max_iters=300,
            position_tolerance=1e-3,
            axis_tolerance=np.deg2rad(1.0),
        )

        aligned_axis = (
            robot.get_tcp_world_pose(tool)[:3, :3]
            @ np.array([1.0, 0.0, 0.0])
        )
        self.assertTrue(reached)
        self.assertLess(info["axis_error"], np.deg2rad(1.0))
        np.testing.assert_allclose(aligned_axis, [0.0, 1.0, 0.0], atol=0.01)


if __name__ == "__main__":
    unittest.main()
