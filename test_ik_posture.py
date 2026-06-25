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


if __name__ == "__main__":
    unittest.main()
