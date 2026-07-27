import numpy as np

from core.robot import Robot


class StubMesh:
    def __init__(self, vertices):
        self.vertices = np.array(vertices, dtype=float)
        self.center_mass = np.mean(self.vertices, axis=0)
        self.moment_inertia = np.eye(3)
        self.centroid = self.center_mass


def test_default_tcp_offset_remains_zero_when_no_user_tcp_is_defined():
    robot = Robot()
    robot.add_link("base")
    tool = robot.add_link("tool")
    tool.mesh = StubMesh([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (1.0, 0.0, 4.0)])
    robot.add_joint("joint_1", "base", "tool")

    robot.update_kinematics()
    robot.ensure_tcp_transform(tool)

    assert np.allclose(tool.auto_tcp_offset, np.array([0.0, 0.0, 0.0]))

    pose_before = robot.get_tcp_world_pose(tool)
    robot.joints["joint_1"].current_value = 90.0
    robot.update_kinematics()
    pose_after = robot.get_tcp_world_pose(tool)

    assert np.allclose(pose_before[:3, 3], np.array([0.0, 0.0, 0.0]))
    assert np.allclose(pose_after[:3, 3], np.array([0.0, 0.0, 0.0]))
    assert np.allclose(tool.auto_tcp_offset, np.array([0.0, 0.0, 0.0]))
