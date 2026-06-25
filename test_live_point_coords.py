"""
Tests for Live Point/TCP coordinate calculations.
"""
import unittest
import numpy as np
from core.robot import Robot
from core.import_units import get_engine_units_per_cm
from ui.mixins.navigation_mixin import NavigationMixin


class _Mesh:
    def __init__(self, vertices):
        self.vertices = np.array(vertices, dtype=float)
        mins = self.vertices.min(axis=0)
        maxs = self.vertices.max(axis=0)
        self.bounds = (mins[0], maxs[0], mins[1], maxs[1], mins[2], maxs[2])
        self.center_mass = self.vertices.mean(axis=0)
        self.centroid = self.center_mass
        self.moment_inertia = np.eye(3)


class _NavHarness(NavigationMixin):
    def __init__(self, robot):
        self.robot = robot


class LivePointCoordinateTest(unittest.TestCase):
    def test_live_point_hud_matches_marker_position(self):
        """Verify HUD coordinates match the marker position in grid units."""
        robot = Robot()

        base = robot.add_link('base')
        tool = robot.add_link('tool')
        robot.base_link = base

        joint = robot.add_joint('j1', 'base', 'tool')
        joint.current_value = 0.0

        test_pos_grid = np.array([150.0, 250.0, 350.0], dtype=float)
        tool.t_offset = np.eye(4)
        tool.t_offset[:3, 3] = test_pos_grid
        robot.update_kinematics()

        tcp_pose = robot.get_tcp_world_pose(tool)
        marker_pos_grid = tcp_pose[:3, 3]

        ratio = get_engine_units_per_cm()
        hud_x_cm = marker_pos_grid[0] / ratio
        hud_y_cm = marker_pos_grid[1] / ratio
        hud_z_cm = marker_pos_grid[2] / ratio

        expected_hud_x = test_pos_grid[0] / ratio
        expected_hud_y = test_pos_grid[1] / ratio
        expected_hud_z = test_pos_grid[2] / ratio

        self.assertAlmostEqual(hud_x_cm, expected_hud_x, places=6)
        self.assertAlmostEqual(hud_y_cm, expected_hud_y, places=6)
        self.assertAlmostEqual(hud_z_cm, expected_hud_z, places=6)
        np.testing.assert_array_almost_equal(marker_pos_grid, test_pos_grid, decimal=6)
        np.testing.assert_array_almost_equal(
            np.array([hud_x_cm, hud_y_cm, hud_z_cm]) * ratio,
            marker_pos_grid,
            decimal=6,
        )

    def test_tcp_pose_applies_parent_chain_and_final_offset(self):
        robot = Robot()
        base = robot.add_link('base')
        link1 = robot.add_link('link1')
        tool = robot.add_link('tool')
        robot.base_link = base

        base.t_offset[:3, 3] = [10.0, 0.0, 0.0]
        link1.t_offset[:3, 3] = [0.0, 20.0, 0.0]
        tool.t_offset[:3, 3] = [0.0, 0.0, 30.0]

        j1 = robot.add_joint('j1', 'base', 'link1')
        j2 = robot.add_joint('j2', 'link1', 'tool')
        j1.current_value = 90.0
        j2.current_value = 0.0
        robot.set_tcp_transform('tool', position=[5.0, 0.0, 0.0])
        robot.update_kinematics()

        tcp_pos = robot.get_tcp_world_pose(tool)[:3, 3]
        expected = np.array([-10.0, 5.0, 30.0])
        np.testing.assert_allclose(tcp_pos, expected, atol=1e-6)

    def test_auto_tcp_offset_tracks_gripper_tip_geometry(self):
        robot = Robot()
        base = robot.add_link('base')
        hand = robot.add_link('hand')
        finger_a = robot.add_link('finger_a', _Mesh([[-1, 0, 0], [1, 0, 0], [0, 0, 10]]))
        finger_b = robot.add_link('finger_b', _Mesh([[-1, 0, 0], [1, 0, 0], [0, 0, 10]]))
        robot.base_link = base

        robot.add_joint('wrist', 'base', 'hand')
        ja = robot.add_joint('grip_a', 'hand', 'finger_a')
        jb = robot.add_joint('grip_b', 'hand', 'finger_b')
        ja.is_gripper = True
        jb.is_gripper = True
        finger_a.t_offset[:3, 3] = [-5.0, 0.0, 0.0]
        finger_b.t_offset[:3, 3] = [5.0, 0.0, 0.0]
        robot.update_kinematics()

        harness = _NavHarness(robot)
        harness._refresh_auto_tcp_offset(hand)
        np.testing.assert_allclose(hand.auto_tcp_offset, [0.0, 0.0, 10.0], atol=1e-6)
        np.testing.assert_allclose(
            robot.get_tcp_world_pose(hand)[:3, 3],
            [0.0, 0.0, 10.0],
            atol=1e-6,
        )

        finger_b.t_offset[:3, 3] = [9.0, 0.0, 0.0]
        robot.update_kinematics()
        harness._refresh_auto_tcp_offset(hand)
        np.testing.assert_allclose(hand.auto_tcp_offset, [2.0, 0.0, 10.0], atol=1e-6)


if __name__ == '__main__':
    unittest.main()