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


class _DummyCanvas:
    def __init__(self):
        self.grid_units_per_cm = get_engine_units_per_cm()
        self.hud_calls = []
        self.marker_calls = []

    def update_hud_coords(self, x, y, z, status=None, render=True):
        self.hud_calls.append((x, y, z, status, render))

    def update_live_point_marker(self, point_world, render=True):
        self.marker_calls.append((np.array(point_world, dtype=float), render))

    def clear_live_point_marker(self):
        self.marker_calls.append(None)


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

    def test_tcp_offset_is_user_defined_local_point(self):
        robot = Robot()
        base = robot.add_link('base')
        hand = robot.add_link('hand')
        robot.base_link = base

        robot.add_joint('wrist', 'base', 'hand')
        robot.set_tcp_transform('hand', position=[3.0, 4.0, 5.0])
        robot.update_kinematics()

        harness = _NavHarness(robot)
        world_pose = robot.get_tcp_world_pose(hand)[:3, 3]
        np.testing.assert_allclose(world_pose, [3.0, 4.0, 5.0], atol=1e-6)

        harness._refresh_auto_tcp_offset(hand)
        np.testing.assert_allclose(hand.auto_tcp_offset, [3.0, 4.0, 5.0], atol=1e-6)

    def test_update_live_ui_uses_application_tool_point_logic(self):
        robot = Robot()
        base = robot.add_link('base')
        tool = robot.add_link('tool')
        robot.base_link = base
        robot.add_joint('j1', 'base', 'tool')
        robot.update_kinematics()

        harness = _NavHarness(robot)
        harness.canvas = _DummyCanvas()
        harness._get_preferred_tcp_link = lambda: tool
        harness._refresh_auto_tcp_offset = lambda link: None
        harness.custom_tcp_name = 'tool'

        expected_world = np.array([60.0, 30.0, 20.0], dtype=float)
        harness.get_link_tool_point = lambda link, return_vec=False: (expected_world.copy(), np.zeros(3), None)

        harness.update_live_ui(render=False)

        ratio = harness.canvas.grid_units_per_cm
        np.testing.assert_allclose(harness.current_live_point_cm, expected_world / ratio, atol=1e-6)
        self.assertEqual(harness.canvas.hud_calls[-1][:3], tuple(expected_world / ratio))

    def test_update_live_ui_maintains_locked_live_point_when_robot_pose_changes(self):
        robot = Robot()
        base = robot.add_link('base')
        tool = robot.add_link('tool')
        robot.base_link = base
        robot.add_joint('j1', 'base', 'tool')
        robot.update_kinematics()

        harness = _NavHarness(robot)
        harness.canvas = _DummyCanvas()
        harness._get_preferred_tcp_link = lambda: tool
        harness._refresh_auto_tcp_offset = lambda link: None
        harness.custom_tcp_name = 'tool'

        harness.get_link_tool_point = lambda link, return_vec=False: (np.array([100.0, 50.0, 20.0], dtype=float), np.zeros(3), None)
        harness.update_live_ui(render=False)

        harness.live_point_locked = True
        harness.locked_live_point = np.array([10.0, 20.0, 30.0], dtype=float)
        harness.get_link_tool_point = lambda link, return_vec=False: (np.array([200.0, 100.0, 60.0], dtype=float), np.zeros(3), None)

        harness.update_live_ui(render=False)

        self.assertEqual(harness.current_live_point_cm, (10.0, 20.0, 30.0))
        self.assertEqual(harness.canvas.hud_calls[-1][:3], (10.0, 20.0, 30.0))

    def test_update_live_ui_falls_back_to_robot_pose_when_tool_point_is_zero(self):
        robot = Robot()
        base = robot.add_link('base')
        tool = robot.add_link('tool')
        robot.base_link = base
        robot.add_joint('j1', 'base', 'tool')
        tool.t_offset[:3, 3] = np.array([12.0, 7.0, 3.0], dtype=float)
        robot.update_kinematics()

        harness = _NavHarness(robot)
        harness.canvas = _DummyCanvas()
        harness._get_preferred_tcp_link = lambda: tool
        harness._refresh_auto_tcp_offset = lambda link: None
        harness.custom_tcp_name = 'tool'
        harness.get_link_tool_point = lambda link, return_vec=False: (np.zeros(3), np.zeros(3), None)

        harness.update_live_ui(render=False)

        ratio = harness.canvas.grid_units_per_cm
        expected_cm = np.array([12.0, 7.0, 3.0], dtype=float) / ratio
        np.testing.assert_allclose(harness.current_live_point_cm, expected_cm, atol=1e-6)

    def test_compute_top_face_center_falls_back_for_robot_without_tcp(self):
        class FaceMesh:
            def __init__(self):
                self.vertices = np.array([
                    [0.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0],
                    [1.0, 1.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [1.0, 0.0, 1.0],
                    [1.0, 1.0, 1.0],
                    [0.0, 1.0, 1.0],
                ], dtype=float)
                self.faces = np.array([
                    [4, 5, 6],
                    [4, 6, 7],
                ], dtype=int)
                self.center_mass = self.vertices.mean(axis=0)
                self.centroid = self.center_mass
                self.moment_inertia = np.eye(3)

        robot = Robot()
        base = robot.add_link('base')
        top_link = robot.add_link('top', mesh=FaceMesh())
        robot.base_link = base
        robot.add_joint('j1', 'base', 'top')
        robot.update_kinematics()

        harness = _NavHarness(robot)
        harness.canvas = _DummyCanvas()
        harness._get_preferred_tcp_link = lambda: None

        top_face_center = harness._compute_robot_top_face_center_point()
        np.testing.assert_allclose(top_face_center, [0.5, 0.5, 1.0], atol=1e-6)

        harness.update_live_ui(render=False)
        ratio = harness.canvas.grid_units_per_cm
        np.testing.assert_allclose(harness.current_live_point_cm, np.array([0.5, 0.5, 1.0]) / ratio, atol=1e-6)

    def test_update_live_ui_falls_back_to_top_face_when_tcp_has_no_live_point(self):
        class FaceMesh:
            def __init__(self):
                self.vertices = np.array([
                    [0.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0],
                    [1.0, 1.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [1.0, 0.0, 1.0],
                    [1.0, 1.0, 1.0],
                    [0.0, 1.0, 1.0],
                ], dtype=float)
                self.faces = np.array([
                    [4, 5, 6],
                    [4, 6, 7],
                ], dtype=int)
                self.center_mass = self.vertices.mean(axis=0)
                self.centroid = self.center_mass
                self.moment_inertia = np.eye(3)

        robot = Robot()
        base = robot.add_link('base')
        top_link = robot.add_link('top', mesh=FaceMesh())
        robot.base_link = base
        robot.add_joint('j1', 'base', 'top')
        robot.update_kinematics()

        harness = _NavHarness(robot)
        harness.canvas = _DummyCanvas()
        harness._get_preferred_tcp_link = lambda: top_link

        harness.update_live_ui(render=False)
        ratio = harness.canvas.grid_units_per_cm
        np.testing.assert_allclose(harness.current_live_point_cm, np.array([0.5, 0.5, 1.0]) / ratio, atol=1e-6)

    def test_update_live_ui_falls_back_to_top_face_for_gripper_link_without_explicit_tcp(self):
        class FaceMesh:
            def __init__(self):
                self.vertices = np.array([
                    [0.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0],
                    [1.0, 1.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [1.0, 0.0, 1.0],
                    [1.0, 1.0, 1.0],
                    [0.0, 1.0, 1.0],
                ], dtype=float)
                self.faces = np.array([
                    [4, 5, 6],
                    [4, 6, 7],
                ], dtype=int)
                self.center_mass = self.vertices.mean(axis=0)
                self.centroid = self.center_mass
                self.moment_inertia = np.eye(3)

        robot = Robot()
        base = robot.add_link('base')
        gripper = robot.add_link('gripper', mesh=FaceMesh())
        robot.base_link = base
        g_joint = robot.add_joint('g1', 'base', 'gripper')
        g_joint.is_gripper = True
        robot.update_kinematics()

        harness = _NavHarness(robot)
        harness.canvas = _DummyCanvas()
        harness._get_preferred_tcp_link = lambda: gripper

        harness.update_live_ui(render=False)
        ratio = harness.canvas.grid_units_per_cm
        np.testing.assert_allclose(harness.current_live_point_cm, np.array([0.5, 0.5, 1.0]) / ratio, atol=1e-6)

    def test_top_face_point_can_be_attached_as_a_real_tcp(self):
        class FaceMesh:
            def __init__(self):
                self.vertices = np.array([
                    [0.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0],
                    [1.0, 1.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [1.0, 0.0, 1.0],
                    [1.0, 1.0, 1.0],
                    [0.0, 1.0, 1.0],
                ], dtype=float)
                self.faces = np.array([
                    [4, 5, 6],
                    [4, 6, 7],
                ], dtype=int)
                self.center_mass = self.vertices.mean(axis=0)
                self.centroid = self.center_mass
                self.moment_inertia = np.eye(3)

        robot = Robot()
        base = robot.add_link('base')
        top_link = robot.add_link('top', mesh=FaceMesh())
        robot.base_link = base
        joint = robot.add_joint('j1', 'base', 'top')
        robot.update_kinematics()

        harness = _NavHarness(robot)
        data = harness._compute_robot_top_face_center_point_data()
        assert data is not None
        top_point, link_name, local_point = data
        assert link_name == 'top'
        np.testing.assert_allclose(top_point, [0.5, 0.5, 1.0], atol=1e-6)
        np.testing.assert_allclose(local_point, [0.5, 0.5, 1.0], atol=1e-6)

        robot.set_tcp_transform(link_name, position=local_point)
        robot.update_kinematics()
        pose_before = robot.get_tcp_world_pose(top_link)[:3, 3].copy()
        joint.current_value = 90.0
        robot.update_kinematics()
        pose_after = robot.get_tcp_world_pose(top_link)[:3, 3].copy()

        assert not np.allclose(pose_before, pose_after)
        np.testing.assert_allclose(pose_before, [0.5, 0.5, 1.0], atol=1e-6)

    def test_update_live_ui_removes_stale_fixed_live_marker_when_unlocked(self):
        class _Actor:
            def __init__(self):
                self.user_matrix = np.eye(4)

        class _Renderer:
            def __init__(self):
                self.actors = {"fixed_live_point_marker": _Actor()}

        class _Plotter:
            def __init__(self):
                self.renderer = _Renderer()
                self.render_calls = 0
            def render(self):
                self.render_calls += 1
            def remove_actor(self, name):
                self.renderer.actors.pop(name, None)

        class _Canvas:
            def __init__(self):
                self.grid_units_per_cm = get_engine_units_per_cm()
                self.plotter = _Plotter()
                self.hud_calls = []
                self.marker_calls = []
                self.tcp_marker_calls = []
            def update_hud_coords(self, x, y, z, status=None, render=True):
                self.hud_calls.append((x, y, z, status, render))
            def update_live_point_marker(self, point_world, render=True):
                self.marker_calls.append(np.array(point_world, dtype=float))
            def update_live_tcp_marker(self, point_world):
                self.tcp_marker_calls.append(np.array(point_world, dtype=float))

        robot = Robot()
        base = robot.add_link('base')
        tool = robot.add_link('tool')
        robot.base_link = base
        robot.add_joint('j1', 'base', 'tool')
        tool.t_offset[:3, 3] = np.array([10.0, 5.0, 2.0], dtype=float)
        robot.set_tcp_transform('tool', position=[3.0, 0.0, 0.0])
        robot.update_kinematics()

        harness = _NavHarness(robot)
        harness.canvas = _Canvas()
        harness._get_preferred_tcp_link = lambda: tool
        harness.custom_tcp_name = 'tool'

        harness.update_live_ui(render=False)
        expected = robot.get_tcp_world_pose(tool)[:3, 3]
        assert 'fixed_live_point_marker' not in harness.canvas.plotter.renderer.actors
        np.testing.assert_allclose(harness.canvas.tcp_marker_calls[-1], expected, atol=1e-6)

    def test_update_live_ui_updates_tcp_marker_from_live_coordinates(self):
        class _Actor:
            def __init__(self):
                self.user_matrix = np.eye(4)

        class _Renderer:
            def __init__(self):
                self.actors = {}

        class _Plotter:
            def __init__(self):
                self.renderer = _Renderer()
                self.render_calls = 0
            def render(self):
                self.render_calls += 1

        class _Canvas:
            def __init__(self):
                self.grid_units_per_cm = get_engine_units_per_cm()
                self.plotter = _Plotter()
                self.hud_calls = []
                self.marker_calls = []
                self.tcp_marker_calls = []
            def update_hud_coords(self, x, y, z, status=None, render=True):
                self.hud_calls.append((x, y, z, status, render))
            def update_live_point_marker(self, point_world, render=True):
                self.marker_calls.append(np.array(point_world, dtype=float))
            def update_live_tcp_marker(self, point_world):
                self.tcp_marker_calls.append(np.array(point_world, dtype=float))
                if "traj_tcp_marker" not in self.plotter.renderer.actors:
                    self.plotter.renderer.actors["traj_tcp_marker"] = _Actor()
                self.plotter.renderer.actors["traj_tcp_marker"].user_matrix[:3, 3] = np.array(point_world, dtype=float) * self.grid_units_per_cm

        robot = Robot()
        base = robot.add_link('base')
        tool = robot.add_link('tool')
        robot.base_link = base
        joint = robot.add_joint('j1', 'base', 'tool')
        tool.t_offset[:3, 3] = np.array([10.0, 5.0, 2.0], dtype=float)
        robot.set_tcp_transform('tool', position=[3.0, 0.0, 0.0])
        robot.update_kinematics()

        harness = _NavHarness(robot)
        harness.canvas = _Canvas()
        harness._get_preferred_tcp_link = lambda: tool
        harness.custom_tcp_name = 'tool'

        harness.update_live_ui(render=False)
        expected = robot.get_tcp_world_pose(tool)[:3, 3]
        np.testing.assert_allclose(harness.canvas.tcp_marker_calls[-1], expected, atol=1e-6)

    def test_update_live_ui_keeps_explicit_tcp_point_when_force_top_face_is_requested(self):
        robot = Robot()
        base = robot.add_link('base')
        tool = robot.add_link('tool')
        robot.base_link = base
        joint = robot.add_joint('j1', 'base', 'tool')
        tool.t_offset[:3, 3] = np.array([12.0, 7.0, 3.0], dtype=float)
        robot.set_tcp_transform('tool', position=[2.0, 0.0, 0.0])
        robot.update_kinematics()

        harness = _NavHarness(robot)
        harness.canvas = _DummyCanvas()
        harness._get_preferred_tcp_link = lambda: tool
        harness.custom_tcp_name = 'tool'

        harness.update_live_ui(render=False, force_top_face=True)
        ratio = harness.canvas.grid_units_per_cm
        expected_before = robot.get_tcp_world_pose(tool)[:3, 3] / ratio
        np.testing.assert_allclose(harness.current_live_point_cm, expected_before, atol=1e-6)

        joint.current_value = 90.0
        robot.update_kinematics()
        harness.update_live_ui(render=False, force_top_face=True)
        expected_after = robot.get_tcp_world_pose(tool)[:3, 3] / ratio
        np.testing.assert_allclose(harness.current_live_point_cm, expected_after, atol=1e-6)

    def test_auto_lock_live_point_attaches_tcp_instead_of_world_lock(self):
        from ui.panels.gripper_panel import GripperPanel

        robot = Robot()
        base = robot.add_link('base')
        tool = robot.add_link('tool')
        finger = robot.add_link('finger')
        robot.base_link = base
        robot.add_joint('j1', 'base', 'tool')
        grip = robot.add_joint('grip', 'tool', 'finger')
        grip.is_gripper = True
        robot.update_kinematics()

        class _MW:
            def __init__(self, robot):
                self.robot = robot
                self.custom_tcp_name = None
                self.live_point_locked = True
                self.locked_live_point = (99.0, 98.0, 97.0)
                self.locked_live_point_link_name = None
                self.locked_live_point_local = None
                self.simulation_tab = type('S', (), {'live_point_locked': True, 'locked_live_point': (99.0, 98.0, 97.0)})()
                self.calls = []
            def log(self, msg):
                self.calls.append(msg)
            def update_live_ui(self, render=False):
                self.calls.append(('update_live_ui', render))
            def _configure_default_tcp(self):
                self.robot.ensure_tcp_transform(tool)
                self.custom_tcp_name = 'tool'
                return True

        dummy = type('D', (), {})()
        dummy.mw = _MW(robot)

        GripperPanel._auto_lock_live_point(dummy)

        assert dummy.mw.custom_tcp_name == 'tool'
        assert dummy.mw.live_point_locked is False
        assert dummy.mw.locked_live_point is None
        assert dummy.mw.locked_live_point_link_name == 'tool'
        assert dummy.mw.locked_live_point_local is None
        np.testing.assert_allclose(robot.get_tcp_world_pose(tool)[:3, 3], [0.0, 0.0, 0.0], atol=1e-6)

    def test_live_point_ignores_local_gripper_motion(self):
        robot = Robot()
        base = robot.add_link('base')
        flange = robot.add_link('flange')
        finger = robot.add_link('finger')
        robot.base_link = base
        robot.add_joint('arm', 'base', 'flange')
        grip = robot.add_joint('grip', 'flange', 'finger')
        grip.is_gripper = True
        finger.t_offset[:3, 3] = np.array([10.0, 0.0, 0.0], dtype=float)
        robot.update_kinematics()

        robot.set_tcp_transform('flange', position=[0.0, 0.0, 0.0])
        robot.update_kinematics()
        before = robot.get_tcp_world_pose(flange)[:3, 3].copy()

        grip.current_value = 45.0
        robot.update_kinematics()
        after = robot.get_tcp_world_pose(flange)[:3, 3].copy()

        np.testing.assert_allclose(after, before, atol=1e-6)

    def test_rigid_tcp_resolver_collapses_nested_gripper_parts_to_flange(self):
        from ui.main_window import MainWindow

        robot = Robot()
        base = robot.add_link('base')
        flange = robot.add_link('flange')
        finger = robot.add_link('finger')
        suction = robot.add_link('suction')
        robot.base_link = base
        robot.add_joint('arm', 'base', 'flange')
        grip = robot.add_joint('grip', 'flange', 'finger')
        grip.is_gripper = True
        robot.add_joint('cup', 'finger', 'suction')
        robot.update_kinematics()

        harness = type('MW', (), {'robot': robot})()

        assert MainWindow._is_gripper_child_link(harness, suction) is True
        assert MainWindow._resolve_rigid_tcp_link(harness, suction) is flange

    def test_default_tcp_ignores_stale_base_live_point_on_make_robo(self):
        from ui.main_window import MainWindow

        robot = Robot()
        base = robot.add_link('base')
        arm = robot.add_link('arm')
        flange = robot.add_link('flange')
        finger = robot.add_link('finger')
        robot.base_link = base
        base.is_base = True
        robot.add_joint('shoulder', 'base', 'arm')
        robot.add_joint('wrist', 'arm', 'flange')
        grip = robot.add_joint('grip', 'flange', 'finger')
        grip.is_gripper = True
        robot.set_tcp_transform('base', position=[0.0, -50.0, 15.0])
        robot.update_kinematics()

        harness = type('MW', (), {
            'robot': robot,
            'custom_tcp_name': 'base',
            'log': lambda self, msg: None,
            '_resolve_rigid_tcp_link': MainWindow._resolve_rigid_tcp_link,
            '_get_preferred_tcp_link': MainWindow._get_preferred_tcp_link,
        })()

        preferred = MainWindow._get_preferred_tcp_link(harness, include_current=False)
        assert preferred is flange
        assert MainWindow._configure_default_tcp(harness, include_current=False) is True
        assert harness.custom_tcp_name == 'flange'
        assert robot.get_tcp_local_transform(flange)[:3, 3].tolist() == [0.0, 0.0, 0.0]


    def test_committed_make_robo_tcp_wins_over_later_candidates(self):
        from ui.main_window import MainWindow

        robot = Robot()
        base = robot.add_link('base')
        arm = robot.add_link('arm')
        flange = robot.add_link('flange')
        finger = robot.add_link('finger')
        robot.base_link = base
        base.is_base = True
        robot.add_joint('shoulder', 'base', 'arm')
        robot.add_joint('wrist', 'arm', 'flange')
        grip = robot.add_joint('grip', 'flange', 'finger')
        grip.is_gripper = True
        robot.update_kinematics()

        harness = type('MW', (), {
            'robot': robot,
            'custom_tcp_name': 'flange',
            'locked_live_point_link_name': None,
            'locked_live_point_local': None,
            'log': lambda self, msg: None,
            '_resolve_rigid_tcp_link': MainWindow._resolve_rigid_tcp_link,
            '_get_preferred_tcp_link': MainWindow._get_preferred_tcp_link,
        })()

        assert MainWindow._commit_live_point_tcp(harness) is True
        assert harness.locked_live_point_link_name == 'flange'

        robot.set_tcp_transform('arm', position=[5.0, 0.0, 0.0])
        harness.custom_tcp_name = 'arm'

        preferred = MainWindow._get_preferred_tcp_link(harness)
        assert preferred is flange

    def test_manual_live_point_pick_cannot_override_committed_make_robo_tcp(self):
        from ui.panels.simulation_panel import SimulationPanel

        robot = Robot()
        base = robot.add_link('base')
        arm = robot.add_link('arm')
        flange = robot.add_link('flange')
        robot.base_link = base
        robot.add_joint('shoulder', 'base', 'arm')
        robot.add_joint('wrist', 'arm', 'flange')
        robot.update_kinematics()

        class _MW:
            def __init__(self):
                self.robot = robot
                self.custom_tcp_name = 'flange'
                self.locked_live_point_link_name = 'flange'
                self.logs = []
                self.toasts = []
                self.updated = False
            def _resolve_rigid_tcp_link(self, link):
                return link
            def log(self, msg):
                self.logs.append(msg)
            def show_toast(self, msg, kind='info'):
                self.toasts.append((msg, kind))
            def update_live_ui(self):
                self.updated = True

        panel = type('P', (), {'main_window': _MW()})()
        SimulationPanel._on_custom_lp_picked(panel, 'arm', np.array([10.0, 0.0, 0.0]), None)

        assert panel.main_window.custom_tcp_name == 'flange'
        assert panel.main_window.locked_live_point_link_name == 'flange'
        assert getattr(arm, 'custom_tcp_offset', None) is None
        assert panel.main_window.updated is True


    def test_make_robo_commit_preserves_displayed_live_point_as_local_tcp(self):
        from ui.main_window import MainWindow

        robot = Robot()
        base = robot.add_link('base')
        arm = robot.add_link('arm')
        flange = robot.add_link('flange')
        finger = robot.add_link('finger')
        robot.base_link = base
        base.is_base = True
        robot.add_joint('shoulder', 'base', 'arm')
        wrist = robot.add_joint('wrist', 'arm', 'flange')
        grip = robot.add_joint('grip', 'flange', 'finger')
        grip.is_gripper = True
        flange.t_offset[:3, 3] = np.array([10.0, 0.0, 20.0], dtype=float)
        robot.update_kinematics()

        harness = type('MW', (), {
            'robot': robot,
            'custom_tcp_name': None,
            'locked_live_point_link_name': None,
            'locked_live_point_local': None,
            'log': lambda self, msg: None,
            '_resolve_rigid_tcp_link': MainWindow._resolve_rigid_tcp_link,
            '_get_preferred_tcp_link': MainWindow._get_preferred_tcp_link,
            '_set_tcp_world_point': MainWindow._set_tcp_world_point,
        })()

        displayed_world = np.array([12.0, -7.0, 45.0], dtype=float)
        assert MainWindow._configure_default_tcp(harness, include_current=False, world_point=displayed_world) is True
        assert harness.custom_tcp_name == 'flange'
        assert MainWindow._commit_live_point_tcp(harness, world_point=displayed_world) is True
        np.testing.assert_allclose(robot.get_tcp_world_pose(flange)[:3, 3], displayed_world, atol=1e-6)

        local_before = robot.get_tcp_local_transform(flange)[:3, 3].copy()
        wrist.current_value = 45.0
        robot.update_kinematics()
        np.testing.assert_allclose(robot.get_tcp_local_transform(flange)[:3, 3], local_before, atol=1e-6)
        assert not np.allclose(robot.get_tcp_world_pose(flange)[:3, 3], displayed_world)





if __name__ == '__main__':
    unittest.main()