import unittest

import numpy as np

from core.robot import Robot as RobotModel
from elabs import Robot
from elabs.runtime import simulation_context


class DummyCanvas:
    grid_units_per_cm = 1.0

    def update_transforms(self, robot):
        self.last_robot = robot


class DummyWindow:
    def __init__(self):
        self.robot = RobotModel()
        base = self.robot.add_link("base")
        shoulder = self.robot.add_link("Shoulder")
        elbow = self.robot.add_link("Elbow")
        self.robot.base_link = base
        shoulder.t_offset[:3, 3] = np.array([0.0, 0.0, 10.0])
        elbow.t_offset[:3, 3] = np.array([10.0, 0.0, 0.0])
        self.robot.add_joint("Shoulder", "base", "Shoulder")
        self.robot.add_joint("Elbow", "Shoulder", "Elbow")
        self.robot.joints["Shoulder"].min_limit = -90.0
        self.robot.joints["Shoulder"].max_limit = 90.0
        self.robot.joints["Elbow"].min_limit = -45.0
        self.robot.joints["Elbow"].max_limit = 45.0
        self.robot.update_kinematics()

        self.canvas = DummyCanvas()
        self.current_speed = 50
        self.messages = []

    def log(self, message):
        self.messages.append(str(message))

    def on_speed_change(self, value):
        self.current_speed = value

    def _get_preferred_tcp_link(self):
        return self.robot.links["Elbow"]

    def _start_joint_animation(self, joint_ids, child_names, target_deg_list, **_kwargs):
        for joint_id, value in zip(joint_ids, target_deg_list):
            self.robot.set_joint_value(joint_id, value, propagate_relations=True)
        self.robot.update_kinematics()

    def move_joint_animated(self, joint_id, target_angle, **_kwargs):
        self.robot.set_joint_value(joint_id, target_angle, propagate_relations=True)
        return True

    def update_live_ui(self, render=True):
        self.live_ui_updated = render


class DummyPanel:
    def __init__(self):
        self.mw = DummyWindow()
        self.is_running = True


class ElabsRobotAPITest(unittest.TestCase):
    def test_robot_binds_to_active_simulation_and_clamps_joint_limits(self):
        panel = DummyPanel()
        with simulation_context(panel):
            robot = Robot()
            robot.set_speed(120)
            reached = robot.move_joint("Shoulder", 120)

        self.assertEqual(panel.mw.current_speed, 100)
        self.assertEqual(reached, 90.0)
        self.assertEqual(panel.mw.robot.joints["Shoulder"].current_value, 90.0)

    def test_home_and_feedback_use_live_robot_model(self):
        panel = DummyPanel()
        with simulation_context(panel):
            robot = Robot()
            robot.move_joint("Shoulder", 30)
            robot.move_joint("Elbow", -30)
            robot.home()
            feedback = robot.feedback()

        self.assertEqual(feedback["joints"]["Shoulder"], 0.0)
        self.assertEqual(feedback["joints"]["Elbow"], 0.0)
        self.assertEqual(len(feedback["tcp"]), 3)


if __name__ == "__main__":
    unittest.main()
