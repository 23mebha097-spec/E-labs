"""Simple Python facade for controlling the E-Labs virtual robot."""

import time

import numpy as np

from .runtime import get_simulation


class RobotAPIError(RuntimeError):
    """Raised when a robotics API command cannot be completed."""


class Gripper:
    """Gripper controls exposed as ``robot.gripper``."""

    def __init__(self, robot_api):
        self._robot_api = robot_api

    def open(self):
        """Open configured gripper fingers."""
        return self._robot_api._move_gripper(close=False)

    def close(self):
        """Close configured gripper fingers."""
        return self._robot_api._move_gripper(close=True)


class Robot:
    """
    User-facing E-Labs robotics API.

    When created inside the E-Labs Code panel, ``Robot()`` attaches to the live
    simulation automatically. A simulation object can also be passed explicitly
    for tests or custom integrations.
    """

    def __init__(self, simulation=None, blocking=True):
        self.simulation = simulation if simulation is not None else get_simulation()
        if self.simulation is None:
            raise RobotAPIError(
                "No active E-Labs simulation is bound. Run this script from the E-Labs Code panel "
                "or pass a simulation object to Robot(...)."
            )
        self.blocking = bool(blocking)
        self.gripper = Gripper(self)

    @property
    def _mw(self):
        return getattr(self.simulation, "mw", self.simulation)

    @property
    def _panel(self):
        if hasattr(self.simulation, "is_running"):
            return self.simulation
        return getattr(self._mw, "experiment_tab", None)

    @property
    def _robot_model(self):
        model = getattr(self._mw, "robot", None)
        if model is None:
            raise RobotAPIError("No robot model is loaded in the active E-Labs simulation.")
        return model

    def _is_running(self):
        panel = self._panel
        return bool(getattr(panel, "is_running", True))

    def _ensure_running(self):
        if not self._is_running():
            raise RobotAPIError("Robot script execution has been stopped.")

    def _log(self, message):
        logger = getattr(self._mw, "log", None)
        if callable(logger):
            logger(str(message))

    def _process_events(self):
        try:
            from PyQt5 import QtWidgets

            QtWidgets.QApplication.processEvents()
        except Exception:
            pass

    def _preferred_tcp_link(self):
        getter = getattr(self._mw, "_get_preferred_tcp_link", None)
        if callable(getter):
            tcp_link = getter()
            if tcp_link is not None:
                return tcp_link

        leaves = [
            link for link in self._robot_model.links.values()
            if getattr(link, "parent_joint", None) is not None and not getattr(link, "child_joints", [])
        ]
        if leaves:
            return leaves[-1]
        return None

    def _cm_ratio(self):
        canvas = getattr(self._mw, "canvas", None)
        return float(getattr(canvas, "grid_units_per_cm", 1.0) or 1.0)

    def _sync_after_direct_model_change(self):
        self._robot_model.update_kinematics()
        canvas = getattr(self._mw, "canvas", None)
        if canvas is not None and hasattr(canvas, "update_transforms"):
            canvas.update_transforms(self._robot_model)
        update_live_ui = getattr(self._mw, "update_live_ui", None)
        if callable(update_live_ui):
            update_live_ui(render=True)
        self._process_events()

    def _resolve_joint_name(self, name):
        model = self._robot_model
        if name in model.joints:
            return name

        lowered = str(name).lower()
        matches = [joint_name for joint_name in model.joints if joint_name.lower() == lowered]
        if len(matches) == 1:
            return matches[0]

        for joint_name, joint in model.joints.items():
            child = getattr(joint, "child_link", None)
            child_name = getattr(child, "name", "")
            if child_name.lower() == lowered:
                return joint_name

        available = ", ".join(model.joints.keys()) or "none"
        raise RobotAPIError(f"Unknown joint '{name}'. Available joints: {available}")

    def _joint_targets_for_home(self):
        targets = {}
        for joint_name, joint in self._robot_model.joints.items():
            value = self._robot_model.home_joint_values.get(joint_name, 0.0)
            targets[joint_name] = float(np.clip(value, joint.min_limit, joint.max_limit))
        return targets

    def _animate_joint_targets(self, targets):
        if not targets:
            return False

        joint_ids = []
        child_names = []
        target_values = []
        for joint_name, value in targets.items():
            joint = self._robot_model.joints[joint_name]
            joint_ids.append(joint_name)
            child_names.append(joint.child_link.name if joint.child_link else joint_name)
            target_values.append(float(value))

        starter = getattr(self._mw, "_start_joint_animation", None)
        if callable(starter):
            starter(joint_ids, child_names, target_values, blocking=self.blocking)
        else:
            for joint_name, value in targets.items():
                self._robot_model.set_joint_value(joint_name, value, propagate_relations=True)
            self._sync_after_direct_model_change()
        return True

    def home(self):
        """Animate all joints to configured home values, or zero when no home is configured."""
        self._ensure_running()
        self._log("Python API: moving robot to joint home.")
        return self._animate_joint_targets(self._joint_targets_for_home())

    def set_speed(self, percent):
        """Set animation speed as a percentage from 0 to 100."""
        self._ensure_running()
        value = int(round(float(np.clip(percent, 0, 100))))
        speed_setter = getattr(self._mw, "on_speed_change", None)
        if callable(speed_setter):
            speed_setter(value)
        else:
            setattr(self._mw, "current_speed", value)
        self._log(f"Python API: speed set to {value}%.")
        return value

    def move_joint(self, joint_name, angle):
        """Move a named joint to an absolute angle in degrees, respecting its limits."""
        self._ensure_running()
        resolved = self._resolve_joint_name(joint_name)
        joint = self._robot_model.joints[resolved]
        requested = float(angle)
        target = float(np.clip(requested, joint.min_limit, joint.max_limit))
        if abs(target - requested) > 1e-9:
            self._log(
                f"Python API: '{resolved}' target {requested:.2f} deg clamped to "
                f"{target:.2f} deg ({joint.min_limit:.2f}..{joint.max_limit:.2f})."
            )

        mover = getattr(self._mw, "move_joint_animated", None)
        if callable(mover):
            ok = bool(mover(resolved, target, blocking=self.blocking))
        else:
            ok = self._robot_model.set_joint_value(resolved, target, propagate_relations=True)
            self._sync_after_direct_model_change()
        if not ok:
            raise RobotAPIError(f"Unable to move joint '{joint_name}'.")
        return target

    def move(self, joint_name, angle):
        """Alias for ``move_joint``."""
        return self.move_joint(joint_name, angle)

    def move_tcp(self, x, y, z):
        """Move the TCP/live point to XYZ coordinates in centimeters using IK."""
        self._ensure_running()
        tcp_link = self._preferred_tcp_link()
        if tcp_link is None:
            raise RobotAPIError("No TCP/live point link is available for move_tcp(...).")

        mover = getattr(self._mw, "_move_tcp_to_xyz", None)
        if callable(mover):
            success, info = mover(float(x), float(y), float(z), tcp_link, blocking=self.blocking)
        else:
            ratio = self._cm_ratio()
            target_pose = self._robot_model.get_tcp_world_pose(tcp_link)
            target_pose[:3, 3] = np.array([float(x), float(y), float(z)], dtype=float) * ratio
            success, info = self._robot_model.inverse_kinematics_pose(
                target_pose,
                tcp_link,
                position_tolerance=max(0.01 * ratio, 0.01),
                orientation_tolerance=1e6,
                orientation_weight=0.0,
            )
            if success:
                self._sync_after_direct_model_change()

        if not success:
            raise RobotAPIError(f"Unable to reach TCP target ({x}, {y}, {z}) cm.")
        return info

    def move_xyz(self, x, y, z):
        """Alias for ``move_tcp``."""
        return self.move_tcp(x, y, z)

    def wait(self, seconds):
        """Wait while keeping the E-Labs UI responsive."""
        self._ensure_running()
        duration = max(0.0, float(seconds))
        end_time = time.time() + duration
        while time.time() < end_time:
            self._ensure_running()
            self._process_events()
            time.sleep(min(0.01, max(0.0, end_time - time.time())))
        return duration

    def get_joint(self, joint_name):
        """Return the current angle of a joint in degrees."""
        resolved = self._resolve_joint_name(joint_name)
        return float(self._robot_model.joints[resolved].current_value)

    def get_joint_names(self):
        """Return the available joint names."""
        return list(self._robot_model.joints.keys())

    def get_tcp(self):
        """Return current TCP/live point XYZ coordinates in centimeters."""
        tcp_link = self._preferred_tcp_link()
        if tcp_link is None:
            return [0.0, 0.0, 0.0]
        pose = self._robot_model.get_tcp_world_pose(tcp_link)
        return (pose[:3, 3] / self._cm_ratio()).astype(float).tolist()

    def feedback(self):
        """Return a real-time state snapshot for scripts and panels."""
        return {
            "joints": {name: self.get_joint(name) for name in self.get_joint_names()},
            "tcp": self.get_tcp(),
            "speed": float(getattr(self._mw, "current_speed", 50.0)),
        }

    def log(self, message):
        """Write a message to the E-Labs console."""
        self._log(message)

    def clear(self):
        """Clear the E-Labs console when available."""
        console = getattr(self._mw, "console", None)
        if console is not None and hasattr(console, "clear"):
            console.clear()

    def _move_gripper(self, close):
        self._ensure_running()
        controller = getattr(self._mw, "_control_gripper_fingers", None)
        if not callable(controller):
            raise RobotAPIError("The active E-Labs simulation does not expose gripper control.")

        targets = controller(close=close, apply=False)
        if not targets:
            raise RobotAPIError("No configured gripper joints found.")

        action = "closing" if close else "opening"
        self._log(f"Python API: {action} gripper.")
        return self._animate_joint_targets(targets)
