"""
Simple robot demo: build a minimal 2-joint robot, show FK, then run IK to a target.

Run from repository root:
    python -m examples.simple_robot_demo

"""
from core.robot import Robot
import numpy as np


def build_simple_robot():
    r = Robot()
    base = r.add_link("base")
    l1 = r.add_link("link1")
    l2 = r.add_link("link2")
    r.base_link = base

    # Set link frame offsets (cm units used by this project)
    l1.t_offset[:3, 3] = np.array([0.0, 0.0, 10.0])
    l2.t_offset[:3, 3] = np.array([10.0, 0.0, 0.0])

    # Create joints connecting the links
    r.add_joint("j1", "base", "link1")
    r.add_joint("j2", "link1", "link2")

    # Optional: tighten joint limits to sensible values
    r.joints["j1"].min_limit = -90.0
    r.joints["j1"].max_limit = 90.0
    r.joints["j2"].min_limit = -90.0
    r.joints["j2"].max_limit = 90.0

    return r, l2


def main():
    r, tcp = build_simple_robot()

    # Set an initial pose by joint angles (degrees)
    r.set_joint_value("j1", 30.0)
    r.set_joint_value("j2", -20.0)
    r.update_kinematics()

    print("Initial joint values:")
    for name, joint in r.joints.items():
        print(f"  {name}: {joint.current_value} deg")

    print("TCP world transform (4x4):")
    print(r.get_tcp_world_pose(tcp))

    # Now attempt an IK target (Cartesian):
    target = np.eye(4)
    target[:3, 3] = np.array([12.0, 0.0, 8.0])

    success, info = r.inverse_kinematics_pose(target, tcp)
    print("\nIK result:", success)
    print("IK info keys:", list(info.keys()))
    if success:
        print("Solved joint values:")
        for k, v in info.get("joint_values", {}).items():
            print(f"  {k}: {v:.3f} deg")


if __name__ == "__main__":
    main()
