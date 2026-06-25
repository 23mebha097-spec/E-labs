"""
Draw square demo: generate a square path with the PathPlanner, profile it, and
check reachability with the robot model.

Run from repository root:
    python -m examples.draw_square_demo

"""
from core.robot import Robot
from core.path_planner import PathPlanner, WorkspacePlan
import numpy as np


def build_simple_robot():
    r = Robot()
    base = r.add_link("base")
    l1 = r.add_link("link1")
    l2 = r.add_link("link2")
    r.base_link = base
    l1.t_offset[:3, 3] = np.array([0.0, 0.0, 10.0])
    l2.t_offset[:3, 3] = np.array([10.0, 0.0, 0.0])
    r.add_joint("j1", "base", "link1")
    r.add_joint("j2", "link1", "link2")
    return r, l2


def main():
    r, tcp = build_simple_robot()

    # Create a planner with a workspace sized to the robot
    planner = PathPlanner(WorkspacePlan(width=40.0, height=30.0))

    # Generate a square (local workspace coordinates)
    square_pts = planner.generate_square(center_x=0.0, center_y=0.0, side_length=20.0, z_height=0.0, num_points=80)

    # Smooth / interpolate the path and apply a velocity profile
    interp = planner.interpolate_path(square_pts, method="catmull", num_points=240)
    traj = planner.apply_velocity_profile(interp, max_vel=15.0, max_accel=8.0)

    # Check IK reachability for each point
    reachable = planner.check_reachability(r, tcp, traj)
    count_ok = sum(1 for ok in reachable if ok)
    pct = 100.0 * count_ok / max(1, len(reachable))

    print(f"Trajectory points: {len(traj.points)}")
    print(f"Reachable points: {count_ok} ({pct:.1f}%)")
    print("Fully reachable?:", traj.is_fully_reachable())


if __name__ == "__main__":
    main()
