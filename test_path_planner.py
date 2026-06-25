import numpy as np
import sys
from core.path_planner import WorkspacePlan, PathTrajectory, PathPlanner
from core.robot import Robot

def run_tests():
    print("==================================================")
    print("          STARTING PATH PLANNER CORE TESTS        ")
    print("==================================================")
    
    # ----------------------------------------------------
    # TEST 1: WorkspacePlan Geometry & Transformation Layer
    # ----------------------------------------------------
    print("\n--- Test 1: WorkspacePlan Transformations & Boundaries ---")
    origin = np.array([10.0, -5.0, 0.0]) # in cm
    workspace = WorkspacePlan(width=100.0, height=80.0, grid_size=10.0, safe_margin=5.0, origin=origin, inclination_deg=0.0)
    
    # 1.1 Bounds validation
    # limit_x = 50 - 5 = 45; limit_y = 40 - 5 = 35
    assert workspace.validate_workspace_bounds([40.0, 30.0, 0.0]) == True, "Point should be inside bounds"
    assert workspace.validate_workspace_bounds([46.0, 30.0, 0.0]) == False, "Point X should exceed limit"
    assert workspace.validate_workspace_bounds([40.0, -36.0, 0.0]) == False, "Point Y should exceed limit"
    print("[SUCCESS] Workspace boundaries validated successfully.")
    
    # 1.2 Transformations
    # Base in World Frame is shifted by (100, 200, 50) and rotated 90 degrees around Z axis
    theta = np.pi / 2.0
    r_base_world = np.array([
        [np.cos(theta), -np.sin(theta), 0.0],
        [np.sin(theta), np.cos(theta), 0.0],
        [0.0, 0.0, 1.0]
    ])
    base_world = np.eye(4)
    base_world[:3, :3] = r_base_world
    base_world[:3, 3] = [100.0, 200.0, 50.0]
    
    # Transform local workspace point to World
    # Workspace center is offset by (10, -5, 0) w.r.t Base.
    # So pt_loc = [0, 0, 0] w.r.t Workspace -> [10, -5, 0] w.r.t Base
    # Rotated by 90 deg around Z:
    # x_world = 100 + (10 * 0 - (-5) * 1) = 105
    # y_world = 200 + (10 * 1 + (-5) * 0) = 210
    # z_world = 50 + 0 = 50
    pt_local = np.array([0.0, 0.0, 0.0])
    pt_world = workspace.convert_local_to_world(pt_local, base_world)
    expected_world = np.array([105.0, 210.0, 50.0])
    assert np.allclose(pt_world, expected_world), f"Expected {expected_world}, got {pt_world}"
    
    # Invert transform: World -> Local
    pt_local_reconstructed = workspace.convert_world_to_local(pt_world, base_world)
    assert np.allclose(pt_local, pt_local_reconstructed), f"Expected {pt_local}, got {pt_local_reconstructed}"
    print("[SUCCESS] Workspace coordinate transformations (Local <-> World) validated successfully.")
    
    # ----------------------------------------------------
    # TEST 2: PathTrajectory Parametric Modeling & Sampling
    # ----------------------------------------------------
    print("\n--- Test 2: PathTrajectory Interpolation & Statistics ---")
    points = np.array([
        [0.0, 0.0, 0.0],
        [10.0, 0.0, 0.0],
        [10.0, 10.0, 0.0]
    ])
    timestamps = np.array([0.0, 1.0, 2.0])
    
    trajectory = PathTrajectory(points=points, timestamps=timestamps)
    
    # 2.1 Total length
    # Segment 1 length = 10, Segment 2 length = 10, Total = 20
    assert np.isclose(trajectory.get_total_length(), 20.0), f"Length should be 20, got {trajectory.get_total_length()}"
    print("[SUCCESS] Path cumulative distance statistics correct.")
    
    # 2.2 Time-sampling
    # At t = 0.5: intermediate between point 0 and 1: [5, 0, 0]
    pt, norm, rot, vel, acc, reach = trajectory.sample_at_time(0.5)
    expected_pt = np.array([5.0, 0.0, 0.0])
    assert np.allclose(pt, expected_pt), f"Expected {expected_pt}, got {pt}"
    assert np.allclose(norm, [0.0, 0.0, 1.0]), "Default normal should be Z-up"
    assert np.allclose(rot, np.eye(3)), "Default orientation should be Identity"
    print("[SUCCESS] Trajectory time-sampling interpolation matches analytical outputs.")
    
    # ----------------------------------------------------
    # TEST 3: PathPlanner Geometric Generation & Profiling
    # ----------------------------------------------------
    print("\n--- Test 3: PathPlanner Generators & Motion Profilers ---")
    planner = PathPlanner(workspace)
    
    # 3.1 Shape generation
    square_pts = planner.generate_square(center_x=0.0, center_y=0.0, side_length=30.0, z_height=10.0, num_points=100)
    assert square_pts.shape == (100, 3), "Square points shape incorrect"
    assert np.allclose(square_pts[0], [-15.0, -15.0, 10.0]), f"Square start point incorrect: {square_pts[0]}"
    
    wave_pts = planner.generate_wave(start_x=-20.0, start_y=0.0, end_x=20.0, end_y=0.0, amplitude=5.0, periods=2, z_height=8.0, num_points=80)
    assert wave_pts.shape == (80, 3), "Sinusoidal wave points shape incorrect"
    print("[SUCCESS] Square and Sine Wave paths generated successfully.")
    
    # 3.2 Spline interpolation
    interpolated = planner.interpolate_path(points, method="cubic", num_points=50)
    assert len(interpolated) == 50, f"Interpolated size should be 50, got {len(interpolated)}"
    print("[SUCCESS] Path interpolation (Catmull-Rom / Spline) completes without errors.")
    
    # 3.3 Trapezoidal velocity profiling
    traj_profiled = planner.apply_velocity_profile(points, max_vel=10.0, max_accel=5.0)
    assert len(traj_profiled.timestamps) == 3, "Profile should match point dimensions"
    assert traj_profiled.timestamps[0] == 0.0, "Start time should be 0.0"
    assert traj_profiled.timestamps[-1] > 0.0, "End time should be positive"
    print(f"[SUCCESS] Trapezoidal velocity profile calculated successfully. End time: {traj_profiled.timestamps[-1]:.3f} s.")
    
    # ----------------------------------------------------
    # TEST 4: IK Batch Verification & Safety Margin checks
    # ----------------------------------------------------
    print("\n--- Test 4: Batch IK Reachability Verification ---")
    
    # Construct a simple 2-DOF planar robot arm for actual IK testing
    robot = Robot()
    base_link = robot.add_link("base")
    base_link.is_base = True
    robot.base_link = base_link
    
    link1 = robot.add_link("link1")
    link1.t_offset = np.eye(4)
    link1.t_offset[0, 3] = 10.0 # Length of 10 cm in X
    
    j1 = robot.add_joint("joint1", "base", "link1")
    j1.axis = np.array([0.0, 0.0, 1.0])
    j1.min_limit = -180.0
    j1.max_limit = 180.0
    j1.origin = np.array([0.0, 0.0, 0.0])
    
    link2 = robot.add_link("link2")
    link2.t_offset = np.eye(4)
    link2.t_offset[0, 3] = 10.0 # Length of 10 cm in X
    
    j2 = robot.add_joint("joint2", "link1", "link2")
    j2.axis = np.array([0.0, 0.0, 1.0])
    j2.min_limit = -180.0
    j2.max_limit = 180.0
    j2.origin = np.array([0.0, 0.0, 0.0])
    
    robot.update_kinematics()
    
    print(f"Manually constructed test robot. Links: {list(robot.links.keys())}, Joints: {list(robot.joints.keys())}")
    
    # Define a path of points in workspace local frame
    # Base link has t_world = eye(4). Workspace origin is [10, -5, 0].
    # Local point [5, 5, 0] -> Base coordinates: [15, 0, 0].
    # Since total length of arm is 20 cm (10 + 10), [15, 0, 0] is reachable.
    # Local point [50, 50, 0] -> Base coords: [60, 45, 0] -> exceeds workspace boundaries AND physical arm reach -> unreachable.
    test_path = np.array([
        [5.0, 5.0, 0.0],     # Reachable
        [50.0, 50.0, 0.0]    # Out of bounds & Out of reach
    ])
    traj = PathTrajectory(points=test_path)
    
    # Backup initial joint angles
    initial_joints = {name: joint.current_value for name, joint in robot.joints.items()}
    
    flags = planner.check_reachability(robot, "link2", traj, orientation_weight=0.0)
    print(f"Reachability flags computed: {flags}")
    
    # Assert correctness of flags:
    # Point 0 is inside boundary and within reach -> True
    # Point 1 is out of bounds -> False (checked by validate_workspace_bounds)
    assert flags[0] == True, "First point [5, 5, 0] should be reachable"
    assert flags[1] == False, "Second point [50, 50, 0] should be unreachable (out of bounds)"
    
    # Verify robot state was restored
    for name, initial_val in initial_joints.items():
        current_val = robot.joints[name].current_value
        assert np.isclose(initial_val, current_val, atol=1e-5), f"Joint {name} was not restored!"
        
    print("[SUCCESS] Batch IK solver check completed with zero joint displacement side-effects.")

    # ----------------------------------------------------
    # TEST 5: PathExecutor States & Playback Logic
    # ----------------------------------------------------
    print("\n--- Test 5: PathExecutor Simulation Playback & States ---")
    from core.path_executor import PathExecutor, ExecutionState
    
    executor = PathExecutor()
    assert executor.state == ExecutionState.IDLE, "Initial state should be IDLE"
    
    # 5.1 Load trajectory
    pts_traj = np.array([
        [0.0, 0.0, 0.0],
        [10.0, 0.0, 0.0]
    ])
    times_traj = np.array([0.0, 2.0]) # 2 seconds duration
    traj_exec = PathTrajectory(points=pts_traj, timestamps=times_traj)
    executor.load_trajectory(traj_exec)
    assert executor.state == ExecutionState.READY, "State should be READY after load"
    assert np.isclose(executor.total_duration, 2.0), "Duration should be 2.0"
    
    # 5.2 Play state transitions
    executor.play()
    assert executor.state == ExecutionState.RUNNING, "State should be RUNNING after play"
    
    # 5.3 Deterministic tick step
    sample = executor.tick(dt=1.0)
    assert sample is not None, "Tick should return valid state"
    pt, norm, rot, vel, acc, reach = sample
    assert np.allclose(pt, [5.0, 0.0, 0.0]), f"Expected [5.0, 0.0, 0.0], got {pt}"
    assert executor.state == ExecutionState.RUNNING
    
    # 5.4 Pause & Resume
    executor.pause()
    assert executor.state == ExecutionState.PAUSED, "State should be PAUSED after pause"
    assert executor.tick(dt=0.5) is None, "Tick should return None when paused"
    
    executor.play()
    assert executor.state == ExecutionState.RUNNING, "State should be RUNNING after resume"
    
    # 5.5 Completion
    sample_end = executor.tick(dt=1.0) # Reach elapsed_time = 2.0
    assert sample_end is not None
    assert np.allclose(sample_end[0], [10.0, 0.0, 0.0])
    assert executor.state == ExecutionState.COMPLETED, "State should be COMPLETED"
    
    # 5.6 Reset / Stop
    executor.reset()
    assert executor.state == ExecutionState.READY, "State should be READY after reset"
    assert executor.elapsed_time == 0.0, "Elapsed time should reset to 0.0"
    
    # 5.7 Unreachable halt warning
    unreachable_traj = PathTrajectory(points=pts_traj, timestamps=times_traj, reachable_flags=[True, False])
    executor.load_trajectory(unreachable_traj)
    executor.play()
    assert executor.state == ExecutionState.RUNNING
    # Tick past t=0 should interpolate between point 0 (reachable) and point 1 (unreachable) -> reachable = False
    sample_unreach = executor.tick(dt=1.0)
    assert sample_unreach is not None
    assert executor.state == ExecutionState.ERROR, "State should go to ERROR on unreachable point"
    
    print("[SUCCESS] PathExecutor state machine and timing logic validated successfully.")
    
    print("\n==================================================")
    print("          ALL PATH PLANNER CORE TESTS PASSED!      ")
    print("==================================================")

if __name__ == "__main__":
    run_tests()
