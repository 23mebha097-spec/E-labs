"""
Test script to diagnose why the robot doesn't reach (30, -30, 30)
even though IK reports 0.000 cm error.
"""

import sys
import os
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, PROJECT_ROOT)

from core.robot import Robot
import numpy as np

# Load the robot
robot = Robot()
trn_file = os.path.join(PROJECT_ROOT, "assets", "default_robot.trn")
if os.path.exists(trn_file):
    robot.load_from_trn(trn_file)
else:
    print(f"Error: {trn_file} not found")
    sys.exit(1)

# Target point
target_point = np.array([30.0, -30.0, 30.0])

# Get the TCP link (end effector)
links = list(robot.links.values())
tcp_link = None

def chain_len(link):
    return len(robot.get_kinematic_chain(link))

# Try to find the best TCP link
leaf_candidates = [l for l in links if not l.child_joints]
if leaf_candidates:
    tcp_link = max(leaf_candidates, key=chain_len)

if not tcp_link:
    tcp_link = max(links, key=chain_len)

print(f"Using TCP link: {tcp_link.name}")

# Test 1: Solve IK for the target point
print(f"\n=== Test 1: Solve IK for target ({target_point[0]}, {target_point[1]}, {target_point[2]}) ===")

# Create target pose (same orientation as current)
target_tcp_pose = robot.get_tcp_world_pose(tcp_link).copy()
target_tcp_pose[:3, 3] = target_point  # Set position

success, info = robot.inverse_kinematics_pose(
    target_tcp_pose,
    tcp_link,
    max_iters=3000,
    position_tolerance=0.01,
    orientation_tolerance=1e6,
    orientation_weight=0.0,
)

print(f"IK Success: {success}")
print(f"Position Error: {info.get('position_error', 'N/A'):.4f} cm")
print(f"Orientation Error: {info.get('orientation_error', 'N/A'):.4f} rad")

if success:
    # Get the kinematic chain
    chain = robot.get_kinematic_chain(tcp_link)
    print(f"Kinematic chain: {[j.name for j in chain]}")
    
    # Print the IK solution
    print("\nIK Solution (joint angles in degrees):")
    joint_values = info.get('joint_values', {})
    for joint_name in [j.name for j in chain]:
        print(f"  {joint_name}: {joint_values.get(joint_name, 0.0):.2f}°")
    
    # Verify the solution by computing FK
    print("\nVerifying IK solution with FK:")
    current_tcp_pose = robot.get_tcp_world_pose(tcp_link)
    current_pos = current_tcp_pose[:3, 3]
    print(f"Current TCP position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
    print(f"Target position:     ({target_point[0]:.3f}, {target_point[1]:.3f}, {target_point[2]:.3f})")
    error = np.linalg.norm(current_pos - target_point)
    print(f"Error: {error:.4f} cm")
    
    # Test 2: Try moving manually through animation steps
    print("\n=== Test 2: Simulate animation steps ===")
    
    # Save current state
    old_angles = {name: joint.current_value for name, joint in robot.joints.items()}
    
    # Extract target angles from chain only
    ordered_joint_ids = [joint.name for joint in chain]
    target_angles = [robot.joints[jid].current_value for jid in ordered_joint_ids]
    current_angles = [old_angles[jid] for jid in ordered_joint_ids]
    
    print(f"\nJoints to animate: {ordered_joint_ids}")
    print(f"Current angles: {[f'{a:.2f}' for a in current_angles]}")
    print(f"Target angles:  {[f'{a:.2f}' for a in target_angles]}")
    
    # Simulate animation (manually move from current to target)
    num_steps = 50
    for step in range(num_steps + 1):
        alpha = step / num_steps
        for idx, joint_id in enumerate(ordered_joint_ids):
            interpolated = current_angles[idx] + (target_angles[idx] - current_angles[idx]) * alpha
            robot.set_joint_value(joint_id, interpolated, propagate_relations=True)
        
        if step % 10 == 0 or step == num_steps:
            tcp_pose = robot.get_tcp_world_pose(tcp_link)
            pos = tcp_pose[:3, 3]
            error = np.linalg.norm(pos - target_point)
            print(f"Step {step:2d}/{num_steps}: Position ({pos[0]:7.3f}, {pos[1]:7.3f}, {pos[2]:7.3f}), Error: {error:.4f} cm")
    
    # Restore
    for name, val in old_angles.items():
        robot.joints[name].current_value = val
    robot.update_kinematics()
    
else:
    print(f"IK failed: {info}")
    print("\nThe point (30, -30, 30) may not be reachable for this robot configuration.")
