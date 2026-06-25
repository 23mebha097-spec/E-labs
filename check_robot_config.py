from core.robot import Robot
import numpy as np

# Load the robot
robot = Robot()

# Print robot structure
print("=== ROBOT STRUCTURE ===")
print(f"Number of links: {len(robot.links)}")
print(f"Number of joints: {len(robot.joints)}")

# Print all links
print("\n=== LINKS ===")
for name, link in robot.links.items():
    tcp_offset = getattr(link, "custom_tcp_offset", None)
    print(f"{name}:")
    print(f"  Position: {link.t_offset[:3, 3]}")
    print(f"  Has custom TCP: {tcp_offset is not None}")

# Print all joints
print("=== JOINTS ===")
for name, joint in robot.joints.items():
    parent_name = joint.parent_link.name if joint.parent_link else None
    child_name = joint.child_link.name if joint.child_link else None
    print(f"{name}:")
    print(f"  Parent: {parent_name}, Child: {child_name}")
    print(f"  Type: {joint.joint_type}")
    print(f"  Limits: [{joint.limits[0]:.2f}, {joint.limits[1]:.2f}]")
    print(f"  Current value: {joint.current_value:.2f}")

# Find TCP link
links = list(robot.links.values())
tcp_candidates = [l for l in links if getattr(l, "custom_tcp_offset", None) is not None]
if tcp_candidates:
    tcp_link = max(tcp_candidates, key=lambda l: len(robot.get_kinematic_chain(l)))
    print(f"\n=== TCP LINK: {tcp_link.name} ===")
    tcp_pose = robot.get_tcp_world_pose(tcp_link)
    print(f"Current TCP pose position (grid units): {tcp_pose[:3, 3]}")
    print(f"Current TCP pose position (cm): {tcp_pose[:3, 3] / 10}")
    
    # Try to reach home position
    target_cm = np.array([30.0, -30.0, 30.0])
    target_grid = target_cm * 10
    print(f"\nTarget position (cm): {target_cm}")
    print(f"Target position (grid units): {target_grid}")
    
    # Check if reachable
    workspace = robot.compute_workspace(tcp_link, sample_spacing=10)
    if workspace and workspace.get("ok"):
        print(f"\n=== WORKSPACE ===")
        print(f"Workspace OK: {workspace.get('ok')}")
        print(f"Workspace center: {workspace.get('center')}")
        print(f"Workspace radius: {workspace.get('radius')}")
        print(f"Distance from center to target: {np.linalg.norm(target_grid - workspace.get('center', [0,0,0]))}")
