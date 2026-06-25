import sys
import os
import zipfile
import json
import numpy as np
import trimesh

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.robot import Robot, Link, Joint

def load_robot_from_trn(file_path):
    temp_dir = "scratch/temp_project_check_custom"
    os.makedirs(temp_dir, exist_ok=True)

    print(f"Loading project from: {file_path}")
    with zipfile.ZipFile(file_path, 'r') as zipf:
        zipf.extractall(temp_dir)

    json_path = os.path.join(temp_dir, "robot.json")
    with open(json_path, 'r') as f:
        robot_data = json.load(f)

    # Recreate the scale factor logic
    links = robot_data.get("links", [])
    has_modern_metadata = any(link.get("import_metadata") for link in links) or bool(
        robot_data.get("ui_state", {}).get("import_preferences")
    )
    
    legacy_scale_factor = 1.0
    if not has_modern_metadata:
        max_extent = 0.0
        max_translation = 0.0
        inspected = 0
        for link in links:
            mesh_rel_path = link.get("mesh_file")
            if not mesh_rel_path:
                continue
            mesh_path = os.path.join(temp_dir, mesh_rel_path)
            if not os.path.exists(mesh_path):
                continue
            mesh = trimesh.load(mesh_path)
            if isinstance(mesh, trimesh.Scene):
                mesh = mesh.to_mesh()
            if hasattr(mesh, "bounds"):
                bounds = np.array(mesh.bounds, dtype=float)
                if bounds.shape == (2, 3):
                    extent = float(np.max(bounds[1] - bounds[0]))
                    max_extent = max(max_extent, extent)
            t_offset = np.array(link.get("t_offset", np.eye(4)), dtype=float)
            if t_offset.shape == (4, 4):
                max_translation = max(max_translation, float(np.max(np.abs(t_offset[:3, 3]))))
            inspected += 1
        if inspected and max_extent <= 2.0 and max_translation <= 2.0:
            legacy_scale_factor = 1000.0

    print(f"Legacy scale factor: {legacy_scale_factor}")

    # Build the robot model
    robot = Robot()
    for l_data in robot_data["links"]:
        name = l_data["name"]
        mesh_rel_path = l_data["mesh_file"]
        mesh_path = os.path.join(temp_dir, mesh_rel_path)
        
        mesh = None
        if os.path.exists(mesh_path):
            try:
                raw_mesh = trimesh.load(mesh_path)
                mesh = raw_mesh.to_mesh() if isinstance(raw_mesh, trimesh.Scene) else raw_mesh
                if abs(legacy_scale_factor - 1.0) > 1e-12:
                    mesh.apply_scale(legacy_scale_factor)
            except Exception as e:
                print(f"Failed to load mesh {mesh_path}: {e}")
                
        link = robot.add_link(name, mesh)
        link.color = l_data.get("color", "lightgray")
        link.is_base = l_data.get("is_base", False)
        link.t_offset = np.array(l_data["t_offset"])
        if abs(legacy_scale_factor - 1.0) > 1e-12:
            link.t_offset[:3, 3] *= legacy_scale_factor
            
        if l_data.get("custom_tcp_offset") is not None:
            link.custom_tcp_offset = np.array(l_data["custom_tcp_offset"], dtype=float)
            if abs(legacy_scale_factor - 1.0) > 1e-12:
                link.custom_tcp_offset *= legacy_scale_factor
                
        link.custom_tcp_rpy_deg = l_data.get("custom_tcp_rpy_deg", [0.0, 0.0, 0.0])
        link.is_sim_obj = l_data.get("is_sim_obj", False)
        link.pick_pos = l_data.get("pick_pos", [0.0, 0.0, 0.0])
        link.place_pos = l_data.get("place_pos", [0.0, 0.0, 0.0])
        if abs(legacy_scale_factor - 1.0) > 1e-12:
            link.pick_pos = (np.array(link.pick_pos) * legacy_scale_factor).tolist()
            link.place_pos = (np.array(link.place_pos) * legacy_scale_factor).tolist()
            
        if link.is_base:
            robot.base_link = link

    for j_data in robot_data["joints"]:
        name = j_data["name"]
        parent_name = j_data["parent_link"]
        child_name = j_data["child_link"]
        
        if parent_name in robot.links and child_name in robot.links:
            joint = robot.add_joint(name, parent_name, child_name)
            joint.joint_type = j_data.get("joint_type", "revolute")
            joint.origin = np.array(j_data["origin"])
            joint.axis = np.array(j_data["axis"])
            if abs(legacy_scale_factor - 1.0) > 1e-12:
                joint.origin *= legacy_scale_factor
            joint.min_limit = j_data.get("min_limit", -180.0)
            joint.max_limit = j_data.get("max_limit", 180.0)
            joint.current_value = j_data.get("current_value", 0.0)

    robot.joint_relations = robot_data.get("joint_relations", {})
    
    # Try restoring home values or current values from file
    home_vals = {}
    for j_data in robot_data.get("joints", []):
        home_vals[j_data["name"]] = float(j_data.get("current_value", 0.0))
        
    ui_state = robot_data.get("ui_state", {})
    saved_home = ui_state.get("home_joint_values")
    if isinstance(saved_home, dict):
        for name, val in saved_home.items():
            if name in robot.joints:
                home_vals[name] = float(val)
                
    robot.home_joint_values = home_vals
    robot.reset_to_home()
    return robot

def main():
    candidate_paths = [
        r"C:\Users\Bhavin\OneDrive\Desktop\bHaViN\MANIPULATOR\1804.trn",
        r"c:\Users\Bhavin\OneDrive\Desktop\bHaViN\E-labs\assets\default_robot.trn"
    ]
    
    selected_path = None
    for path in candidate_paths:
        if os.path.exists(path):
            selected_path = path
            break
            
    if not selected_path:
        print("No .trn file found!")
        sys.exit(1)
        
    robot = load_robot_from_trn(selected_path)
    
    links_list = list(robot.links.values())
    def chain_len(link):
        return len(robot.get_kinematic_chain(link))
        
    tcp_candidates = [l for l in links_list if getattr(l, "custom_tcp_offset", None) is not None]
    if tcp_candidates:
        tcp_link = max(tcp_candidates, key=chain_len)
    else:
        tcp_link = max(links_list, key=chain_len)
        
    print(f"TCP Link: {tcp_link.name}")
    
    # Print current configuration
    chain = robot.get_kinematic_chain(tcp_link)
    print("Kinematic chain:")
    for j in chain:
        print(f"  {j.name}: limits=[{j.min_limit}, {j.max_limit}], val={j.current_value:.2f}, axis={j.axis}, origin={j.origin}")

    ratio = 10.0 # From ENGINE_UNITS_PER_CM
    print(f"Ratio (units per cm): {ratio}")
    
    curr_tcp_pose = robot.get_tcp_world_pose(tcp_link)
    print(f"Current TCP position in world (cm): {curr_tcp_pose[:3, 3] / ratio}")
    
    # We want to solve for target X: 30.0, Y: 20.0, Z: 30.0 cm
    target_pos_cm = np.array([30.0, 20.0, 30.0])
    target_world = target_pos_cm * ratio
    print(f"Target position in world (cm): {target_pos_cm}")
    print(f"Target position in world (mm): {target_world}")
    
    target_tcp_pose = curr_tcp_pose.copy()
    target_tcp_pose[:3, 3] = target_world
    
    # Try solving IK with navigation_mixin settings:
    print("\n--- Solving IK with navigation_mixin settings ---")
    success, info = robot.inverse_kinematics_pose(
        target_tcp_pose,
        tcp_link,
        max_iters=1000,
        position_tolerance=max(0.1 * ratio, 0.1),
        orientation_tolerance=1e6,
        orientation_weight=0.0,
        joint_change_weight=0.35,
    )
    print(f"Success: {success}")
    if success:
        robot.update_kinematics()
        final_tcp_pose = robot.get_tcp_world_pose(tcp_link)
        print(f"Reached position: {final_tcp_pose[:3, 3] / ratio} cm")
        print(f"Position error (cm): {np.linalg.norm(final_tcp_pose[:3, 3] - target_world) / ratio:.6f}")
        print("Solved Joint Angles:")
        for name, val in info["joint_values"].items():
            print(f"  {name}: {val:.2f}°")
    else:
        print("Solve failed!")
        # Let's inspect what's inside info
        robot.update_kinematics()
        final_tcp_pose = robot.get_tcp_world_pose(tcp_link)
        print(f"Failed reached position: {final_tcp_pose[:3, 3] / ratio} cm")
        print(f"Failed reached position error (cm): {np.linalg.norm(final_tcp_pose[:3, 3] - target_world) / ratio:.6f}")
        print("Best joint values found:")
        for name, val in info["joint_values"].items():
            print(f"  {name}: {val:.2f}°")
            
    # Let's run a check: what if we set joint_change_weight=0.0?
    print("\n--- Solving IK with joint_change_weight=0.0 ---")
    robot.reset_to_home()
    success, info = robot.inverse_kinematics_pose(
        target_tcp_pose,
        tcp_link,
        max_iters=1000,
        position_tolerance=max(0.1 * ratio, 0.1),
        orientation_tolerance=1e6,
        orientation_weight=0.0,
        joint_change_weight=0.0,
    )
    print(f"Success: {success}")
    if success:
        robot.update_kinematics()
        final_tcp_pose = robot.get_tcp_world_pose(tcp_link)
        print(f"Reached position: {final_tcp_pose[:3, 3] / ratio} cm")
        print(f"Position error (cm): {np.linalg.norm(final_tcp_pose[:3, 3] - target_world) / ratio:.6f}")
        print("Solved Joint Angles:")
        for name, val in info["joint_values"].items():
            print(f"  {name}: {val:.2f}°")
    else:
        print("Solve failed!")
        robot.update_kinematics()
        final_tcp_pose = robot.get_tcp_world_pose(tcp_link)
        print(f"Failed reached position: {final_tcp_pose[:3, 3] / ratio} cm")
        print(f"Failed reached position error (cm): {np.linalg.norm(final_tcp_pose[:3, 3] - target_world) / ratio:.6f}")
        print("Best joint values found:")
        for name, val in info["joint_values"].items():
            print(f"  {name}: {val:.2f}°")

if __name__ == "__main__":
    main()
