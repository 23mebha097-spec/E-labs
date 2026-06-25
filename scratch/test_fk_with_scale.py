import os
import zipfile
import json
import numpy as np
import trimesh
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from core.robot import Robot

def load_robot():
    file_path = r"C:\Users\Bhavin\OneDrive\Desktop\bHaViN\MANIPULATOR\1804.trn"
    temp_dir = "scratch/temp_fk_check_scaled"
    os.makedirs(temp_dir, exist_ok=True)
    with zipfile.ZipFile(file_path, 'r') as zipf:
        zipf.extractall(temp_dir)
    with open(os.path.join(temp_dir, "robot.json"), 'r') as f:
        robot_data = json.load(f)
    
    robot = Robot()
    legacy_scale_factor = 1000.0  # Apply the scale factor
    for l_data in robot_data["links"]:
        name = l_data["name"]
        mesh_rel_path = l_data["mesh_file"]
        mesh_path = os.path.join(temp_dir, mesh_rel_path)
        mesh = trimesh.load(mesh_path) if os.path.exists(mesh_path) else None
        if isinstance(mesh, trimesh.Scene):
            mesh = mesh.to_mesh()
        link = robot.add_link(name, mesh)
        link.is_base = l_data.get("is_base", False)
        link.t_offset = np.array(l_data["t_offset"])
        link.t_offset[:3, 3] *= legacy_scale_factor
        if l_data.get("custom_tcp_offset") is not None:
            link.custom_tcp_offset = np.array(l_data["custom_tcp_offset"], dtype=float) * legacy_scale_factor
        if link.is_base:
            robot.base_link = link
            
    for j_data in robot_data["joints"]:
        name = j_data["name"]
        joint = robot.add_joint(name, j_data["parent_link"], j_data["child_link"])
        joint.joint_type = j_data.get("joint_type", "revolute")
        joint.origin = np.array(j_data["origin"]) * legacy_scale_factor
        joint.axis = np.array(j_data["axis"])
        joint.min_limit = j_data.get("min_limit", -180.0)
        joint.max_limit = j_data.get("max_limit", 180.0)
        joint.current_value = j_data.get("current_value", 0.0)
        
    robot.joint_relations = robot_data.get("joint_relations", {})
    robot.update_kinematics()
    return robot

def main():
    robot = load_robot()
    tcp_link = robot.links["07"]
    
    screenshot_vals = {
        "joint_01_02": -136.43,
        "joint_02_03": -90.00,
        "joint_03_04": -3.75,
        "joint_04_05": 59.78,
        "joint_05_06": 180.00,
        "joint_06_07": 0.00
    }
    
    for name, val in screenshot_vals.items():
        if name in robot.joints:
            robot.joints[name].current_value = val
            
    robot.update_kinematics()
    
    ratio = 10.0
    tcp_pose = robot.get_tcp_world_pose(tcp_link)
    pos_cm = tcp_pose[:3, 3] / ratio
    print(f"TCP Position for screenshot values (with scale): {pos_cm.tolist()} cm")
    
    target_pos = np.array([30.0, 20.0, 30.0])
    err = np.linalg.norm(pos_cm - target_pos)
    print(f"Error to target (30, 20, 30): {err:.4f} cm")
    
    live_pos = np.array([30.77, 18.72, 30.16])
    err_live = np.linalg.norm(pos_cm - live_pos)
    print(f"Error to live point in screenshot (30.77, 18.72, 30.16): {err_live:.4f} cm")

if __name__ == "__main__":
    main()
