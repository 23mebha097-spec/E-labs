import os
import zipfile
import json
import numpy as np
import trimesh
import sys
from scipy.optimize import minimize

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from core.robot import Robot

def load_robot():
    file_path = r"C:\Users\Bhavin\OneDrive\Desktop\bHaViN\MANIPULATOR\1804.trn"
    temp_dir = "scratch/temp_fk_check_scaled"
    with zipfile.ZipFile(file_path, 'r') as zipf:
        zipf.extractall(temp_dir)
    with open(os.path.join(temp_dir, "robot.json"), 'r') as f:
        robot_data = json.load(f)
    
    robot = Robot()
    legacy_scale_factor = 1000.0
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
    
    target_pos = np.array([30.77, 18.72, 30.16]) * 10.0 # to mm
    
    def loss(x):
        robot.joints["joint_01_02"].current_value = x[0]
        robot.joints["joint_02_03"].current_value = x[1]
        robot.joints["joint_03_04"].current_value = x[2]
        robot.joints["joint_04_05"].current_value = x[3]
        robot.joints["joint_05_06"].current_value = x[4]
        robot.joints["joint_06_07"].current_value = x[5]
        robot.update_kinematics()
        tcp_pos = robot.get_tcp_world_pose(tcp_link)[:3, 3]
        return np.linalg.norm(tcp_pos - target_pos)
        
    bounds = [
        (-180, 180),
        (-90, 90),
        (-90, 90),
        (-90, 90),
        (-180, 180),
        (0, 45)
    ]
    
    res = minimize(loss, x0=[-129.66, -90.0, -26.0, 26.0, 180.0, 0.0], bounds=bounds)
    print("Optimization success:", res.success)
    print("Function value (residual error in mm):", res.fun)
    print("Joint values:")
    joint_names = ["joint_01_02", "joint_02_03", "joint_03_04", "joint_04_05", "joint_05_06", "joint_06_07"]
    for name, val in zip(joint_names, res.x):
        print(f"  {name}: {val:.4f}°")

if __name__ == "__main__":
    main()
