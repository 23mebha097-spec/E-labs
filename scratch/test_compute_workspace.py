import os
import sys
import json
import zipfile
import tempfile
import time
import numpy as np
import trimesh

sys.path.append(r"c:\Users\Bhavin\OneDrive\Desktop\bHaViN\E-labs")
from core.robot import Robot
from core.path_planner import WorkspacePlan

def test_workspace():
    file_path = r"C:\Users\Bhavin\OneDrive\Desktop\bHaViN\MANIPULATOR\1804.trn"
    if not os.path.exists(file_path):
        print("ERROR: File does not exist!")
        return

    robot = Robot()
    with tempfile.TemporaryDirectory() as temp_dir:
        with zipfile.ZipFile(file_path, 'r') as zipf:
            zipf.extractall(temp_dir)

        json_path = os.path.join(temp_dir, "robot.json")
        with open(json_path, 'r') as f:
            robot_data = json.load(f)

        for l_data in robot_data["links"]:
            name = l_data["name"]
            mesh_path = os.path.join(temp_dir, l_data["mesh_file"])
            if not os.path.exists(mesh_path):
                continue
            raw_mesh = trimesh.load(mesh_path)
            if isinstance(raw_mesh, trimesh.Scene):
                mesh = raw_mesh.to_mesh()
            else:
                mesh = raw_mesh
            link = robot.add_link(name, mesh)
            link.is_base = l_data.get("is_base", False)
            link.t_offset = np.array(l_data["t_offset"])
            if link.is_base:
                robot.base_link = link

        for j_data in robot_data["joints"]:
            name = j_data["name"]
            parent_name = j_data["parent_link"]
            child_name = j_data["child_link"]
            if parent_name in robot.links and child_name in robot.links:
                joint = robot.add_joint(name, parent_name, child_name)
                joint.origin = np.array(j_data["origin"])
                joint.axis = np.array(j_data["axis"])
                joint.min_limit = j_data.get("min_limit", -180.0)
                joint.max_limit = j_data.get("max_limit", 180.0)
                joint.current_value = j_data.get("current_value", 0.0)

        robot.update_kinematics()
        
        tcp_link_name = list(robot.links.keys())[-1]
        tcp_link_obj = robot.links[tcp_link_name]

        print("Measuring compute_workspace(max_samples=5000)...")
        start = time.time()
        res = robot.compute_workspace(tcp_link_obj, max_samples=5000)
        end = time.time()
        print(f"Done compute_workspace in {end - start:.4f} seconds!")

        print("Measuring auto_calculate_inclined_workspace...")
        reach = 0.0
        for joint in robot.joints.values():
            reach += np.linalg.norm(joint.origin)
        if reach < 10.0:
            reach = 100.0

        candidates = [
            (0.50 * reach, 0.25 * reach),
            (0.40 * reach, 0.20 * reach),
            (0.60 * reach, 0.30 * reach)
        ]

        def is_point_reachable(pos_world):
            dist_from_base = np.linalg.norm(pos_world - robot.base_link.t_world[:3, 3])
            if dist_from_base > reach * 1.05:
                return False

            target_pose = np.eye(4, dtype=float)
            target_pose[:3, 3] = pos_world
            
            saved_joints = {n: j.current_value for n, j in robot.joints.items()}
            
            success, _ = robot.inverse_kinematics_pose(
                target_tcp_pose=target_pose,
                tcp_link=tcp_link_obj,
                max_iters=20,
                position_tolerance=1.0,
                orientation_weight=0.0,
            )
            for n, val in saved_joints.items():
                robot.joints[n].current_value = val
            return success

        start = time.time()
        base_world = robot.base_link.t_world
        best_center = None
        best_size = (0.0, 0.0)
        for cx, cz in candidates:
            center_local_base = np.array([cx, 0.0, cz], dtype=float)
            center_world = (base_world @ np.append(center_local_base, 1.0))[:3]
            
            if not is_point_reachable(center_world):
                continue
                
            valid_w, valid_h = 0.0, 0.0
            sizes_to_test = [20.0, 35.0, 50.0, 70.0, 90.0]
            for size in sizes_to_test:
                if size > 0.8 * reach:
                    break
                    
                temp_ws = WorkspacePlan(width=size, height=size, origin=center_local_base, inclination_deg=45.0)
                test_pts_local = [
                    [-size/2.0, -size/2.0, 0.0],
                    [size/2.0, -size/2.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [-size/2.0, size/2.0, 0.0],
                    [size/2.0, size/2.0, 0.0]
                ]
                
                all_reachable = True
                for pt in test_pts_local:
                    pt_world = temp_ws.convert_local_to_world(pt, base_world)
                    if not is_point_reachable(pt_world):
                        all_reachable = False
                        break
                        
                if all_reachable:
                    valid_w = size
                    valid_h = size
                else:
                    break
                    
            if valid_w > best_size[0]:
                best_size = (valid_w, valid_h)
                best_center = center_local_base
        end = time.time()
        print(f"Done auto_calculate_inclined_workspace in {end - start:.4f} seconds!")

if __name__ == "__main__":
    test_workspace()
