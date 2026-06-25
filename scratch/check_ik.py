import sys
import os
import numpy as np

# Use offscreen platform for Qt so we can run headless
os.environ["QT_QPA_PLATFORM"] = "offscreen"

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from PyQt5 import QtWidgets, QtCore
    from ui.main_window import MainWindow
    from core.robot import Robot
except Exception as e:
    print(f"Import error: {e}")
    sys.exit(1)

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    
    # Load default robot project
    project_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "assets", "default_robot.trn")
    print(f"Loading project from: {project_path}")
    
    success = window.load_project_from_path(project_path, show_dialogs=False, auto_finalize=True)
    if not success:
        print("Failed to load project.")
        sys.exit(1)
        
    robot = window.robot
    print("\n--- Robot Configuration ---")
    print(f"Base link: {robot.base_link.name if robot.base_link else None}")
    
    tcp_link = window._get_preferred_tcp_link()
    print(f"Preferred TCP link: {tcp_link.name if tcp_link else None}")
    
    chain = robot.get_kinematic_chain(tcp_link)
    print("Kinematic chain:")
    for j in chain:
        print(f"  Joint: {j.name} (type={j.joint_type}, limits=[{j.min_limit}, {j.max_limit}], axis={j.axis}, origin={j.origin}, val={j.current_value})")
        
    print("\n--- Solving IK for target (30, 20, 30) cm ---")
    target_pos_cm = np.array([30.0, 20.0, 30.0])
    ratio = window.units_per_cm
    print(f"units_per_cm ratio: {ratio}")
    target_world = target_pos_cm * ratio
    print(f"Target world (mm): {target_world}")
    
    # Let's see current TCP pose in world
    curr_tcp_pose = robot.get_tcp_world_pose(tcp_link)
    print(f"Current TCP position: {curr_tcp_pose[:3, 3] / ratio} cm")
    
    target_tcp_pose = curr_tcp_pose.copy()
    target_tcp_pose[:3, 3] = target_world
    
    # Run inverse kinematics
    success, info = robot.inverse_kinematics_pose(
        target_tcp_pose,
        tcp_link,
        max_iters=1000,
        position_tolerance=max(0.1 * ratio, 0.1),
        orientation_tolerance=1e6,
        orientation_weight=0.0,
        joint_change_weight=0.35,
    )
    
    print(f"\nSolve Success: {success}")
    print("Solve Info:")
    for k, v in info.items():
        if k in ("joint_values", "joint_motion_deg"):
            print(f"  {k}:")
            for jname, val in v.items():
                print(f"    {jname}: {val:.4f}")
        elif isinstance(v, np.ndarray):
            print(f"  {k}: {v.tolist()}")
        else:
            print(f"  {k}: {v}")
            
    # Check final TCP position
    final_tcp_pose = robot.get_tcp_world_pose(tcp_link)
    final_pos_cm = final_tcp_pose[:3, 3] / ratio
    print(f"\nFinal TCP Position: {final_pos_cm.tolist()} cm")
    print(f"Distance to target: {np.linalg.norm(final_pos_cm - target_pos_cm):.4f} cm")

if __name__ == "__main__":
    main()
