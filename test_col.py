import sys
sys.path.append('.')
from core.project_manager import ProjectManager

def main():
    pm = ProjectManager()
    robot = pm.load_robot('scratch/temp_project_check_custom/robot.json')
    robot.update_kinematics()
    collision = robot.has_self_collision()
    print(f"Collision detected: {collision}")

if __name__ == '__main__':
    main()
