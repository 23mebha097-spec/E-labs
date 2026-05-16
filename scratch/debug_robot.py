import sys
import os

# Add the project root to sys.path
project_root = r"c:\Users\Bhavin\OneDrive\Desktop\bHaViN\E-labs"
if project_root not in sys.path:
    sys.path.append(project_root)

from core.robot import Robot, Link

def debug_robot():
    robot = Robot()
    # In a real scenario, the robot would be loaded from a project file.
    # But I can't easily load the user's current project state.
    # However, I can look at the main.py to see how it initializes.
    print("Checking joints...")

if __name__ == "__main__":
    debug_robot()
