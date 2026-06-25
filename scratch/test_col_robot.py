import sys
sys.path.append('.')
import json
import numpy as np

# Mock GUI components to load project
from PyQt5.QtWidgets import QApplication
app = QApplication(sys.argv)

from ui.main_window import MainWindow

mw = MainWindow()
# Load the user's project
with open('robot.json', 'r') as f:
    data = json.load(f)

# Manually load the project state (simplified)
mw.new_project()
mw.robot.joints = {}
mw.robot.links = {}

# It's easier to just use project_mixin logic, let's trigger open project
mw._load_project_file('robot.json')

# Check collision
tcp_link = mw._get_preferred_tcp_link()
print("Collision:", mw.robot.has_self_collision(tcp_link=tcp_link))

for name, joint in mw.robot.joints.items():
    print(f"Joint {name}: {joint.current_value} deg")
