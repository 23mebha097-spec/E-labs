import sys
sys.path.append('.')
import json
import numpy as np
from core.project_manager import load_project_to_robot # or whatever

# let's just inspect the mesh bounds of the json
with open(r'c:\Users\Bhavin\OneDrive\Desktop\bHaViN\E-labs\scratch\temp_inspect_all\robot.json', 'r') as f:
    data = json.load(f)

for child_name, ui_data in data.get('ui_state', {}).get('joint_panel_joints', {}).items():
    print(child_name, ui_data.get('mesh_path'))
