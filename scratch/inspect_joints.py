import os
import sys
import json
import zipfile
import tempfile
import numpy as np

file_path = r"C:\Users\Bhavin\OneDrive\Desktop\bHaViN\MANIPULATOR\1804.trn"
with tempfile.TemporaryDirectory() as temp_dir:
    with zipfile.ZipFile(file_path, 'r') as zipf:
        zipf.extractall(temp_dir)
    json_path = os.path.join(temp_dir, "robot.json")
    with open(json_path, 'r') as f:
        robot_data = json.load(f)
    print("Joints origins:")
    for j in robot_data["joints"]:
        print(f"  {j['name']}: origin={j['origin']}, axis={j['axis']}")
