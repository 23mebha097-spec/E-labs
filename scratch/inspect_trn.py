import os
import zipfile
import json

file_path = r"C:\Users\Bhavin\OneDrive\Desktop\bHaViN\MANIPULATOR\1804.trn"
temp_dir = "scratch/temp_inspect"
os.makedirs(temp_dir, exist_ok=True)

with zipfile.ZipFile(file_path, 'r') as zipf:
    zipf.extractall(temp_dir)

json_path = os.path.join(temp_dir, "robot.json")
with open(json_path, 'r') as f:
    data = json.load(f)

print("Joint Relations:")
print(json.dumps(data.get("joint_relations"), indent=2))

print("\nJoints:")
for j in data["joints"]:
    print(f"  Name: {j['name']}, parent={j['parent_link']}, child={j['child_link']}, current_value={j.get('current_value')}")
