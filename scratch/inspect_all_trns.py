import os
import zipfile
import json

def inspect_trn(file_path):
    try:
        temp_dir = "scratch/temp_inspect_all"
        os.makedirs(temp_dir, exist_ok=True)
        with zipfile.ZipFile(file_path, 'r') as zipf:
            zipf.extractall(temp_dir)
        json_path = os.path.join(temp_dir, "robot.json")
        if not os.path.exists(json_path):
            return
        with open(json_path, 'r') as f:
            data = json.load(f)
        
        joints = data.get("joints", [])
        joint_names = [j["name"] for j in joints]
        
        # Let's print if it matches the number of joints or if we want to print details
        print(f"\nFile: {file_path}")
        print(f"  Joints in model ({len(joints)}):")
        for j in joints:
            print(f"    {j['name']}: val={j.get('current_value'):.2f}, limits=[{j.get('min_limit'):.1f}, {j.get('max_limit'):.1f}]")
            
        ui_state = data.get("ui_state", {})
        print(f"  Speed in UI: {ui_state.get('current_speed')}")
        
    except Exception as e:
        print(f"  Error inspecting {file_path}: {e}")

def main():
    paths = [
        r"C:\Users\Bhavin\OneDrive\Desktop\bHaViN\KOZMAC\last.trn",
        r"C:\Users\Bhavin\OneDrive\Desktop\bHaViN\MANIPULATOR\0603.trn",
        r"C:\Users\Bhavin\OneDrive\Desktop\bHaViN\MANIPULATOR\0703.trn",
        r"C:\Users\Bhavin\OneDrive\Desktop\bHaViN\MANIPULATOR\final.trn",
        r"C:\Users\Bhavin\OneDrive\Desktop\bHaViN\MANIPULATOR\r1n.trn",
        r"C:\Users\Bhavin\OneDrive\Desktop\bHaViN\MANIPULATOR\restart.trn",
        r"C:\Users\Bhavin\OneDrive\Desktop\bHaViN\what\f4.trn"
    ]
    for p in paths:
        if os.path.exists(p):
            inspect_trn(p)

if __name__ == "__main__":
    main()
