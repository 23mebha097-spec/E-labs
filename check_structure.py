import zipfile, json
import sys
sys.path.insert(0, '.')

with zipfile.ZipFile('assets/default_robot.trn', 'r') as zf:
    with zf.open('robot.json') as f:
        data = json.load(f)
        
        # Print links and joints
        print('Links:')
        for link in data.get('links', []):
            name = link.get('name')
            print(f'  {name}')
            
        print('\nJoints (parent -> child):')
        for joint in data.get('joints', []):
            name = joint.get('name')
            parent = joint.get('parent')
            child = joint.get('child')
            print(f'  {name}: {parent} -> {child}')
