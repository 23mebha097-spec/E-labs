import zipfile
import json
import os

trn_file = 'assets/default_robot.trn'
if os.path.exists(trn_file):
    with zipfile.ZipFile(trn_file, 'r') as zf:
        print('Files in default_robot.trn:')
        zf.printdir()
        print('\n' + '='*50)
        with zf.open('robot.json') as f:
            robot_data = json.load(f)
            print(f'Number of links: {len(robot_data.get("links", []))}')
            print(f'Number of joints: {len(robot_data.get("joints", []))}')
            print(f'Number of joint relations: {len(robot_data.get("joint_relations", []))}')
            
            jr = robot_data.get('joint_relations', [])
            if jr:
                print(f'Joint relations:')
                for rel in jr:
                    print(f'  {rel}')
            else:
                print('No joint relations found')
                
            # Print joint info
            print(f'\nJoints:')
            for joint in robot_data.get('joints', []):
                print(f'  {joint.get("name")}: limits [{ joint.get("min_limit")}, {joint.get("max_limit")}]')
else:
    print(f'{trn_file} not found')
