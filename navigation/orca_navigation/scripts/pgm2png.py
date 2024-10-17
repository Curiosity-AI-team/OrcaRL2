import cv2
import sys
import yaml
import os
from pathlib import Path

if len(sys.argv) > 1:
    if len(sys.argv) == 4:
        map_name = ''.join(sys.argv[1])
        nav_path_name = ''.join(sys.argv[2])
        rmf_path_name = ''.join(sys.argv[3])
    else:
        print("Error: folder_path must be provided")
else:
    print("Error: map_name must be provided")
    sys.exit()

print(f"MAP NAME is {map_name}")
print(f"NAV PATH NAME is {nav_path_name}")
print(f"RMF PATH NAME is {rmf_path_name}")

# script_dir = Path(__file__).resolve().parent.parent.parent.parent
    
# directory = os.path.join(script_dir, f"{rmf_path_name}/maps/{map_name}")
directory = os.path.join(f"{rmf_path_name}/{map_name}")
if not os.path.exists(directory):
    os.makedirs(directory)

input_path = os.path.join(f"{nav_path_name}/2d_map/{map_name}.pgm")
# output_path = os.path.join(script_dir, f"{rmf_path_name}/maps/{map_name}/{map_name}.png")
output_path = os.path.join(f"{rmf_path_name}/{map_name}/{map_name}.png")

input_img = cv2.imread(input_path, -1)
cv2.imwrite(output_path, input_img)

yaml_input_path = os.path.join(f"{nav_path_name}/2d_map/{map_name}.yaml")
# yaml_output_path = os.path.join(script_dir, f"{rmf_path_name}/maps/{map_name}/{map_name}.param.yaml")
yaml_output_path = os.path.join(f"{rmf_path_name}/{map_name}/{map_name}.param.yaml")

with open(yaml_input_path) as file:
    original_data = yaml.safe_load(file)

origin = original_data['origin']
scale = original_data['resolution']
height, width = input_img.shape

new_params = {
    '/**': {  
        'ros__parameters': {
            'translation_x': origin[0],
            'translation_y': round(height*scale+origin[1], 2),
            'rotation': 0.0,
            'scale': scale
        }
    }
}

with open(yaml_output_path, 'w') as file:
    yaml.dump(new_params, file, default_flow_style=False)

print("Done!")
