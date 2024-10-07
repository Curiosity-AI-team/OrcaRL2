import cv2
import sys
import yaml
import os
from pathlib import Path

if len(sys.argv) > 1:
    map_name = ' '.join(sys.argv[1:])
else:
    print("Error: map_name must be provided")
    sys.exit()
print(f"MAP NAME is {map_name}")

script_dir = Path(__file__).resolve().parent.parent.parent.parent
    
directory = os.path.join(script_dir, f"simulation/rmf_demos/rmf_demos_maps/maps/{map_name}")
if not os.path.exists(directory):
    os.makedirs(directory)

input_path = os.path.join(script_dir, f"navigation/orca_navigation/2d_map/{map_name}.pgm")
output_path = os.path.join(script_dir, f"simulation/rmf_demos/rmf_demos_maps/maps/{map_name}/{map_name}.png")

input_img = cv2.imread(input_path, -1)
cv2.imwrite(output_path, input_img)

yaml_input_path = os.path.join(script_dir, f"navigation/orca_navigation/2d_map/{map_name}.yaml")
yaml_output_path = os.path.join(script_dir, f"simulation/rmf_demos/rmf_demos_maps/maps/{map_name}/{map_name}.param.yaml")

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
