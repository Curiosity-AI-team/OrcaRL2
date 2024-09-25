import cv2
import sys
import yaml
import os

if len(sys.argv) > 1:
    map_name = ' '.join(sys.argv[1:])
else:
    print("Error: map_name must be provided")
    sys.exit()
print(f"MAP NAME is {map_name}")

directory = f"/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/simulation/rmf_demos/rmf_demos_maps/maps/{map_name}"
if not os.path.exists(directory):
    os.makedirs(directory)

input = cv2.imread(f"/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/2d_map/{map_name}.pgm", -1)
cv2.imwrite(f"/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/simulation/rmf_demos/rmf_demos_maps/maps/{map_name}/{map_name}.png", input)


with open(f"/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/navigation/orca_navigation/2d_map/{map_name}.yaml") as file:
    original_data = yaml.safe_load(file)

origin = original_data['origin']

# Create a new dictionary with the required parameters
new_params = {
    '/**': {  # Puts all parameters in every node
        'ros__parameters': {
            'translation_x': origin[0],
            'translation_y': origin[1],
            'rotation': 0.0,  # Default value
            'scale': 1.0      # Default value
        }
    }
}

# Update the new_params dictionary with the origin values
# new_params.update({'translation_x': origin[0], 'translation_y': origin[1]})

# Write the new YAML file
with open(f"/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/simulation/rmf_demos/rmf_demos_maps/maps/{map_name}/{map_name}.param.yaml", 'w') as file:
    yaml.dump(new_params, file, default_flow_style=False)


print("Done!")