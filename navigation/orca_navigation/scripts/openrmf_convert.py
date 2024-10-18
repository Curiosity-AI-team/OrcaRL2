import yaml
import json
import sys
import itertools
import os
from pathlib import Path
import random
import time

def load_yaml(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Error: File not found at {file_path}")
        sys.exit()
    except yaml.YAMLError as exc:
        print(f"Error in YAML file: {exc}")
        sys.exit()

def initialize_fleet_config():
    return {
        'rmf_fleet': {
            'name': "customRobot",
            'fleet_manager': {
                'ip': "127.0.0.1",
                'port': 22011,
                'user': "some_user",
                'password': "some_password"
            },
            'limits': {
                'linear': [0.5, 0.75],
                'angular': [0.6, 2.0]
            },
            'profile': {
                'footprint': 0.3,
                'vicinity': 0.5
            },
            'reversible': True,
            'battery_system': {
                'voltage': 12.0,
                'capacity': 24.0,
                'charging_current': 5.0
            },
            'mechanical_system': {
                'mass': 20.0,
                'moment_of_inertia': 10.0,
                'friction_coefficient': 0.22
            },
            'ambient_system': {
                'power': 20.0
            },
            'tool_system': {
                'power': 0.0
            },
            'recharge_threshold': 0.10,
            'recharge_soc': 1.0,
            'publish_fleet_state': 10.0,
            'account_for_battery_drain': True,
            'task_capabilities': {
                'loop': True,
                'delivery': True,
                'clean': False,
                'finishing_request': "park"
            }
        },
        'robots': {}
    }

def extract_robot_info(building_data):
    crowd_sim = building_data.get('crowd_sim', {})
    agent_groups = crowd_sim.get('agent_groups', [])
    return [agent_name for group in agent_groups for agent_name in group.get('agents_name', [])]

def initialize_vertex_lists():
    return {
        'is_holding_point': [],
        'is_parking_spot': [],
        'spawn_robot_name_x': [],
        'spawn_robot_name_y': [],
        'charger_waypoint': [],
        'dropoff_ingestor': [],
        'pickup_dispenser': [],
        'pickup_dispenser_name': [],
        'dropoff_ingestor_name': [],
        'is_cleaning_zone': []
    }

def parse_vertices(vertices, vertex_lists):
    for vertex in vertices:
        if isinstance(vertex[-1], dict):
            attributes = vertex[-1]
            if attributes.get('is_charger', [None, False])[1]:
                vertex_lists['charger_waypoint'].append(vertex[3])
            if attributes.get('is_holding_point', [None, False])[1]:
                vertex_lists['is_holding_point'].append(vertex[3])
            if attributes.get('is_parking_spot', [None, False])[1]:
                vertex_lists['is_parking_spot'].append(vertex[3])
            if attributes.get('spawn_robot_name', [None, False])[1]:
                vertex_lists['spawn_robot_name_x'].append(vertex[0])
                vertex_lists['spawn_robot_name_y'].append(vertex[1])
            if attributes.get('dropoff_ingestor', [None, False])[1]:
                vertex_lists['dropoff_ingestor'].append(vertex[3])
            if attributes.get('pickup_dispenser', [None, False])[1]:
                vertex_lists['pickup_dispenser'].append(vertex[3])
            if attributes.get('is_cleaning_zone', [None, False])[1]:
                vertex_lists['is_cleaning_zone'].append(vertex[3])
        if len(vertex) > 4 and isinstance(vertex[4], dict):
            if 'pickup_dispenser' in vertex[4]:
                vertex_lists['pickup_dispenser_name'].append(vertex[4]['pickup_dispenser'][1])
            if 'dropoff_ingestor' in vertex[4]:
                vertex_lists['dropoff_ingestor_name'].append(vertex[4]['dropoff_ingestor'][1])

def generate_robot_config(fleet_config, robot_name, charger_waypoint, level_name, robot_index):
    fleet_config['robots'][robot_name] = {
        'robot_config': {
            'max_delay': 15.0
        },
        'rmf_config': {
            'robot_state_update_frequency': 10.0,
            'start': {
                'map_name': level_name,
                'waypoint': charger_waypoint[robot_index] if charger_waypoint else None,
                'orientation': 0.0
            },
            'charger': {
                'waypoint': charger_waypoint[robot_index] if charger_waypoint else None
            }
        }
    }

def write_yaml(output_file_path, data):

    directory = os.path.dirname(output_file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)

    with open(output_file_path, 'w') as file:
        yaml.dump(data, file, default_flow_style=False)
    print(f"YAML file generated at {output_file_path}")

def combine_lists(list_a, list_b):
    combinations = list(itertools.product(list_a, list_b))
    return sorted(set(combinations))

def generate_delivery_options(result, dict_pickup, dict_dropoff):
    delivery_options = {}
    for i, combination in enumerate(result, start=1):
        delivery_options[f"task_{i}"] = {
            "pickup_place_name": combination[0],
            "pickup_dispenser": dict_pickup[combination[0]],
            "dropoff_place_name": combination[1],
            "dropoff_ingestor": dict_dropoff[combination[1]]
        }
    return delivery_options

def generate_json_config(delivery_options, unique_elements, is_cleaning_zone):
    json_config = {
        "world_name": "World",
        "valid_task": [],
        "task": {
            "Delivery": {
                "option": delivery_options if delivery_options else {}
            },
            "Loop": {
                "places": unique_elements if unique_elements else []
            },
            "Clean": {
                "option": is_cleaning_zone if is_cleaning_zone else []
            },
            "Station": {},
            "Patrol": {},
            "Charging": {}
        }
    }

    if delivery_options:
        json_config["valid_task"].append("Delivery")
    else:
        json_config["task"]["Delivery"] = {}
    if unique_elements:
        json_config["valid_task"].append("Loop")
    else:
        json_config["task"]["Loop"] = {}
    if is_cleaning_zone:
        json_config["valid_task"].append("Clean")
    else:
        json_config["task"]["Clean"] = {}

    return json_config

def write_json(output_file_path, data):

    directory = os.path.dirname(output_file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)

    with open(output_file_path, "w") as json_file:
        json.dump(data, json_file, indent=4)
    print(f"JSON file generated at {output_file_path}")

def write_ros_tasks(output_ros_tasks, json_config):

    # Ensure the output directory exists
    directory = os.path.dirname(output_ros_tasks)
    if not os.path.exists(directory):
        os.makedirs(directory)

    with open(output_ros_tasks, "w") as md_file:
        md_file.write("# ROS2 Command List\n\n")

        # Write delivery tasks to markdown
        if json_config["task"]["Delivery"] != {}:
            md_file.write("## Delivery Tasks\n\n")
            delivery_data = json_config["task"]["Delivery"]["option"]
            for task_id, task_info in delivery_data.items():
                pickup_place_name = task_info.get('pickup_place_name', 'N/A')
                pickup_dispenser = task_info.get('pickup_dispenser', 'N/A')
                dropoff_place_name = task_info.get('dropoff_place_name', 'N/A')
                dropoff_ingestor = task_info.get('dropoff_ingestor', 'N/A')
                command = (f"ros2 run rmf_demos_tasks dispatch_delivery "
                        f"-p {pickup_place_name} -ph {pickup_dispenser} "
                        f"-d {dropoff_place_name} -dh {dropoff_ingestor} --use_sim_time")
                md_file.write(f"- `{command}`\n")

        # Write loop tasks to markdown
        if json_config["task"]["Loop"] != {}:
            md_file.write("\n## Loop Tasks\n\n")
            loop_data = json_config["task"]["Loop"]["places"]
            for i in range(len(loop_data)):
                rand_n1 = random.randint(0, len(loop_data) - 1)
                time.sleep(0.001)
                rand_n2 = random.randint(0, len(loop_data) - 1)
                time.sleep(0.001)
                r_n3 = random.randint(0, len(loop_data) - 1)
                command = (f"ros2 run rmf_demos_tasks dispatch_patrol "
                        f"-p {loop_data[rand_n1]} {loop_data[rand_n2]} "
                        f"-n {r_n3} --use_sim_time")
                md_file.write(f"- `{command}`\n")

    print(f"ROS commands generated at {output_ros_tasks}")

def read_yaml_scale(file_path):
    try:
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        
        # Access the nested structure
        data = data['/**']['ros__parameters']
        
        return data
    
    except FileNotFoundError:
        print(f"Error: File '{file_path}' not found.")
        sys.exit()
    except KeyError:
        print("Error: Invalid YAML structure.")
        sys.exit()
    except Exception as e:
        print(f"An error occurred: {e}")
        sys.exit()

def modify_cmakelist(cmake_file_path, map_name):

    lenght_map_indices = 0

    # Open and read the contents of the file
    with open(cmake_file_path, "r") as file:
        cmake_content = file.readlines()

    # Modify the "install(DIRECTORY ...)" command
    for i, line in enumerate(cmake_content):
        if "install(DIRECTORY" in line:
            map_name_list = []
            # Find the corresponding DESTINATION line and add the custom text
            while "DESTINATION" not in cmake_content[i]:
                map_name_list.append(cmake_content[i])
                i += 1

            lenght_map_indices = len([i for i, item in enumerate(map_name_list) if map_name in item])

            if lenght_map_indices == 0:
            # Add the custom text before the closing parenthesis
                cmake_content[i] = "  " + map_name + "\n" + cmake_content[i]

    if lenght_map_indices == 0:
        # Write the modified content back to the file
        with open(cmake_file_path, "w") as file:
            file.writelines(cmake_content)

        # Notify the user that the modification is complete
        print(f"Added install command to {cmake_file_path}")
    else:
        print(f"No map added to {cmake_file_path}")
        
if __name__ == "__main__":
    if len(sys.argv) > 1:
        map_name = ' '.join(sys.argv[1:])
    else:
        print("Error: map_name must be provided")
        sys.exit()

    script_dir = Path(__file__).resolve().parent.parent.parent.parent
    
    input_file_path = os.path.join(script_dir, f'simulation/rmf_demos/rmf_demos_maps/maps/{map_name}/{map_name}.building.yaml')
    # input_file_path = f'/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/simulation/rmf_demos/rmf_demos_maps/maps/{map_name}/{map_name}.building.yaml'
    print(f"Input file path: {input_file_path}")
    building_data = load_yaml(input_file_path)
    fleet_config = initialize_fleet_config()

    agent_names = extract_robot_info(building_data)
    print(f"Agent names: {agent_names}")

    levels = building_data.get('levels', {})
    level_name = next(iter(levels.keys()), 'Not found')
    print(f"Level names: {level_name}")
    l1_data = levels.get(level_name, {})
    vertices = l1_data.get('vertices', [])

    vertex_lists = initialize_vertex_lists()
    parse_vertices(vertices, vertex_lists)

    for i, robot_name in enumerate(agent_names):
        generate_robot_config(fleet_config, robot_name, vertex_lists['charger_waypoint'], level_name, i)

    combined_list = vertex_lists['is_holding_point'] + vertex_lists['is_parking_spot'] + vertex_lists['pickup_dispenser'] + vertex_lists['dropoff_ingestor']
    unique_elements = [element for element in set(combined_list) if element]

    dict_pickup = dict(zip(vertex_lists['pickup_dispenser'], vertex_lists['pickup_dispenser_name']))
    dict_dropoff = dict(zip(vertex_lists['dropoff_ingestor'], vertex_lists['dropoff_ingestor_name']))

    result = combine_lists(vertex_lists['pickup_dispenser'], vertex_lists['dropoff_ingestor'])
    delivery_options = generate_delivery_options(result, dict_pickup, dict_dropoff)

    json_config = generate_json_config(delivery_options, unique_elements, vertex_lists['is_cleaning_zone'])

    print("For the fake client, this is the tf coordnates from map->base_footprint to test spawn")
    yaml_input_path = os.path.join(script_dir, f"simulation/rmf_demos/rmf_demos_maps/maps/{map_name}/{map_name}.param.yaml")
    map_yaml = read_yaml_scale(yaml_input_path)

    scale = map_yaml['scale']
    map_x = map_yaml['translation_x']
    map_y = map_yaml['translation_y']

    x_list = vertex_lists["spawn_robot_name_x"]
    y_list = vertex_lists["spawn_robot_name_y"]
    for i in range(0,len(x_list)):
        print(f"{i}: x= {round(-map_x - x_list[i]*scale,2)} y= {round(-map_y + y_list[i]*scale,2)}")

    print("Finish!")
    print("#"*80)

    for i, robot_name in enumerate(agent_names):
        fleet_config['rmf_fleet']['name'] = robot_name        
        # output_yaml_path = f'/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/simulation/rmf_demos/rmf_demos/config/{map_name}/robot_config.yaml'
        output_yaml_path = os.path.join(script_dir, f'simulation/rmf_demos/rmf_demos/config/{map_name}/robot_config{i}.yaml')
        write_yaml(output_yaml_path, fleet_config)

    # output_json_path = f"/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/simulation/rmf_demos/rmf_demos_dashboard_resources/{map_name}/dashboard_config.json"
    output_json_path = os.path.join(script_dir, f'simulation/rmf_demos/rmf_demos_dashboard_resources/{map_name}/dashboard_config.json')
    write_json(output_json_path, json_config)

    output_ros_tasks = os.path.join(script_dir, f'GENERATED_ROS_COMMANDS.md')
    write_ros_tasks(output_ros_tasks, json_config)

    output_cmakelist = "/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/simulation/rmf_demos/rmf_demos_dashboard_resources/CMakeLists.txt"
    modify_cmakelist(output_cmakelist, map_name)