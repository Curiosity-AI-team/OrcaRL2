import rclpy
from rclpy.node import Node
import time
import os
import json
import sys
import numpy as np
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
END_COORDINATE = "tf_end"

class TaskGenerator(Node):
    def __init__(self, robot_name, mode, *argv):
        super().__init__('task_generator_node')
        
        # ROS2 does not use rospkg directly, assuming package path is known or managed differently
        self.package_path = '/path/to/moveit_python'  # Update this path as necessary

        if mode not in ["check_json_files", "delete_json_sim_content", "delete_json_temp"]:
            self.bot = None
            if robot_name == "robot":
                robot_name = self.bot.get_group_names()[0]
                robot_name = robot_name.replace("_arm", "")
            elif not self.bot.get_group_names()[0] == f"{robot_name}_arm":
                print(f"Wrong robot name: {robot_name}")
                sys.exit()

        print(f"task_generator_node: | robot:{robot_name} | mode:{mode} |")
        self.arguments = sys.argv
        self.robot = robot_name
        self.home_dir = os.path.expanduser('~')
        self.mode = mode
        self.task_executer = True  # Set to False to disable execution to manipulator
        self.slow_move = True  # Set to False to run at max speed
        self.joint_data1 = {}
        self.joint_data2 = {}
        self.joint_data3 = {}
        self.name = None
        self.position = None
        self.rate = self.create_rate(10)  # Hz

        mode_actions = {
            "get_robot_param": self.get_robot_param,
            "joints_position": self.joints_position,
            "end_coordinate": self.end_coordinate,
            "spawn_object": self.spawn_object,
            "attach_object": self.attach_object,
            "detach_object": self.detach_object,
            "remove_object": self.remove_object,
            "gripper_open": self.gripper_open,
            "gripper_close": self.gripper_close,
            "choose_pipeline": self.choose_pipeline,
            "choose_follow_mode": self.choose_follow_mode,
            "clear_scene": self.clear_scene,
            "check_json_files": self.check_json_files,
            "delete_json_sim_content": self.delete_json_sim_content,
            "delete_json_temp": self.delete_json_temp,
        }

        action = mode_actions.get(mode)
        if action:
            action()
        else:
            print("Mode name error")
            print("Available modes:", list(mode_actions.keys()))
            sys.exit()

def main(args=None):
    rclpy.init(args=args)
    task_generator = TaskGenerator('robot', 'get_robot_param')  # Example usage
    rclpy.spin(task_generator)
    task_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
