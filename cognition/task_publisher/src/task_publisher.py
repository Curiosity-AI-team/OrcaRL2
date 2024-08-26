#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import json
import uuid
import yaml
from rmf_task_msgs.msg import ApiRequest, ApiResponse
from rmf_fleet_msgs.msg import FleetState, RobotMode
from typing import Tuple
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter
# Qos
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

# Read the map file in function timer_callback
# Read the json file/input from rasa in timer_callback_test and then publish to ros

class BodyActionPublisher(Node):
    def __init__(self):
        super().__init__('bodyaction_publisher')
        api_req_qos_profile = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)
        self.task_api_req_pub = self.create_publisher(ApiRequest, '/task_api_requests', api_req_qos_profile)
        timer_period = 2  # seconds
        self.timer1 = self.create_timer(timer_period, self.timer_callback)
        self.timer2 = self.create_timer(timer_period, self.timer_callback_test)
        self.map_data = None

    def timer_callback(self):
        file_path = "/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/simulation/rmf_demos/rmf_demos_maps/maps/office/office.building.yaml"
        try:
            with open(file_path, 'r') as file:
                self.map_data = yaml.safe_load(file)
        except FileNotFoundError:
            print(f"File {file_path} not found.")
            return None
        except yaml.YAMLError as e:
            print(f"YAML error occurred: {e}")
            return None
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            return None

    def timer_callback_test(self):

        if self.map_data == None:
            print(f"map_data is None")
            return None
        
        file_path = '/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/cognition/json_data/my_json.json'
        try:
            with open(file_path, 'r') as file:
                request_json = json.load(file)
        except FileNotFoundError:
            print(f"Error: File {file_path} not found.")
            return None
        except json.JSONDecodeError:
            print(f"Error: Invalid JSON format in {file_path}")
            return None
        
        for task in request_json:
            task_id, err_msg = self.submit_task_request(task)
            print(f"Task: {task_id} Error: {err_msg}")
            print("-"*80)

    def submit_task_request(self, req_json) -> Tuple[str, str]:
        """
        Task Submission - This function will trigger a ros srv call to the
        dispatcher node, and return a response. Function will return a Task ID

        Args:
            req_json: task description in json format
        Returns:
            task_id, error_msg: if submission failed
        """
        # construct task request json from legacy format
        request_json, err_msg = self.convert_task_description(req_json)
        if request_json is None:
            self.get_logger().error(err_msg)
            return "", err_msg
        payload = {
            "type": "dispatch_task_request",
            "request": request_json
        }

        msg = ApiRequest()
        msg.request_id = "demos_" + str(uuid.uuid4())
        msg.json_msg = json.dumps(payload)
        self.task_api_req_pub.publish(msg)
        self.get_logger().info(f'Publish task request {msg}')

        # Note: API Response or can wait for response
        # TODO: listen to "/task_api_responses"
        return msg.request_id, ""  # success
    
    def convert_task_description(self, task_json):
        """
        Convert a json task req format to legacy dashboard json msg format
        :note: The 'start time' here is the "Duration" from now.
        """
        # default request fields
        request = {
            "priority": {"type": "binary", "value": 0},
            "labels": ["rmf_demos.simple_api_server"],
            "description": {}
        }
        try:
            if (("task_type" not in task_json) or
                ("start_time" not in task_json) or
                    ("description" not in task_json)):
                raise Exception("Key value is incomplete")

            if ("priority" in task_json):
                priority_val = int(task_json["priority"])
                if (priority_val < 0):
                    raise Exception("Priority value is less than 0")
                request["priority"]["value"] = priority_val

            # Refer to task schemas
            # https://github.com/open-rmf/rmf_ros2/blob/redesign_v2/rmf_fleet_adapter/schemas
            desc = task_json["description"]
            if task_json["task_type"] == "Clean":
                request["category"] = "clean"
                request["description"]["zone"] = desc["cleaning_zone"]

            elif task_json["task_type"] == "Loop":
                request["category"] = "patrol"
                request["description"]["places"] = [
                    desc["start_name"],
                    desc["finish_name"]]
                request["description"]["rounds"] = int(desc["num_loops"])

            elif task_json["task_type"] == "Delivery":
                request["category"] = "delivery"
                request["description"]["pickup"] = {
                    "place": desc["pickup_place_name"],
                    "handler": desc["pickup_dispenser"],
                    "payload": []}
                request["description"]["dropoff"] = {
                    "place": desc["dropoff_place_name"],
                    "handler": desc["dropoff_ingestor"],
                    "payload": []}
                
            else:
                raise Exception("Invalid TaskType")

            # Calc earliest_start_time, convert "Duration from now(min)"
            # to unix_milli epoch time
            rclpy.spin_once(self, timeout_sec=0.0)
            rostime_now = self.get_clock().now()
            unix_milli_time = round(rostime_now.nanoseconds/1e6)
            unix_milli_time += int(task_json["start_time"]*60*1000)
            request["unix_millis_earliest_start_time"] = unix_milli_time
        except KeyError as ex:
            return None, f"Missing Key value in json body: {ex}"
        except Exception as ex:
            return None, str(ex)
        print("return", request)
        return request, ""
    
def main(args=None):
    rclpy.init(args=args)
    bodyaction_publisher = BodyActionPublisher()

    try:
        if len(sys.argv) > 1:
            input_command = sys.argv[1]
            print(f"Received command: {input_command}")
        rclpy.spin(bodyaction_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        bodyaction_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
