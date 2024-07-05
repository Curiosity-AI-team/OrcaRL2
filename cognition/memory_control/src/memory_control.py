#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import PoseStamped
from fiducial_msgs.msg import FiducialTransformArray
from yolov8_msgs.msg import DetectionArray
import json
import socket
import time
import datetime
from random import uniform
import uuid
from geopy.geocoders import Nominatim
import sys
import os
import socket


class MyCombinedNode(Node):
    def __init__(self):
        super().__init__('my_combined_node')
        self.map_name = "odom_cmd"
        self.robot_name = "base_link"
        # Initialize the tf2 buffer and listener for TransformListener functionality
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.handle_timer)
        self.geolocator = Nominatim(user_agent="myGeocoder")
        # Subscriptions for Subscriber functionality
        # self.subscription_pose =        self.create_subscription(PoseStamped,   'gps_pose', self.listener_callback_pose, 10)
        self.subscription_image =       self.create_subscription(Image,         '/camera_rgb/image_raw', self.listener_callback_image, 10)
        self.subscription_gps =         self.create_subscription(NavSatFix,     '/gps/fix', self.listener_callback_gps, 10)
        self.subscription_fiducial =    self.create_subscription(FiducialTransformArray,      '/fiducial_transforms', self.listener_callback_fiducial, 10)
        self.subscription_yolo =        self.create_subscription(DetectionArray, '/detection_data', self.listener_callback_yolo, 10)
        
        # Data storage
        self.gps_data = None
        self.gps_time = None

        self.pose_data = None
        self.pose_time = None

        self.image_data = None
        self.image_time = None

        self.fiducial_data = None
        self.fiducial_time = None

        self.detection_data = None
        self.detection_time = None
        
        self.data3 = {}

    def get_address_from_coords(self, latitude, longitude):
        location = self.geolocator.reverse([latitude, longitude], exactly_one=True)
        return location.raw['address']

    def handle_timer(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.map_name, self.robot_name, rclpy.time.Time())
            # self.get_logger().info(f'Transform: {trans.transform}')
            self.pose_data = trans.transform
        
            if self.gps_data and self.image_data and self.fiducial_data: # and self.detection_data:
                update = True
                if self.gps_time-time.time() > 1:
                    self.get_logger().info(f"gps not updated")
                    update = False 
                if self.image_time-time.time() > 1:
                    self.get_logger().info(f"image not updated")
                    update = False
                if self.fiducial_time-time.time() > 1:
                    self.get_logger().info(f"fiducial not updated")
                    update = False
                # if self.detection_time-time.time() > 1:
                #     print("detection not updated")
                #     update = False
                
                if update == True:
                    self.get_logger().info(f'Trigger generate transform')
                    # check coordinate here
                    self.generate_json()
            else:
                self.get_logger().info('Waiting for all data to be available.')

        except tf2_ros.LookupException:
            self.get_logger().info('Transform not found')
        except tf2_ros.ConnectivityException:
            self.get_logger().info('Connectivity Error')
        except tf2_ros.ExtrapolationException:
            self.get_logger().info('Extrapolation Error')

    def listener_callback_gps(self, msg):
        self.gps_time = time.time()
        self.gps_data = msg

    def listener_callback_image(self, msg):
        self.image_time = time.time()
        self.image_data = msg

    def listener_callback_fiducial(self, msg):
        self.fiducial_time = time.time()
        for transform in msg.transforms:
            # print(f"ID: {transform.fiducial_id}, Coodrinate X: {transform.transform.translation.x}")
            self.fiducial_data = transform

    def listener_callback_yolo(self, msg):
        # Handle YOLO detection data
        pass

    def generate_json(self):
        file_path = '/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/cognition/database/output.json'
        json_data = {
            "_id": str(datetime.datetime.now()),
            "picture": "/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/cognition/database/",
            "address": str(self.get_local_ip()),
            "latitude": self.gps_data.latitude if self.gps_data else None,
            "longitude": self.gps_data.longitude if self.gps_data else None,
            "altitude": self.gps_data.altitude if self.gps_data else None,
            "map": self.map_name,
            "map_frame": [{"x": 0, "y": 0, "z": 0, "rx": 0, "ry": 0, "rz": 0, "rw": 1}],
            "robot": self.robot_name,
            "robot_frame": [{"x": self.pose_data.translation.x, "y": self.pose_data.translation.y, "z": self.pose_data.translation.z, "rx": self.pose_data.rotation.x, "ry": self.pose_data.rotation.y, "rz": self.pose_data.rotation.z, "rw": self.pose_data.rotation.w}],
            "target": self.fiducial_data.fiducial_id,
            "target_frame": [{"x": self.fiducial_data.transform.translation.x, "y": self.fiducial_data.transform.translation.y, "z": self.fiducial_data.transform.translation.z, "rx": self.fiducial_data.transform.rotation.x, "ry": self.fiducial_data.transform.rotation.y, "rz": self.fiducial_data.transform.rotation.z, "rw": self.fiducial_data.transform.rotation.w}],
            "emotion_0": "reserved",
            "emotion_1": "reserved",
            "emotion_2": "reserved",
            "reserved1": "reserved",
            "reserved2": "reserved",
            "reserved3": "reserved" 
        }

        self.data3[time.time()] = json_data
        self.save_json(self.data3, file_path)


    def save_json(self, data_local, save_path=None, save_mod="append"):
        if save_path == None:
            # save_path = f'{self.package_path}/tasks/{self.robot}/test.json'
            save_path = "/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/cognition/database/output.json"
        try:
            with open(save_path, 'r') as file:
                if os.stat(save_path).st_size == 0:
                    data = []
                else:
                    data = json.load(file)
        except FileNotFoundError:
            data = []
        except json.JSONDecodeError:
            # Attempt to repair or bypass corrupted JSON data
            print(f"File are corrupted: {save_path}")
            sys.exit()

        if save_mod=="append":
            data.append(data_local)
        elif save_mod=="extend":
            data.extend(data_local)
        else:
            print(f"save_mod are incorrect!")
            sys.exit()
        with open(save_path, 'w') as file:
            json.dump(data, file, indent=1)

    def get_local_ip(self):
        try:
            # Try to find an IPv4 address
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip_address = s.getsockname()[0]
            s.close()
            return ip_address
        except Exception as e:
            print(f"Error getting local IP: {e}")
            return None

    def trigger_zone():
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MyCombinedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
