#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import PoseStamped
from fiducial_msgs.msg import Fiducial
from yolov8_msgs.msg import DetectionArray
import json
import socket
import time
import datetime
from random import uniform
import uuid
from geopy.geocoders import Nominatim

class MyCombinedNode(Node):
    def __init__(self):
        super().__init__('my_combined_node')
        
        # Initialize the tf2 buffer and listener for TransformListener functionality
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.handle_timer)
        self.geolocator = Nominatim(user_agent="myGeocoder")
        # Subscriptions for Subscriber functionality
        self.subscription_pose =        self.create_subscription(PoseStamped,   'gps_pose', self.listener_callback_pose, 10)
        self.subscription_image =       self.create_subscription(Image,         'camera_image', self.listener_callback_image, 10)
        self.subscription_gps =         self.create_subscription(NavSatFix,     'gps_data', self.listener_callback_gps, 10)
        self.subscription_fiducial =    self.create_subscription(Fiducial,      'fiducial_data', self.listener_callback_fiducial, 10)
        self.subscription_yolo =        self.create_subscription(DetectionArray, 'detection_data', self.listener_callback_yolo, 10)
        
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


    def get_address_from_coords(self, latitude, longitude):
        location = self.geolocator.reverse([latitude, longitude], exactly_one=True)
        return location.raw['address']

    def handle_timer(self):
        # try:
        #     trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        #     self.get_logger().info(f'Transform: {trans.transform}')
        
            if self.gps_data and self.pose_data and self.image_data and self.fiducial_data and self.detection_data:
                update = True
                if self.gps_time-time.time() > 1:
                    print("gps not updated")
                    update = False
                if self.pose_time-time.time() > 1:
                    print("pose not updated")
                    update = False       
                if self.image_time-time.time() > 1:
                    print("image not updated")
                    update = False
                if self.fiducial_time-time.time() > 1:
                    print("fiducial not updated")
                    update = False
                if self.detection_time-time.time() > 1:
                    print("detection not updated")
                    update = False
                
                if update == True:
                    # check coordinate here
                    self.generate_json()

            else:
                self.get_logger().info('Waiting for all data to be available.')

        # except tf2_ros.LookupException:
        #     self.get_logger().info('Transform not found')
        # except tf2_ros.ConnectivityException:
        #     self.get_logger().info('Connectivity Error')
        # except tf2_ros.ExtrapolationException:
        #     self.get_logger().info('Extrapolation Error')

    def listener_callback_gps(self, msg):
        self.gps_data = msg

    def listener_callback_pose(self, msg):
        self.pose_data = msg

    def listener_callback_image(self, msg):
        self.image_data = msg

    def listener_callback_fiducial(self, msg):
        # Handle fiducial data
        pass

    def listener_callback_yolo(self, msg):
        # Handle YOLO detection data
        pass

    def generate_json(self):
        json_data = {
            "_id": str(uuid.uuid4()),
            "registered": datetime.datetime.now(),
            "picture": "http://placehold.it/32x32",
            "address": self.get_address_from_coords(self.gps_data.latitude, self.gps_data.longitude) if self.gps_data else None,
            "latitude": self.gps_data.latitude if self.gps_data else None,
            "longitude": self.gps_data.longitude if self.gps_data else None,
            "altitude": self.gps_data.altitude if self.gps_data else None,
            "map": "map",
            "map_frame": [{"x": 0, "y": 0, "z": 0, "rx": 0, "ry": 0, "rz": 0, "rw": 1}],
            "robot": "base_link",
            "robot_frame": [{"x": 0, "y": 0, "z": 0, "rx": 0, "ry": 0, "rz": 0, "rw": 1}],
            "target": "target",
            "target_frame": [{"x": 0, "y": 0, "z": 0, "rx": 0, "ry": 0, "rz": 0, "rw": 1}],
            "emotion_name": "reserved",
            "emotion_1": "reserved",
            "emotion_2": "reserved",
            "reserved1": "reserved",
            "reserved2": "reserved",
            "reserved3": "reserved"
        }
        with open('/home/vboxuser/orca_robot/colcon_ws/src/OrcaRL2/cognition/database/output.json', 'w') as json_file:
            json.dump(json_data, json_file, indent=4)

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
