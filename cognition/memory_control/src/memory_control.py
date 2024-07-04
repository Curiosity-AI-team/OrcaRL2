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
        self.subscription_pose = self.create_subscription(PoseStamped, 'gps_pose', self.listener_callback_pose, 10)
        self.subscription_image = self.create_subscription(Image, 'camera_image', self.listener_callback_image, 10)
        self.subscription_gps = self.create_subscription(NavSatFix, 'gps_data', self.listener_callback_gps, 10)
        self.subscription_fiducial = self.create_subscription(Fiducial, 'fiducial_data', self.listener_callback_fiducial, 10)
        self.subscription_yolo = self.create_subscription(DetectionArray, 'detection_data', self.listener_callback_yolo, 10)
        
        # Data storage
        self.gps_data = None
        self.pose_data = None
        self.image_data = None

    def get_address_from_coords(self, latitude, longitude):
        location = self.geolocator.reverse([latitude, longitude], exactly_one=True)
        return location.raw['address']

    def handle_timer(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.get_logger().info(f'Transform: {trans.transform}')
        except tf2_ros.LookupException:
            self.get_logger().info('Transform not found')
        except tf2_ros.ConnectivityException:
            self.get_logger().info('Connectivity Error')
        except tf2_ros.ExtrapolationException:
            self.get_logger().info('Extrapolation Error')

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
            "registered": time.time(),
            "picture": "http://placehold.it/32x32",
            "address": self.get_address_from_coords(self.gps_data.latitude, self.gps_data.longitude),
            "latitude": self.gps_data.latitude if self.gps_data else None,
            "longitude": self.gps_data.longitude if self.gps_data else None,
            "altitude": self.gps_data.altitude if self.gps_data else None,
            "map": "map",
            "map_frame": "map data coordinate from tf buffer",
            "robot": "base_link",
            "robot_frame": "base link data coordinate from tf buffer",
            "target": "target",
            "target_frame": "target data coordinate from tf buffer",
            "emotion_name": "Happy",
            "emotion_1": uniform(-90.0, 90.0),
            "emotion_2": uniform(-90.0, 90.0),
            "reserved1": None,
            "reserved2": None,
            "reserved3": None
        }
        with open('output.json', 'w') as json_file:
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
