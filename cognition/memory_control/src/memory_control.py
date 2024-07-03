import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import json
import socket

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription_odom = self.create_subscription(Odometry, 'odom', self.listener_callback_odom, 10)
        self.subscription_pose = self.create_subscription(PoseStamped, 'gps_pose', self.listener_callback_pose, 10)
        self.subscription_image = self.create_subscription(Image, 'image_data', self.listener_callback_image, 10)
        self.odom_data = None
        self.pose_data = None
        self.image_data = None

    def listener_callback_odom(self, msg):
        self.odom_data = msg
        self.log_data()

    def listener_callback_pose(self, msg):
        self.pose_data = msg
        self.log_data()

    def listener_callback_image(self, msg):
        self.image_data = msg
        self.log_data()

    def log_data(self):
        if self.odom_data and self.pose_data and self.image_data:
            data = {
                'odom': {
                    'position': {
                        'x': self.odom_data.pose.pose.position.x,
                        'y': self.odom_data.pose.pose.position.y,
                        'z': self.odom_data.pose.pose.position.z
                    },
                    'orientation': {
                        'x': self.odom_data.pose.pose.orientation.x,
                        'y': self.odom_data.pose.pose.orientation.y,
                        'z': self.odom_data.pose.pose.orientation.z,
                        'w': self.odom_data.pose.pose.orientation.w
                    }
                },
                'pose': {
                    'latitude': self.pose_data.pose.position.x,  # Assuming conversion is done elsewhere
                    'longitude': self.pose_data.pose.position.y
                },
                'image': 'Image data here',  # Placeholder for actual image handling
                'ip_address': socket.gethostbyname(socket.gethostname())
            }
            with open('data.json', 'w') as outfile:
                json.dump(data, outfile, indent=4)

def main(args=None):
    rclpy.init(args=args)
    my_subscriber = MySubscriber()
    rclpy.spin(my_subscriber)
    my_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
