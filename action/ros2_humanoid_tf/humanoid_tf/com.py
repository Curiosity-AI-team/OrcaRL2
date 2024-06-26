# Copyright 2020 Bold Hearts
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys

from geometry_msgs.msg import TransformStamped, Vector3
from lxml import objectify
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String
import tf2_ros
from visualization_msgs.msg import Marker


class COM(Node):
    def __init__(self):
        super().__init__("com")

        self.links = None
        self.total_mass = None

        # Set up TF listening
        self.tf2_buffer = tf2_ros.Buffer(node=self)
        self.tf2_listener = tf2_ros.TransformListener(buffer=self.tf2_buffer, node=self)

        # Listen to robot description
        self.create_subscription(
            String,
            "/robot_description",
            self.robot_description_callback,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
        )

        # Publisher for visualisation topic
        self.com_marker_pub = self.create_publisher(
            Marker, "/com/viz", rclpy.qos.qos_profile_sensor_data
        )

        # TF2 broadcaster
        self.tf2_broadcast = tf2_ros.TransformBroadcaster(self)

        self.create_timer(0.1, self.timer_callback)

    def robot_description_callback(self, msg):
        # Parse URDF XML into a tree of objects
        self.urdf_tree = objectify.fromstring(msg.data)

        # Find links that have inertial properties
        self.links = {
            l.get("name"): l
            for l in self.urdf_tree.link
            if l.find("inertial") is not None
        }

        # Calculate total robot mass
        total_mass = 0.0
        self.get_logger().info(f"# links: {len(self.links)}")
        for n, l in self.links.items():
            m = float(l.inertial.mass.get("value"))
            total_mass += m
            self.get_logger().debug(f"{n}: {m}")

        self.total_mass = total_mass
        self.get_logger().info(f"Total mass: {self.total_mass}")

    def timer_callback(self):
        # Wait until we have received and parsed the robot description
        if self.total_mass is None:
            return

        # Create transformation message
        # Describes the COM in the base_link frame coordinates
        com_msg = TransformStamped()
        com_msg.header.stamp = self.get_clock().now().to_msg()
        com_msg.header.frame_id = "base_link"
        com_msg.child_frame_id = "com"

        # Determine COM as weighted average of link coordinates,
        # weighted by their mass
        try:
            for name, link in self.links.items():
                transform = self.tf2_buffer.lookup_transform(
                    "base_link", name, rclpy.time.Time()
                )

                translation = transform.transform.translation
                link_mass = float(self.links[name].inertial.mass.get("value"))
                com_msg.transform.translation.x += translation.x * link_mass
                com_msg.transform.translation.y += translation.y * link_mass
                com_msg.transform.translation.z += translation.z * link_mass

            com_msg.transform.translation.x /= self.total_mass
            com_msg.transform.translation.y /= self.total_mass
            com_msg.transform.translation.z /= self.total_mass

            # Create and publish a marker for visualisation
            marker = _create_marker(com_msg.transform.translation)
            marker.header = com_msg.header
            self.com_marker_pub.publish(marker)

            # Broadcast transformation
            self.tf2_broadcast.sendTransform(com_msg)
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warning(f"Error transform: {name} - {e.args}")


def _create_marker(translation: Vector3) -> Marker:
    marker = Marker()
    marker.ns = "com"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.position.x = translation.x
    marker.pose.position.y = translation.y
    marker.pose.position.z = translation.z
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 0.8
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    return marker


def main(argv=sys.argv[1:]):
    rclpy.init(args=argv)

    try:
        node = COM()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
