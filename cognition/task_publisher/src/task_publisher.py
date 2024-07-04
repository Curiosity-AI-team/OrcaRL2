#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import sys
from robot_interfaces.srv import ModePoseStamped  # Assuming the service definition is in robot_interfaces

class BodyActionPublisher(Node):
    def __init__(self):
        super().__init__('bodyaction_publisher')
        self.publisher_ = self.create_publisher(String, 'bodyaction', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.client = self.create_client(ModePoseStamped, 'get_instruction')

    def timer_callback(self):
        with open('bodyaction.yaml', 'r') as file:
            body_action = yaml.safe_load(file)
        
        action_str = yaml.dump(body_action)
        msg = String()
        msg.data = f"Action: {action_str}"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.send_request()

    def send_request(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            return
        request = ModePoseStamped.Request()
        request.mode = 'some_mode'  # Example mode
        request.pose = PoseStamped()  # Example PoseStamped, fill with actual data
        future = self.client.call_async(request)
        future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        try:
            response = future.result()
            if response.result:
                self.get_logger().info('Service call succeeded')
            else:
                self.get_logger().info('Service call failed')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

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
