import rclpy
from rclpy.node import Node
import yaml
from std_msgs.msg import String
import sys

class BodyActionPublisher(Node):
    def __init__(self):
        super().__init__('bodyaction_publisher')
        self.publisher_ = self.create_publisher(String, 'bodyaction', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        with open('bodyaction.yaml', 'r') as file:
            body_action = yaml.safe_load(file)
        
        action_str = yaml.dump(body_action)
        msg = String()
        msg.data = f"Action: {action_str}"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

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
