#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool  # This will be replaced with our custom service
from example_interfaces.srv import AddTwoInts  # Placeholder for custom service import

class TaskGenerator(Node):
    def __init__(self, robot_name, mode, *argv):
        super().__init__('task_generator_node')
        self.service = self.create_service(AddTwoInts, 'get_instruction', self.handle_get_instruction)

    def handle_get_instruction(self, request, response):
        try:
            self.get_instruction(request.mode, request.pose)
            response.result = True
        except Exception as e:
            self.get_logger().error(f'Failed to execute mode {request.mode}: {str(e)}')
            response.result = False
        return response

    def get_instruction(self, mode, tf_target):
        move_actions = {
            "move_head": self.move_head,

            "arm_random": self.arm_random,
            "arm_touch": self.arm_touch,
            "arm_grasp": self.arm_grasp,
            "arm_put": self.arm_put,
            "arm_point": self.arm_point,
            "arm_move": self.arm_move,
            "arm_rest": self.arm_rest,

            "body_sit": self.body_sit,
            "body_run": self.body_run,
            "body_walk": self.body_walk,
            "body_crawl": self.body_crawl,
            "body_stand": self.body_stand,
            "body_lie_down": self.body_lie_down,
            "body_fall_over": self.body_fall_over,

            "leg_jump": self.leg_jump,
            "leg_kick": self.leg_kick,
        }
        action = move_actions.get(mode)
        if action:
            action(tf_target)
        else:
            self.get_logger().error("Mode name error")
            self.get_logger().error("Available moves: " + ", ".join(list(move_actions.keys())))
            return False
        return True


    def move_head(self):
        pass

    def arm_random(self):
        pass
    def arm_touch(self):
        pass
    def arm_grasp(self):
        pass
    def arm_put(self):
        pass
    def arm_point(self):
        pass
    def arm_move(self):
        pass
    def arm_rest(self):
        pass


    def body_sit(self):
        pass
    def body_run(self):
        pass
    def body_walk(self):
        pass
    def body_crawl(self):
        pass
    def body_stand(self):
        pass
    def body_lie_down(self):
        pass
    def body_fall_over(self):
        pass


    def leg_jump(self):
        pass
    def leg_kick(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    task_generator = TaskGenerator('robot', 'get_robot_param')
    rclpy.spin(task_generator)
    task_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
