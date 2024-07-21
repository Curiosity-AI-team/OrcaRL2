#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import can
import cantools
import numpy as np

CHANNELS_NUMBER = 4

CAN_ID = 0x281
PAYLOAD_LENGTH = 8

LINEAR_VELOCITY_MAX = 1.2
LINEAR_VELOCITY_MIN = -1.2

ANGULAR_VELOCITY_MAX = 1.2
ANGULAR_VELOCITY_MIN = -1.2

#CH_MAX      = [1920, 1920, 1920, 1920]
#CH_MIN      = [1090, 1090, 1090, 1090]
CH_MAX       = [1800, 1920, 1920, 1920]
CH_MIN       = [1200, 1090, 1090, 1090]
CH_ZERO_MAX  = [1520, 1520, 1520, 1520]
CH_ZERO_MIN  = [1480, 1480, 1480, 1480]
CH_ZERO_TRIM = [True, True, False, False]
CH_INVERT    = [True, False, False, False]

STEERING_CHANNEL = 0
THROTTLE_CHANNEL = 1
DISARM_CHANNEL   = 2
AUTO_CHANNEL     = 3

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control')
        # self.declare_parameter('dbc', '/home/rover/gr_platform2/colcon_ws/src/control/joystick_control/config/joystick.dbc')
        # can_dbc = self.get_parameter('dbc').get_parameter_value().string_value
        can_dbc = '/home/rover/gr_platform2/colcon_ws/src/control/joystick_control/config/joystick.dbc'
        self.can_db = cantools.database.load_file(can_dbc)

        self.can_bus = can.Bus("can0", bustype="socketcan")
        self.can_bus.set_filters([{"can_id": 0x281, "can_mask": 0xFFF, "extended": False}])

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel_joy', 1)
        self.auto_trig_pub = self.create_publisher(Bool, 'joy_auto_trigger', 1)
        self.disarm_trig_pub = self.create_publisher(Bool, 'joy_disarm_trigger', 1)

        self.vel_msg = Twist()
        self.auto_trig_msg = Bool()
        self.disarm_trig_msg = Bool()

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.auto_trig_prev = None
        self.disarm_trig_prev = None
        self.init_trigger = False
        self.channels = [0] * CHANNELS_NUMBER

        self.create_timer(0.1, self.read_can_bus)

    def pwm_to_ratio(self, pwm_value, channel_idx):
        ratio = float(pwm_value - CH_MIN[channel_idx])/(CH_MAX[channel_idx] - CH_MIN[channel_idx])
        if CH_INVERT[channel_idx]:
            ratio = 1.0 - ratio
        return ratio

    def read_can_bus(self):
        for can_msg in self.can_bus:
            if len(can_msg.data) != 8:
                continue
            
            if all(byte == 0 for byte in can_msg.data):
                continue
            
            data = self.can_db.decode_message('Joystick', can_msg.data)
            self.process_joystick_data(data)

    def process_joystick_data(self, data):
        # Process data here and publish messages
        for i in range(CHANNELS_NUMBER):
            self.channels[i] = data['Channel_{}'.format(i)]

        for i in range(CHANNELS_NUMBER):
            self.channels[i] = np.clip(self.channels[i], CH_MIN[i], CH_MAX[i]) 
            if CH_ZERO_TRIM[i] and CH_ZERO_MIN[i] <= self.channels[i] and self.channels[i] <= CH_ZERO_MAX[i]:
                self.channels[i] = -1

        if self.channels[THROTTLE_CHANNEL] == -1:
            linear_velocity = 0.0
        else:
            factor = float(self.channels[THROTTLE_CHANNEL] - CH_MIN[THROTTLE_CHANNEL]) / (CH_MAX[THROTTLE_CHANNEL] - CH_MIN[THROTTLE_CHANNEL])
            if CH_INVERT[THROTTLE_CHANNEL]:
                factor = 1.0 - factor
            linear_velocity = ((1.0 - factor) * LINEAR_VELOCITY_MIN) + (LINEAR_VELOCITY_MAX * factor)

        if self.channels[STEERING_CHANNEL] == -1:
            angular_velocity = 0.0
        else:
            factor = float(self.channels[STEERING_CHANNEL] - CH_MIN[STEERING_CHANNEL])/(CH_MAX[STEERING_CHANNEL] - CH_MIN[STEERING_CHANNEL])
            if CH_INVERT[STEERING_CHANNEL]:
                factor = 1.0 - factor
            angular_velocity = ((1.0 - factor) * ANGULAR_VELOCITY_MIN) + (ANGULAR_VELOCITY_MAX * factor)

        auto_trig_ratio = self.pwm_to_ratio(self.channels[AUTO_CHANNEL], AUTO_CHANNEL)
        auto_trig = auto_trig_ratio > 0.5
        if self.auto_trig_prev is None:
            self.auto_trig_prev = auto_trig
        elif self.auto_trig_prev != auto_trig:
            self.auto_trig_prev = auto_trig
        self.auto_trig_msg.data = auto_trig
        self.auto_trig_pub.publish(self.auto_trig_msg)

        disarm_trig_ratio = self.pwm_to_ratio(self.channels[DISARM_CHANNEL], DISARM_CHANNEL)
        disarm_trig = disarm_trig_ratio > 0.5
        
        if disarm_trig == True:
            self.init_trigger = True

        if self.init_trigger:
            if self.disarm_trig_prev is None:
                self.disarm_trig_prev = disarm_trig
            elif self.disarm_trig_prev != disarm_trig:
                # print(disarm_trig)
                self.disarm_trig_prev = disarm_trig
                self.disarm_trig_msg.data = disarm_trig
                self.disarm_trig_pub.publish(self.disarm_trig_msg)
        
        self.vel_msg.linear.x = linear_velocity
        self.vel_msg.angular.z = angular_velocity
        self.vel_pub.publish(self.vel_msg)
        pass

def main(args=None):
    rclpy.init(args=args)
    joystick_control_node = JoystickControlNode()
    rclpy.spin(joystick_control_node)
    joystick_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
