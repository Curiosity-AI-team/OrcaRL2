#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from mavros_msgs.msg import VFR_HUD
from std_msgs.msg import Bool
import can
import cantools
import time
import numpy as np
import struct

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.declare_parameters(namespace='', parameters=[
            ('dbc', None),
        ])
        dbc_file = self.get_parameter('dbc').get_parameter_value().string_value
        self.db = cantools.database.load_file(dbc_file)
        self.bus = can.Bus("can0", bustype="socketcan")

        self.bus.set_filters([
            {"can_mask": 0xFFF, "can_id": 0x01 | (0 << 5), "extended": False},
            {"can_mask": 0xFFF, "can_id": 0x01 | (1 << 5), "extended": False},
            {"can_mask": 0xFFF, "can_id": 0x01 | (2 << 5), "extended": False},
            {"can_mask": 0xFFF, "can_id": 0x01 | (3 << 5), "extended": False},
            
            {"can_mask": 0xFFF, "can_id": 0x09 | (0 << 5), "extended": False},
            {"can_mask": 0xFFF, "can_id": 0x09 | (1 << 5), "extended": False},
            {"can_mask": 0xFFF, "can_id": 0x09 | (2 << 5), "extended": False},
            {"can_mask": 0xFFF, "can_id": 0x09 | (3 << 5), "extended": False},
        ])

        self.vel_sub = self.create_subscription(Twist, 'diff_cont/cmd_vel_unstamped', self.cmd_vel_callback, 10)
        self.disarm_sub = self.create_subscription(Bool, 'cmd_trig', self.arm_callback, 10)
        # Uncomment and modify other subscribers as needed

        self.axis_list = [0, 1, 2, 3]
        self.control_dir = [1, 1, 1, -1]
        self.active_state = [1, 1, 1, 1]
        times = time.time()
        self.buffer = [times,times,times,times] 
        self.l_distance = 0.6
        self.last_msg_time = self.get_clock().now().to_sec()
        self.arm_msg = False
        self.ardupilot_vel_x = 0.0
        self.ardupilot_ang_z = 0.0
        self.vel_lin_setpoint = 0.0
        self.vel_ang_setpoint = 0.0
        self.DEBUG = False

    def arm_callback(self, msg):
        self.arm_msg = msg.data

    def cmd_vel_callback(self, msg):
        self.last_msg_time = self.get_clock().now().to_sec()
        self.vel_lin_setpoint = 1.2 * np.clip(msg.linear.x * 1.5, -4.0, 4.0)
        self.vel_ang_setpoint = 5.0 * np.clip(msg.angular.z * 1.5, -4.0, 4.0)

    def main_loop(self):

        for msg in self.bus:
            # Check the ros alive
            if rclpy.is_shutdown():
                print("bye")
                break
            
            if self.arm_msg:
                for i in self.axis_list:

                    # For each axis, check the error. Activate to clean and start the axis if detected
                    if(msg.arbitration_id == (i << 5 | 0x01)):
                        errorCode = msg.data[0] | msg.data[1] << 8 | msg.data[2] << 16 | msg.data[3] << 24
                        if not errorCode == 0x0:
                            print(f"Driver axis {i} error code: {str(hex(errorCode))}")
                            self.active_state[i] = 1
                    
                    # For each axis, check the timeout. Activate to clean and start the axis if timeout
                    if (time.time()-self.buffer[i]) > 1:
                        self.active_state[i] = 1
                    else:
                        if self.active_state[i] == 1:
                            self.active_state[i] = 0
                            data = self.db.encode_message('Clear_Errors', {})
                            self.bus.send(can.Message(arbitration_id=(0x18 | (i << 5)), is_extended_id=False, data=data))
                            data = self.db.encode_message('Set_Axis_State', {'Axis_Requested_State': 0x08})
                            self.bus.send(can.Message(arbitration_id=(0x07 | (i << 5)), is_extended_id=False, data=data))

                if (msg.arbitration_id & 0x1F) == 0x09:

                    axis_id = msg.arbitration_id >> 5
                    self.buffer[axis_id]=time.time()

                    # err_vel = 1.0*(vel_lin_setpoint - ardupilot_vel_x)
                    # err_ang = 1.0*(vel_ang_setpoint - ardupilot_ang_z)

                    data = self.db.decode_message('Get_Encoder_Estimates', msg.data)
                    v_cur = data['Vel_Estimate']

                    if time.time() - self.last_msg_time > 1:
                        wheel = 0
                    else: 
                        if axis_id%2 == 0:
                            wheel  = self.vel_lin_setpoint - self.vel_ang_setpoint*self.l_distance/2
                        else:
                            wheel = self.vel_lin_setpoint + self.vel_ang_setpoint*self.l_distance/2

                    self.bus.send(can.Message(arbitration_id=(axis_id << 5 | 0x00D), data=struct.pack('<ff', wheel*self.control_dir[axis_id], 0.0), is_extended_id=False))

                    if self.DEBUG:
                        if axis_id == 0:
                            # print(f"{round(err_vel,3)} {round(err_ang,3)}")
                            print(f"{round(self.vel_lin_setpoint,3)} {round(self.vel_ang_setpoint,3)}")

            time.sleep(0.004) # Set the delay for reducing the computational resources

        payload = self.db.encode_message('Set_Input_Torque', {'Input_Torque': 0.0})
        for axis_id in self.axis_list:
            msg = can.Message(arbitration_id=((axis_id << 5) | 0x00E), data=payload, is_extended_id=False)
            self.bus.send(msg)
        payload = self.db.encode_message('Set_Axis_State', {'Axis_Requested_State': 0x01})
        for axis_id in self.axis_list:
            msg = can.Message(arbitration_id=((axis_id << 5) | 0x007), data=payload, is_extended_id=False)
            self.bus.send(msg)


def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    try:
        control_node.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
