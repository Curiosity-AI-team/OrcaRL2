#!/usr/bin/env python3

from time import time
import sys
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from ZLAC8030L_CAN_controller.canopen_controller import MotorController
from differential_drive import DiffDrive
from pid import PID
import math
from zlac8030l_ros2.msg import State

class Driver(Node):
    def __init__(self):
        super().__init__('zlac_motor_driver')
        
        self._can_channel = self.get_parameter('can_channel', 'can0').value
        self._bus_type = self.get_parameter('bus_type', 'socketcan').value
        self._bitrate = self.get_parameter('bitrate', 500000).value
        self._eds_file = self.get_parameter('eds_file', '').value
        self._torque_mode = self.get_parameter('torque_mode', False ).value
        self._kp = self.get_parameter('vel_kp', 100).value
        self._ki = self.get_parameter('vel_ki', 10).value
        self._kd = self.get_parameter('vel_kd', 0).value
        # Create PIDs, one for each wheel
        self._vel_pids = {"fl":PID(kp=self._kp, ki=self._ki, kd=self._kd), "bl":PID(kp=self._kp, ki=self._ki, kd=self._kd), "br":PID(kp=self._kp, ki=self._ki, kd=self._kd), "fr":PID(kp=self._kp, ki=self._ki, kd=self._kd)}

        
        # Stores current wheel speeds [rpm]
        self._current_whl_rpm = {"fl": 0.0, "bl": 0.0, "br": 0.0, "fr": 0.0}
        # Target RPM
        self._target_whl_rpm = {"fl": 0.0, "bl": 0.0, "br": 0.0, "fr": 0.0}
        # Target torque; when slef._control_mode="torque"
        self._target_current = {"fl": 0.0, "bl": 0.0, "br": 0.0, "fr": 0.0}
        
        self._wheel_ids = {"fl":4, "bl":4, "br":4, "fr":4}
        self._flip_direction = {"fl": -1, "bl": -1, "br": 1, "fr": 1}

        self._wheel_radius = self.get_parameter('wheel_radius', 0.358).value
        self._track_width = self.get_parameter('track_width', 0.6).value
        self._max_vx = self.get_parameter('max_vx', 2.0).value
        self._max_w = self.get_parameter('max_w', 1.57).value
        self._max_lin_accel = self.get_parameter('max_lin_accel', 10).value
        self._max_ang_accel = self.get_parameter('max_ang_accel', 15).value
        self._odom_frame = self.get_parameter('odom_frame', 'odom_link').value
        self._robot_frame = self.get_parameter('robot_frame', 'base_link').value
        self._loop_rate = self.get_parameter('loop_rate', 100.0).value
        self._cmd_timeout = self.get_parameter('cmd_timeout', 0.1).value
        self._pub_tf = self.get_parameter('pub_tf', False).value


        self._diff_drive = DiffDrive(self._wheel_radius, self._track_width)
        self._last_odom_dt = time()
        self._last_cmd_t = time()
        self.arm_msg = False

        try:
            if (self._torque_mode):
                mode='torque'
            else:
                 mode='velocity'   
            self._network = MotorController(channel=self._can_channel, bustype=self._bus_type, bitrate=self._bitrate, node_ids=None, debug=True, eds_file=self._eds_file, mode=mode)
        except Exception as e:
            self.get_logger().error("Could not create CAN network object. Error: %s" % str(e))
            exit(0)

        self.get_logger().warn("\n ** cmd_vel must be published at rate more than %s Hz ** \n" % (1 / self._cmd_timeout))

        # ------------------- Subscribers ----------------#
        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmdVelCallback, 10)
        self.arm_callback_subscriber = self.create_subscription(Bool, 'cmd_trig', self.ArmCallback, 10)
        # ------------------- Publishers ----------------#
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._vel_pub = self.create_publisher(Float64, 'forward_vel', 10)

        self._motor_state_pub_dict = {}
        for wheel in ["fl", "bl", "br", "fr"]:
            self._motor_state_pub_dict[wheel] = self.create_publisher(State, wheel+"_motor/state", 10)

        # ------------------- Services ----------------#

        # TF broadcaster
        self._tf_br = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info(f'** Driver initialization is done **')

    def rpmToRps(self, rpm):
        return rpm / 9.5493

    def rpsToRpm(self, rad):
        return rad * 9.5493

    def applyControls(self):
        """
        Computes and applyies control signals based on the control mode (velocity vs. torque)
        """
        if (self._torque_mode):
            try:
                err_rpm = {"fr":0, "fl":0, "br":0, "bl":0}
                for t in ["fl", "bl", "br", "fr"]:
                    v_dict = self._network.getVelocity(node_id=self._wheel_ids[t])
                    vel = v_dict['value']* self._flip_direction[t] # flipping is required for odom
                    self._current_whl_rpm[t] = v_dict['value']

                    err_rpm[t] = self._target_whl_rpm[t] - self._current_whl_rpm[t]
                    self._target_current[t] = self._vel_pids["fr"].update(err_rpm[t])

            except Exception as e:
                self.get_logger().error("[applyControls] Error in getting wheel velocity: %s. Check driver connection", e)
        
        
            try:
                for t in ["fl", "bl", "br", "fr"]:
                    self._network.setTorque( node_id=self._wheel_ids[t], current_mA=self._target_current[t])
            except Exception as e:
                self.get_logger().error("[applyControls] Error in setting wheel torque: %s", e)
                
                
        else:
            # Send target velocity to the controller
            try:
                for t in ["fl", "bl", "br", "fr"]:
                    self._network.setVelocity(node_id=self._wheel_ids[t], vel=self._target_whl_rpm[t])
                
            except Exception as e:
                self.get_logger().error("[applyControls] Error in setting wheel velocity: %s", e)

    def ArmCallback(self, msg):
        self.arm_msg = msg.data

    
    def cmdVelCallback(self, msg):
        sign_x = -1 if msg.linear.x <0 else 1
        sign_w = -1 if msg.angular.z <0 else 1
        
        vx = msg.linear.x
        w = msg.angular.z

        # Initialize final commanded velocities,, after applying constraitns
        v_d = vx
        w_d = w

        # Limit velocity by acceleration
        current_t = time()
        dt = current_t - self._last_cmd_t
        self._last_cmd_t = current_t
        odom = self._diff_drive.calcRobotOdom(dt)
        current_v = odom['v']
        current_w = odom['w']

        # Figure out the max acceleration sign
        dv = vx-current_v
        abs_dv = abs(dv)
        if (abs_dv > 0):
            lin_acc = (abs_dv/dv)*self._max_lin_accel
        else:
            lin_acc = self._max_lin_accel

        dw = w-current_w
        abs_dw = abs(w-current_w)
        if (abs_dw > 0):
            ang_acc = dw/abs_dw * self._max_ang_accel
        else:
            ang_acc = self._max_ang_accel

        # Maximum acceptable velocity given the acceleration constraints, and current velocity
        max_v = current_v + dt*lin_acc
        max_w = current_w + dt*ang_acc

        # Compute & compare errors to deceide whether to scale down the desired velocity
        # For linear vel
        ev_d = abs(vx-current_v)
        ev_max = abs(max_v - current_v)
        if ev_d > ev_max:
            v_d=max_v

        # For angular vel
        ew_d = abs(w-current_w)
        ew_max = abs(max_w - current_w)
        if ew_d > ew_max:
            w_d = max_w

        if (abs(v_d) > self._max_vx):
            self.get_logger().warning("Commanded linear velocity %s is more than maximum magnitude %s", sign_x*vx, sign_x*self._max_vx)
            v_d = sign_x * self._max_vx
        if (abs(w_d) > self._max_w):
            self.get_logger().warning("Commanded angular velocity %s is more than maximum magnitude %s", sign_w*w, sign_w*self._max_w)
            w_d = sign_w * self._max_w

        # Compute wheels velocity commands [rad/s]
        (wl, wr) = self._diff_drive.calcWheelVel(v_d,w_d)

        # TODO convert rad/s to rpm
        wl_rpm = self.rpsToRpm(wl)
        wr_rpm = self.rpsToRpm(wr)
        self._target_whl_rpm["fl"] = wl_rpm * self._flip_direction["fl"]
        self._target_whl_rpm["bl"] = wl_rpm * self._flip_direction["bl"]
        self._target_whl_rpm["br"] = wr_rpm * self._flip_direction["br"]
        self._target_whl_rpm["fr"] = wr_rpm * self._flip_direction["fr"]

        # Apply control in the main loop
        

    class Quaternion:
        w: float
        x: float
        y: float
        z: float

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = self.Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        return q   

    def pubOdom(self):
        """Computes & publishes odometry msg
        """
        try:
            for t in ["fl", "bl", "br", "fr"]:

                v_dict = self._network.getVelocity(node_id=self._wheel_ids[t])
                vel = v_dict['value']* self._flip_direction[t] # flipping is required for odom
                self._current_whl_rpm[t] = v_dict['value']
                if t=="fl":
                    self._diff_drive._fl_vel = self.rpmToRps(vel)
                if t=="fr":
                    self._diff_drive._fr_vel = self.rpmToRps(vel)
                if t=="bl":
                    self._diff_drive._bl_vel = self.rpmToRps(vel)
                if t=="br":
                    self._diff_drive._br_vel = self.rpmToRps(vel)
        except Exception as e :
            self.get_logger().error(" Error in pubOdom: %s. Check driver connection", e)
            #self.get_logger().error("Availabled nodes = %s", self._network._network.scanner.nodes)

        now = time()

        dt= now - self._last_odom_dt
        self._last_odom_dt = now

        odom = self._diff_drive.calcRobotOdom(dt)

        msg = Odometry()

        time_stamp = self.get_clock().now()
        msg.header.stamp = time_stamp
        msg.header.frame_id=self._odom_frame
        msg.child_frame_id = self._robot_frame

        msg.pose.pose.position.x = odom["x"]
        msg.pose.pose.position.y = odom["y"]
        msg.pose.pose.position.z = 0.0
        odom_quat = self.quaternion_from_euler(0, 0, odom['yaw'])
        msg.pose.pose.orientation.x = odom_quat[0]
        msg.pose.pose.orientation.y = odom_quat[1]
        msg.pose.pose.orientation.z = odom_quat[2]
        msg.pose.pose.orientation.w = odom_quat[3]
        # pose covariance
        msg.pose.covariance[0] = 1000.0 # x-x
        msg.pose.covariance[7] = 1000.0 # y-y
        msg.pose.covariance[14] = 1000.0 # z-z
        msg.pose.covariance[21] = 1000.0 # roll
        msg.pose.covariance[28] = 1000.0 # pitch
        msg.pose.covariance[35] = 1000.0 # yaw

        # For twist, velocities are w.r.t base_link. So, only x component (forward vel) is used
        msg.twist.twist.linear.x = odom['v']
        msg.twist.twist.linear.y = 0 #odom['y_dot']
        msg.twist.twist.angular.z = odom['w']
        msg.twist.covariance[0] = 0.1 # vx
        msg.twist.covariance[7] = 0.1 # vx
        msg.twist.covariance[14] = 1000.0 # vz
        msg.twist.covariance[21] = 1000.0 # omega_x
        msg.twist.covariance[28] = 1000.0 # omega_y
        msg.twist.covariance[35] = 0.1 # omega_z
        
        self._odom_pub.publish(msg)
        if self._pub_tf:
            # Send TF
            self._tf_br.sendTransform((odom['x'],odom['y'],0),odom_quat,time_stamp,self._robot_frame,self._odom_frame)

        msg = Float64()
        msg.data = odom["v"] # Forward velocity
        self._vel_pub.publish(msg)

    def pubMotorState(self):
        for t in ["fl", "bl", "br", "fr"]:
            msg = State()
            msg.header.stamp = self.get_clock().now()
            msg.nodeid = self._wheel_ids[t]
            
            # Voltage
            try:
                volts_dict = self._network.getVoltage(self._wheel_ids[t])
                volts = volts_dict['value']
                msg.voltage = volts
            except:
                pass

            # Target current in mA
            msg.targetcurrentma = self._target_current[t]
            # Target current in A
            msg.targetcurrenta = self._target_current[t]/1000.0
            
            # Motor current
            try:
                curr_dict = self._network.getMotorCurrent(self._wheel_ids[t])
                curr = curr_dict['value']
                msg.current = curr
            except:
                pass

            # Error Code
            try:
                err_dict = self._network.getErrorCode(self._wheel_ids[t])
                code = err_dict['value']
                msg.errorcode = code
            except:
                pass

            # Current speed, rpm
            try:
                msg.actualspeed = self._current_whl_rpm[t]
            except:
                pass

            # Target speed, rpm
            try:
                msg.targetspeed = self._target_whl_rpm[t]
            except:
                pass

            self._motor_state_pub_dict[t].publish(msg)


    def mainLoop(self):
        rate = self.create_rate(self._loop_rate)

        while rclpy.ok():
            now = time()

            dt = now - self._last_cmd_t
            if (dt > self._cmd_timeout):
                # set zero velocity
                for t in ["fl", "bl", "br", "fr"]:
                    self._target_whl_rpm[t]=0.0

            if self.arm_msg:
                # Apply controls
                self.applyControls()

            # Publish wheel odom
            self.pubOdom()
            # Publish Motors state
            self.pubMotorState()
            
            rate.sleep()
  

if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    try:
        driver = Driver()

        driver.mainLoop()
        driver._network.stop()
    except rclpy.ROSInterruptException:
        driver._network.stop()
        driver._network.disconnectNetwork()