// Copyright 2019 Bold Hearts
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <utility>
#include <fstream>

#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "ik_walk/ik_walk.hpp"

#include "IKWalk/IKWalk.hpp"

using namespace std::chrono_literals;

namespace ik_walk
{

IkWalkWrapper::IkWalkWrapper(const rclcpp::NodeOptions & options) 
: rclcpp::Node{"ik_walk", options},
  enabled(false),
  ramp_triggered(false),
  phase_(0.0),
  phase_previous_(0.0)
{
  params_ = get_default_IKWalkParameters();
  arm_outputs_ = get_default_ArmOutputs();

  params_.stepGain = 0.0;
  params_.lateralGain = 0.0;
  params_.turnGain = 0.0;
  params_.enabledGain = 0.0;

  RCLCPP_INFO(get_logger(), "Create publisher for /cm730/joint_commands");
  jointCommandPub_ = create_publisher<mx_joint_controller_msgs::msg::JointCommand>(
    "/cm730/joint_commands", 10);

  RCLCPP_INFO(get_logger(), "Create subscription for /walking/command");
  walkingCommandSub_ = create_subscription<ik_walk_msgs::msg::WalkingCommand>(
    "/walking/command", 
    rclcpp::ServicesQoS(),
    [=](const ik_walk_msgs::msg::WalkingCommand::SharedPtr msg){
      RCLCPP_INFO(get_logger(), "walking/command received  \t direction: %f \t urgency: %f \t enabledGain: %f", msg->stepgain, msg->urgency, msg->enabledgain);
      std::tie(params_.stepGain, params_.lateralGain, params_.turnGain, params_.enabledGain) = IkWalkWrapper::calculate_params(msg);
      RCLCPP_INFO(get_logger(), "params received : stepGain: %f \t lateralGain: %f \t turnGain: %f \t enabledGain: %f", 
                                params_.stepGain, params_.lateralGain, params_.turnGain, params_.enabledGain);
    }
  );

  RCLCPP_INFO(get_logger(), "Create subscriber to /imu/data");
  imuDataSub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data",
    rclcpp::SensorDataQoS(),
    std::bind(&IkWalkWrapper::imuCallback, this, std::placeholders::_1));

  auto loop =
    [this]() -> void {

      if(!enabled && params_.enabledGain > 0.0){
        if(phase_previous_ > phase_){
          if(ramp_triggered) params_.enabledGain = 0.0;
          ramp_triggered = true;
        }
        if(ramp_triggered) params_.enabledGain = 1.0 - phase_;
      }else{
        ramp_triggered = false;
      }
      if(params_.enabledGain < 0.1) params_.enabledGain = 0.0;

      // cloning previous phase
      phase_previous_ = phase_;

      bool success = Rhoban::IKWalk::walk(
        params_,         // Walk parameters
        8.0 / 1000.0,    // 125Hz Time step
        phase_,          // Current walk phase will be updated)
        outputs_);       // Result target position (updated)

      if (!success) {
        // The requested position not feasible (don't update phase)
        RCLCPP_ERROR(get_logger(), "Inverse Kinematics error. Position not reachable.");
      } else {
        // checking whether the joint commands have changed, then publish
        if (outputs_has_changed(outputs_previous_, outputs_)){
          RCLCPP_DEBUG(get_logger(), "phase: %f \t prev: %f \t enabledGain: %f \t enabled: %i \t ramp: %i", phase_, phase_previous_, params_.enabledGain, enabled ? 1 : 0, ramp_triggered ? 1 : 0);
          // phase_ = phase;
          // simple_balance correction
          // TODO Test and update the factors for balancing
          //Rhoban::IKWalkOutputs  outputs_balance = simpleBalance(outputs_);
          //auto jointMsg = create_joint_command(outputs_balance, arm_outputs_);
          auto jointMsg = create_joint_command(outputs_, arm_outputs_);
          jointCommandPub_->publish(std::move(jointMsg)); 
        }
      }
      // cloning outputs for next delta calculation
      outputs_previous_ = clone_outputs( outputs_);
    };

  loopTimer_ = create_wall_timer(8ms, loop);
}

Rhoban::IKWalkOutputs IkWalkWrapper::clone_outputs ( Rhoban::IKWalkOutputs b) {
    Rhoban::IKWalkOutputs a;
    a.left_hip_roll     = b.left_hip_roll    ;
    a.left_hip_pitch    = b.left_hip_pitch   ;
    a.left_knee         = b.left_knee        ;
    a.left_ankle_pitch  = b.left_ankle_pitch ;
    a.left_ankle_roll   = b.left_ankle_roll  ;
    a.left_hip_yaw      = b.left_hip_yaw     ;
    a.right_hip_yaw     = b.right_hip_yaw    ;
    a.right_hip_roll    = b.right_hip_roll   ;
    a.right_hip_pitch   = b.right_hip_pitch  ;
    a.right_knee        = b.right_knee       ;
    a.right_ankle_pitch = b.right_ankle_pitch;
    a.right_ankle_roll  = b.right_ankle_roll ;
    return a;
}

bool IkWalkWrapper::outputs_has_changed ( Rhoban::IKWalkOutputs a, Rhoban::IKWalkOutputs b) {
    return 
      (a.left_hip_roll     != b.left_hip_roll     ) ||
      (a.left_hip_pitch    != b.left_hip_pitch    ) ||
      (a.left_knee         != b.left_knee         ) ||
      (a.left_ankle_pitch  != b.left_ankle_pitch  ) ||
      (a.left_ankle_roll   != b.left_ankle_roll   ) ||
      (a.left_hip_yaw      != b.left_hip_yaw      ) ||
      (a.right_hip_yaw     != b.right_hip_yaw     ) ||
      (a.right_hip_roll    != b.right_hip_roll    ) ||
      (a.right_hip_pitch   != b.right_hip_pitch   ) ||
      (a.right_knee        != b.right_knee        ) ||
      (a.right_ankle_pitch != b.right_ankle_pitch ) ||
      (a.right_ankle_roll  != b.right_ankle_roll  );
}

IkWalkWrapper::~IkWalkWrapper()
{
}


//Callback function saving the imu values
void IkWalkWrapper::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuMsg) {
  linear_acceleration_.x() = imuMsg->linear_acceleration.x;
  linear_acceleration_.y() = imuMsg->linear_acceleration.y;
  linear_acceleration_.z() = imuMsg->linear_acceleration.z;
  angular_velocity_.x() = imuMsg->angular_velocity.x;
  angular_velocity_.y() = imuMsg->angular_velocity.y;
  angular_velocity_.z() = imuMsg->angular_velocity.z;
}

//Function adjusting the IkWalk outputs using the imu-values
Rhoban::IKWalkOutputs IkWalkWrapper::simpleBalance(Rhoban::IKWalkOutputs walking_outputs) {
    struct Rhoban::IKWalkOutputs balanced_outputs;
    balanced_outputs = walking_outputs;

    //std::ifstream infile("correction_values.txt");
    
    // Setting the weight how much of the imu value is used to correct the roll and pitch motors
    float correction_roll = 0;
    float arm_correction_roll = 0;
    //correction_roll = 1;

    float correction_pitch = 0;
    float arm_correction_pitch = 0;
    //correction_pitch = 1;

    //infile >> correction_roll >> correction_pitch >> arm_correction_roll >> arm_correction_pitch;

   
    /* Perform moving avrage for X IMU values */
    movWinX.push_back(linear_acceleration_.x());
//    movWinX.push_back(angular_velocity_.x());
    RCLCPP_INFO(get_logger(), "Before erase: %ld", movWinX.size());
    if(movWinX.size() > WIN_SIZE) movWinX.erase(movWinX.begin());
    RCLCPP_INFO(get_logger(), "After erase: %ld", movWinX.size());
    double xAvg = 0;
    for(std::vector<double>::iterator i = movWinX.begin(); i != movWinX.end(); i++) xAvg += *i;
    //RCLCPP_INFO(get_logger(), "inside loop");
    xAvg /= movWinX.size();
//    xAvg = linear_acceleration_.x();
    RCLCPP_INFO(get_logger(), "xAvg %f", xAvg);

    /* Perform moving avrage for Y IMU values */
    movWinY.push_back(linear_acceleration_.y());
//    movWinY.push_back(angular_velocity_.y());
    RCLCPP_INFO(get_logger(), "Before Y erase: %ld", movWinY.size());
    if(movWinY.size() > WIN_SIZE) movWinY.erase(movWinY.begin());
    RCLCPP_INFO(get_logger(), "After Y erase: %ld", movWinY.size());
    double yAvg = 0;
    for(std::vector<double>::iterator i = movWinY.begin(); i != movWinY.end(); i++) yAvg += *i;
    yAvg /= movWinY.size();
//    yAvg = linear_acceleration_.y();
    RCLCPP_INFO(get_logger(), "yAvg %f", yAvg);
      
    //roll correction - different for left and right (only one leg adjusted)
    if (yAvg < 0.0) {
    balanced_outputs.left_hip_roll = (1.0 + yAvg * correction_roll) * walking_outputs.left_hip_roll;
    balanced_outputs.left_ankle_roll = walking_outputs.left_ankle_roll + (balanced_outputs.left_hip_roll - walking_outputs.left_hip_roll);
    arm_outputs_.left_shoulder_roll = (1.0 - (yAvg) * arm_correction_roll) * arm_outputs_.left_shoulder_roll;
    }
    else {
    balanced_outputs.right_hip_roll = (1.0 + yAvg * correction_roll) * walking_outputs.right_hip_roll;
    balanced_outputs.right_ankle_roll = walking_outputs.right_ankle_roll + (balanced_outputs.right_hip_roll - walking_outputs.right_hip_roll);
    arm_outputs_.right_shoulder_roll = (1.0 + (yAvg) * arm_correction_roll) * arm_outputs_.right_shoulder_roll;
    }
   
    //pitch correction same for front and back (both legs adjusted)
    balanced_outputs.left_hip_pitch = (1.0 + (xAvg-0.1) * correction_pitch) * walking_outputs.left_hip_pitch;
    balanced_outputs.right_hip_pitch = (1.0 + (xAvg-0.1) * correction_pitch) * walking_outputs.right_hip_pitch;
    balanced_outputs.left_ankle_pitch = walking_outputs.left_ankle_pitch + (balanced_outputs.left_hip_pitch - walking_outputs.left_hip_pitch);
    balanced_outputs.right_ankle_pitch = walking_outputs.right_ankle_pitch + (balanced_outputs.right_hip_pitch - walking_outputs.right_hip_pitch);
    arm_outputs_.left_shoulder_pitch = (1.0 + (xAvg-0.1) * arm_correction_pitch) * arm_outputs_.left_shoulder_pitch;
    arm_outputs_.right_shoulder_pitch = (1.0 - (xAvg-0.1) * arm_correction_pitch) * arm_outputs_.right_shoulder_pitch;

    return balanced_outputs;
}




float IkWalkWrapper::clamp(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

std::tuple<float, float, float, float> IkWalkWrapper::calculate_params(const ik_walk_msgs::msg::WalkingCommand::SharedPtr msg) {
  /*
  * msg.x  - direction - in radians - zero straight ahead, positive clockwise direction
  * msg.y  - urgency - scale the default stepGain  and lateralGain
  * for walk on the spot send ( anything, 0, !0)  
  * msg.z  - walk -  0 for stop  1 for continue / start moving
  * for walk on the spot send ( anything, 0, !0)  
  * 
  * default x stepGain = 0.02
  */
  
  enabled = true;
  if(msg->enabledgain == 0.0) enabled = false;

  if(!enabled) RCLCPP_INFO(get_logger(), "Told to stop"); // TODO

//  if (msg->enabledgain == 0.0) {  
//    // stop moving at the end of the current cycle
//    return std::make_tuple(0.0, 0.0, 0.0, 0.0);
//  }
//  else {
    // default stepGain = 0.02, scale by urgency
    // clamp to maximum value of 0.1 - arbitrarily chosen
    float maxStepGain = 0.1;
    float defaultStepGain = 0.02;
    float stepGain = clamp(defaultStepGain * msg->urgency, -maxStepGain, maxStepGain);
    float lateralGain = clamp(tan(msg->stepgain)*stepGain, 0, stepGain);

    // TODO we need a strategy for apportioning rotation between lateralGain and turnGain
    float turnGain = msg->turngain;
    float enabledGain = enabled ? 1.0 : params_.enabledGain;

    return std::make_tuple(stepGain, lateralGain, turnGain, enabledGain);
//  }
}

mx_joint_controller_msgs::msg::JointCommand::UniquePtr IkWalkWrapper::create_joint_command(
  const Rhoban::IKWalkOutputs & positions, ArmOutputs & arm_outputs)
{
  auto jointMsg = std::make_unique<mx_joint_controller_msgs::msg::JointCommand>();
  jointMsg->name.push_back("shoulder-pitch-l");
  jointMsg->position.push_back(arm_outputs.left_shoulder_pitch);
  jointMsg->name.push_back("shoulder-roll-l");
  jointMsg->position.push_back(arm_outputs.left_shoulder_roll);

  jointMsg->name.push_back("hip-yaw-l");
  jointMsg->position.push_back(positions.left_hip_yaw);
  jointMsg->name.push_back("hip-roll-l");
  jointMsg->position.push_back(positions.left_hip_roll);
  jointMsg->name.push_back("hip-pitch-l");
  jointMsg->position.push_back(-1 * positions.left_hip_pitch);
  jointMsg->name.push_back("knee-l");
  jointMsg->position.push_back(-1 * positions.left_knee);
  jointMsg->name.push_back("ankle-pitch-l");
  jointMsg->position.push_back(positions.left_ankle_pitch);
  jointMsg->name.push_back("ankle-roll-l");
  jointMsg->position.push_back(positions.left_ankle_roll);

  jointMsg->name.push_back("shoulder-pitch-r");
  jointMsg->position.push_back(arm_outputs.right_shoulder_pitch);
  jointMsg->name.push_back("shoulder-roll-r");
  jointMsg->position.push_back(arm_outputs.right_shoulder_roll);

  jointMsg->name.push_back("hip-yaw-r");
  jointMsg->position.push_back(positions.right_hip_yaw);
  jointMsg->name.push_back("hip-roll-r");
  jointMsg->position.push_back(positions.right_hip_roll);
  jointMsg->name.push_back("hip-pitch-r");
  jointMsg->position.push_back(positions.right_hip_pitch);
  jointMsg->name.push_back("knee-r");
  jointMsg->position.push_back(positions.right_knee);
  jointMsg->name.push_back("ankle-pitch-r");
  jointMsg->position.push_back(-1 * positions.right_ankle_pitch);
  jointMsg->name.push_back("ankle-roll-r");
  jointMsg->position.push_back(positions.right_ankle_roll);

  return jointMsg;
}

ik_walk::ArmOutputs IkWalkWrapper::get_default_ArmOutputs()
{
  struct ArmOutputs arm_outputs;
  /**
   * basic pitch for the shoulders
   */
  arm_outputs.left_shoulder_pitch = 0.4; 
  arm_outputs.right_shoulder_pitch = -0.4;

  /**
   * basic roll for the shoulders
   */
  arm_outputs.left_shoulder_roll = 0.5;
  arm_outputs.right_shoulder_roll = -0.5;

  return arm_outputs;
}

Rhoban::IKWalkParameters IkWalkWrapper::get_default_IKWalkParameters()
{
  struct Rhoban::IKWalkParameters params;

  /**
   * Model leg typical length between
   * each rotation axis in meters
   */
  params.distHipToKnee = declare_parameter("dist_hip_to_knee", double{0.128});
  params.distKneeToAnkle = declare_parameter("dist_knee_to_ankle", double{0.1415});
  params.distAnkleToGround = declare_parameter("dist_ankle_to_ground", double{0.059});

  /**
   * Distance between the two feet in lateral
   * axis while in zero position
   */
  params.distFeetLateral = declare_parameter("dist_feet_lateral", double{0.075});

  /**
   * Complete (two legs) walk cycle frequency
   * in Hertz
   */
  params.freq = declare_parameter("freq", double{1.7});

  /**
   * Global gain multiplying all time
   * dependant movement between 0 and 1.
   * Control walk enabled/disabled smoothing.
   * 0 is walk disabled.
   * 1 is walk fully enabled
   */
  params.enabledGain = declare_parameter("enabled_gain", double{1.0});

  /**
   * Length of double support phase
   * in phase time
   * (between 0 and 1)
   * 0 is null double support and full single support
   * 1 is full double support and null single support
   */
  params.supportPhaseRatio = declare_parameter("support_phase_ratio", double{0.2});

  /**
   * Lateral offset on default foot
   * position in meters (foot lateral distance)
   * 0 is default
   * > 0 is both feet external offset
   */
  params.footYOffset = declare_parameter("foot_y_offset", double{-0.015});

  /**
   * Forward length of each foot step
   * in meters
   * >0 goes forward
   * <0 goes backward
   * (dynamic parameter)
   */
  params.stepGain = declare_parameter("step_gain", double{0.0});

  /**
   * Vertical rise height of each foot
   * in meters (positive)
   */
  params.riseGain = declare_parameter("rise_gain", double{0.05});

  /**
   * Angular yaw rotation of each
   * foot for each step in radian.
   * 0 does not turn
   * >0 turns left
   * <0 turns right
   * (dynamic parameter)
   */
  params.turnGain = declare_parameter("turn_gain", double{0.0});

  /**
   * Lateral length of each foot step
   * in meters.
   * >0 goes left
   * <0 goes right
   * (dynamic parameter)
   */
  params.lateralGain = declare_parameter("lateral_gain", double{0.0});

  /**
   * Vertical foot offset from trunk
   * in meters (positive)
   * 0 is in init position
   * > 0 set the robot lower to the ground
   */
  params.trunkZOffset = declare_parameter("trunk_z_offset", double{0.065});

  /**
   * Lateral trunk oscillation amplitude
   * in meters (positive)
   */
  params.swingGain = declare_parameter("swing_gain", double{0.02});

  /**
   * Lateral angular oscillation amplitude
   * of swing trunkRoll in radian
   */
  params.swingRollGain = declare_parameter("swing_roll_gain", double{0.0});

  /**
   * Phase shift of lateral trunk oscillation
   * between 0 and 1
   */
  params.swingPhase = declare_parameter("swing_phase", double{0.3});

  /**
   * Foot X-Z spline velocities
   * at ground take off and ground landing.
   * Step stands for X and rise stands for Z
   * velocities.
   * Typical values ranges within 0 and 5.
   * >0 for DownVel is having the foot touching the
   * ground with backward velocity.
   * >0 for UpVel is having the foot going back
   * forward with non perpendicular tangent.
   */
  params.stepUpVel = declare_parameter("step_up_vel", double{4.0});
  params.stepDownVel = declare_parameter("step_down_vel", double{4.0});
  params.riseUpVel = declare_parameter("rise_up_vel", double{4.0});
  params.riseDownVel = declare_parameter("rise_down_vel", double{4.0});

  /**
   * Time length in phase time
   * where swing lateral oscillation
   * remains on the same side
   * between 0 and 0.5
   */
  params.swingPause = declare_parameter("swing_pause", double{0.0});

  /**
   * Swing lateral spline velocity (positive).
   * Control the "smoothness" of swing trajectory.
   * Typical values are between 0 and 5.
   */
  params.swingVel = declare_parameter("swing_vel", double{4.0});

  /**
   * Forward trunk-foot offset
   * with respect to foot in meters
   * >0 moves the trunk forward
   * <0 moves the trunk backward
   */
  params.trunkXOffset = declare_parameter("trunk_x_offset", double{0.02});

  /**
   * Lateral trunk-foot offset
   * with respect to foot in meters
   * >0 moves the trunk on the left
   * <0 moves the trunk on the right
   */
  params.trunkYOffset = declare_parameter("trunk_y_offset", double{0.0});

  /**
   * Trunk angular rotation
   * around Y in radian
   * >0 bends the trunk forward
   * <0 bends the trunk backward
   */
  params.trunkPitch = declare_parameter("trunk_pitch", double{0.15});

  /**
   * Trunk angular rotation
   * around X in radian
   * >0 bends the trunk on the right
   * <0 bends the trunk on the left
   */
  params.trunkRoll = declare_parameter("trunk_roll", double{0.0});

  /**
   * Add extra offset on X, Y and Z
   * direction on left and right feet
   * in meters
   * (Can be used for example to implement
   * dynamic kick)
   */
  params.extraLeftX = declare_parameter("extra_left_x", double{0.0});
  params.extraLeftY = declare_parameter("extra_left_y", double{0.0});
  params.extraLeftZ = declare_parameter("extra_left_z", double{0.0});
  params.extraRightX = declare_parameter("extra_right_x", double{0.0});
  params.extraRightY = declare_parameter("extra_right_y", double{0.0});
  params.extraRightZ = declare_parameter("extra_right_z", double{0.0});

  /**
   * Add extra angular offset on
   * Yaw, Pitch and Roll rotation of
   * left and right foot in radians
   */
  params.extraLeftYaw = declare_parameter("extra_left_yaw", double{0.0});
  params.extraLeftPitch = declare_parameter("extra_left_pitch", double{0.0});
  params.extraLeftRoll = declare_parameter("extra_left_roll", double{0.1});
  params.extraRightYaw = declare_parameter("extra_right_yaw", double{0.0});
  params.extraRightPitch = declare_parameter("extra_right_pitch", double{0.0});
  params.extraRightRoll = declare_parameter("extra_right_roll", double{-0.1});

  return params;
}

}  // namespace ik_walk
