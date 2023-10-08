
//
// Created by qiayuan on 1/24/22.
//

#pragma once

#include "unitree_legged_sdk/safety.h"
#include "unitree_legged_sdk/udp.h"

#include "unitree_arm_sdk/unitree_arm.h"
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <hardware_interface/joint_command_interface.h>

#include <legged_hw/LeggedHW.h>

namespace legged {
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

struct UnitreeMotorData {
  double pos_, vel_, tau_;                 // state
  double posDes_, velDes_, kp_, kd_, ff_;  // command
};

struct UnitreeImuData {
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

class UnitreeHW : public LeggedHW {
 public:
  UnitreeHW() = default;
  ~UnitreeHW();
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /** \brief Communicate with hardware. Get data, status of robot.
   *
   * Call @ref UNITREE_LEGGED_SDK::UDP::Recv() to get robot's state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref UNITREE_LEGGED_SDK::UDP::Recv(). Publish actuator
   * current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& time, const ros::Duration& period) override;


  void gripperCB(const control_msgs::GripperCommandGoalConstPtr& msg);

 private:
  hardware_interface::PositionJointInterface grip_pos_interface;
  bool setupJoints();

  bool setupImu();

  bool setupContactSensor(ros::NodeHandle& nh);

  std::shared_ptr<UNITREE_LEGGED_SDK::UDP> udp_;
  std::shared_ptr<UNITREE_LEGGED_SDK::Safety> safety_;
  UNITREE_LEGGED_SDK::LowState lowState_{};
  UNITREE_LEGGED_SDK::LowCmd lowCmd_{};

  bool has_gripper;
  UNITREE_ARM_SDK::UnitreeArm* arm;
  /* Gripper Action */
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction>* gripper_as;
  control_msgs::GripperCommandFeedback gripper_feedback;
  control_msgs::GripperCommandResult gripper_result;
  UnitreeMotorData gripperJointData_{};
  double gripper_epsilon = 0.01;
  double gripper_position_cmd{};
  double gripper_effort_cmd{};

  std::vector<std::string> arm_joint_names_;

  size_t dogIndex_ = 12;
  UnitreeMotorData jointData_[18]{};  // NOLINT(modernize-avoid-c-arrays)
  UnitreeImuData imuData_{};
  bool contactState_[4]{};  // NOLINT(modernize-avoid-c-arrays)

  int powerLimit_{};
  int contactThreshold_{};
};

}  // namespace legged
