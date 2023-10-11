//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
//#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/WbcBase.h>

#include "legged_controllers/SafetyChecker.h"
#include <legged_msgs/ee_state.h>

//first trial
#include <legged_msgs/joints_acc.h>
#include "legged_interface/visualization/legged_visualization.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class LeggedController : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                               ContactSensorInterface> {
 public:
  LeggedController() = default;
  ~LeggedController() override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override { mpcRunning_ = false; }

 protected:
  virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  legged_msgs::ee_state createEeStateMsg(ocs2::scalar_t time, ocs2::vector_t state);
  void armEeState(vector_t& ee_state);
  void armControlLaw(const ros::Time& time);

  //first trial
  legged_msgs::joints_acc createJointAccMsg(ocs2::scalar_t time, ocs2::vector_t joints_acc);
  bool armJointsAcc(vector_t& joints_acc);


  // Interface
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  std::shared_ptr<PinocchioEndEffectorKinematics> armEeKinematicsPtr_;
  
  std::vector<HybridJointHandle> hybridJointHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

  // State Estimation
  SystemObservation currentObservation_;
  vector_t measuredRbdState_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

  // Whole Body Control
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<LeggedVisualizer> robotVisualizer_;
  ros::Publisher observationPublisher_, eeStatePublisher_;

  //first_trial
  ros::Publisher jointAccPublisher_;

 private:
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  benchmark::RepeatedTimer mpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;
  bool is_ch = false;
};

class LeggedCheaterController : public LeggedController {
 protected:
  void setupStateEstimate(const std::string& taskFile, bool verbose) override;
};

//class MPCController : public LeggedController{
//public:
//    MpcController() = default;
//    ~MpcController() = default;
//
//private:
//    void setHybridJointHardware(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh);
//    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
//    void setupWbc(ros::NodeHandle& controller_nh, const std::string& taskFile);
//    void updateJointState(vector_t& jointPos, vector_t& jointVel);
//    void updateControlLaw(const vector_t &posDes, const vector_t &velDes, const vector_t &torque);
//
//    std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>> cmd_pub_[6];
//    ros::Subscriber arm_joint_sub_;
//    realtime_tools::RealtimeBuffer<sensor_msgs::JointState> joint_state_buffer_;
//};

}  // namespace legged
