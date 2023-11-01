
//
// Created by qiayuan on 1/24/22.
//

#include "legged_unitree_hw/UnitreeHW.h"

namespace legged {
bool UnitreeHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!LeggedHW::init(root_nh, robot_hw_nh)) {
    return false;
  }
  std::cerr << "Begin!!!!" <<std::endl;
  ROS_INFO("HW Initializing...");
  robot_hw_nh.param<bool>("UnitreeGripperYN", has_gripper, true);
  std::string controller_ip;
  robot_hw_nh.param<std::string>("udp_to_controller/controller_ip", controller_ip, "127.0.0.1");
  int sdk_own_port, controller_port;
  robot_hw_nh.param<int>("udp/port_to_sdk", controller_port, 8872);
  robot_hw_nh.param<int>("udp_to_controller/own_port", sdk_own_port, 8872);

  robot_hw_nh.getParam("power_limit", powerLimit_);

  setupJoints();
  setupImu();
  setupContactSensor(robot_hw_nh);

  // Quadruped setting
  // std::cerr << "nitree Quad Initializing..." <<std::endl;
  ROS_INFO("Unitree Quad Initializing...");
  udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL);
  udp_->InitCmdData(lowCmd_);

  std::string robot_type;
  root_nh.getParam("robot_type", robot_type);
  if (robot_type == "a1") {
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::A1);
  } else if (robot_type == "aliengo" || robot_type == "aliengoZ1") {
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::Aliengo);
  } else {
    ROS_FATAL("Unknown robot type: %s", robot_type.c_str());
    return false;
  }

  // TODO Arm setting
  // arm = new UNITREE_ARM_SDK::UnitreeArm(controller_ip, sdk_own_port, controller_port);
  arm = new UNITREE_ARM_SDK::UnitreeArm("127.0.0.1");
  
  std::cerr << "Unitree arm Initializing..." <<std::endl;
  ROS_INFO("Unitree Arm Initializing...");

  arm->init();
  std::cerr << " arm Initialized" <<std::endl;

  // get initial pos
  for(int i(0); i<6; i++) {
      jointData_[i+12].pos_ = arm->armState.q[i];
      jointData_[i+12].posDes_ = arm->armState.q[i];
  }
  gripper_position_cmd = arm->armState.gripperState.angle;


    /* Set UnitreeArm SDK Class */
    gripper_as = new actionlib::SimpleActionServer<control_msgs::GripperCommandAction>("z1_gripper", boost::bind(&UnitreeHW::gripperCB, this, _1), false);
    gripper_as->start();

    std::cerr << " Gripper Initialized" <<std::endl;

  ROS_INFO("\033[1;32m [MmHW]: Hardware start successfully! \033[0m");
  return true;
}

UnitreeHW::~UnitreeHW(){
    arm->armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::Passive;
    arm->sendRecv();
}

//maybe_modifiable
void UnitreeHW::read(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  // Quadruped information
  udp_->Recv();
  udp_->GetRecv(lowState_);

  for (int i = 0; i < 12; ++i) {
    jointData_[i].pos_ = lowState_.motorState[i].q;
    jointData_[i].vel_ = lowState_.motorState[i].dq;
    jointData_[i].tau_ = lowState_.motorState[i].tauEst;
  }

  imuData_.ori_[0] = lowState_.imu.quaternion[1];
  imuData_.ori_[1] = lowState_.imu.quaternion[2];
  imuData_.ori_[2] = lowState_.imu.quaternion[3];
  imuData_.ori_[3] = lowState_.imu.quaternion[0];
  imuData_.angularVel_[0] = lowState_.imu.gyroscope[0];
  imuData_.angularVel_[1] = lowState_.imu.gyroscope[1];
  imuData_.angularVel_[2] = lowState_.imu.gyroscope[2];
  imuData_.linearAcc_[0] = lowState_.imu.accelerometer[0];
  imuData_.linearAcc_[1] = lowState_.imu.accelerometer[1];
  imuData_.linearAcc_[2] = lowState_.imu.accelerometer[2];

  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactState_[i] = lowState_.footForce[i] > contactThreshold_;
  }

  //TODO Arm Information
    for(int i(0); i<6; i++) {
        jointData_[i+12].pos_ = arm->armState.q[i];
        jointData_[i+12].vel_ = arm->armState.dq[i];
        jointData_[i+12].tau_ = arm->armState.tau[i];
    }

    if(has_gripper)
    {
        gripperJointData_.pos_ = arm->armState.gripperState.angle;
        gripperJointData_.vel_ = arm->armState.gripperState.speed;
        gripperJointData_.tau_ = arm->armState.gripperState.tau;
    }



  // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
  std::vector<std::string> names = hybridJointInterface_.getNames();
  for (const auto& name : names) {
    HybridJointHandle handle = hybridJointInterface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setVelocityDesired(0.);
    handle.setKd(3.);
  }
}

void UnitreeHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  // Quadruped Write
  for (int i = 0; i < 12; ++i) {
    lowCmd_.motorCmd[i].q = static_cast<float>(jointData_[i].posDes_);
    lowCmd_.motorCmd[i].dq = static_cast<float>(jointData_[i].velDes_);
    lowCmd_.motorCmd[i].Kp = static_cast<float>(jointData_[i].kp_);
    lowCmd_.motorCmd[i].Kd = static_cast<float>(jointData_[i].kd_);
    lowCmd_.motorCmd[i].tau = static_cast<float>(jointData_[i].ff_);
  }
  safety_->PositionLimit(lowCmd_);
  safety_->PowerProtect(lowCmd_, lowState_, powerLimit_);
  udp_->SetSend(lowCmd_);
  udp_->Send();


  //TODO Arm Write
    // arm->armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::LowCmd;
    // Eigen::Matrix<double, 6, 1> tau_ff;
    // for(int i(0); i<6; i++) {
    //     // arm->armCmd.q_d[i] = static_cast<float>(jointData_[i+12].posDes_);
    //     arm->armCmd.q_d[i] = static_cast<float>(jointData_[i+12].pos_ + jointData_[i+12].velDes_* 0.001);
    //     arm->armCmd.dq_d[i] = static_cast<float>(jointData_[i+12].velDes_);
    //     arm->armCmd.Kp[i] = static_cast<float>(jointData_[i+12].kp_);
    //     arm->armCmd.Kd[i] = static_cast<float>(jointData_[i+12].kd_);
    //     tau_ff[i] = jointData_[i+12].ff_;
    // }
    // arm->armCmd.setTau(tau_ff);

    arm->armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointSpeedCtrl;
    for(int i(0); i<6; i++) {
        arm->armCmd.dq_d[i] = jointData_[i+12].velDes_;
    }

    // arm->armCmd.mode = (mode_t)UNITREE_ARM_SDK::ArmMode::JointPositionCtrl;
    // for(int i(0); i<6; i++) {
    //     arm->armCmd.q_d[i] = jointData_[i+12].posDes_;
    //     arm->armCmd.dq_d[i] = jointData_[i+12].velDes_;
    // }



    if(has_gripper) {
        arm->armCmd.gripperCmd.angle = gripper_position_cmd;
        arm->armCmd.gripperCmd.maxTau = gripper_effort_cmd;
        arm->armCmd.gripperCmd.epsilon_inner = gripper_epsilon;
        arm->armCmd.gripperCmd.epsilon_outer = gripper_epsilon;
    }

    arm->sendRecv();

}

bool UnitreeHW::setupJoints() {
  // Quadruped leg joints
  for (const auto& joint : urdfModel_->joints_) {
    int leg_index = 0;
    int joint_index = 0;
    if (joint.first.find("RF") != std::string::npos) {
      leg_index = UNITREE_LEGGED_SDK::FR_;
    } else if (joint.first.find("LF") != std::string::npos) {
      leg_index = UNITREE_LEGGED_SDK::FL_;
    } else if (joint.first.find("RH") != std::string::npos) {
      leg_index = UNITREE_LEGGED_SDK::RR_;
    } else if (joint.first.find("LH") != std::string::npos) {
      leg_index = UNITREE_LEGGED_SDK::RL_;
    } else {
      continue;
    }

    if (joint.first.find("HAA") != std::string::npos) {
      joint_index = 0;
    } else if (joint.first.find("HFE") != std::string::npos) {
      joint_index = 1;
    } else if (joint.first.find("KFE") != std::string::npos) {
      joint_index = 2;
    } else {
      continue;
    }

    int index = leg_index * 3 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].posDes_, &jointData_[index].velDes_,
                                                           &jointData_[index].kp_, &jointData_[index].kd_, &jointData_[index].ff_));
  }

  // Arm joints
    arm_joint_names_.resize(6);
    for (size_t i = 0; i < 6; ++i) {
        arm_joint_names_[i] = "joint" + boost::lexical_cast<std::string>(i+1);
        hardware_interface::JointStateHandle state_handle(arm_joint_names_[i], &jointData_[dogIndex_+i].pos_,
                                                          &jointData_[dogIndex_+i].vel_, &jointData_[dogIndex_+i].tau_);
        jointStateInterface_.registerHandle(state_handle);
        hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[dogIndex_+i].posDes_, &jointData_[dogIndex_+i].velDes_,
                                                               &jointData_[dogIndex_+i].kp_, &jointData_[dogIndex_+i].kd_, &jointData_[dogIndex_+i].ff_));
    }

  return true;
}

bool UnitreeHW::setupImu() {
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("unitree_imu", "unitree_imu", imuData_.ori_, imuData_.oriCov_,
                                                                         imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_,
                                                                         imuData_.linearAccCov_));
  imuData_.oriCov_[0] = 0.0012;
  imuData_.oriCov_[4] = 0.0012;
  imuData_.oriCov_[8] = 0.0012;

  imuData_.angularVelCov_[0] = 0.0004;
  imuData_.angularVelCov_[4] = 0.0004;
  imuData_.angularVelCov_[8] = 0.0004;

  return true;
}

bool UnitreeHW::setupContactSensor(ros::NodeHandle& nh) {
  nh.getParam("contact_threshold", contactThreshold_);
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactSensorInterface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
  }
  return true;
}


void UnitreeHW::gripperCB(const control_msgs::GripperCommandGoalConstPtr& msg)
{
    gripper_position_cmd = msg->command.position;
    gripper_effort_cmd = msg->command.max_effort;

    ros::Rate r(5);

    Timer timer(2.0);
    while (true)
    {
        gripper_feedback.position = arm->armState.gripperState.angle;
        gripper_feedback.effort = arm->armState.gripperState.tau;
        gripper_feedback.reached_goal = arm->armState.gripperState.reached_goal;
        gripper_feedback.stalled = arm->armState.gripperState.stalled;

        if(gripper_as->isPreemptRequested() || !ros::ok() || (timer.wait_time() < 0))
        {
            gripper_result.position = gripper_feedback.position;
            gripper_result.effort = gripper_feedback.effort;
            gripper_result.reached_goal = gripper_feedback.reached_goal;
            gripper_result.stalled = gripper_feedback.stalled;
            gripper_as->setPreempted(gripper_result);
            break;
        }

        gripper_as->publishFeedback(gripper_feedback);

        if(std::fabs(gripper_position_cmd - arm->armState.gripperState.angle)<0.01)
        {
            if(gripper_feedback.stalled)
            {
                ROS_INFO("[Gripper] Reach goal.");
                gripper_result.position = gripper_feedback.position;
                gripper_result.effort = gripper_feedback.effort;
                gripper_result.reached_goal = gripper_feedback.reached_goal;
                gripper_result.stalled = gripper_feedback.stalled;
                gripper_as->setSucceeded(gripper_result);
                break;
            }
        }

        r.sleep();
    }
}

}  // namespace legged
