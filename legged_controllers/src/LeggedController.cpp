//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp> // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include "legged_controllers/LeggedController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged
{
  bool LeggedController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh)
  {
    // Initialize OCS2
    std::string urdfFile;
    std::string taskFile;
    std::string referenceFile;
    controller_nh.getParam("/urdfFile", urdfFile);
    controller_nh.getParam("/taskFile", taskFile);
    controller_nh.getParam("/referenceFile", referenceFile);
    bool verbose = false;
    loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

    setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
    setupMpc();
    setupMrt();
    // Visualization
    ros::NodeHandle nh;

    CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
    eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                        leggedInterface_->modelSettings().contactNames3DoF);
    std::vector<std::string> eeName{leggedInterface_->modelSettings().info.eeFrame};
    armEeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                           eeName);

    robotVisualizer_ = std::make_shared<LeggedVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(),
                                                          leggedInterface_->modelSettings(),
                                                          *eeKinematicsPtr_,
                                                          *armEeKinematicsPtr_,
                                                          nh);

    // Hardware interface
    // link all joints in simulation/HW
    auto *hybridJointInterface = robot_hw->get<HybridJointInterface>();
    std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                         "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE",
                                         "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    for (const auto &joint_name : joint_names)
    {
      hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
    }
    auto *contactInterface = robot_hw->get<ContactSensorInterface>();
    for (const auto &name : leggedInterface_->modelSettings().contactNames3DoF)
    {
      contactHandles_.push_back(contactInterface->getHandle(name));
    }
    imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu");

    // State estimation
    setupStateEstimate(taskFile, verbose);

    // Whole body control
    wbc_ = std::make_shared<HierarchicalWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                             *eeKinematicsPtr_, *armEeKinematicsPtr_);
    wbc_->loadTasksSetting(taskFile, verbose);

    // Safety Checker
    safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

    is_ch = false; // QQQ

    return true;
  }

  void LeggedController::starting(const ros::Time &time)
  {
    // Initial state
    currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);

    // first trial
    ROS_INFO_STREAM("HAHAHAHAHAHAHAH" << leggedInterface_->getCentroidalModelInfo().stateDim);

    updateStateEstimation(time, ros::Duration(0.002));
    currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
    currentObservation_.mode = ModeNumber::STANCE;

    // Initial end-effector target
    vector_t EeInitTarget(7), initTarget(leggedInterface_->getInitialState().size() + 7);
    EeInitTarget.head(3) << 0.8, 0.0, 0.34; // + 0.056 = 0.436
    EeInitTarget.tail(4) << Eigen::Quaternion<scalar_t>(1.0, 0.0, 0.0, 0.0).coeffs();

    vector_t initState = leggedInterface_->getInitialState();
    vector_t armInitState = initState.tail(6); // 6

    initTarget << currentObservation_.state.head(24), armInitState, EeInitTarget;

    TargetTrajectories target_trajectories({currentObservation_.time}, {initTarget}, {currentObservation_.input});

    std::cout << "armInitState: " << armInitState.transpose() << std::endl;

    // Set the first observation and command and wait for optimization to finish
    mpcMrtInterface_->setCurrentObservation(currentObservation_);
    mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
    ROS_INFO_STREAM("Waiting for the initial policy ...");
    while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok())
    {
      mpcMrtInterface_->advanceMpc();
      ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();

      // ROS_INFO_STREAM("i am sleeping.");
    }
    ROS_INFO_STREAM("Initial policy has been received.");

    mpcRunning_ = true;
  }

  void LeggedController::update(const ros::Time &time, const ros::Duration &period)
  {
    // State Estimate
    updateStateEstimation(time, period);
    //  if(currentObservation_.time > 20 && !is_ch) {
    //      leggedInterface_->getSwitchedModelReferenceManagerPtr()->setChickenHead(true);
    //      std::cerr << "chicken head begin" << std::endl;
    //      is_ch = true;
    //  }

    // Update the current state of the system
    mpcMrtInterface_->setCurrentObservation(currentObservation_);

    // Load the latest MPC policy
    mpcMrtInterface_->updatePolicy();
    // Evaluate the current policy
    vector_t optimizedState, optimizedInput;
    size_t plannedMode = 0; // The mode that is active at the time the policy is evaluated at.
    mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

    //  // Whole body control
    currentObservation_.input = optimizedInput;

    wbcTimer_.startTimer();
    vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec(), currentObservation_.time);
    wbcTimer_.endTimer();

    vector_t torque = x.tail(18);

    vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
    vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

    // Safety check, if failed, stop the controller
    if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput))
    {
      ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
      stopRequest(time);
    }

    // quadruped joint torque
    //  if(currentObservation_.time > 0.0){
    //      for (size_t j = 0; j < 12; ++j) {
    //          hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 3, torque(j));
    //      }
    //  }

    // arm joint torque
    vector_t kp_arm, kd_arm;
    kp_arm.resize(6);
    kd_arm.resize(6);

    //according to the structure of the manipulation, the kp for j4 and j5 are about the same
    //j3 is bigger ,so do j2 , j1, j6 is the smallest.

    // kp_arm << 600, 700, 800, 300, 300, 100; 
    // kp_arm << 100, 100, 100, 100, 100, 150;
    // kp_arm << 500, 600, 500, 400, 300, 0;
    // kp_arm << 500, 1000, 800, 600, 500, 400;
    kp_arm << 500, 500, 500 ,500 ,500 ,100;

    // kd_arm << 10, 10, 20, 10, 10, 2;      
    // kd_arm << 10, 10, 10, 10, 10, 1;
    // kd_arm << 5, 5, 5, 5, 5, 0;
    kd_arm << 5, 10, 5, 5, 5, 0;

    //test the performance of every single joint on Z1.
    // vector_t posDes_test, velDes_test;
    // posDes_test.resize(6);
    // velDes_test.resize(6);

    // int joint_test_num = 5;
    // double joint_test_vel = 0.1;

    // velDes_test << 0,0,0,0,0,0;
    // velDes_test[joint_test_num] = joint_test_vel; 
    // double posDes_single = hybridJointHandles_[12+joint_test_num].getPosition() + joint_test_vel * 0.001;
    // posDes_test << 0,0,0,0,0,0;
    // posDes_test[joint_test_num] = posDes_single;

    //Low-pass filtering
    static vector_t last_velDes = Eigen::Matrix<double,6,1>::Zero();
    static vector_t last_posDes = Eigen::Matrix<double,6,1>::Zero();
    static vector_t current_velDes = Eigen::Matrix<double,6,1>::Zero();
    static vector_t current_posDes = Eigen::Matrix<double,6,1>::Zero();

    //The tuning parameter for L-P fiter
    double alpha = 0.1;
    //set commands for arm's joints
    for (size_t j = 12; j < 18; ++j)
    { // 12, 18

      current_posDes[j-12] = (posDes[j]* alpha + last_posDes[j-12] * (1-alpha));
      current_velDes[j-12] = (velDes[j]*alpha + last_velDes[j-12] * (1-alpha));

      hybridJointHandles_[j].setCommand(current_posDes[j-12], current_velDes[j-12], kp_arm[j-12], kd_arm[j-12], 0); //able to reach,but quiver at beginning
      // hybridJointHandles_[j].setCommand(posDes[j], velDes[j],0, 0, 0);
      // hybridJointHandles_[j].setCommand(0, 0, 0, 0, 0);
      // hybridJointHandles_[j].setCommand(posDes_test[j-12], velDes_test[j-12],kp_arm[j-12], kd_arm[j-12],0);
      last_posDes[j-12] = current_posDes[j-12];
      last_velDes[j-12] = current_velDes[j-12];
    }

    //    armControlLaw(time);

    // Visualization
    robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());

    // Publish the observation. Only needed for the command interface
    observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
    vector_t ee_state(7);
    armEeState(ee_state);
    eeStatePublisher_.publish(createEeStateMsg(currentObservation_.time, ee_state));

    //publish the joint debug datas
    vector_t joints_acc(6);
    vector_t vel_desire(6);
    vector_t pos_desire(6);
    vector_t vel_armJoints(6);
    vector_t pos_armJoints(6);
    for (int i =0;i<6;i++)
    {
      // vel_desire[i] = velDes[12+i];
      // pos_desire[i] = posDes[12+i];
      vel_desire[i] = current_velDes[i];
      pos_desire[i] = current_posDes[i];
      vel_armJoints[i] = hybridJointHandles_[12+i].getVelocity();
      pos_armJoints[i] = hybridJointHandles_[12+i].getPosition();
    }
    
    if(armJointsAcc(currentObservation_.time, joints_acc))
    {
      armJointsPublisher_.publish(createArmJointsMsg(currentObservation_.time, joints_acc, vel_desire, pos_desire, vel_armJoints, pos_armJoints));
    }
  }

  void LeggedController::armControlLaw(const ros::Time &time)
  {
    static double t_0 = time.toSec();
    vector_t kp_arm, kd_arm;
    kp_arm.resize(6);
    kd_arm.resize(6);

    // modifiable
    // kp_arm << 100, 100, 500, 200, 100, 10; 
    // kp_arm << 100, 100, 100, 100, 100, 5;
    kp_arm << 500, 600, 500, 400, 300, 200;

    // kd_arm << 10, 10, 20, 10, 10, 2;      
    // kd_arm << 10, 10, 10, 10, 10, 1;
    kd_arm << 5, 5, 5, 5, 5, 5;

    Eigen::Matrix<double, 6, 1> posDes, jointPos, jointVel;
    posDes << 0.0, 1.85, -1.15, -0.85, 0, 0;
    if ((time.toSec() - t_0) <= 10)
      posDes = Eigen::Matrix<double, 6, 1>::Zero() * (1 - (time.toSec() - t_0) / 10.0) + posDes * (time.toSec() - t_0) / 10.0;

    jointPos = currentObservation_.state.segment<6>(24);
    //    std::cerr << "posDes: " << posDes.transpose() << std::endl;
    //    std::cerr << "posMea: " << jointPos.transpose() << std::endl;

    for (size_t j = 0; j < 6; ++j)
    { // 12, 18
      double tau = kp_arm[j] * (posDes[j] - jointPos[j]);
      //        hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 3, torque(j));
      hybridJointHandles_[j + 12].setCommand(posDes[j], 0.0, kp_arm[j], kd_arm[j], 0);
    }
  }

  void LeggedController::updateStateEstimation(const ros::Time &time, const ros::Duration &period)
  {
    vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
    contact_flag_t contacts;
    Eigen::Quaternion<scalar_t> quat;
    contact_flag_t contactFlag;
    vector3_t angularVel, linearAccel;
    matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

    for (size_t i = 0; i < hybridJointHandles_.size(); ++i)
    {
      jointPos(i) = hybridJointHandles_[i].getPosition();
      jointVel(i) = hybridJointHandles_[i].getVelocity();
    }
    for (size_t i = 0; i < contacts.size(); ++i)
    {
      contactFlag[i] = contactHandles_[i].isContact();
    }
    for (size_t i = 0; i < 4; ++i)
    {
      quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
    }
    for (size_t i = 0; i < 3; ++i)
    {
      angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
      linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
    }
    for (size_t i = 0; i < 9; ++i)
    {
      orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
      angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
      linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
    }

    stateEstimate_->updateJointStates(jointPos, jointVel);
    stateEstimate_->updateContact(contactFlag);
    stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
    measuredRbdState_ = stateEstimate_->update(time, period);
    currentObservation_.time += period.toSec();
    scalar_t yawLast = currentObservation_.state(9);
    currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
    currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
    currentObservation_.mode = stateEstimate_->getMode();
  }

  LeggedController::~LeggedController()
  {
    controllerRunning_ = false;
    if (mpcThread_.joinable())
    {
      mpcThread_.join();
    }
    std::cerr << "########################################################################";
    std::cerr << "\n### MPC Benchmarking";
    std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
    std::cerr << "########################################################################";
    std::cerr << "\n### WBC Benchmarking";
    std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
  }

  void LeggedController::setupLeggedInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile,
                                              bool verbose)
  {
    // Soft constraint for friction cone
    leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
    leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
  }

  void LeggedController::setupMpc()
  {
    mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                    leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
    rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                      leggedInterface_->getCentroidalModelInfo());

    const std::string robotName = "legged_robot";
    ros::NodeHandle nh;
    // Gait receiver
    auto gaitReceiverPtr =
        std::make_shared<GaitReceiver>(nh, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
    // ROS ReferenceManager
    auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
    rosReferenceManagerPtr->subscribe(nh);
    mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
    mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
    observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
    eeStatePublisher_ = nh.advertise<legged_msgs::ee_state>(robotName + "_mpc_observation_ee_state", 1);

    // first trial
    armJointsPublisher_ = nh.advertise<legged_msgs::arm_joints_debug>(robotName + "_Z1_joints_debug", 10);
  }

  void LeggedController::setupMrt()
  {
    mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
    mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
    mpcTimer_.reset();

    controllerRunning_ = true;
    mpcThread_ = std::thread([&]()
                             {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    } });
    setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
  }

  void LeggedController::setupStateEstimate(const std::string &taskFile, bool verbose)
  {
    stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
    dynamic_cast<KalmanFilterEstimate &>(*stateEstimate_).loadSettings(taskFile, verbose);
    currentObservation_.time = 0;
  }

  void LeggedCheaterController::setupStateEstimate(const std::string & /*taskFile*/, bool /*verbose*/)
  {
    stateEstimate_ = std::make_shared<FromTopicStateEstimate>(leggedInterface_->getPinocchioInterface(),
                                                              leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
    currentObservation_.time = 0;
  }

  legged_msgs::ee_state LeggedController::createEeStateMsg(ocs2::scalar_t time, ocs2::vector_t state)
  {
    legged_msgs::ee_state ee_state_msg;

    ee_state_msg.time = time;

    ee_state_msg.state.resize(7);
    for (size_t i = 0; i < 7; ++i)
    {
      ee_state_msg.state[i] = static_cast<float>(state(i));
    }

    return ee_state_msg;
  }

  // first trial
  legged_msgs::arm_joints_debug LeggedController::createArmJointsMsg(ocs2::scalar_t time, ocs2::vector_t joints_acc, \
  ocs2::vector_t vel_d, ocs2::vector_t pos_d, ocs2::vector_t vel, ocs2::vector_t pos)
  {
    legged_msgs::arm_joints_debug arm_msgs;

    arm_msgs.time = time;

    arm_msgs.acc_s.resize(6);
    arm_msgs.vel_d.resize(6);
    arm_msgs.pos_d.resize(6);
    arm_msgs.vel.resize(6);
    arm_msgs.pos.resize(6);
    for (size_t i = 0; i < 6; i++)
    {
      arm_msgs.acc_s[i] = static_cast<_Float64>(joints_acc[i]);
      arm_msgs.vel_d[i] = static_cast<_Float64>(vel_d[i]);
      arm_msgs.pos_d[i] = static_cast<_Float64>(pos_d[i]);
      arm_msgs.vel[i] = static_cast<_Float64>(vel[i]);
      arm_msgs.pos[i] = static_cast<_Float64>(pos[i]);
    }

    return arm_msgs;
  }

  /*bool LeggedController::armJointsAcc(ocs2::scalar_t time, vector_t &joints_acc)
  {
    static bool is_first_time = true;
    static bool is_second_time = false;
    static vector_t last_joint_pose(6);
    static vector_t last_joint_vel(6);
    static vector_t current_joint_vel(6);
    static ocs2::scalar_t last_time = 0;
    // At least the third time enterring this function can actually get the acceleration of 6 joints
    if (is_first_time)
    {
      is_first_time = false;
      last_joint_pose = currentObservation_.state.segment<6>(24);
      is_second_time = true;
      last_time = time;
      return false;  //fail to get accelerations
    }
    else if (is_second_time) //the second time
    {
      is_second_time = false;
      for (size_t i = 0; i < 6; i++)
      {
        // TODO: To consider the interval?
        last_joint_vel[i] = (currentObservation_.state(24 + i) - last_joint_pose(i)) / (time-last_time);
      }
      last_joint_pose = currentObservation_.state.segment<6>(24);
      last_time = time;
      return false; //fail to get accelerations
    }

    else
    {
      for (size_t i = 0; i < 6; i++)
      {
        current_joint_vel[i] = (currentObservation_.state(24 + i) - last_joint_pose(i)) / (time-last_time);
        joints_acc[i] = (current_joint_vel[i] - last_joint_vel[i])/(time-last_time);
      }
      last_joint_vel = current_joint_vel;
      last_joint_pose = currentObservation_.state.segment<6>(24);
      last_time = time;
      return true; //successfully get the accelerations
    }
  } */
  

 bool LeggedController::armJointsAcc(ocs2::scalar_t time, vector_t &joints_acc)
  {
    static bool is_first_time = true;
    static vector_t last_joint_vel(6);
    static vector_t current_joint_vel(6);
    static ocs2::scalar_t last_time = time;
    // At least the second time enterring this function can actually get the acceleration of 6 joints
    if (is_first_time)
    {
      is_first_time = false;
      for(size_t i(0); i<6; i++)
      {
        last_joint_vel[i] = hybridJointHandles_[i+12].getVelocity();
      }
      return false;  //fail to get accelerations
    }
    else
    {
      for (size_t i = 0; i < 6; i++)
      {
        current_joint_vel[i] = hybridJointHandles_[12+i].getVelocity();
        joints_acc[i] = (current_joint_vel[i] - last_joint_vel[i])/(time-last_time);
      }
      last_joint_vel = current_joint_vel;
      last_time = time;
      return true; //successfully get the accelerations
    }
  }
  
  void LeggedController::armEeState(vector_t &ee_state)
  {
    armEeKinematicsPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());

    vector_t qMeasured_ = centroidal_model::getGeneralizedCoordinates(currentObservation_.state, leggedInterface_->getCentroidalModelInfo());

    const auto &model = leggedInterface_->getPinocchioInterface().getModel();
    auto &data = leggedInterface_->getPinocchioInterface().getData();

    pinocchio::forwardKinematics(model, data, qMeasured_);
    pinocchio::updateFramePlacements(model, data);

    const auto armEePos = armEeKinematicsPtr_->getPosition(vector_t());

    const size_t frame_idx = model.getBodyId(armEeKinematicsPtr_->getIds()[0]);
    const auto armEeOriQuat = matrixToQuaternion(data.oMf[frame_idx].rotation());

    ee_state.segment<4>(3) = vector_t(armEeOriQuat.coeffs());
    ee_state.segment<3>(0) = armEePos[0];
  }

} // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerBase)
