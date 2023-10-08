//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/subscriber.h>
#include <legged_msgs/ee_state.h>
#include <mutex>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>


namespace legged {
using namespace ocs2;

class TargetTrajectoriesPublisher final {
 public:
  using GoalPoseToTargetTrajectories = std::function<TargetTrajectories(
            const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
            const SystemObservation& observation, const SystemObservation& eeState)>;
  using CmdToTargetTrajectories = std::function<TargetTrajectories(
          const vector_t& cmd, const vector_t& lastEeTarget, 
          const SystemObservation& observation, const SystemObservation& eeState)>;
  

  TargetTrajectoriesPublisher(::ros::NodeHandle& nh, const std::string& topicPrefix, GoalPoseToTargetTrajectories goalToTargetTrajectories,
                              CmdToTargetTrajectories cmdVelToTargetTrajectories)
      : server_("simple_marker"),
        goalToTargetTrajectories_(std::move(goalToTargetTrajectories)),
        cmdVelToTargetTrajectories_(std::move(cmdVelToTargetTrajectories)) {
    // Trajectories publisher
    targetTrajectoriesPublisher_.reset(new TargetTrajectoriesRosPublisher(nh, topicPrefix));

    // robot state observation subscriber
    auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    };
    observationSub_ = nh.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

    // initial end-effector Target
    lastEeTarget_ = vector_t::Zero(7);
    lastEeTarget_.head(3) << 0.8, 0., 0.44;
    lastEeTarget_.tail(4) << Eigen::Quaternion<scalar_t>(1.0, 0., 0., 0.).coeffs();

    // end-effector pose observation subscriber
    auto eePoseCallback = [this](const legged_msgs::ee_state::ConstPtr& msg){
        std::lock_guard<std::mutex> lock(latestObservationEeMutex_);
        {
            legged_msgs::ee_state eeState = *msg;
            latestObservationEe_.time = eeState.time;
            latestObservationEe_.state.resize(eeState.state.size());
            for (size_t i = 0; i < eeState.state.size(); i++) {
                const auto& state = eeState.state[i];
                latestObservationEe_.state[i] = static_cast<scalar_t>(state);
            }
        }
    };
    eePoseSub_ = nh.subscribe<legged_msgs::ee_state>(topicPrefix + "_mpc_observation_ee_state", 1, eePoseCallback);

    // cmd_vel subscriber
    auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
      if (latestObservation_.time == 0.0) {
        return;
      }

      vector_t cmdVel = vector_t::Zero(4);
      cmdVel[0] = msg->linear.x;
      cmdVel[1] = msg->linear.y;
      cmdVel[2] = msg->linear.z;
      cmdVel[3] = msg->angular.z;

      const auto trajectories = cmdVelToTargetTrajectories_(cmdVel, lastEeTarget_,
                                                              latestObservation_, latestObservationEe_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    cmdVelSub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelCallback);

      // interactive marker
      menuHandler_.insert("Send target pose", boost::bind(&TargetTrajectoriesPublisher::processFeedback, this, _1));
      auto interactiveMarker = createInteractiveMarker();
      server_.insert(interactiveMarker);
      menuHandler_.apply(server_, interactiveMarker.name);
      server_.applyChanges();
  }

 private:
    visualization_msgs::InteractiveMarker createInteractiveMarker() const;
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    interactive_markers::MenuHandler menuHandler_;
    interactive_markers::InteractiveMarkerServer server_;

  GoalPoseToTargetTrajectories goalToTargetTrajectories_;
  CmdToTargetTrajectories cmdVelToTargetTrajectories_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;

  ::ros::Subscriber observationSub_, cmdVelSub_, eePoseSub_;

  mutable std::mutex latestObservationMutex_, latestObservationEeMutex_;
  SystemObservation latestObservation_, latestObservationEe_;

  vector_t lastEeTarget_;
};

}  // namespace legged
