//
// Created by qiayuan on 2022/7/24.
//

#include "legged_controllers/TargetTrajectoriesPublisher.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

using namespace legged;

namespace {
scalar_t TARGET_DISPLACEMENT_VELOCITY;
scalar_t TARGET_ROTATION_VELOCITY;
scalar_t COM_HEIGHT;
vector_t DEFAULT_JOINT_STATE(18);
scalar_t TIME_TO_TARGET;
}  // namespace


visualization_msgs::InteractiveMarker TargetTrajectoriesPublisher::createInteractiveMarker() const {
    visualization_msgs::InteractiveMarker interactiveMarker;
    interactiveMarker.header.frame_id = "odom";
    interactiveMarker.header.stamp = ros::Time::now();
    interactiveMarker.name = "Goal";
    interactiveMarker.scale = 0.2;
    interactiveMarker.description = "Right click to send command";
    interactiveMarker.pose.position.x = 0.8;
    interactiveMarker.pose.position.y = 0.0;
    interactiveMarker.pose.position.z = 0.44;
    interactiveMarker.pose.orientation.x = 0.0;
    interactiveMarker.pose.orientation.y = 0.0;
    interactiveMarker.pose.orientation.z = 0.0;
    interactiveMarker.pose.orientation.w = 1.0;

    // create a grey box marker
    const auto boxMarker = []() {
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
        marker.color.a = 0.5;
        return marker;
    }();

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl boxControl;
    boxControl.always_visible = 1;
    boxControl.markers.push_back(boxMarker);
    boxControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

    // add the control to the interactive marker
    interactiveMarker.controls.push_back(boxControl);

    // create a control which will move the box
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::InteractiveMarkerControl control;

    control.orientation.w = sqrt(2)/2;
    control.orientation.x = sqrt(2)/2;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactiveMarker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactiveMarker.controls.push_back(control);

    control.orientation.w = sqrt(2)/2;
    control.orientation.x = 0;
    control.orientation.y = sqrt(2)/2;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactiveMarker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactiveMarker.controls.push_back(control);

    control.orientation.w = sqrt(2)/2;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = sqrt(2)/2;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactiveMarker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactiveMarker.controls.push_back(control);

    return interactiveMarker;
}

void TargetTrajectoriesPublisher::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    const Eigen::Vector3d EePosition(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
    const Eigen::Quaterniond EeOrientation(feedback->pose.orientation.w, feedback->pose.orientation.x,
                                           feedback->pose.orientation.y, feedback->pose.orientation.z);

    // get TargetTrajectories
    const auto targetTrajectories = goalToTargetTrajectories_(EePosition, EeOrientation,
                                                              latestObservation_, latestObservationEe_);

    // publish TargetTrajectories
    targetTrajectoriesPublisher_->publishTargetTrajectories(targetTrajectories);

    // update last ee target
    lastEeTarget_ << EePosition, EeOrientation.coeffs();
}


/**
* Choose a proper total time to arrive the target
*/
scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) {
  // position
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dz = desiredBaseDisplacement(2);
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy + dz * dz);
  const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;

  // rotation
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t& droll = desiredBaseDisplacement(4);
  const scalar_t& dpitch = desiredBaseDisplacement(5);
  const scalar_t ratation = std::sqrt(dyaw * dyaw + droll * droll + dpitch * dpitch);
  const scalar_t rotationTime = ratation / TARGET_ROTATION_VELOCITY;

  return std::max(rotationTime, displacementTime);
}


TargetTrajectories targetPoseToTargetTrajectories(const vector_t& EeTargetPose,
                                                  const vector_t& BaseTargetPose,
                                                  const SystemObservation& observation,
                                                  const SystemObservation& eeState,
                                                  const scalar_t& targetReachingTime) {
  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  const vector_t EeCurrentPose = eeState.state;
  vector_t BaseCurrenPose = observation.state.segment<6>(6);
  BaseCurrenPose(2) = COM_HEIGHT;
  BaseCurrenPose(4) = 0;
  BaseCurrenPose(5) = 0;

  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size() + 7));
  stateTrajectory[0] << vector_t::Zero(6), BaseCurrenPose, DEFAULT_JOINT_STATE, EeCurrentPose;
  stateTrajectory[1] << vector_t::Zero(6), BaseTargetPose, DEFAULT_JOINT_STATE, EeTargetPose;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}


TargetTrajectories cmdVelToTargetTrajectories(const vector_t& cmdVel,
                                              const vector_t& lastEeTarget,
                                              const SystemObservation& observation,
                                              const SystemObservation& eeState) {
  const vector_t BaseCurrentPose = observation.state.segment<6>(6);
  const Eigen::Matrix<scalar_t, 3, 1> zyx = BaseCurrentPose.tail(3);
  vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3); // world frame

  const scalar_t timeToTarget = TIME_TO_TARGET;
  const vector_t BaseTargetPose = [&]() {
    vector_t target(6);
    target(0) = BaseCurrentPose(0) + cmdVelRot(0) * timeToTarget;
    target(1) = BaseCurrentPose(1) + cmdVelRot(1) * timeToTarget;
    target(2) = COM_HEIGHT;
    target(3) = BaseCurrentPose(3) + cmdVel(3) * timeToTarget;
    target(4) = 0;
    target(5) = 0;
    return target;
  }();

  const vector_t EeTargetPose = lastEeTarget.head(7);
  SystemObservation eeStateLast = eeState;
  eeStateLast.state = EeTargetPose;

  // target reaching duration
  const scalar_t targetReachingTime = observation.time + timeToTarget;
  auto trajectories = targetPoseToTargetTrajectories(EeTargetPose, BaseTargetPose, observation, eeStateLast, targetReachingTime);
  trajectories.stateTrajectory[0].head(3) = cmdVelRot;
  trajectories.stateTrajectory[1].head(3) = cmdVelRot;
  return trajectories;
}


/**
 * Converts the pose of the interactive marker to TargetTrajectories.
 */
TargetTrajectories EEgoalPoseToTargetTrajectories(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                                  const SystemObservation& observation, const SystemObservation& eeState) {

    // current pose
    const vector_t EeCurrentPose = eeState.state;
    const vector_t BaseCurrenPose = observation.state.segment<6>(6);
    // target pose
    const vector_t EeTargetPose = (vector_t(7) << position, orientation.coeffs()).finished();

    const vector_t BaseTargetPose = [&](){
        vector_t target(6);
        target.setZero();
//        target = BaseCurrenPose;
        target(0) = position(0) - 0.8;
        target(1) = position(1);
        target(2) = COM_HEIGHT;
//        target(3) = BaseCurrenPose(3); // keep current body yaw angle
        target(4) = 0;
        target(5) = 0;
        return target;
    }();

    // time
    const vector_t deltaError = [&](){
        vector_t delta(6);
        delta.segment<3>(0) = EeTargetPose.segment<3>(0) - EeCurrentPose.segment<3>(0);

        Eigen::Quaterniond q_current(EeCurrentPose[6], EeCurrentPose[3], EeCurrentPose[4], EeCurrentPose[5]);
        Eigen::Quaterniond q_target(EeTargetPose[6], EeTargetPose[3], EeTargetPose[4], EeTargetPose[5]);

        delta.segment<3>(3) = quaternionDistance(q_current, q_target);

        return delta;
    }();
    const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(deltaError);

    // traj
    return targetPoseToTargetTrajectories(EeTargetPose, BaseTargetPose, observation, eeState, targetReachingTime);
}


int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string referenceFile;
  std::string taskFile;
  nodeHandle.getParam("/referenceFile", referenceFile);
  nodeHandle.getParam("/taskFile", taskFile);

  loadData::loadCppDataType(referenceFile, "comHeight", COM_HEIGHT);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET);

  TargetTrajectoriesPublisher target_pose_command(nodeHandle, robotName, &EEgoalPoseToTargetTrajectories, &cmdVelToTargetTrajectories);

  ros::spin();
  // Successful exit
  return 0;
}
