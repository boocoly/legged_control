//
// Created by qiayuan on 2022/7/1.
//
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_wbc/WbcBase.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <utility>

namespace legged {
WbcBase::WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                 const PinocchioEndEffectorKinematics& eeKinematics,  const PinocchioEndEffectorKinematics& armEeKinematics)
    : pinocchioInterfaceMeasured_(pinocchioInterface),
      pinocchioInterfaceDesired_(pinocchioInterface),
      info_(std::move(info)),
      mapping_(info_),
      eeKinematics_(eeKinematics.clone()),
      armEeKinematics_(armEeKinematics.clone()) 
{
  numDecisionVars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts;
  qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  qDesired_ = vector_t(info_.generalizedCoordinatesNum);
  vDesired_ = vector_t(info_.generalizedCoordinatesNum);
  inputLast_ = vector_t::Zero(info_.inputDim);
  baseAccDesired_ = vector_t(6);

  const auto& model = pinocchioInterfaceMeasured_.getModel();
  armEeFrameIdx_ = model.getBodyId(armEeKinematics_->getIds()[0]);

  jb_ = matrix_t(6, info_.generalizedCoordinatesNum);
  djb_ = matrix_t(6, info_.generalizedCoordinatesNum);;

  je_ = matrix_t(6, info_.generalizedCoordinatesNum);
  dje_ = matrix_t(6, info_.generalizedCoordinatesNum);;

  jointKp_ = matrix_t::Zero(6, 6);
  jointKd_ = matrix_t::Zero(6, 6);
  d_arm_ = vector_t::Zero(6);

  armEeLinearKp_ = matrix_t::Zero(3, 3);
  armEeLinearKd_ = matrix_t::Zero(3, 3);
  armEeAngularKp_ = matrix_t::Zero(3, 3);
  armEeAngularKd_ = matrix_t::Zero(3, 3);
  d_ee_ = vector_t::Zero(3);
  da_ee_ = vector_t::Zero(3);

  zyx2xyz_.setZero();
  zyx2xyz_ << 0., 0., 1., 
              0., 1., 0.,
              1., 0., 0.;
    // ee linear
    armEeLinearKp_(0, 0) = 2000;
    armEeLinearKp_(1, 1) = 2000;
    armEeLinearKp_(2, 2) = 2000;

    armEeLinearKd_(0, 0) = 75;
    armEeLinearKd_(1, 1) = 75;
    armEeLinearKd_(2, 2) = 75;

    armEeAngularKp_(0, 0) = 2000;
    armEeAngularKp_(1, 1) = 2000;
    armEeAngularKp_(2, 2) = 2000;

    armEeAngularKd_(0, 0) = 75;
    armEeAngularKd_(1, 1) = 75;
    armEeAngularKd_(2, 2) = 75;

    // base
    baseHeightKp_ = 100;
    baseHeightKd_ = 10;

    baseAngularKp_ = 100;
    baseAngularKd_ = 10;

    // swing leg
    swingKp_ = 350;
    swingKd_ = 37;


//    ros::NodeHandle wbc_nh;
//    ros::NodeHandle nh_weight = ros::NodeHandle(wbc_nh, "wbc");
//    dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<legged_wbc::WbcWeightConfig>>(nh_weight);
//    dynamic_reconfigure::Server<legged_wbc::WbcWeightConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
//        dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
//    };
//    dynamic_srv_->setCallback(cb);
}

//void WbcBase::dynamicCallback(legged_wbc::WbcWeightConfig &config, uint32_t) {
//
//    // ee linear
//    armEeLinearKp_(0, 0) = config.kp_ee_linear_x;
//    armEeLinearKp_(1, 1) = config.kp_ee_linear_y;
//    armEeLinearKp_(2, 2) = config.kp_ee_linear_z;
//
//    armEeLinearKd_(0, 0) = config.kd_ee_linear_x;
//    armEeLinearKd_(1, 1) = config.kd_ee_linear_y;
//    armEeLinearKd_(2, 2) = config.kd_ee_linear_z;
//
//    armEeAngularKp_(0, 0) = config.kp_ee_angular_x;
//    armEeAngularKp_(1, 1) = config.kp_ee_angular_y;
//    armEeAngularKp_(2, 2) = config.kp_ee_angular_z;
//
//    armEeAngularKd_(0, 0) = config.kd_ee_angular_x;
//    armEeAngularKd_(1, 1) = config.kd_ee_angular_y;
//    armEeAngularKd_(2, 2) = config.kd_ee_angular_z;
//
//    // base
//    baseHeightKp_ = config.baseHeightKp;
//    baseHeightKd_ = config.baseHeightKd;
//
//    baseAngularKp_ = config.kp_base_angular;
//    baseAngularKd_ = config.kd_base_angular;
//
//    // swing leg
//    swingKp_ = config.swing_Kp;
//    swingKd_ = config.swing_Kd;
//
//    ROS_INFO_STREAM("\033[32m Update the wbc param. \033[0m");
//}

vector_t WbcBase::update(const vector_t& stateDesired, const vector_t& inputDesired,
                          const vector_t& rbdStateMeasured, size_t mode, scalar_t period, scalar_t time) {
  contactFlag_ = modeNumber2StanceLeg(mode);
  numContacts_ = 0;
  for (bool flag : contactFlag_) {
    if (flag) {
      numContacts_++;
    }
  }

  updateMeasured(rbdStateMeasured);
  updateDesired(stateDesired, inputDesired, period);

  return {};
}

void WbcBase::updateMeasured(const vector_t& rbdStateMeasured) {
  qMeasured_.setZero();
  vMeasured_.setZero();

  qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);
  qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
  qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);
  vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
  vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
  vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

  const auto& model = pinocchioInterfaceMeasured_.getModel();
  auto& data = pinocchioInterfaceMeasured_.getData();

  // For floating base EoM task
  pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::crba(model, data, qMeasured_);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);
  j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }

  // For not contact motion task
  pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
  dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }

  // For base motion tracking task
  Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> base_j, base_dj;
  base_j.setZero(6, info_.generalizedCoordinatesNum);
  base_dj.setZero(6, info_.generalizedCoordinatesNum);
  pinocchio::getFrameJacobian(model, data, model.getBodyId("base"), pinocchio::LOCAL_WORLD_ALIGNED, base_j);
  pinocchio::getFrameJacobianTimeVariation(model, data, model.getBodyId("base"), pinocchio::LOCAL_WORLD_ALIGNED, base_dj);
  jb_.setZero(); jb_ = base_j;
  djb_.setZero(); djb_ = base_dj;

  // For end-effector motion task
  je_ = matrix_t(6, info_.generalizedCoordinatesNum);
  dje_ = matrix_t(6, info_.generalizedCoordinatesNum);
  je_.setZero();
  dje_.setZero();
  pinocchio::getFrameJacobian(model, data, armEeFrameIdx_, pinocchio::LOCAL_WORLD_ALIGNED, je_);
  pinocchio::getFrameJacobianTimeVariation(model, data, armEeFrameIdx_, pinocchio::LOCAL_WORLD_ALIGNED, dje_);
  
}

void WbcBase::updateDesired(const vector_t& stateDesired, const vector_t& inputDesired, ocs2::scalar_t period) {
  const auto& model = pinocchioInterfaceDesired_.getModel();
  auto& data = pinocchioInterfaceDesired_.getData();

  qDesired_.setZero();
  vDesired_.setZero();

  mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);
  qDesired_ = mapping_.getPinocchioJointPosition(stateDesired);
  pinocchio::forwardKinematics(model, data, qDesired_);
  pinocchio::computeJointJacobians(model, data, qDesired_);
  pinocchio::updateFramePlacements(model, data);
  updateCentroidalDynamics(pinocchioInterfaceDesired_, info_, qDesired_);
  vDesired_ = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);
  pinocchio::forwardKinematics(model, data, qDesired_, vDesired_);

  // update base acc desired
  jointAccel_ = centroidal_model::getJointVelocities(inputDesired - inputLast_, info_) / period;
  inputLast_ = inputDesired;

  const auto& A = getCentroidalMomentumMatrix(pinocchioInterfaceDesired_);
  const Matrix6 Ab = A.template leftCols<6>();
  const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
  auto Aj = A.rightCols(info_.actuatedDofNum);
  const auto ADot = pinocchio::dccrba(model, data, qDesired_, vDesired_);
  Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
  centroidalMomentumRate.noalias() -= ADot * vDesired_;
  centroidalMomentumRate.noalias() -= Aj * jointAccel_;

  baseAccDesired_.setZero();
  baseAccDesired_ = AbInv * centroidalMomentumRate;
}

vector_t  WbcBase::updateCmd(ocs2::vector_t x_optimal) {
    auto& data = pinocchioInterfaceMeasured_.getData();

    matrix_t Mj, Jj_T;
    vector_t hj;
    Mj = data.M.bottomRows(info_.actuatedDofNum);
    Jj_T = j_.transpose().bottomRows(info_.actuatedDofNum);
    hj = data.nle.bottomRows(info_.actuatedDofNum);
    matrix_t a = (matrix_t(info_.actuatedDofNum, getNumDecisionVars())<< Mj, -Jj_T).finished();

    vector_t torque_optimal = a * x_optimal + hj;

    vector_t cmd = (vector_t(numDecisionVars_ + info_.actuatedDofNum)<<x_optimal, torque_optimal).finished();

    return cmd;
}

// Tracking base xy linear motion task
Task WbcBase::formulateBaseXYLinearAccelTask() {
  matrix_t a(2, numDecisionVars_);
  vector_t b(a.rows());

  a.setZero();
  b.setZero();

  a.block(0, 0, 2, 2) = matrix_t::Identity(2, 2);

  b = baseAccDesired_.segment<2>(0);

  return {a, b, matrix_t(), vector_t()};
}


// Tracking base angular motion task
Task WbcBase::formulateBaseAngularMotionTask(){
    matrix_t a(3, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 0, 3, info_.generalizedCoordinatesNum) = jb_.block(3, 0, 3, info_.generalizedCoordinatesNum);

    vector3_t eulerAngles = qMeasured_.segment<3>(3);

    // from derivative euler to angular
    vector3_t vMeasuredGlobal =
            getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngles, vMeasured_.segment<3>(3));
    vector3_t vDesiredGlobal =
            getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngles, vDesired_.segment<3>(3));

    // from euler to rotation
    vector3_t eulerAnglesDesired;
    eulerAnglesDesired << qDesired_.segment<3>(3);
    matrix3_t rotationBaseMeasuredToWorld =
            getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAngles);
    matrix3_t rotationBaseReferenceToWorld =
            getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAnglesDesired);

    vector3_t error = rotationErrorInWorld<scalar_t>(rotationBaseReferenceToWorld, rotationBaseMeasuredToWorld);

    // desired acc
    vector3_t accDesired = getGlobalAngularAccelerationFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngles,
                                                                   vDesired_.segment<3>(3), baseAccDesired_.segment<3>(3));

//    baseAngularKp_ = 100.0; baseAngularKd_ = 10.0;
    b = accDesired + baseAngularKp_  * error + baseAngularKd_ * (vDesiredGlobal - vMeasuredGlobal)
                 - djb_.block(3, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;

    return {a, b, matrix_t(), vector_t()};
}


// Tracking base height motion task
Task WbcBase::formulateBaseHeightMotionTask() {
    matrix_t a(1, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();
    a.block(0, 2, 1, 1) = matrix_t::Identity(1, 1);

//    baseHeightKp_ = 100; baseHeightKd_ =10;
    b[0] = baseAccDesired_[2] + baseHeightKp_ * (qDesired_[2] - qMeasured_[2])
           + baseHeightKd_ * (vDesired_[2] - vMeasured_[2]);

    return {a, b, matrix_t(), vector_t()};
}


// EoM
// [Mb, -J^Tb]x = -hb
Task WbcBase::formulateFloatingBaseEomTask() {
    matrix_t a(6, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();

    auto& data = pinocchioInterfaceMeasured_.getData();

    matrix_t Mb, Jb_T;
    vector_t hb;
    Mb = data.M.topRows(6);
    hb = data.nle.topRows(6);
    Jb_T = j_.transpose().topRows(6);

    a << Mb, -Jb_T;
    b = -hb;

    return {a, b, matrix_t(), matrix_t()};
}


//Task WbcBase::formulateFloatingBaseEomTask() {
//  auto& data = pinocchioInterfaceMeasured_.getData();
//
//  matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
//  s.block(0, 0, info_.actuatedDofNum, 6).setZero();
//  s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();
//
//  matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, numDecisionVars_) << data.M, -j_.transpose(), -s.transpose()).finished();
//  vector_t b = -data.nle;
//
//  return {a, b, matrix_t(), vector_t()};
//}


// torque limit
// tau_min - hj <= [Mj, -Jj^T] <= tau_max - hj
Task WbcBase::formulateTorqueLimitsTask() {
    matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
    vector_t f(d.rows());
    d.setZero();
    f.setZero();

    auto& data = pinocchioInterfaceMeasured_.getData();

    matrix_t Mj, Jj_T;
    vector_t hj;
    Mj = data.M.bottomRows(info_.actuatedDofNum);
    Jj_T = j_.transpose().bottomRows(info_.actuatedDofNum);
    hj = data.nle.bottomRows(info_.actuatedDofNum);

    d.block(0, 0, info_.actuatedDofNum, numDecisionVars_) << Mj, -Jj_T;
    d.block(info_.actuatedDofNum, 0, info_.actuatedDofNum, numDecisionVars_) << -Mj, Jj_T;
    auto legTorqueLimits = torqueLimits_.segment<3>(0);
    auto armTorqueLimits = torqueLimits_(3) * Eigen::Matrix<double, 6, 1>::Ones();


    f << legTorqueLimits, legTorqueLimits, legTorqueLimits, legTorqueLimits, armTorqueLimits,
            legTorqueLimits, legTorqueLimits, legTorqueLimits, legTorqueLimits, armTorqueLimits;
    f.segment<18>(0) -= hj;
    f.segment<18>(info_.actuatedDofNum) += hj;

    return {matrix_t(), vector_t(), d, f};
}

//Task WbcBase::formulateTorqueLimitsTask() {
//  matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
//  d.setZero();
//  matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
//  d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum, info_.actuatedDofNum) = i;
//  d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
//          info_.actuatedDofNum) = -i;
//  vector_t f(2 * info_.actuatedDofNum);
//  // for (size_t l = 0; l < 2 * info_.actuatedDofNum / 3; ++l) {
//  //   f.segment<3>(3 * l) = torqueLimits_;
//  // }
//  // quadruped joint torque limit
//  for (size_t l = 0; l < 4; ++l) {
//    f.segment<3>(3 * l) = torqueLimits_.segment<3>(0);
//  }
//  // arm joint torque limit
//  f.segment<6>(12) = torqueLimits_(3) * Eigen::Matrix<double, 6, 1>::Ones();
//  f.segment<18>(18) = f.segment<18>(0);
//
//  return {matrix_t(), vector_t(), d, f};
//}

// [J, 0] x = -\dot J v
Task WbcBase::formulateNoContactMotionTask() {
  matrix_t a(3 * numContacts_, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
    if (contactFlag_[i]) {
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      j++;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}


// no contact:
// [0, I] x = 0
// contact:
// [0, C] x <= 0
Task WbcBase::formulateFrictionConeTask() {
  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  a.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (!contactFlag_[i]) {
      a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
    }
  }
  vector_t b(a.rows());
  b.setZero();

  matrix_t frictionPyramic(5, 3);  // clang-format off
  frictionPyramic << 0, 0, -1,
                     1, 0, -frictionCoeff_,
                    -1, 0, -frictionCoeff_,
                     0, 1, -frictionCoeff_,
                     0,-1, -frictionCoeff_;  // clang-format on

  matrix_t d(5 * numContacts_ + 3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  d.setZero();
  j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (contactFlag_[i]) {
      d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramic;
    }
  }
  vector_t f = Eigen::VectorXd::Zero(d.rows());

  return {a, b, d, f};
}

Task WbcBase::formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period) {
  matrix_t a(6, numDecisionVars_);
  a.setZero();
  a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

  vector_t jointAccel = centroidal_model::getJointVelocities(inputDesired - inputLast_, info_) / period;
  inputLast_ = inputDesired;
  mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);

  const auto& model = pinocchioInterfaceDesired_.getModel();
  auto& data = pinocchioInterfaceDesired_.getData();
  const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
  const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);

  const auto& A = getCentroidalMomentumMatrix(pinocchioInterfaceDesired_);
  const Matrix6 Ab = A.template leftCols<6>();
  const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
  const auto Aj = A.rightCols(info_.actuatedDofNum);
  const auto ADot = pinocchio::dccrba(model, data, qDesired, vDesired);
  Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
  centroidalMomentumRate.noalias() -= ADot * vDesired;
  centroidalMomentumRate.noalias() -= Aj * jointAccel;

  Vector6 b = AbInv * centroidalMomentumRate;

  return {a, b, matrix_t(), vector_t()};
}


// [J, 0] x = \dot V - \dotJ v
Task WbcBase::formulateSwingLegTask() {
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
  std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
  std::vector<vector3_t> posDesired = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velDesired = eeKinematics_->getVelocity(vector_t(), vector_t());

  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (!contactFlag_[i]) {
      vector3_t accel = swingKp_ * (posDesired[i] - posMeasured[i]) + swingKd_ * (velDesired[i] - velMeasured[i]);
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      j++;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}


Task WbcBase::formulateArmJointNomalTrackingTask()
{
    matrix_t a(6, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, info_.generalizedCoordinatesNum-6, 6, 6) = matrix_t::Identity(6, 6);

    matrix_t jointKp, jointKd;
    jointKp = matrix_t::Zero(6, 6);
    jointKd = matrix_t::Zero(6, 6);

//    d_arm_ << 0.0, 1.85, -0.532, -0.35, 0, 0;
//    d_arm_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    for (int i = 0; i < 6; ++i) {
        jointKp(i, i) = 4000; //jointKp_ 4000
        jointKd(i, i) = 75; //jointKd_ 75
    }

    b = jointKp * (qDesired_.segment<6>(info_.generalizedCoordinatesNum-6)
            - qMeasured_.segment<6>(info_.generalizedCoordinatesNum-6))
        + jointKd * (vDesired_.segment<6>(info_.generalizedCoordinatesNum-6)
                - vMeasured_.segment<6>(info_.generalizedCoordinatesNum-6));

//    b = jointKp * (d_arm_
//                   - qMeasured_.segment<6>(info_.generalizedCoordinatesNum-6))
//        + jointKd * (d_arm_
//                     - vMeasured_.segment<6>(info_.generalizedCoordinatesNum-6));

    return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateEeLinearMotionTrackingTask() {
    matrix_t a(3, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();

    // current
    armEeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
    std::vector<vector3_t> posMeasured = armEeKinematics_->getPosition(vector_t());
    std::vector<vector3_t> velMeasured = armEeKinematics_->getVelocity(vector_t(), vector_t());

    // desired
    armEeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
    std::vector<vector3_t> posDesired = armEeKinematics_->getPosition(vector_t());
    std::vector<vector3_t> velDesired = armEeKinematics_->getVelocity(vector_t(), vector_t());


    vector3_t linear_acc = armEeLinearKp_ * (posDesired[0] - posMeasured[0])
            + armEeLinearKd_ * (velDesired[0] - velMeasured[0]);
//    std::cerr << "Ee pos desired:: " << posDesired[0].transpose() << std::endl;
//    std::cerr << "Ee pos measured:: " << posMeasured[0].transpose() << std::endl;
//    std::cerr << "Ee vel desired:: " << velDesired[0].transpose() << std::endl;
//    std::cerr << "Ee vel measured:: " << velMeasured[0].transpose() << std::endl;
    
    a.block(0, 0, 3, info_.generalizedCoordinatesNum) = je_.block(0, 0, 3, info_.generalizedCoordinatesNum);

    b = linear_acc - dje_.block(0, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;

    return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateEeAngularMotionTrackingTask(){
    matrix_t a(3, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();

    // measure
    const auto& Mmodel = pinocchioInterfaceMeasured_.getModel();
    auto& Mdata = pinocchioInterfaceMeasured_.getData();

    matrix3_t rotationEeMeasuredToWorld = Mdata.oMf[armEeFrameIdx_].rotation();
    vector3_t armCurrentEeAngularVel = pinocchio::getFrameVelocity(Mmodel, Mdata, armEeFrameIdx_,
                                                                   pinocchio::LOCAL_WORLD_ALIGNED).angular();

    // desired
    const auto& Dmodel = pinocchioInterfaceDesired_.getModel();
    auto& Ddata = pinocchioInterfaceDesired_.getData();
    matrix3_t rotationEeReferenceToWorld = Ddata.oMf[armEeFrameIdx_].rotation();
    vector3_t armDesiredEeAngularVel = pinocchio::getFrameVelocity(Dmodel, Ddata, armEeFrameIdx_,
                                                   pinocchio::LOCAL_WORLD_ALIGNED).angular();

    // error
    vector3_t error = rotationErrorInWorld<scalar_t>(rotationEeReferenceToWorld, rotationEeMeasuredToWorld);

    matrix_t arm_dj_tmp = matrix_t(6, info_.generalizedCoordinatesNum);
    arm_dj_tmp.setZero();
    arm_dj_tmp = dje_;
    arm_dj_tmp.block(3, 3, 3, 3).setZero();

    a.block(0, 0, 3, info_.generalizedCoordinatesNum) = je_.block(3, 0, 3, info_.generalizedCoordinatesNum);
    a.block(0, 3, 3, 3).setZero();


//    b =  armEeAngularKp_ * error + armEeAngularKd_ * (armDesiredEeAngularVel - armCurrentEeAngularVel)
//        - arm_dj_tmp.block(3, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
    b =  armEeAngularKp_ * error + armEeAngularKd_ * (- armCurrentEeAngularVel)
         - arm_dj_tmp.block(3, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;

    return {a, b, matrix_t(), vector_t()};
}


// [0, I] x = GRFs
Task WbcBase::formulateContactForceTask(const vector_t& inputDesired) const {
  matrix_t a(3 * info_.numThreeDofContacts, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();

  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
  }
  b = inputDesired.head(a.rows());

  return {a, b, matrix_t(), vector_t()};
}

void WbcBase::loadTasksSetting(const std::string& taskFile, bool verbose) {
  // Load task file
  torqueLimits_ = vector_t((info_.actuatedDofNum-6) / 4 + 1);
  loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torqueLimits_);
  if (verbose) {
    std::cerr << "\n #### Torque Limits Task:";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "\n #### HAA HFE KFE: " << torqueLimits_.transpose() << "\n";
    std::cerr << " #### =============================================================================\n";
  }
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "frictionConeTask.";
  if (verbose) {
    std::cerr << "\n #### Friction Cone Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, frictionCoeff_, prefix + "frictionCoefficient", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

}

}  // namespace legged
