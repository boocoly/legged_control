//
// Created by qiayuan on 2022/7/1.
//

#pragma once

#include "legged_wbc/Task.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

//#include <dynamic_reconfigure/server.h>
//#include "legged_wbc/WbcWeightConfig.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

// Decision Variables: x = [\ddot q^T, F^T]^T
class WbcBase {
  using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

 public:
  WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
          const PinocchioEndEffectorKinematics& eeKinematics, const PinocchioEndEffectorKinematics& armEeKinematics);

  virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

  virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                          scalar_t period, scalar_t time);

 protected:
  void updateMeasured(const vector_t& rbdStateMeasured);
  void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired, ocs2::scalar_t period);
  vector_t updateCmd(vector_t x_optimal);

  size_t getNumDecisionVars() const { return numDecisionVars_; }

  Task formulateFloatingBaseEomTask();
  Task formulateTorqueLimitsTask();
  Task formulateNoContactMotionTask();
  Task formulateFrictionConeTask();

  Task formulateBaseHeightMotionTask();
  Task formulateBaseAngularMotionTask();
  Task formulateBaseXYLinearAccelTask();
  Task formulateSwingLegTask();

  Task formulateArmJointNomalTrackingTask();
  Task formulateEeLinearMotionTrackingTask();
  Task formulateEeAngularMotionTrackingTask();

  Task formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);
  Task formulateContactForceTask(const vector_t& inputDesired) const;

 private:
//  void dynamicCallback(legged_wbc::WbcWeightConfig& config, uint32_t /*level*/);
//  std::shared_ptr<dynamic_reconfigure::Server<legged_wbc::WbcWeightConfig>> dynamic_srv_{};

  size_t numDecisionVars_;
  PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
  CentroidalModelInfo info_;
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_, armEeKinematics_;
  CentroidalModelPinocchioMapping mapping_;

  vector_t qMeasured_, vMeasured_, inputLast_;
  vector_t qDesired_, vDesired_, baseAccDesired_;
  vector_t jointAccel_;
  matrix_t j_, dj_;
  matrix_t je_, dje_;
  matrix_t jb_, djb_;
  contact_flag_t contactFlag_{};
  size_t numContacts_{};

  // Task Parameters:
  vector_t torqueLimits_;
  scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{};

  scalar_t baseHeightKp_{}, baseHeightKd_{};
  scalar_t baseAngularKp_{}, baseAngularKd_{};

  matrix_t jointKp_, jointKd_;
  // Desired angle of arm joint
  vector_t d_arm_;

  matrix_t armEeLinearKp_{}, armEeLinearKd_{};
  matrix_t armEeAngularKp_{}, armEeAngularKd_{};
  // Desired position and orientation of the end-effector
  vector_t d_ee_, da_ee_;

  size_t armEeFrameIdx_{};

  Eigen::Matrix3d zyx2xyz_;
};


template <typename T>
using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
/*!
 * Compute the pseudo inverse of a matrix
 * @param matrix : input matrix
 * @param sigmaThreshold : threshold for singular values being zero
 * @param invMatrix : output matrix
 */
template <typename T>
void pseudoInverse(DMat<T> const& matrix, double sigmaThreshold, DMat<T>& invMatrix)
{
    if ((1 == matrix.rows()) && (1 == matrix.cols())) {
        invMatrix.resize(1, 1);
        if (matrix.coeff(0, 0) > sigmaThreshold) {
            invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
        }
        else {
            invMatrix.coeffRef(0, 0) = 0.0;
        }
        return;
    }

    Eigen::JacobiSVD<DMat<T>> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // not sure if we need to svd.sort()... probably not
    int const nrows(svd.singularValues().rows());
    DMat<T> invS;
    invS = DMat<T>::Zero(nrows, nrows);
    for (int ii(0); ii < nrows; ++ii)
    {
        if (svd.singularValues().coeff(ii) > sigmaThreshold) {
            invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
        }
        else {
            // invS.coeffRef(ii, ii) = 1.0/ sigmaThreshold;
            // printf("sigular value is too small: %f\n",
            // svd.singularValues().coeff(ii));
        }
    }
    invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();
}

}  // namespace legged
