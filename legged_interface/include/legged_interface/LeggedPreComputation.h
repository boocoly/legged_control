//
// Created by skywoodsz on 2023/2/21.
//

#ifndef SRC_LeggedPRECOMPUTATION_H
#define SRC_LeggedPRECOMPUTATION_H

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include "legged_interface/common/ModelSettings.h"
#include "ocs2_legged_robot/constraint/EndEffectorLinearConstraint.h"
#include "ocs2_legged_robot/foot_planner/SwingTrajectoryPlanner.h"

namespace legged{
using namespace ocs2;
using namespace legged_robot;

/** Callback for caching and reference update */
class LeggedPreComputation : public PreComputation {
public:
    LeggedPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                              const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings);
    ~LeggedPreComputation() override = default;

    LeggedPreComputation* clone() const override { return new LeggedPreComputation(*this); }

    void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

    const std::vector<EndEffectorLinearConstraint::Config>& getEeNormalVelocityConstraintConfigs() const { return eeNormalVelConConfigs_; }

    PinocchioInterface& getPinocchioInterface() { return pinocchioInterface_; }
    const PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

protected:
    LeggedPreComputation(const LeggedPreComputation& other);

private:
    PinocchioInterface pinocchioInterface_;
    CentroidalModelInfo info_;
    const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
    std::unique_ptr<CentroidalModelPinocchioMapping> mappingPtr_;
    const ModelSettings settings_;

    std::vector<EndEffectorLinearConstraint::Config> eeNormalVelConConfigs_;
};

}

#endif //SRC_LeggedPRECOMPUTATION_H
