//
// Created by skywoodsz on 2023/2/21.
//

#ifndef SRC_LeggedROBOTDYNAMICSAD_H
#define SRC_LeggedROBOTDYNAMICSAD_H

#include <ocs2_core/dynamics/SystemDynamicsBase.h>

#include <ocs2_centroidal_model/PinocchioCentroidalDynamicsAD.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "legged_interface/common/ModelSettings.h"

namespace legged{
using namespace ocs2;

class LeggedDynamicsAD final : public SystemDynamicsBase{
public:
    LeggedDynamicsAD(const PinocchioInterface& pinocchioInterface, const CentroidalModelInfo& info, const std::string& modelName,
                 const ModelSettings& modelSettings);

    ~LeggedDynamicsAD() override = default;

    LeggedDynamicsAD* clone() const override {return new LeggedDynamicsAD(*this);}

    vector_t computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) override;
    VectorFunctionLinearApproximation linearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                          const PreComputation& preComp) override;

private:
    LeggedDynamicsAD(const LeggedDynamicsAD& rhs) = default;

    PinocchioCentroidalDynamicsAD pinocchioCentroidalDynamicsAd_;
};

}


#endif //SRC_LeggedROBOTDYNAMICSAD_H
