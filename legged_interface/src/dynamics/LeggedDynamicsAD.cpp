//
// Created by skywoodsz on 2023/2/21.
//

#include "legged_interface/dynamics/LeggedDynamicsAD.h"

namespace legged {
    using namespace ocs2;
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedDynamicsAD::LeggedDynamicsAD(const PinocchioInterface &pinocchioInterface, const CentroidalModelInfo &info,
                           const std::string &modelName,
                           const ModelSettings &modelSettings)
        : pinocchioCentroidalDynamicsAd_(pinocchioInterface, info, modelName, modelSettings.modelFolderCppAd,
                                         modelSettings.recompileLibrariesCppAd, modelSettings.verboseCppAd) {}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LeggedDynamicsAD::computeFlowMap(scalar_t time, const vector_t &state, const vector_t &input,
                                      const PreComputation &preComp) {
    return pinocchioCentroidalDynamicsAd_.getValue(time, state, input);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LeggedDynamicsAD::linearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                             const PreComputation& preComp) {
    return pinocchioCentroidalDynamicsAd_.getLinearApproximation(time, state, input);
}
}