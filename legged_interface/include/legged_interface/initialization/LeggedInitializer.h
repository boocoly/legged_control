//
// Created by skywoodsz on 2023/2/28.
//

#ifndef SRC_LeggedINITIALIZER_H
#define SRC_LeggedINITIALIZER_H

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/initialization/Initializer.h>

#include "ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h"

namespace legged{
using namespace ocs2;
using namespace legged_robot;

class LeggedInitializer final : public Initializer{
public:
    LeggedInitializer(CentroidalModelInfo info, const SwitchedModelReferenceManager& referenceManager,
                  bool extendNormalizedMomentum = false);
    ~LeggedInitializer() override = default;
    LeggedInitializer* clone() const override;

    void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

private:
    LeggedInitializer(const LeggedInitializer& other) = default;
    const CentroidalModelInfo info_;
    const SwitchedModelReferenceManager* referenceManagerPtr_;
    const bool extendNormalizedMomentum_;
};
}


#endif //SRC_LeggedINITIALIZER_H
