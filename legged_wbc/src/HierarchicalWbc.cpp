//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/HierarchicalWbc.h"

#include "legged_wbc/HoQp.h"

namespace legged {
vector_t HierarchicalWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                                 scalar_t period, scalar_t time) {
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);

  Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask()
          + formulateNoContactMotionTask() + formulateFrictionConeTask();
  Task taskInit = formulateArmJointNomalTrackingTask();

//  Task task1 = formulateBaseHeightMotionTask() + formulateBaseAngularMotionTask() +
//                formulateEeLinearMotionTrackingTask() + formulateEeAngularMotionTrackingTask()
//                + formulateSwingLegTask() * 100;
    Task task1 = formulateBaseHeightMotionTask() + formulateBaseAngularMotionTask() +
                    formulateArmJointNomalTrackingTask() + formulateSwingLegTask() * 100;

  Task task2 = formulateContactForceTask(inputDesired) + formulateBaseXYLinearAccelTask();  //formulateArmJointNomalTrackingTask()



  HoQp hoQp(task2, std::make_shared<HoQp>(taskInit, std::make_shared<HoQp>(task0)));
  vector_t x_optimal = hoQp.getSolutions();
  return WbcBase::updateCmd(x_optimal);
//    if(time < 10)
//    {
//        HoQp hoQp(task2, std::make_shared<HoQp>(taskInit, std::make_shared<HoQp>(task0)));
//        vector_t x_optimal = hoQp.getSolutions();
//        return WbcBase::updateCmd(x_optimal);
//    }
//    else
//    {
//        HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0))); //
//        vector_t x_optimal = hoQp.getSolutions();
//        return WbcBase::updateCmd(x_optimal);
//    }
}


void HierarchicalWbc::loadTasksSetting(const std::string& taskFile, bool verbose) {
  WbcBase::loadTasksSetting(taskFile, verbose);

}

}  // namespace legged
