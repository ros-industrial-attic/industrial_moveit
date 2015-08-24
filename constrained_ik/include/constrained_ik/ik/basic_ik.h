/**
* @file basic_ik.h
* @brief Basic IK Solver, solves for 6DOF cartesian goal.
* @author dsolomon
* @date Sep 23, 2013
* @version TODO
* @bug No known bugs
*
* @copyright Copyright (c) 2013, Southwest Research Institute
*
* @license Software License Agreement (Apache License)\n
* \n
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at\n
* \n
* http://www.apache.org/licenses/LICENSE-2.0\n
* \n
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
#ifndef BASIC_IK_H
#define BASIC_IK_H

#include "constrained_ik/constrained_ik.h"
#include "constrained_ik/constraints/avoid_joint_limits.h"
#include "constrained_ik/constraints/goal_pose.h"
#include "constrained_ik/enum_types.h"

namespace constrained_ik
{
namespace basic_ik
{

/** @brief Basic IK Solver, solves for 6DOF cartesian goal. */
class Basic_IK : public Constrained_IK
{
public:
  Basic_IK(): goal_pose_(new constraints::GoalPose), avoid_joint_limits_(new constraints::AvoidJointLimits)
  {
    addConstraint(goal_pose_, constraint_types::primary);
    addConstraint(avoid_joint_limits_, constraint_types::primary);
    avoid_joint_limits_->setDebug(false);
    Eigen::Vector3d w_ori;
    w_ori << 1,1,1;
    goal_pose_->setWeightOrientation(w_ori);
  }
  ~Basic_IK() {}

protected:

  constraints::GoalPose* goal_pose_;
  constraints::AvoidJointLimits* avoid_joint_limits_;

}; // class Basic_IK

} // namespace basic_ik
} // namespace constrained_ik


#endif // BASIC_IK_H

