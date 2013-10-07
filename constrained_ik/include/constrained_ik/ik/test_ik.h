/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef TEST_IK_H
#define TEST_IK_H

#include "constrained_ik/constrained_ik.h"
#include "constrained_ik/constraints/avoid_joint_limits.h"
#include "constrained_ik/constraints/goal_position.h"
#include "constrained_ik/constraints/goal_orientation.h"
#include "constrained_ik/constraints/goal_tool_orientation.h"
#include "constrained_ik/constraints/goal_minimize_change.h"
#include "constrained_ik/constraints/goal_zero_jvel.h"

namespace constrained_ik
{
namespace test_ik
{

class Test_IK : public Constrained_IK
{
public:
  Test_IK():  position_(new constraints::GoalPosition),
              orientation_(new constraints::GoalOrientation),
              tool_orientation_(new constraints::GoalToolOrientation),
              avoid_joint_limits_(new constraints::AvoidJointLimits),
              min_change_(new constraints::GoalMinimizeChange),
              zero_vel_(new constraints::GoalZeroJVel)
  {
    addConstraint(position_);
    addConstraint(orientation_);
//    addConstraint(tool_orientation_);
    addConstraint(avoid_joint_limits_);
    addConstraint(min_change_);
//    addConstraint(zero_vel_);

    Eigen::Vector3d w_ori;
    w_ori << 1,0.1,1;
//    tool_orientation_->setWeight(w_ori);
    orientation_->setWeight(w_ori);

//    avoid_joint_limits_->setWeight(.25);
    min_change_->setWeight(.3);
//    zero_vel_->setWeight(.7);
  }
  ~Test_IK() {};

protected:

  constraints::GoalPosition* position_;
  constraints::GoalOrientation* orientation_;
  constraints::GoalToolOrientation* tool_orientation_;
  constraints::AvoidJointLimits* avoid_joint_limits_;
  constraints::GoalMinimizeChange* min_change_;
  constraints::GoalZeroJVel* zero_vel_;

}; // class Test_IK

} // namespace test_ik
} // namespace constrained_ik


#endif // TEST_IK_H

