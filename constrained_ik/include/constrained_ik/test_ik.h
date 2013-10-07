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

#include "constrained_ik.h"
#include "constraints/avoid_joint_limits.h"
#include "constraints/goal_position.h"
#include "constraints/goal_tool_orientation.h"

namespace constrained_ik
{
namespace test_ik
{

class Test_IK : public Constrained_IK
{
public:
  Test_IK(): position_(new constraints::GoalPosition), tool_orientation_(new constraints::GoalToolOrientation), avoid_joint_limits_(new constraints::AvoidJointLimits)
  {
    addConstraint(position_);
    addConstraint(tool_orientation_);
    addConstraint(avoid_joint_limits_);
    Eigen::Vector3d w_ori;
    w_ori << 0,1,1;
    tool_orientation_->setWeight(w_ori);
  }
  ~Test_IK() {};

protected:

  constraints::GoalPosition* position_;
  constraints::GoalToolOrientation* tool_orientation_;
  constraints::AvoidJointLimits* avoid_joint_limits_;

}; // class Test_IK

} // namespace test_ik
} // namespace constrained_ik


#endif // TEST_IK_H

