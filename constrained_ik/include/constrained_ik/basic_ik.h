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


#ifndef BASIC_IK_H
#define BASIC_IK_H

#include "constrained_ik.h"
#include "constraints/avoid_joint_limits.h"
#include "constraints/goal_pose.h"

namespace constrained_ik
{
namespace basic_ik
{

/**
 * \brief Basic IK Solver
 *          - solve for 6DOF cartesian goal
 */
class Basic_IK : public Constrained_IK
{
public:
  Basic_IK()  {
    addConstraint(new constraints::GoalPose());
    addConstraint(new constraints::AvoidJointLimits());
  }
  ~Basic_IK() {};

}; // class Basic_IK

} // namespace basic_ik
} // namespace constrained_ik


#endif // BASIC_IK_H

