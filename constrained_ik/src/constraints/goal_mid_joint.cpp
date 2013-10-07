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

#include "constrained_ik/constrained_ik.h"
#include "constrained_ik/constraints/goal_mid_joint.h"

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;

// initialize limits/tolerances to default values
GoalMidJoint::GoalMidJoint() : Constraint(), weight_(1.0)
{
}

Eigen::VectorXd GoalMidJoint::calcError()
{
    VectorXd err = mid_range_ - state_.joints;
    err *= weight_;
    return err;
}

Eigen::MatrixXd GoalMidJoint::calcJacobian()
{
    size_t n = state_.joints.size();    // number of joints
    MatrixXd  J = MatrixXd::Identity(n,n) * weight_;
    return J;
}

void GoalMidJoint::init(const Constrained_IK *ik)
{
  Constraint::init(ik);

  // initialize joint/thresholding limits
  MatrixXd joint_limits = ik->getKin().getLimits();
  mid_range_ = joint_limits.col(1) - joint_limits.col(0);
}

} // namespace constraints
} // namespace constrained_ik

