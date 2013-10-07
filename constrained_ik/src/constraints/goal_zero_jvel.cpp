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
#include "constrained_ik/constraints/goal_zero_jvel.h"

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;

// initialize limits/tolerances to default values
GoalZeroJVel::GoalZeroJVel() : Constraint(), weight_(1.0)
{
}

Eigen::VectorXd GoalZeroJVel::calcError()
{
    return VectorXd::Zero(numJoints());
}

Eigen::MatrixXd GoalZeroJVel::calcJacobian()
{
    size_t n = numJoints();    // number of joints
    MatrixXd  J = MatrixXd::Identity(n,n) * weight_;
    return J;
}

} // namespace constraints
} // namespace constrained_ik

