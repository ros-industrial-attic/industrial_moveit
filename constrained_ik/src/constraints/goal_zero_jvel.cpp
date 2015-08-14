/**
* @file goal_zero_jvel.cpp
* @brief Constraint to dampen movement by driving joint velocity to zero in each iteration
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

constrained_ik::ConstraintResults GoalZeroJVel::evalConstraint(const SolverState &state) const
{
  constrained_ik::ConstraintResults output;
  GoalZeroJVel::ConstraintData cdata(state);

  output.error = calcError(cdata);
  output.jacobian = calcJacobian(cdata);
  output.status = checkStatus(cdata);

  return output;
}

Eigen::VectorXd GoalZeroJVel::calcError(const GoalZeroJVel::ConstraintData &cdata) const
{
    return VectorXd::Zero(numJoints());
}

Eigen::MatrixXd GoalZeroJVel::calcJacobian(const GoalZeroJVel::ConstraintData &cdata) const
{
    size_t n = numJoints();    // number of joints
    MatrixXd  J = MatrixXd::Identity(n,n) * weight_;
    return J;
}

} // namespace constraints
} // namespace constrained_ik

