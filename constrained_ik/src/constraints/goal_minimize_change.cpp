/**
 * @file goal_minimize_change.cpp
 * @brief Constraint to pushes joints back towards their starting position
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
#include "constrained_ik/constraints/goal_minimize_change.h"

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;

// initialize limits/tolerances to default values
GoalMinimizeChange::GoalMinimizeChange() : Constraint(), weight_(1.0)
{
}

Eigen::VectorXd GoalMinimizeChange::calcError()
{
    VectorXd err = state_.joint_seed - state_.joints;
    err *= weight_;
    return err;
}

Eigen::MatrixXd GoalMinimizeChange::calcJacobian()
{
    size_t n = numJoints();    // number of joints
    MatrixXd  J = MatrixXd::Identity(n,n) * weight_;
    return J;
}

} // namespace constraints
} // namespace constrained_ik

