/**
* @file goal_position.cpp
* @brief Constraint to specify cartesian goal position (XYZ)
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
#include "constrained_ik/constraints/goal_position.h"
#include <ros/assert.h>

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;

// initialize limits/tolerances to default values
GoalPosition::GoalPosition() : Constraint(), pos_err_tol_(0.001), weight_(Vector3d::Ones())
{
}

constrained_ik::ConstraintResults GoalPosition::evalConstraint(const SolverState &state) const
{
  constrained_ik::ConstraintResults output;
  GoalPosition::GoalPositionData cdata(state);

  output.error = calcError(cdata);
  output.jacobian = calcJacobian(cdata);
  output.status = checkStatus(cdata);

  return output;
}

Eigen::VectorXd GoalPosition::calcError(const GoalPosition::GoalPositionData &cdata) const
{
  Vector3d goalPos = cdata.state_.goal.translation();
  Vector3d estPos  = cdata.state_.pose_estimate.translation();
  Vector3d err = (goalPos - estPos).cwiseProduct(weight_);

  ROS_ASSERT( err.rows() == 3 );
  return err;
}

// translate cartesian errors into joint-space errors
Eigen::MatrixXd GoalPosition::calcJacobian(const GoalPosition::GoalPositionData &cdata) const
{
  MatrixXd tmpJ;
  if (!ik_->getKin().calcJacobian(cdata.state_.joints, tmpJ))
    throw std::runtime_error("Failed to calculate Jacobian");
  MatrixXd  J = tmpJ.topRows(3);

  // weight each row of J
  for (size_t ii=0; ii<3; ++ii)
      J.row(ii) *= weight_(ii);

  ROS_ASSERT( J.rows() == 3);
  return J;
}

double GoalPosition::calcDistance(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2)
{
  return (p2.translation() - p1.translation()).norm();
}

bool GoalPosition::checkStatus(const GoalPosition::GoalPositionData &cdata) const
{
  // check to see if we've reached the goal position
  if (cdata.pos_err_ < pos_err_tol_)
    return true;

  return false;
}

GoalPosition::GoalPositionData::GoalPositionData(const SolverState &state): ConstraintData(state)
{
  pos_err_ = calcDistance(state.goal, state.pose_estimate);
}

} // namespace constraints
} // namespace constrained_ik

