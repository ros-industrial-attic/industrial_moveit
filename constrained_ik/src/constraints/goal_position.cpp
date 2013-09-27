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
#include "constrained_ik/constraints/goal_position.h"
#include <ros/assert.h>

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;

// initialize limits/tolerances to default values
GoalPosition::GoalPosition() : Constraint(), pos_err_tol_(0.001), pos_err_(0.0)
{
}

Eigen::VectorXd GoalPosition::calcError()
{
  Vector3d goalPos = state_.goal.translation();
  Vector3d estPos  = state_.pose_estimate.translation();
  Vector3d err = goalPos - estPos;

  ROS_ASSERT( err.rows() == 3 );
  return err;
}

// translate cartesian errors into joint-space errors
Eigen::MatrixXd GoalPosition::calcJacobian()
{
  MatrixXd tmpJ;
  if (!ik_->getKin().calcJacobian(state_.joints, tmpJ))
    throw std::runtime_error("Failed to calculate Jacobian");
  MatrixXd  J = tmpJ.topRows(3);

  ROS_ASSERT( J.rows() == 3);
  return J;
}

double GoalPosition::calcDistance(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2)
{
  return (p2.translation() - p1.translation()).norm();
}

bool GoalPosition::checkStatus() const
{
  // check to see if we've reached the goal position
  if (pos_err_ < pos_err_tol_)
    return true;

  return Constraint::checkStatus();
}

void GoalPosition::reset()
{
  Constraint::reset();
  pos_err_ = 0;
}

void GoalPosition::update(const SolverState &state)
{
  Constraint::update(state);

  pos_err_ = calcDistance(state.goal, state.pose_estimate);
}

} // namespace constraints
} // namespace constrained_ik

