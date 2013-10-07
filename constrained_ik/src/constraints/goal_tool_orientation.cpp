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
#include "constrained_ik/constraints/goal_tool_orientation.h"
#include <ros/ros.h>

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;

// initialize limits/tolerances to default values
GoalToolOrientation::GoalToolOrientation() : Constraint(), rot_err_tol_(0.009), rot_err_(0.0), weight_(Vector3d::Ones())
{
}

double GoalToolOrientation::calcAngle(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2)
{
  Quaterniond q1(p1.rotation()), q2(p2.rotation());
  return q1.angularDistance(q2);
}

Eigen::Vector3d GoalToolOrientation::calcAngleError(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2)
{
  Eigen::AngleAxisd r12(p1.rotation().transpose()*p2.rotation());   // rotation from p1 -> p2
  double theta = Constrained_IK::rangedAngle(r12.angle());          // TODO: move rangedAngle to utils class
  return p1.rotation() * r12.axis() * theta;                        // axis k * theta expressed in frame0
}

Eigen::VectorXd GoalToolOrientation::calcError()
{
  //rotate jacobian into tool frame by premultiplying by otR.transpose()
  Vector3d err = state_.pose_estimate.rotation().transpose() * calcAngleError(state_.pose_estimate, state_.goal);

  err = err.cwiseProduct(weight_);
  ROS_ASSERT(err.rows() == 3);
  return err;
}

// translate cartesian errors into joint-space errors
Eigen::MatrixXd GoalToolOrientation::calcJacobian()
{
  MatrixXd tmpJ;
  if (!ik_->getKin().calcJacobian(state_.joints, tmpJ))
    throw std::runtime_error("Failed to calculate Jacobian");

  //rotate jacobian into tool frame by premultiplying by otR.transpose()
  MatrixXd J = state_.pose_estimate.rotation().transpose() * tmpJ.bottomRows(3);

  // weight each row of J
  for (size_t ii=0; ii<3; ++ii)
      J.row(ii) *= weight_(ii);

  ROS_ASSERT(J.rows() == 3);
  return J;
}

bool GoalToolOrientation::checkStatus() const
{
  // check to see if we've reached the goal orientation
  if (rot_err_ < rot_err_tol_)
    return true;

  return Constraint::checkStatus();
}

void GoalToolOrientation::reset()
{
  Constraint::reset();
  rot_err_ = 0;
}

void GoalToolOrientation::update(const SolverState &state)
{
  Constraint::update(state);

  rot_err_ = calcAngle(state.goal, state.pose_estimate);
}

} // namespace constraints
} // namespace constrained_ik

