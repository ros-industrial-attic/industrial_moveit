/**
 * @file goal_orientation.cpp
 * @brief Constraint to specify Cartesian goal orientation (XYZ rotation)
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
#include "constrained_ik/constraints/goal_orientation.h"
#include <ros/ros.h>

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;

// initialize limits/tolerances to default values
GoalOrientation::GoalOrientation() : Constraint(), rot_err_tol_(0.009), weight_(Vector3d::Ones())
{
}

constrained_ik::ConstraintResults GoalOrientation::evalConstraint(const SolverState &state) const
{
  constrained_ik::ConstraintResults output;
  GoalOrientation::GoalOrientationData cdata(state);

  output.error = calcError(cdata);
  output.jacobian = calcJacobian(cdata);
  output.status = checkStatus(cdata);

  return output;
}

double GoalOrientation::calcAngle(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2)
{
  Quaterniond q1(p1.rotation()), q2(p2.rotation());
  return q1.angularDistance(q2);
}

Eigen::Vector3d GoalOrientation::calcAngleError(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2)
{
  Eigen::AngleAxisd r12(p1.rotation().transpose()*p2.rotation());   // rotation from p1 -> p2
  double theta = Constrained_IK::rangedAngle(r12.angle());          // TODO: move rangedAngle to utils class
  return p1.rotation() * r12.axis() * theta;                        // axis k * theta expressed in frame0
}

Eigen::VectorXd GoalOrientation::calcError(const GoalOrientation::GoalOrientationData &cdata) const
{
  Vector3d err = calcAngleError(cdata.state_.pose_estimate, cdata.state_.goal);
  err = err.cwiseProduct(weight_);
  ROS_ASSERT(err.rows() == 3);
  return err;
}

// translate cartesian errors into joint-space errors
Eigen::MatrixXd GoalOrientation::calcJacobian(const GoalOrientation::GoalOrientationData &cdata) const
{
  MatrixXd tmpJ;
  if (!ik_->getKin().calcJacobian(cdata.state_.joints, tmpJ))
    throw std::runtime_error("Failed to calculate Jacobian");
  MatrixXd J = tmpJ.bottomRows(3);

  // weight each row of J
  for (size_t ii=0; ii<3; ++ii)
      J.row(ii) *= weight_(ii);

  ROS_ASSERT(J.rows() == 3);
  return J;
}

bool GoalOrientation::checkStatus(const GoalOrientation::GoalOrientationData &cdata) const
{
  // check to see if we've reached the goal orientation
  if (cdata.rot_err_ < rot_err_tol_)
    return true;

  return false;
}

GoalOrientation::GoalOrientationData::GoalOrientationData(const SolverState &state): ConstraintData(state)
{
  rot_err_ = calcAngle(state_.goal, state_.pose_estimate);
}


} // namespace constraints
} // namespace constrained_ik

