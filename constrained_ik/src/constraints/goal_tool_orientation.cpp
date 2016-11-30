/**
 * @file goal_tool_orientation.cpp
 * @brief Constraint to specify cartesian goal tool orientation (XYZ rotation).
 *
 * @author dsolomon
 * @date Sep 23, 2013
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "constrained_ik/constrained_ik.h"
#include "constrained_ik/constraints/goal_tool_orientation.h"
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(constrained_ik::constraints::GoalToolOrientation, constrained_ik::Constraint)

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;

// initialize limits/tolerances to default values
GoalToolOrientation::GoalToolOrientation() : GoalOrientation()
{
}

constrained_ik::ConstraintResults GoalToolOrientation::evalConstraint(const SolverState &state) const
{
  constrained_ik::ConstraintResults output;
  GoalOrientation::GoalOrientationData cdata(state);

  output.error = calcError(cdata);
  output.jacobian = calcJacobian(cdata);
  output.status = checkStatus(cdata);

  return output;
}

Eigen::VectorXd GoalToolOrientation::calcError(const GoalOrientation::GoalOrientationData &cdata) const
{
  //rotate jacobian into tool frame by premultiplying by otR.transpose()
  Vector3d err = cdata.state_.pose_estimate.rotation().transpose() * calcAngleError(cdata.state_.pose_estimate, cdata.state_.goal);

  err = err.cwiseProduct(weight_);
  ROS_ASSERT(err.rows() == 3);
  return err;
}

// translate cartesian errors into joint-space errors
Eigen::MatrixXd GoalToolOrientation::calcJacobian(const GoalOrientation::GoalOrientationData &cdata) const
{
  MatrixXd tmpJ;
  if (!ik_->getKin().calcJacobian(cdata.state_.joints, tmpJ))
    throw std::runtime_error("Failed to calculate Jacobian");

  //rotate jacobian into tool frame by premultiplying by otR.transpose()
  MatrixXd J = cdata.state_.pose_estimate.rotation().transpose() * tmpJ.bottomRows(3);

  // weight each row of J
  for (size_t ii=0; ii<3; ++ii)
      J.row(ii) *= weight_(ii);

  ROS_ASSERT(J.rows() == 3);
  return J;
}

} // namespace constraints
} // namespace constrained_ik

