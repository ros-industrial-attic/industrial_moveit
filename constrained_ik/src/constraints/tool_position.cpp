/**
* @file tool_position.cpp
* @brief Constraint to specify cartesian goal position in tool frame (XYZ rotation).
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
#include "constrained_ik/constraints/tool_position.h"
#include <ros/ros.h>

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;

// initialize limits/tolerances to default values
ToolPosition::ToolPosition() : GoalPosition()
{
}

Eigen::VectorXd ToolPosition::calcError()
{
  //rotate jacobian into tool frame by premultiplying by otR.transpose()
  Vector3d err = state_.pose_estimate.rotation().transpose() * GoalPosition::calcError();

  ROS_ASSERT(err.rows() == 3);
  return err;
}

// translate cartesian errors into joint-space errors
Eigen::MatrixXd ToolPosition::calcJacobian()
{
  MatrixXd tmpJ;
  if (!ik_->getKin().calcJacobian(state_.joints, tmpJ))
    throw std::runtime_error("Failed to calculate Jacobian");

  //rotate jacobian into tool frame by premultiplying by otR.transpose()
  MatrixXd J = state_.pose_estimate.rotation().transpose() * tmpJ.topRows(3);

  // weight each row of J
  for (size_t ii=0; ii<3; ++ii)
      J.row(ii) *= weight_(ii);

  ROS_ASSERT(J.rows() == 3);
  return J;
}

} // namespace constraints
} // namespace constrained_ik

