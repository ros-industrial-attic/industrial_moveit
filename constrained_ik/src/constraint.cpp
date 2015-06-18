/**
 * @file constraint.cpp
 * @brief Base class for IK-solver Constraints
 * Specify relationship between joint velocities and constraint "error"
 * @author dsolomon
 * @date Sep 15, 2013
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
#include "constrained_ik/constraint.h"
#include "constrained_ik/constrained_ik.h"
#include <ros/ros.h>

using namespace Eigen;

namespace constrained_ik
{

// NOTE: This method does a resize in-place, and may be inefficient if called many times
void Constraint::appendError(Eigen::VectorXd &error, const Eigen::VectorXd &addErr)
{
  if (addErr.rows() == 0) return;

  if (error.rows() == 0)
    error = addErr;
  else
  {
    size_t nAddRows = addErr.rows();
    error.conservativeResize(error.rows() + nAddRows);
    error.tail(nAddRows) = addErr;
  }
}

// NOTE: This method does a resize in-place, and may be inefficient if called many times
void Constraint::appendJacobian(Eigen::MatrixXd &jacobian, const Eigen::MatrixXd &addJacobian)
{
  if(addJacobian.rows() == 0 || addJacobian.cols() == 0)
  {
    ROS_ERROR("trying to add a Jacobian with no data");
  }
  if (jacobian.rows() == 0) // first call gets to set size
  {
    size_t nAddRows = addJacobian.rows();
    jacobian.conservativeResize(jacobian.rows() + nAddRows, addJacobian.cols());
    jacobian.bottomRows(nAddRows) = addJacobian;
  }
  else
  {
    ROS_ASSERT(jacobian.cols() == addJacobian.cols());
    size_t nAddRows = addJacobian.rows();
    jacobian.conservativeResize(jacobian.rows() + nAddRows, Eigen::NoChange);
    jacobian.bottomRows(nAddRows) = addJacobian;
  }
}

int Constraint::numJoints()
{
  return ik_->getKin().numJoints();
}

void Constraint::updateError(Eigen::VectorXd &error)
{
  appendError(error, calcError());
}

void Constraint::updateJacobian(Eigen::MatrixXd &jacobian)
{

  Eigen::MatrixXd addJacobian = calcJacobian();
  appendJacobian(jacobian, addJacobian);
}

} // namespace constrained_ik
