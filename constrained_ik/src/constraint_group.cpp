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

#include "constrained_ik/constraint_group.h"
#include "constrained_ik/constrained_ik.h"
#include <ros/ros.h>

namespace constrained_ik
{

using namespace Eigen;

// initialize limits/tolerances to default values
ConstraintGroup::ConstraintGroup() : Constraint()
{
}

Eigen::VectorXd ConstraintGroup::calcError()
{
  // calculate Error for each constraint
  std::vector<VectorXd> errors;
  for (size_t i=0; i<constraints_.size(); ++i)
    errors.push_back( constraints_[i].calcError() );

  return concatErrors(errors);
}

// translate task-space errors into joint-space errors
Eigen::MatrixXd ConstraintGroup::calcJacobian()
{
  // calculate Jacobians for each constraint
  std::vector<MatrixXd> jacobians;
  for (size_t i=0; i<constraints_.size(); ++i)
    jacobians.push_back( constraints_[i].calcJacobian() );

  return concatJacobians(jacobians);
}

bool ConstraintGroup::checkStatus() const
{
  bool done=true;

  for (size_t i=0; i<constraints_.size(); ++i)
    done &= constraints_[i].checkStatus();

  if (done) return done;

  return Constraint::checkStatus();
}

Eigen::VectorXd ConstraintGroup::concatErrors(const std::vector<Eigen::VectorXd> &errors)
{
  // count # of rows
  size_t nRows = 0;
  for (int i=0; i<errors.size(); ++i)
    nRows += errors[i].rows();

  if (nRows == 0) return VectorXd();

  // concatenate errors into one vector
  VectorXd combinedErr(nRows);
  size_t row=0;
  for (size_t i=0; i<errors.size(); ++i)
  {
    const VectorXd &tmpErr = errors[i];
    if (tmpErr.rows() > 0)
      combinedErr.segment(row, tmpErr.rows()) = tmpErr;
    row += tmpErr.rows();
  }

  return combinedErr;
}

Eigen::MatrixXd ConstraintGroup::concatJacobians(const std::vector<Eigen::MatrixXd> &jacobians)
{
  // count # of rows
  size_t nRows = 0, nCols = 0;
  for (int i=0; i<jacobians.size(); ++i)
  {
    const MatrixXd &tmpJ = jacobians[i];

    nRows += tmpJ.rows();
    nCols = (nCols>0) ? nCols : tmpJ.cols();

    if (tmpJ.cols() > 0 && tmpJ.cols() != nCols)
    {
      ROS_ERROR("Jacobian column mismatch (%d / %d)", tmpJ.cols(), nCols);
      throw std::runtime_error("Jacobian column mismatch");
    }
  }

  if (nRows == 0) return MatrixXd();

  // concatenate jacobians into one matrix
  MatrixXd combinedJ(nRows, jacobians[0].cols());
  size_t row=0;
  for (size_t i=0; i<jacobians.size(); ++i)
  {
    const MatrixXd &tmpJ = jacobians[i];
    if (tmpJ.rows() > 0)
      combinedJ.block(row, 0, tmpJ.rows(), tmpJ.cols()) = tmpJ;
    row += tmpJ.rows();
  }

  return combinedJ;
}

void ConstraintGroup::reset()
{
  Constraint::reset();

  for (size_t i=0; i<constraints_.size(); ++i)
    constraints_[i].reset();
}

void ConstraintGroup::setIK(const Constrained_IK* ik)
{
  ik_ = ik;
  for (size_t i=0; i<constraints_.size(); ++i)
    constraints_[i].setIK(ik);
}

void ConstraintGroup::update(const SolverState &state)
{
  Constraint::update(state);

  for (size_t i=0; i<constraints_.size(); ++i)
    constraints_[i].update(state);
}

} // namespace constrained_ik

