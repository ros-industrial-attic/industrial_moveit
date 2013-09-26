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
  VectorXd err(this->size());
  size_t row=0;

  // concatenate error vectors
  for (size_t i=0; i<constraints_.size(); ++i)
  {
    Constraint &c = constraints_[i];
    err.segment(row, c.size()) = c.calcError();
    row += c.size();
  }

  return err;
}

// translate task-space errors into joint-space errors
Eigen::MatrixXd ConstraintGroup::calcJacobian()
{
  VectorXd J( this->size() );
  size_t row=0;

  // concatenate Jacobian matrices
  for (size_t i=0; i<constraints_.size(); ++i)
  {
    Constraint &c = constraints_[i];
    J.block(row, 0, c.size(), 6) = c.calcJacobian();
    row += c.size();
  }

  return J;
}

bool ConstraintGroup::checkStatus() const
{
  bool done=true;

  for (size_t i=0; i<constraints_.size(); ++i)
    done &= constraints_[i].checkStatus();

  if (done) return done;

  return Constraint::checkStatus();
}

void ConstraintGroup::reset()
{
  Constraint::reset();

  for (size_t i=0; i<constraints_.size(); ++i)
    constraints_[i].reset();
}

unsigned int ConstraintGroup::size() const
{
  unsigned int n=0;
  for (size_t i=0; i<constraints_.size(); ++i)
    n += constraints_[i].size();

  return n;
}


void ConstraintGroup::update(const SolverState &state)
{
  Constraint::update(state);

  for (size_t i=0; i<constraints_.size(); ++i)
    constraints_[i].update(state);
}

} // namespace constrained_ik

