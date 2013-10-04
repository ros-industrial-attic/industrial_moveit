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

void ConstraintGroup::add(Constraint* constraint)
{
  if (initialized_)
    constraint->init(ik_);

  constraints_.push_back(constraint);
}

Eigen::VectorXd ConstraintGroup::calcError()
{
  VectorXd error;
  for (size_t i=0; i<constraints_.size(); ++i)
    constraints_[i].updateError(error);

  return error;
}

// translate task-space errors into joint-space errors
Eigen::MatrixXd ConstraintGroup::calcJacobian()
{
  MatrixXd jacobian;
  for (size_t i=0; i<constraints_.size(); ++i)
    constraints_[i].updateJacobian(jacobian);

  return jacobian;
}

bool ConstraintGroup::checkStatus() const
{
  bool done=true;

  for (size_t i=0; i<constraints_.size(); ++i)
    done &= constraints_[i].checkStatus();

  if (done) return done;

  return Constraint::checkStatus();
}

void ConstraintGroup::init(const Constrained_IK* ik)
{
  Constraint::init(ik);

  for (size_t i=0; i<constraints_.size(); ++i)
    constraints_[i].init(ik);
}

void ConstraintGroup::reset()
{
  Constraint::reset();

  for (size_t i=0; i<constraints_.size(); ++i)
    constraints_[i].reset();
}

void ConstraintGroup::update(const SolverState &state)
{
  Constraint::update(state);

  for (size_t i=0; i<constraints_.size(); ++i)
    constraints_[i].update(state);
}

} // namespace constrained_ik

