/**
 * @file constraint_group.cpp
 * @brief Base class for IK-solver Constraints
 *
 * Specify relationship between joint velocities and constraint "error"
 *
 * @author dsolomon
 * @date Sep 15, 2013
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

constrained_ik::ConstraintResults ConstraintGroup::evalConstraint(const SolverState &state) const
{
  constrained_ik::ConstraintResults output;
  for (size_t i=0; i<constraints_.size(); ++i)
    output.append(constraints_[i].evalConstraint(state));

  return output;
}

void ConstraintGroup::init(const Constrained_IK* ik)
{
  Constraint::init(ik);

  for (size_t i=0; i<constraints_.size(); ++i)
    constraints_[i].init(ik);
}

} // namespace constrained_ik

