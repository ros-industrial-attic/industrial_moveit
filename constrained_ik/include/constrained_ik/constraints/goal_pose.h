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


#ifndef GOAL_POSE_H
#define GOAL_POSE_H

#include "constrained_ik/constraint.h"
#include "goal_position.h"
#include "goal_orientation.h"

namespace constrained_ik
{
namespace constraints
{

/**
 * \brief Constraint to specify cartesian goal pose (XYZ+orientation)
 *          - convenience class, built from goal_position and goal_orientation constraints
  */
class GoalPose : public ConstraintGroup
{
public:
  GoalPose() : ConstraintGroup()
  {
    this->add(new GoalPosition());
    this->add(new GoalOrientation());
  }
  virtual ~GoalPose() {};

}; // class GoalPose

} // namespace constraints
} // namespace constrained_ik


#endif // GOAL_POSE_H

