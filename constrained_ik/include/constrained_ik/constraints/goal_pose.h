/**
 * @file goal_pose.h
 * @brief Constraint to specify cartesian goal pose (XYZ + orientation).
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
#ifndef GOAL_POSE_H
#define GOAL_POSE_H

#include "constrained_ik/constraint.h"
#include "constrained_ik/constraint_group.h"
#include "constrained_ik/constraints/goal_position.h"
#include "constrained_ik/constraints/goal_orientation.h"

namespace constrained_ik
{
namespace constraints
{

 /**
 * @brief Constraint to specify cartesian goal pose (XYZ+orientation)
 * Convenience class, built from goal_position and goal_orientation constraints
 * @todo This should not be a ConstraintGroup because it has to calculate a jacobian twice.
 */
class GoalPose : public ConstraintGroup
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GoalPose() : ConstraintGroup(), position_(new GoalPosition()), orientation_(new GoalOrientation())
  {
    this->add(position_);
    this->add(orientation_);
  }
  /**
   * @brief Setter for the orientation weights
   * @param weight_orientation values to set the orientation weights
   */
  void setWeightOrientation(const Eigen::Vector3d &weight_orientation) {orientation_->setWeight(weight_orientation);}

  /**
   * @brief Setter for the position weights
   * @param weight_position value to set the position weights
   */
  void setWeightPosition(const Eigen::Vector3d &weight_position) {position_->setWeight(weight_position);}
protected:
  GoalPosition* position_; /**< Positional Constraint */
  GoalOrientation* orientation_; /**< Orientation Constraint */

}; // class GoalPose

} // namespace constraints
} // namespace constrained_ik


#endif // GOAL_POSE_H

