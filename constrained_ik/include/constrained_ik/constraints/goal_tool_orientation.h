/**
 * @file goal_tool_orientation.h
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
#ifndef GOAL_TOOL_ORIENTATION_H
#define GOAL_TOOL_ORIENTATION_H

#include "constrained_ik/constraint.h"
#include "constrained_ik/constraints/goal_orientation.h"

namespace constrained_ik
{
namespace constraints
{
/**
 * @class constrained_ik::constraints::GoalToolOrientation
 * @brief Constraint to specify cartesian goal tool orientation (XYZ rotation)
 *
 * @par Examples:
 * All examples are located here @ref goal_tool_orientation_example
 */
class GoalToolOrientation : public GoalOrientation
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GoalToolOrientation();

  /** @brief see base class for documentation*/
  constrained_ik::ConstraintResults evalConstraint(const SolverState &state) const override;

  /**
   * @brief Jacobian is the last three rows of standard jacobian
   * (in tool frame). Equivalent to each axis of rotation expressed in tool frame coordinates.
   * Each row is scaled by the corresponding element of weight_
   * @param cdata The constraint specific data.
   * @return Last 3 rows of standard jacobian expressed in tool frame, scaled by weight_
   */
  Eigen::MatrixXd calcJacobian(const GoalOrientationData &cdata) const override;

  /**
   * @brief Rotation to get from current orientation to goal orientation
   * Resolve into primary vectors (x,y,z) of tool coordinate system
   * Each element is multiplied by corresponding element in weight_
   * @param cdata The constraint specific data.
   * @return Rotation from current to goal expressed in tool frame, scaled by weight_
   */
  Eigen::VectorXd calcError(const GoalOrientationData &cdata) const override;

}; // class GoalToolOrientation

} // namespace constraints
} // namespace constrained_ik


#endif // GOAL_TOOL_ORIENTATION_H

