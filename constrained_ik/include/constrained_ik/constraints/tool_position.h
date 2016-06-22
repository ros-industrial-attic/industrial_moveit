/**
* @file tool_position.h
* @brief Constraint to specify cartesian goal position in tool frame (XYZ rotation).
*
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
#ifndef GOAL_TOOL_POSITION_H
#define GOAL_TOOL_POSITION_H

#include "constrained_ik/constraint.h"
#include "constrained_ik/constraints/goal_position.h"

namespace constrained_ik
{
namespace constraints
{

/** @brief Constraint to specify cartesian goal position in tool frame (XYZ rotation) */
class ToolPosition : public GoalPosition
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ToolPosition();
  virtual ~ToolPosition() {}

  virtual constrained_ik::ConstraintResults evalConstraint(const SolverState &state) const;

  /**
   * @brief Jacobian is the first three rows of standard jacobian
   * expressed in tool frame coordinates.
   * Each row is scaled by the corresponding element of weight_
   * @param cdata, The constraint specific data.
   * @return First 3 rows of standard jacobian expressed in tool frame, scaled by weight_
   */
  virtual Eigen::MatrixXd calcJacobian(const GoalPositionData &cdata) const;

  /**
   * @brief Vector to get from current position to goal position
   * Resolve into primary vectors (x,y,z) of tool coordinate system
   * Each element is multiplied by corresponding element in weight_
   * @param cdata, The constraint specific data.
   * @return Vector from current to goal expressed in tool frame, scaled by weight_
   */
  virtual Eigen::VectorXd calcError(const GoalPositionData &cdata) const;

}; // class ToolPosition

} // namespace constraints
} // namespace constrained_ik


#endif // GOAL_TOOL_ORIENTATION_H

