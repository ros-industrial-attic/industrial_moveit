/**
* @file goal_tool_orientation.h
* @brief Constraint to specify cartesian goal tool orientation (XYZ rotation).
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
#ifndef GOAL_TOOL_ORIENTATION_H
#define GOAL_TOOL_ORIENTATION_H

#include "constrained_ik/constraint.h"
#include "constrained_ik/constraints/goal_orientation.h"

namespace constrained_ik
{
namespace constraints
{

/** @brief Constraint to specify cartesian goal orientation (XYZ rotation) */
class GoalToolOrientation : public GoalOrientation
{
public:
  GoalToolOrientation();
  virtual ~GoalToolOrientation() {}

  /**
   * @brief Jacobian is the last three rows of standard jacobian
   * (in tool frame). Equivalent to each axis of rotation expressed in tool frame coordinates.
   * Each row is scaled by the corresponding element of weight_
   * @return Last 3 rows of standard jacobian expressed in tool frame, scaled by weight_
   */
  virtual Eigen::MatrixXd calcJacobian();

  /**
   * @brief Rotation to get from current orientation to goal orientation
   * Resolve into primary vectors (x,y,z) of tool coordinate system
   * Each element is multiplied by corresponding element in weight_
   * @return Rotation from current to goal expressed in tool frame, scaled by weight_
   */
  virtual Eigen::VectorXd calcError();

}; // class GoalToolOrientation

} // namespace constraints
} // namespace constrained_ik


#endif // GOAL_TOOL_ORIENTATION_H

