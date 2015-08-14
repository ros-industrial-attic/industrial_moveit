/**
* @file goal_position.h
* @brief Constraint to specify cartesian goal position (XYZ)
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
#ifndef GOAL_POSITION_H
#define GOAL_POSITION_H

#include "constrained_ik/constraint.h"
#include <constrained_ik/solver_state.h>

namespace constrained_ik
{
namespace constraints
{

/** @brief Constraint to specify cartesian goal position (XYZ) */
class GoalPosition : public Constraint
{
public:
  struct GoalPositionData: public ConstraintData
  {
    double pos_err_;      /**< @brief current solution error */

    GoalPositionData(const constrained_ik::SolverState &state);
  };

  GoalPosition();
  virtual ~GoalPosition() {}

  virtual constrained_ik::ConstraintResults evalConstraint(const SolverState &state) const;

  /**
   * @brief Calculates distance between two frames
   * @param p1 Current frame
   * @param p2 Goal frame
   * @return Cartesian distance between p1 and p2
   */
  static double calcDistance(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2);

  /**
   * @brief Jacobian is first three rows of standard jacobian
   * (expressed in base frame). Equivalent to each axis of rotation crossed with vector from joint to chain tip.
   * Each row is scaled by the corresponding element of weight_
   * @param cdata, The constraint specific data.
   * @return First 3 rows of standard jacobian scaled by weight_
   */
  virtual Eigen::MatrixXd calcJacobian(const GoalPositionData &cdata) const;

  /**
   * @brief Direction vector from current position to goal position
   * Expressed in base coordinate system
   * Each element is multiplied by corresponding element in weight_
   * @param cdata, The constraint specific data.
   * @return Translation from current to goal scaled by weight_
   */
  virtual Eigen::VectorXd calcError(const GoalPositionData &cdata) const;

  /**
   * @brief Checks termination criteria
   * Termination criteria for this constraint is that positional error is below threshold
   * @param cdata, The constraint specific data.
   * @return True if positional error is below threshold
   */
  virtual bool checkStatus(const GoalPositionData &cdata) const;

  /**
   * @brief Getter for weight_
   * @return weight_
   */
  Eigen::Vector3d getWeight() {return weight_;}

  /**
   * @brief Setter for weight_
   * @param weight Value to assign to weight_
   */
  void setWeight(const Eigen::Vector3d &weight) {weight_ = weight;}

protected:
  double pos_err_tol_;  /**< @brief termination criteria */
  Eigen::Vector3d weight_;
}; // class GoalPosition

} // namespace constraints
} // namespace constrained_ik


#endif // GOAL_POSITION_H

