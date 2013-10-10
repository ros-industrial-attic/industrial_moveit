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


#ifndef GOAL_POSITION_H
#define GOAL_POSITION_H

#include "constrained_ik/constraint.h"

namespace constrained_ik
{
namespace constraints
{

/**
 * \brief Constraint to specify cartesian goal position (XYZ)
  */
class GoalPosition : public Constraint
{
public:
  GoalPosition();
  virtual ~GoalPosition() {};

  /**@brief Calculates distance between two frames
   * @param p1 Current frame
   * @param p2 Goal frame
   * @return Cartesian distance between p1 and p2
   */
  static double calcDistance(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2);

  /**@brief Jacobian is first three rows of standard jacobian
   * (expressed in base frame). Equivalent to each axis of rotation crossed with vector from joint to chain tip.
   * Each row is scaled by the corresponding element of weight_
   * @return First 3 rows of standard jacobian scaled by weight_
   */
  virtual Eigen::MatrixXd calcJacobian();

  /**@brief Direction vector from current position to goal position
   * Expressed in base coordinate system
   * Each element is multiplied by corresponding element in weight_
   * @return Translation from current to goal scaled by weight_
   */
  virtual Eigen::VectorXd calcError();

  /**@brief Checks termination criteria
   * Termination criteria for this constraint is that positional error is below threshold
   * @return True if positional error is below threshold
   */
  virtual bool checkStatus() const;

  /**@brief Getter for weight_
   * @return weight_
   */
  Eigen::Vector3d getWeight() {return weight_;}

  /**@brief Resets constraint. Use this before performing new IK request.
   */
  virtual void reset();

  /**@brief Update internal state of constraint (overrides constraint::update)
   * Sets current positional error
   * @param state SolverState holding current state of IK solver
   */
  virtual void update(const SolverState &state);

  /**@brief Setter for weight_
   * @param weight Value to assign to weight_
   */
  void setWeight(const Eigen::Vector3d &weight) {weight_ = weight;};

protected:
  double pos_err_tol_;  // termination criteria
  double pos_err_;      // current solution error
  Eigen::Vector3d weight_;
}; // class GoalPosition

} // namespace constraints
} // namespace constrained_ik


#endif // GOAL_POSITION_H

