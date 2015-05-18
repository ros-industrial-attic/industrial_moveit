/**
 * @file goal_orientation.h
 * @brief Constraint to specify Cartesian goal orientation (XYZ rotation)
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
#ifndef GOAL_ORIENTATION_H
#define GOAL_ORIENTATION_H

#include "constrained_ik/constraint.h"

namespace constrained_ik
{
namespace constraints
{

/**  @brief Constraint to specify Cartesian goal orientation (XYZ rotation) */
class GoalOrientation : public Constraint
{
public:
  GoalOrientation();
  virtual ~GoalOrientation() {};

  /**
   * @brief Jacobian is the last three rows of standard jacobian(in base frame).
   * Equivalent to each axis of rotation expressed in base frame coordinates.
   * Each row is scaled by the corresponding element of weight_
   * @return Last 3 rows of standard jacobian scaled by weight_
   */
  virtual Eigen::MatrixXd calcJacobian();

  /**
   * @brief Rotation to get from current orientation to goal orientation
   * Resolve into primary vectors (x,y,z) of base coordinate system
   * Each element is multiplied by corresponding element in weight_
   * @return Rotation from current to goal scaled by weight_
   */
  virtual Eigen::VectorXd calcError();

  /**
   * @brief Shortest angle between current orientation and goal orientation
   * @param p1 Current pose
   * @param p2 Goal pose
   * @return Shortest angle between p1 and p2
   */
  static double calcAngle(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2);

  /**
   * @brief Calculate 3-element rotation vector necessary to rotate pose p1 into pose p2
   * @param p1 Current pose
   * @param p2 Goal pose
   * @return x,y,z rotation from p1 to p2
   */
  static Eigen::Vector3d calcAngleError(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2);

  /**
   * @brief Checks termination criteria
   * Termination criteria for this constraint is that angle error is below threshold
   * @return True if angle error is below threshold
   */
  virtual bool checkStatus() const;

  /**
   * @brief Getter for weight_
   * @return weight_
   */
  Eigen::Vector3d getWeight() {return weight_;};

  /** @brief Resets constraint. Use this before performing new IK request. */
  virtual void reset();

  /**
   * @brief Setter for tolerance (termination criteria)
   * @param tol Value to assign to tol_
   */
  void setTolerance(double tol) {rot_err_tol_ = tol;};  //TODO turn tolerance into Vector3d

  /**
   * @brief Setter for weight_
   * @param weight Value to assign to weight_
   */
  void setWeight(const Eigen::Vector3d &weight) {weight_ = weight;};

  /**
   * @brief Update internal state of constraint (overrides constraint::update)
   * Sets current rotation error
   * @param state SolverState holding current state of IK solver
   */
  virtual void update(const SolverState &state);

protected:
  double rot_err_tol_;  /**< @brief termination criteria */
  double rot_err_;      /**< @brief current solution error */
  Eigen::Vector3d weight_;    /**< @brief weight for each direction */

}; // class GoalOrientation

} // namespace constraints
} // namespace constrained_ik


#endif // GOAL_ORIENTATION_H

