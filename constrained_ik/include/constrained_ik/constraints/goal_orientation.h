/**
 * @file goal_orientation.h
 * @brief Constraint to specify Cartesian goal orientation (XYZ rotation)
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
#ifndef GOAL_ORIENTATION_H
#define GOAL_ORIENTATION_H

#include "constrained_ik/constraint.h"
#include <constrained_ik/solver_state.h>

namespace constrained_ik
{
namespace constraints
{
/**
 * @class constrained_ik::constraints::GoalOrientation
 * @brief Constraint to specify Cartesian goal orientation (XYZ rotation)
 *
 * @par Examples:
 * All examples are located here @ref goal_orientation_example
 */
class GoalOrientation : public Constraint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief This structure stores constraint data */
  struct GoalOrientationData: public ConstraintData
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double rot_err_;      /**< @brief current solution orientation error */

    /** @brief see base class for documentation*/
    GoalOrientationData(const constrained_ik::SolverState &state);
  };

  GoalOrientation();

  /** @brief see base class for documentation*/
  constrained_ik::ConstraintResults evalConstraint(const SolverState &state) const override;

  /** @brief see base class for documentation*/
  void loadParameters(const XmlRpc::XmlRpcValue &constraint_xml) override;

  /**
   * @brief Jacobian is the last three rows of standard jacobian(in base frame).
   * Equivalent to each axis of rotation expressed in base frame coordinates.
   * Each row is scaled by the corresponding element of weight_
   * @param cdata The constraint specific data.
   * @return Last 3 rows of standard jacobian scaled by weight_
   */
  virtual Eigen::MatrixXd calcJacobian(const GoalOrientationData &cdata) const;
  /**
   * @brief Rotation to get from current orientation to goal orientation
   * Resolve into primary vectors (x,y,z) of base coordinate system
   * Each element is multiplied by corresponding element in weight_
   * @param cdata The constraint specific data.
   * @return Rotation from current to goal scaled by weight_
   */
  virtual Eigen::VectorXd calcError(const GoalOrientationData &cdata) const;

  /**
   * @brief Shortest angle between current orientation and goal orientation
   * @todo This should be moved to the utils class
   * @param p1 Current pose
   * @param p2 Goal pose
   * @return Shortest angle between p1 and p2
   */
  static double calcAngle(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2);

  /**
   * @brief Calculate 3-element rotation vector necessary to rotate pose p1 into pose p2
   * @todo This should be moved to the utils class
   * @param p1 Current pose
   * @param p2 Goal pose
   * @return x,y,z rotation from p1 to p2
   */
  static Eigen::Vector3d calcAngleError(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2);

  /**
   * @brief Checks termination criteria
   * Termination criteria for this constraint is that angle error is below threshold
   * @param cdata The constraint specific data.
   * @return True if angle error is below threshold
   */
  virtual bool checkStatus(const GoalOrientationData &cdata) const;

  /**
   * @brief Getter for weight_
   * @return weight_
   */
  virtual Eigen::Vector3d getWeight() const {return weight_;}

  /**
   * @brief Setter for weight_
   * @param weight Value to assign to weight_
   */
  virtual void setWeight(const Eigen::Vector3d &weight) {weight_ = weight;}

  /**
   * @brief Getter for rot_err_tol_
   * @return rot_err_tol_
   */
  virtual double getTolerance() const {return rot_err_tol_;}

  /**
   * @brief Setter for tolerance (termination criteria)
   * @param tol Value to assign to tol_
   */
  virtual void setTolerance(double tol) {rot_err_tol_ = tol;}  //TODO turn tolerance into Vector3d

protected:
  double rot_err_tol_;  /**< @brief termination criteria */
  Eigen::Vector3d weight_;    /**< @brief weights used to scale the jocabian and error */

}; // class GoalOrientation

} // namespace constraints
} // namespace constrained_ik


#endif // GOAL_ORIENTATION_H

