/**
 * @file goal_tool_pointing.h
 * @brief Constraint to specify cartesian tool goal pointing (XYZRP)
 *
 * @author Levi Armstrong
 * @date September 18, 2015
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
#ifndef GOAL_TOOL_POINTING_H
#define GOAL_TOOL_POINTING_H

#include "constrained_ik/constraint.h"
#include <constrained_ik/solver_state.h>

namespace constrained_ik
{
namespace constraints
{
/**
 * @class constrained_ik::constraints::GoalToolPointing
 * @brief Constraint to specify cartesian tool goal pointing (XYZRP)
 *
 * @par Examples:
 * All examples are located here @ref goal_tool_pointing_example
 */
class GoalToolPointing : public Constraint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief This structure stores constraint data */
  struct GoalToolPointingData: public ConstraintData
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double pos_err_;      /**< @brief current solution positional error */
    double rot_err_;      /**< @brief current solution rotational error */

    /** @brief see base class for documentation*/
    GoalToolPointingData(const constrained_ik::SolverState &state);
  };

  GoalToolPointing();

  /** @brief see base class for documentation*/
  constrained_ik::ConstraintResults evalConstraint(const SolverState &state) const override;

  /** @brief see base class for documentation*/
  void loadParameters(const XmlRpc::XmlRpcValue &constraint_xml) override;

  /**
   * @brief Jacobian is first five rows of tool jacobian
   * (expressed in tool frame). Equivalent to each axis of rotation crossed with vector from joint to chain tip.
   * Each row is scaled by the corresponding element of weight_
   * @param cdata The constraint specific data.
   * @return First 5 rows of tool jacobian scaled by weight_
   */
  virtual Eigen::MatrixXd calcJacobian(const GoalToolPointingData &cdata) const;

  /**
   * @brief Direction vector from current position to goal position
   * Expressed in tool coordinate system
   * Each element is multiplied by corresponding element in weight_
   * @param cdata The constraint specific data.
   * @return Firt 5 rows from current to goal scaled by weight_
   */
  virtual Eigen::VectorXd calcError(const GoalToolPointingData &cdata) const;

  /**
   * @brief Checks termination criteria
   * Termination criteria for this constraint is that positional and rotational error is below threshold
   * @param cdata The constraint specific data.
   * @return True if positional and rotational error is below threshold
   */
  virtual bool checkStatus(const GoalToolPointingData &cdata) const;

  /**
   * @brief Getter for weight_
   * @return weight_
   */
  virtual Eigen::Matrix<double, 5, 1> getWeight() const {return weight_.diagonal();}

  /**
   * @brief Setter for weight_
   * @param weight Value to assign to weight_
   */
  virtual void setWeight(const Eigen::Matrix<double, 5, 1> &weight) {weight_ = weight.asDiagonal();}

  /**
   * @brief Getter for pos_err_tol_
   * @return pos_err_tol_
   */
  virtual double getPositionTolerance() const {return pos_err_tol_;}

  /**
   * @brief Setter for pos_err_tol_
   * @param position_tolerance Value to assign to pos_err_tol_
   */
  virtual void setPositionTolerance(const double &position_tolerance) {pos_err_tol_ = position_tolerance;}

  /**
   * @brief Getter for rot_err_tol_
   * @return rot_err_tol_
   */
  virtual double getOrientationTolerance() const {return rot_err_tol_;}

  /**
   * @brief Setter for rot_err_tol_
   * @param orientation_tolerance Value to assign to rot_err_tol_
   */
  virtual void setOrientationTolerance(const double &orientation_tolerance) {rot_err_tol_ = orientation_tolerance;}

protected:
  double pos_err_tol_;  /**< @brief termination criteria */
  double rot_err_tol_;  /**< @brief termination criteria */
  Eigen::Matrix<double, 5, 5> weight_; /**< @brief weights used to scale the jocabian and error */

}; // class GoalToolPointing

} // namespace constraints
} // namespace constrained_ik

#endif // GOAL_TOOL_POINTING_H

