/**
* @file goal_tool_pointing.h
* @brief Constraint to specify cartesian tool goal pointing (XYZRP)
* @author Levi Armstrong
* @date September 18, 2015
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
#ifndef GOAL_TOOL_POINTING_H
#define GOAL_TOOL_POINTING_H

#include "constrained_ik/constraint.h"
#include <constrained_ik/solver_state.h>

namespace constrained_ik
{
namespace constraints
{

/** @brief Constraint to specify cartesian goal position (XYZ) */
class GoalToolPointing : public Constraint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  struct GoalToolPointingData: public ConstraintData
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double pos_err_;      /**< @brief current solution positional error */
    double rot_err_;      /**< @brief current solution rotational error */

    GoalToolPointingData(const constrained_ik::SolverState &state);
  };

  GoalToolPointing();
  virtual ~GoalToolPointing() {}

  virtual constrained_ik::ConstraintResults evalConstraint(const SolverState &state) const;

  /**
   * @brief Jacobian is first five rows of tool jacobian
   * (expressed in tool frame). Equivalent to each axis of rotation crossed with vector from joint to chain tip.
   * Each row is scaled by the corresponding element of weight_
   * @param cdata, The constraint specific data.
   * @return First 5 rows of tool jacobian scaled by weight_
   */
  virtual Eigen::MatrixXd calcJacobian(const GoalToolPointingData &cdata) const;

  /**
   * @brief Direction vector from current position to goal position
   * Expressed in tool coordinate system
   * Each element is multiplied by corresponding element in weight_
   * @param cdata, The constraint specific data.
   * @return Firt 5 rows from current to goal scaled by weight_
   */
  virtual Eigen::VectorXd calcError(const GoalToolPointingData &cdata) const;

  /**
   * @brief Checks termination criteria
   * Termination criteria for this constraint is that positional and rotational error is below threshold
   * @param cdata, The constraint specific data.
   * @return True if positional and rotational error is below threshold
   */
  virtual bool checkStatus(const GoalToolPointingData &cdata) const;

  /**
   * @brief Load constraint parameters from XmlRpc::XmlRpcValue
   * @param constraint_xml XmlRpc::XmlRpcValue
   */
  virtual void loadParameters(const XmlRpc::XmlRpcValue &constraint_xml);

  /**
   * @brief Getter for weight_
   * @return weight_
   */
  Eigen::Matrix<double, 5, 1> getWeight() {return weight_.diagonal();}

  /**
   * @brief Setter for weight_
   * @param weight Value to assign to weight_
   */
  void setWeight(const Eigen::Matrix<double, 5, 1> &weight) {weight_ = weight.asDiagonal();}

protected:
  double pos_err_tol_;  /**< @brief termination criteria */
  double rot_err_tol_;  /**< @brief termination criteria */
  Eigen::Matrix<double, 5, 5> weight_;

}; // class GoalToolPointing

} // namespace constraints
} // namespace constrained_ik

#endif // GOAL_TOOL_POINTING_H

