/**
 * @file goal_minimize_change.h
 * @brief Constraint to pushes joints back towards their starting position
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
#ifndef GOAL_MINIMIZE_CHANGE_H
#define GOAL_MINIMIZE_CHANGE_H

#include "constrained_ik/constraint.h"

namespace constrained_ik
{
namespace constraints
{
/**
 * @class constrained_ik::constraints::GoalMinimizeChange
 * @brief Constraint to pushes joints back towards their starting position
 *
 * @par Examples:
 * All examples are located here @ref goal_minimize_change_example
 */
class GoalMinimizeChange : public Constraint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GoalMinimizeChange();

  /** @brief see base class for documentation*/
  constrained_ik::ConstraintResults evalConstraint(const SolverState &state) const override;

  /** @brief see base class for documentation*/
  void loadParameters(const XmlRpc::XmlRpcValue &constraint_xml) override;

  /**
   * @brief Jacobian is identity becasue all joints are affected
   * @param cdata The constraint specific data.
   * @return Identity scaled by weight_
   */
  virtual Eigen::MatrixXd calcJacobian(const ConstraintData &cdata) const;

  /**
   * @brief Joint velocity is difference between starting position and current position
   * @param cdata The constraint specific data.
   * @return Joint difference scaled by weight_
   */
  virtual Eigen::VectorXd calcError(const ConstraintData &cdata) const;

  /**
   * @brief Termination criteria for singularity constraint
   * @param cdata The constraint specific data.
   * @return True always (no termination criteria)
   */
  virtual bool checkStatus(const ConstraintData &cdata) const { return true;}

  /**
   * @brief Getter for weight_
   * @return weight_
   */
  virtual double getWeight() const {return weight_;}

  /**
   * @brief setter for weight_
   * @param weight Value to set weight_ to
   */
  virtual void setWeight(double weight) {weight_ = weight;}

protected:
  double weight_; /**< @brief weights used to scale the jocabian and error */

}; // class GoalMinimizeChange

} // namespace constraints
} // namespace constrained_ik


#endif // GOAL_MINIMIZE_CHANGE_H

