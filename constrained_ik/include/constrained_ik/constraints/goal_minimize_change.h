/**
 * @file goal_minimize_change.h
 * @brief Constraint to pushes joints back towards their starting position
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
#ifndef GOAL_MINIMIZE_CHANGE_H
#define GOAL_MINIMIZE_CHANGE_H

#include "constrained_ik/constraint.h"

namespace constrained_ik
{
namespace constraints
{

/** @brief Constraint to pushes joints back towards their starting position */
class GoalMinimizeChange : public Constraint
{
public:
  GoalMinimizeChange();
  virtual ~GoalMinimizeChange() {}

  /**
   * @brief Jacobian is identity becasue all joints are affected
   * @return Identity scaled by weight_
   */
  virtual Eigen::MatrixXd calcJacobian();

  /**
   * @brief Joint velocity is difference between starting position and current position
   * @return Joint difference scaled by weight_
   */
  virtual Eigen::VectorXd calcError();

  /**
   * @brief Termination criteria for singularity constraint
   * @return True always (no termination criteria)
   */
  virtual bool checkStatus() const { return true;}

  /**
   * @brief Getter for weight_
   * @return weight_
   */
  double getWeight() {return weight_;}

  /**
   * @brief setter for weight_
   * @param weight Value to set weight_ to
   */
  void setWeight(double weight) {weight_ = weight;}

protected:
  double weight_;

}; // class GoalMinimizeChange

} // namespace constraints
} // namespace constrained_ik


#endif // GOAL_MINIMIZE_CHANGE_H

