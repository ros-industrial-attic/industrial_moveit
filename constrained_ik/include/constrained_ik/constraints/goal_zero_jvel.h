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


#ifndef GOAL_ZERO_JVEL_H
#define GOAL_ZERO_JVEL_H

#include "constrained_ik/constraint.h"

namespace constrained_ik
{
namespace constraints
{

class GoalZeroJVel: public Constraint
{
public:
  GoalZeroJVel();
  virtual ~GoalZeroJVel() {};

  virtual Eigen::MatrixXd calcJacobian();
  virtual Eigen::VectorXd calcError();
  virtual bool checkStatus() const {return true;};  //always return true

  double getWeight() {return weight_;}

  void setWeight(double weight) {weight_ = weight;};

protected:
  double weight_;

}; // class GoalZeroJVel

} // namespace constraints
} // namespace constrained_ik


#endif // GOAL_ZERO_JVEL_H

