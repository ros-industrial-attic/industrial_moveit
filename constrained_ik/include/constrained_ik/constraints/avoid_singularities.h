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


#ifndef GOAL_AVOID_SINGULARITIES_H
#define GOAL_AVOID_SINGULARITIES_H

#include "constrained_ik/constraint.h"

namespace constrained_ik
{
namespace constraints
{

class AvoidSingularities: public Constraint
{
public:
    AvoidSingularities();
  virtual ~AvoidSingularities() {};

  virtual Eigen::MatrixXd calcJacobian();
  virtual Eigen::VectorXd calcError();

  double getWeight() {return weight_;}

  void setWeight(double weight) {weight_ = weight;};

  virtual void update(const SolverState &state);

protected:
  double weight_;
  double enable_threshold_, ignore_threshold_; // how small singular value must be to trigger avoidance, how small is too small
  bool avoidance_enabled_;
  Eigen::VectorXd Ui_, Vi_;
  Eigen::MatrixXd jacobian_orig_;   // current jacobian

  Eigen::MatrixXd jacobianPartialDerivative(size_t jntIdx, double eps=1e-6);

}; // class AvoidSingularities

} // namespace constraints
} // namespace constrained_ik


#endif // GOAL_AVOID_SINGULARITIES_H

