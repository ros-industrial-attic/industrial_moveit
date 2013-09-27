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

  virtual Eigen::MatrixXd calcJacobian();
  virtual Eigen::VectorXd calcError();

  virtual void reset();
  virtual void update(const SolverState &state);
  virtual bool checkStatus() const;

  static double calcDistance(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2);

protected:
  double pos_err_tol_;  // termination criteria
  double pos_err_;      // current solution error
}; // class GoalPosition

} // namespace constraints
} // namespace constrained_ik


#endif // GOAL_POSITION_H

