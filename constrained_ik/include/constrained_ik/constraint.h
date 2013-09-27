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


#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "solver_state.h"
#include <boost/scoped_ptr.hpp>
#include <Eigen/Core>

namespace constrained_ik
{

// forward-declarations, to avoid circular inclusion
class Constrained_IK;

/**
 * \brief Base class for IK-solver Constraints
 *          - specify relationship between joint velocities and constraint "error"
 */
class Constraint
{
public:
  Constraint() : initialized_(false), debug_(false) {};
  virtual ~Constraint() {};

  virtual Eigen::VectorXd calcError() = 0;
  virtual Eigen::MatrixXd calcJacobian() = 0;

  virtual bool checkStatus() const { return false; }
  virtual void init(const Constrained_IK* ik) { initialized_=true; ik_ = ik; }
  virtual void reset() { };
  virtual void update(const SolverState &state) { state_ = state; }
  virtual void updateError(Eigen::VectorXd &error);
  virtual void updateJacobian(Eigen::MatrixXd &jacobian);

  static void appendError(Eigen::VectorXd &error, const Eigen::VectorXd &addErr);
  static void appendJacobian(Eigen::MatrixXd &jacobian, const Eigen::MatrixXd &addJacobian);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  bool initialized_;
  bool debug_;
  const Constrained_IK* ik_;
  SolverState state_;

  int numJoints();
}; // class Constraint


} // namespace constrained_ik


#endif // CONSTRAINT_H

