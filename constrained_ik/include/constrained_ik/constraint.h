/**
 * @file constraint.h
 * @brief Base class for IK-solver Constraints
 * Specify relationship between joint velocities and constraint "error"
 * @author dsolomon
 * @date Sep 15, 2013
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
#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "solver_state.h"
#include <boost/scoped_ptr.hpp>
#include <Eigen/Core>
#include <constrained_ik/constraint_results.h>

namespace constrained_ik
{

// forward-declarations, to avoid circular inclusion
class Constrained_IK;

/**
 * @brief Base class for IK-solver Constraints
 * Specify relationship between joint velocities and constraint "error"
 */
class Constraint
{
public:
  struct ConstraintData
  {
    SolverState state_;

    ConstraintData(const constrained_ik::SolverState &state) { state_ = state; }
  };

  Constraint() : initialized_(false), debug_(false), requires_collision_checks_(false) {}
  virtual ~Constraint() {}

  virtual constrained_ik::ConstraintResults evalConstraint(const SolverState &state) const = 0;

  virtual void init(const Constrained_IK* ik) { initialized_=true; ik_ = ik;}

  /**
   * @brief set debug mode
   * @param debug Value to set debug_ to (defaults to true)
   */
  void setDebug(bool debug = true) {debug_= debug;}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool requires_collision_checks() const { return requires_collision_checks_; }

protected:
  bool initialized_;
  bool debug_;
  bool requires_collision_checks_;

  const Constrained_IK* ik_;

  int numJoints() const;
}; // class Constraint


} // namespace constrained_ik


#endif // CONSTRAINT_H

