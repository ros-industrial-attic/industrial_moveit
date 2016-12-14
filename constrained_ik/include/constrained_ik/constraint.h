/**
 * @file constraint.h
 * @brief Base class for IK-solver Constraints
 *
 * Specify relationship between joint velocities and constraint "error"
 *
 * @author dsolomon
 * @date Sep 15, 2013
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
#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "solver_state.h"
#include <boost/scoped_ptr.hpp>
#include <Eigen/Core>
#include <constrained_ik/constraint_results.h>
#include <constrained_ik/constrained_ik_utils.h>
#include <XmlRpc.h>

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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief This structure is to be used by all constraints to store specific data
   * that needs to get updated every iteration of the solver.
   */
  struct ConstraintData
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** @brief This is a copy of the current state of the parent solver */
    SolverState state_;

    /**
     * @brief The constructure class which should be called by the inheriting structure.
     * @param state solvers current state.
     */
    ConstraintData(const constrained_ik::SolverState &state) { state_ = state; }
  };

  Constraint() : initialized_(false), debug_(false) {}

  /**
   * @brief Pure definition for calculating constraint error, jacobian & status
   * @param state solvers current state
   * @return ConstraintResults
   */
  virtual constrained_ik::ConstraintResults evalConstraint(const SolverState &state) const = 0;

  /**
   * @brief Initialize constraint and should be called by any inheriting classes
   * @param ik Pointer to Constrained_IK
   */
  virtual void init(const Constrained_IK* ik) { initialized_=true; ik_ = ik;}

  /**
   * @brief Load constraint parameters from XmlRpc::XmlRpcValue
   * @param constraint_xml XmlRpc::XmlRpcValue
   */
  virtual void loadParameters(const XmlRpc::XmlRpcValue &constraint_xml) {}

  /**
   * @brief set debug mode
   * @param debug Value to set debug_ to (defaults to true)
   */
  void setDebug(bool debug = true) {debug_= debug;}

protected:
  bool initialized_;         /**< True if solver is intialized, otherwise false */
  bool debug_;               /**< Provide control over if certain print statements are output */
  const Constrained_IK* ik_; /**< Pointer to parent solver */

  /**
   * @brief Returns the number of joints
   * @return number of joints
   */
  virtual int numJoints() const;

}; // class Constraint


} // namespace constrained_ik


#endif // CONSTRAINT_H

