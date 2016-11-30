/**
 * @file avoid_singularities.h
 * @brief Constraint to increases dexterity when manipulator is close to singularity
 *
 * Joint velocity is determined by gradient of smallest singular value
 * Constraint is only active when smallest SV is below theshold
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
#ifndef GOAL_AVOID_SINGULARITIES_H
#define GOAL_AVOID_SINGULARITIES_H

#include "constrained_ik/constraint.h"

namespace constrained_ik
{
namespace constraints
{
/**
 * @class constrained_ik::constraints::AvoidSingularities
 * @brief Constraint to increases dexterity when manipulator is close to singularity
 *
 * Joint velocity is determined by gradient of smallest singular value
 * Constraint is only active when smallest SV is below theshold
 *
 * @par Examples:
 * All examples are located here @ref avoid_singularities_example
 */
class AvoidSingularities: public Constraint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief This structure stores constraint data */
  struct AvoidSingularitiesData: public ConstraintData
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    const constraints::AvoidSingularities* parent_; /**< @brief pointer to parent class object */
    bool avoidance_enabled_; /**< Is avoidance enabled */
    double smallest_sv_; /**< smallest singular value */
    Eigen::VectorXd Ui_; /**< U matrix from SVD of the jacobian */
    Eigen::VectorXd Vi_; /**< V matrix from SVD of the jacobian */
    Eigen::MatrixXd jacobian_orig_; /**< current jacobian */

    /** @brief see base class for documentation*/
    AvoidSingularitiesData(const constrained_ik::SolverState &state, const constraints::AvoidSingularities* parent);
  };

  AvoidSingularities();

  /** @brief see base class for documentation*/
  constrained_ik::ConstraintResults evalConstraint(const SolverState &state) const override;

  /** @brief see base class for documentation*/
  void loadParameters(const XmlRpc::XmlRpcValue &constraint_xml) override;

  /**
   * @brief Jacobian for this constraint is identity (all joints may contribute)
   * @param cdata The constraint specific data.
   * @return Identity jacobian scaled by weight
   */
  virtual Eigen::MatrixXd calcJacobian(const AvoidSingularitiesData &cdata) const;

  /**
   * @brief Velocity is gradient of smallest singular value
   * del(sv) = uT * del(J) * v
   * @param cdata The constraint specific data.
   * @return Joint velocity error scaled by weight
   */
  virtual Eigen::VectorXd calcError(const AvoidSingularitiesData &cdata) const;

  /**
   * @brief Termination criteria for singularity constraint
   * @param cdata The constraint specific data.
   * @return True always (no termination criteria)
   */
  virtual bool checkStatus(const AvoidSingularitiesData &cdata) const { return true;} //always return true

  /**
   * @brief Getter for weight_
   * @return weight_
   */
  virtual double getWeight() const {return weight_;}

  /**
   * @brief Setter for weight_
   * @param weight Value to assign to weight_
   */
  virtual void setWeight(double weight) {weight_ = weight;}

  /**
   * @brief Getter for enable_threshold_
   * @return enable_threshold_
   */
  virtual double getEnableThreshold() const {return enable_threshold_;}

  /**
   * @brief Setter for enable_threshold_
   * @param enable_threshold Value to assign to enable_threshold_
   */
  virtual void setEnableThreshold(double enable_threshold) {enable_threshold_ = enable_threshold;}

  /**
   * @brief Getter for ignore_threshold_
   * @return ignore_threshold_
   */
  virtual double getIgnoreThreshold() const {return ignore_threshold_;}

  /**
   * @brief Setter for ignore_threshold_
   * @param ignore_threshold Value to assign to ignore_threshold_
   */
  virtual void setIgnoreThreshold(double ignore_threshold) {ignore_threshold_ = ignore_threshold;}

protected:
  double weight_; /**< @brief weights used to scale the jocabian and error */
  double enable_threshold_; /**< @brief how small singular value must be to trigger avoidance */
  double ignore_threshold_; /**< @brief how small is too small */

  /**
   * @brief Calculates the partial derivative of the jacobian
   * This will calculate the partial derivative of the jacobian with
   * respect to the provided joint index by method of numerical
   * differentiation.
   * @param cdata Constraint specific data
   * @param jntIdx joint index
   * @param eps value for which to peturb the joint value
   * @return partial derivative of the jacobian relative to a single joint
   */
  Eigen::MatrixXd jacobianPartialDerivative(const AvoidSingularitiesData &cdata, size_t jntIdx, double eps=1e-6) const;

}; // class AvoidSingularities

} // namespace constraints
} // namespace constrained_ik


#endif // GOAL_AVOID_SINGULARITIES_H

