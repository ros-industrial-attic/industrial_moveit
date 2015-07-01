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
#ifndef GOAL_AVOID_SINGULARITIES_H
#define GOAL_AVOID_SINGULARITIES_H

#include "constrained_ik/constraint.h"

namespace constrained_ik
{
namespace constraints
{

/**
 * @brief Constraint to increases dexterity when manipulator is close to singularity
 * Joint velocity is determined by gradient of smallest singular value
 * Constraint is only active when smallest SV is below theshold
 */
class AvoidSingularities: public Constraint
{
public:
  struct AvoidSingularitiesData: public ConstraintData
  {
    const constraints::AvoidSingularities* parent_;
    bool avoidance_enabled_;
    double smallest_sv_;
    Eigen::VectorXd Ui_, Vi_;
    Eigen::MatrixXd jacobian_orig_;   // current jacobian

    AvoidSingularitiesData(const constrained_ik::SolverState &state, const constraints::AvoidSingularities* parent);
    virtual ~AvoidSingularitiesData() {}

  };

    AvoidSingularities();
  virtual ~AvoidSingularities() {}

  virtual constrained_ik::ConstraintResults evalConstraint(const SolverState &state) const;

  /**
   * @brief Jacobian for this constraint is identity (all joints may contribute)
   * @return Identity jacobian scaled by weight
   */
  virtual Eigen::MatrixXd calcJacobian(const AvoidSingularitiesData &cdata) const;

  /**
   * @brief Velocity is gradient of smallest singular value
   * del(sv) = uT * del(J) * v
   * @return Joint velocity error scaled by weight
   */
  virtual Eigen::VectorXd calcError(const AvoidSingularitiesData &cdata) const;

  /**
   * @brief Termination criteria for singularity constraint
   * @return True always (no termination criteria)
   */
  virtual bool checkStatus(const AvoidSingularitiesData &cdata) const { return true;} //always return true

  /**
   * @brief Getter for weight_
   * @return weight_
   */
  double getWeight() {return weight_;}

  /**
   * @brief Setter for weight_
   * @param weight Value to assign to weight_
   */
  void setWeight(double weight) {weight_ = weight;}

protected:
  double weight_;
  double enable_threshold_, ignore_threshold_; /**< @brief how small singular value must be to trigger avoidance, how small is too small */

  Eigen::MatrixXd jacobianPartialDerivative(const AvoidSingularitiesData &cdata, size_t jntIdx, double eps=1e-6) const;

}; // class AvoidSingularities

} // namespace constraints
} // namespace constrained_ik


#endif // GOAL_AVOID_SINGULARITIES_H

