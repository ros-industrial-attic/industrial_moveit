/**
 * @file joint_vel_limits.h
 * @brief Constraint to avoid joint velocity limits.
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
#ifndef JOINT_VEL_LIMITS_H_
#define JOINT_VEL_LIMITS_H_

#include "constrained_ik/constraint.h"
#include <vector>

namespace constrained_ik
{
namespace constraints
{
/**
 * @class constrained_ik::constraints::JointVelLimits
 * @brief Constraint to avoid joint velocity limits
 * @todo Need to make the joint velocity limit configurable
 * @todo This class should be renamed to AvoidJointVelLimits
 *
 * @par Examples:
 * All examples are located here @ref avoid_joint_vel_limits_example
 */
class JointVelLimits: public Constraint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** @brief This structure stores constraint data */
  struct JointVelLimitsData: public ConstraintData
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    const constraints::JointVelLimits* parent_; /**< pointer to parent class AvoidObstacles */
    std::vector<int> limited_joints_;  /**< @brief list of joints that will be constrained */
    Eigen::VectorXd jvel_; /**< @brief calculated joint velocities */

    /** @brief See base class for documentation */
    JointVelLimitsData(const constrained_ik::SolverState &state, const constraints::JointVelLimits* parent);
  };

  JointVelLimits();

  /**
   * @brief Initialize constraint (overrides Constraint::init)
   * Initializes internal velocity limit variable
   * Should be called before using class.
   * @param ik Pointer to Constrained_IK used for base-class init
   */
  void init(const Constrained_IK* ik) override;

  /** @brief See base class for documentation */
  constrained_ik::ConstraintResults evalConstraint(const SolverState &state) const override;

  /** @brief See base class for documentation */
  void loadParameters(const XmlRpc::XmlRpcValue &constraint_xml) override;

  /**
   * @brief Creates jacobian rows corresponding to joint velocity limit avoidance
   * Each limited joint gets a 0 row with a 1 in that joint's column
   * @param cdata The constraint specific data.
   * @return Pseudo-Identity scaled by weight_
   */
  virtual Eigen::MatrixXd calcJacobian(const JointVelLimitsData &cdata) const;

  /**
   * @brief Creates vector representing velocity error term corresponding to calcJacobian()
   * Velocity error is difference between current velocity and velocity limit
   * (only applicable for joints outside velocity limits)
   * @param cdata The constraint specific data.
   * @return VectorXd of joint velocities for joint velocity limit avoidance
   */
  virtual Eigen::VectorXd calcError(const JointVelLimitsData &cdata) const;

  /**
   * @brief Checks termination criteria
   * This constraint is satisfied if no joints are beyond velocity limits
   * @param cdata The constraint specific data.
   * @return True if no joints violating veloity limit
   */
  virtual bool checkStatus(const JointVelLimitsData &cdata) const { return cdata.limited_joints_.size() == 0; }

  /**
   * @brief getter for weight_
   * @return weight_
   */
  virtual double getWeight() const {return weight_;}

  /**
   * @brief setter for weight_
   * @param weight Value to set weight_ to
   */
  virtual void setWeight(const double &weight) {weight_ = weight;}

  /**
   * @brief getter for timestep_
   * @return timestep_
   */
  virtual double getTimestep() const {return timestep_;}

  /**
   * @brief setter for timestep_
   * @param timestep Value to set timestep_ to
   */
  virtual void setTimestep(const double &timestep) {timestep_ = timestep;}

protected:
  Eigen::VectorXd vel_limits_; /**< @brief joint velocity limits */
  double weight_;  /**< @brief weights used to scale the jocabian and error */
  double timestep_; /**< @brief timestep used to calculate joint velocities */
};

} /* namespace constraints */
} /* namespace constrained_ik */
#endif /* JOINT_VEL_LIMITS_H_ */
