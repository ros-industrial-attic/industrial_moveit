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
#ifndef JOINT_VEL_LIMITS_H_
#define JOINT_VEL_LIMITS_H_

#include "constrained_ik/constraint.h"
#include <vector>

namespace constrained_ik
{
namespace constraints
{

/** @brief Constraint to avoid joint velocity limits */
class JointVelLimits: public Constraint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  struct JointVelLimitsData: public ConstraintData
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    const constraints::JointVelLimits* parent_;
    std::vector<int> limited_joints_;  /**< @brief list of joints that will be constrained */
    Eigen::VectorXd jvel_;

    JointVelLimitsData(const constrained_ik::SolverState &state, const constraints::JointVelLimits* parent);
    virtual ~JointVelLimitsData() {}

  };

  JointVelLimits();
  virtual ~JointVelLimits() {}

  virtual constrained_ik::ConstraintResults evalConstraint(const SolverState &state) const;

  /**
   * @brief Creates jacobian rows corresponding to joint velocity limit avoidance
   * Each limited joint gets a 0 row with a 1 in that joint's column
   * @param cdata, The constraint specific data.
   * @return Pseudo-Identity scaled by weight_
   */
  virtual Eigen::MatrixXd calcJacobian(const JointVelLimitsData &cdata) const;

  /**
   * @brief Creates vector representing velocity error term corresponding to calcJacobian()
   * Velocity error is difference between current velocity and velocity limit
   * (only applicable for joints outside velocity limits)
   * @param cdata, The constraint specific data.
   * @return VectorXd of joint velocities for joint velocity limit avoidance
   */
  virtual Eigen::VectorXd calcError(const JointVelLimitsData &cdata) const;

  /**
   * @brief Checks termination criteria
   * This constraint is satisfied if no joints are beyond velocity limits
   * @param cdata, The constraint specific data.
   * @return True if no joints violating veloity limit
   */
  virtual bool checkStatus(const JointVelLimitsData &cdata) const { return cdata.limited_joints_.size() == 0; }

  /**
   * @brief Initialize constraint (overrides Constraint::init)
   * Initializes internal velocity limit variable
   * Should be called before using class.
   * @param ik Pointer to Constrained_IK used for base-class init
   */
  virtual void init(const Constrained_IK* ik);

  /**
   * @brief Load constraint parameters from XmlRpc::XmlRpcValue
   * @param constraint_xml XmlRpc::XmlRpcValue
   */
  virtual void loadParameters(const XmlRpc::XmlRpcValue &constraint_xml);

  /**
   * @brief getter for weight_
   * @return weight_
   */
  double getWeight() {return weight_;}

  /**
   * @brief setter for weight_
   * @param weight Value to set weight_ to
   */
  void setWeight(const double &weight) {weight_ = weight;}

protected:
  Eigen::VectorXd vel_limits_;    /**< @brief joint velocity */
  double weight_, timestep_;

};

} /* namespace constraints */
} /* namespace constrained_ik */
#endif /* JOINT_VEL_LIMITS_H_ */
