/**
 * @file avoid_joint_limits.h
 * @brief Constraint to avoid joint position limits.
 *
 * Using cubic velocity ramp, it pushes each joint away from its limits,
 * with a maximimum velocity of 2*threshold*(joint range).
 * Only affects joints that are within theshold of joint limit.
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
#ifndef AVOID_JOINT_LIMITS_H_
#define AVOID_JOINT_LIMITS_H_

#include "constrained_ik/constraint.h"
#include <vector>

namespace constrained_ik
{
namespace constraints
{

/**
 * @brief Constraint to avoid joint position limits
 *
 * Using cubic velocity ramp, it pushes each joint away from its limits,
 * with a maximimum velocity of 2*threshold*(joint range).
 * Only affects joints that are within theshold of joint limit.
 */
class AvoidJointLimits: public Constraint
{
protected:
  /**
   * @brief Stores joint limit constraint data for a single joint.
   */
  struct LimitsT
  {
    double min_pos;       /**< @brief minimum joint position */
    double max_pos;       /**< @brief maximum joint position */
    double lower_thresh;  /**< @brief lower threshold at which limiting begins */
    double upper_thresh;  /**< @brief upper threshold at which limiting begins */
    double e;             /**< @brief threshold as a distance from limit */
    double k3;            /**< @brief factor used in cubic velocity ramp */

    /**
     * @brief Constructor for LimitsT
     * @param minPos Minimum allowed joint position
     * @param maxPos Maximum allowed joint position
     * @param threshold Limiting threshold given as a percentage of joint range
     */
    LimitsT(double minPos, double maxPos, double threshold);

    /**
     * @brief Calculates velocity for joint position avoidance
     * Uses cubic function v = y-y0 = k(x-x0)^3 where k=max_vel/(joint_range/2)^3
     * @param angle Angle to calculate velocity for
     * @param limit Angle limit
     * @return joint velocity to avoid joint limits
     */
    double cubicVelRamp(double angle, double limit) const;
  };

  std::vector<LimitsT> limits_;
  double weight_;
  double threshold_;   /**< @brief threshold (% of range) at which to engage limit avoidance */

public:
  struct AvoidJointLimitsData: public ConstraintData
  {
    std::vector<int> limited_joints_;  /**< @brief list of joints that will be constrained */
    const constraints::AvoidJointLimits* parent_;
    AvoidJointLimitsData(const constrained_ik::SolverState &state, const constraints::AvoidJointLimits* parent);
    virtual ~AvoidJointLimitsData() {}

    /**
     * @brief Check if a given joint is near its lower limit
     * @param idx Index of joint
     * @return True if joint position is within threshold of lower limit
     */
    bool nearLowerLimit(size_t idx) const;

    /**
     * @brief Check if a given joint is near its upper limit
     * @param idx Index of joint
     * @return True if joint position is within threshold of upper limit
     */
    bool nearUpperLimit(size_t idx);
  };

  AvoidJointLimits(): Constraint(), weight_(1.0), threshold_(0.05) {}
  virtual ~AvoidJointLimits() {}

  virtual constrained_ik::ConstraintResults evalConstraint(const SolverState &state) const;

  /**
   * @brief Creates jacobian rows corresponding to joint velocity limit avoidance
   * Each limited joint gets a 0 row with a 1 in that joint's column
   * @param cdata, The constraint specific data
   * @return Pseudo-Identity scaled by weight_
   */
  Eigen::MatrixXd calcJacobian(const AvoidJointLimitsData &cdata) const;

  /**
   * @brief Creates vector representing velocity error term
   * corresponding to calcJacobian()
   * @param cdata, The constraint specific data.
   * @return VectorXd of joint velocities for joint limit avoidance
   */
  Eigen::VectorXd calcError(const AvoidJointLimitsData &cdata) const;

  /**
   * @brief Checks termination criteria
   * There are no termination criteria for this constraint
   * @param cdata, The constraint specific data.
   * @return True
   */
  bool checkStatus(const AvoidJointLimitsData &cdata) const;

  /**
   * @brief Initialize constraint (overrides Constraint::init)
   * Initializes internal limit variables
   * Should be called before using class.
   * @param ik Pointer to Constrained_IK used for base-class init
   */
  virtual void init(const Constrained_IK* ik);

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
};

} /* namespace constraints */
} /* namespace constrained_ik */
#endif /* AVOID_JOINT_LIMITS_H_ */
