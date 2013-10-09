/*
 * avoid_joint_limits.h
 *
 *  Created on: Sep 23, 2013
 *      Author: dsolomon
 */
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

#ifndef AVOID_JOINT_LIMITS_H_
#define AVOID_JOINT_LIMITS_H_

#include "constrained_ik/constraint.h"
#include <vector>

namespace constrained_ik
{
namespace constraints
{

/**@brief Constraint class to avoid joint position limits
 * Using cubic velocity ramp, it pushes each joint away from its limits,
 * with a maximimum velocity of 2*threshold.
 * Only affects joints that are within theshold of joint limit.
 */
class AvoidJointLimits: public Constraint
{
public:
    AvoidJointLimits();
    virtual ~AvoidJointLimits() {};

    /**@brief Creates jacobian rows corresponding to joint limit avoidance
     * @return Identity(n) scaled by weight_
     */
    virtual Eigen::MatrixXd calcJacobian();

    /**@brief Creates vector representing velocity error term
     * corresponding to calcJacobian()
     * @return VectorXd of joint velocities for joint limit avoidance
     */
    virtual Eigen::VectorXd calcError();

    /**@brief Checks termination criteria
     * There are no termination criteria for this constraint
     * @return True
     */
    virtual bool checkStatus() const { return true; }  // always satisfied, even if "close to limits"

    /**@brief Initialize constraint (overrides Constraint::init)
     * Initializes internal limit variables
     * Should be called before using class.
     * @param ik Pointer to Constrained_IK used for base-class init
     */
    virtual void init(const Constrained_IK* ik);

    /**@brief Resets constraint before new use.
     * Call this method before beginning a new IK calculation
     */
    virtual void reset();

    /**@brief Update internal state of constraint (overrides constraint::update)
     * Sets which joints are near limits
     * @param state SolverState holding current state of IK solver
     */
    virtual void update(const SolverState &state);

    /**@brief getter for weight_
     * @return weight_
     */
    double getWeight() {return weight_;};

    /**@brief setter for weight_
     * @param weight Value to set weight_ to
     */
    void setWeight(const double &weight) {weight_ = weight;};

protected:

    struct LimitsT
    {
      double min_pos;       // minimum joint position
      double max_pos;       // maximum joint position
      double range;
      double mid_pos;       // joint position at middle-of-range
      double lower_thresh;  // lower threshold at which limiting begins
      double upper_thresh;  // upper threshold at which limiting begins
//      double max_vel;       // maximum velocity for joint limit update step
//      double min_vel;       // value used for cubic velocity ramp
      double k3;            // factor used in cubic velocity ramp

      LimitsT(double minPos, double maxPos, double threshold);

      /**@brief Calculates velocity for joint position avoidance
       * Uses cubic function y-y0 = k(x-x0)^3 where k=max_vel/(joint_range/2)^3
       * @param angle Angle to calculate velocity for
       * @return joint velocity to avoid joint limits
       */
      double cubicVelRamp(double angle) const;
    };

    std::vector<LimitsT> limits_;
    std::vector<int> limited_joints_;  // list of joints that will be constrained
    double weight_;
    double threshold_;   // threshold (% of range) at which to engage limit avoidance

    /**@brief Check if a given joint is near its lower limit
     * @param idx Index of joint
     * @return True if joint position is within threshold of lower limit
     */
    bool nearLowerLimit(size_t idx);

    /**@brief Check if a given joint is near its upper limit
     * @param idx Index of joint
     * @return True if joint position is within threshold of upper limit
     */
    bool nearUpperLimit(size_t idx);

};

} /* namespace constraints */
} /* namespace constrained_ik */
#endif /* AVOID_JOINT_LIMITS_H_ */
