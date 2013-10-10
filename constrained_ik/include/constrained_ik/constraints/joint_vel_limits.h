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

#ifndef JOINT_VEL_LIMITS_H_
#define JOINT_VEL_LIMITS_H_

#include "constrained_ik/constraint.h"
#include <vector>

namespace constrained_ik
{
namespace constraints
{

/**@brief Constraint to avoid joint velocity limits
 */
class JointVelLimits: public Constraint
{
public:
    JointVelLimits();
    virtual ~JointVelLimits() {};

    /**@brief Creates jacobian rows corresponding to joint velocity limit avoidance
     * Each limited joint gets a 0 row with a 1 in that joint's column
     * @return Pseudo-Identity scaled by weight_
     */
    virtual Eigen::MatrixXd calcJacobian();

    /**@brief Creates vector representing velocity error term corresponding to calcJacobian()
     * Velocity error is difference between current velocity and velocity limit
     * (only applicable for joints outside velocity limits)
     * @return VectorXd of joint velocities for joint velocity limit avoidance
     */
    virtual Eigen::VectorXd calcError();

    /**@brief Checks termination criteria
     * This constraint is satisfied if no joints are beyond velocity limits
     * @return True if no joints violating veloity limit
     */
    virtual bool checkStatus() const { return limited_joints_.size() == 0; }

    /**@brief Initialize constraint (overrides Constraint::init)
     * Initializes internal velocity limit variable
     * Should be called before using class.
     * @param ik Pointer to Constrained_IK used for base-class init
     */
    virtual void init(const Constrained_IK* ik);

    /**@brief Resets constraint before new use.
     * Call this method before beginning a new IK calculation
     */
    virtual void reset();

    /**@brief Update internal state of constraint (overrides constraint::update)
     * Sets which joints are near velocity limits
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
    std::vector<int> limited_joints_;  // list of joints that will be constrained
    Eigen::VectorXd jvel_, vel_limits_;    // joint velocity
    double weight_, timestep_;

};

} /* namespace constraints */
} /* namespace constrained_ik */
#endif /* JOINT_VEL_LIMITS_H_ */
