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

class AvoidJointLimits: public Constraint
{
public:
    AvoidJointLimits();
    virtual ~AvoidJointLimits() {};

    virtual Eigen::MatrixXd calcJacobian();
    virtual Eigen::VectorXd calcError();

    virtual bool checkStatus() const { return true; }  // always satisfied, even if "close to limits"
    virtual void init(const Constrained_IK* ik);
    virtual void reset();
    virtual void update(const SolverState &state);

    static double cubicVelRamp(double angle, double max_angle, double max_vel, double min_angle, double min_vel);

protected:

    struct LimitsT
    {
      double min_pos;       // minimum joint position
      double max_pos;       // maximum joint position
      double mid_pos;       // joint position at middle-of-range
      double lower_thresh;  // lower threshold at which limiting begins
      double upper_thresh;  // upper threshold at which limiting begins
      double max_vel;       // maximum allowable velocity (??)

      LimitsT(double minPos, double maxPos, double threshold);
    };

    std::vector<LimitsT> limits_;
    std::vector<int> limited_joints_;  // list of joints that will be constrained
    double weight_;
    double threshold_;   // threshold (% of range) at which to engage limit avoidance

    //TODO document
    bool nearLowerLimit(size_t idx);
    bool nearUpperLimit(size_t idx);

};

} /* namespace constraints */
} /* namespace constrained_ik */
#endif /* AVOID_JOINT_LIMITS_H_ */
