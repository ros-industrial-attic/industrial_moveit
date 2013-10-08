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

class JointVelLimits: public Constraint
{
public:
    JointVelLimits();
    virtual ~JointVelLimits() {};

    virtual Eigen::MatrixXd calcJacobian();
    virtual Eigen::VectorXd calcError();

    virtual bool checkStatus() const { return limited_joints_.size() == 0; }
    virtual void init(const Constrained_IK* ik);
    virtual void reset();
    virtual void update(const SolverState &state);

    double getWeight() {return weight_;};
    void setWeight(const double &weight) {weight_ = weight;};

protected:
    std::vector<int> limited_joints_;  // list of joints that will be constrained
    Eigen::VectorXd jvel_, vel_limits_;    // joint velocity
    double weight_, timestep_;

};

} /* namespace constraints */
} /* namespace constrained_ik */
#endif /* JOINT_VEL_LIMITS_H_ */
