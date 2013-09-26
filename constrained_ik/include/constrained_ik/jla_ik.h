/*
 * jla_ik.h
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

#ifndef JLA_IK_H_
#define JLA_IK_H_

#include "basic_ik.h"

namespace constrained_ik
{

namespace jla_ik
{

class JLA_IK: public basic_ik::Basic_IK
{
public:
    JLA_IK();
    virtual ~JLA_IK() {};

    void calcInvKin(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed, Eigen::VectorXd &joint_angles);

    bool checkStatus() const;

    void init(const basic_kin::BasicKin &kin);

    double jla_VelCubic(double angle,
                        const double &max_angle, const double &max_vel,
                        const double &min_angle, const double &min_vel) const;

protected:

    Eigen::MatrixXd joint_limits_;  // joint limits
    Eigen::VectorXd joint_ranges_;  // range of joint movement
    Eigen::VectorXd joint_mid_;     // middle of joint
    Eigen::VectorXd jl_threshold_;  // threshold (% of range) at which to engage limit avoidance
    Eigen::VectorXd threshold_lower_, threshold_upper_, max_vel_;
    double weight_jla_;             // max velocity as a percentage of joint range

    //TODO document
    void augmentJErr(Eigen::MatrixXd &J, Eigen::VectorXd &err);

    void augmentJErr2(Eigen::MatrixXd &J);

    void augmentJErr3(Eigen::MatrixXd &J, Eigen::VectorXd &err);

    virtual void update(const Eigen::VectorXd &joints);

};

} /* namespace jla_ik */
} /* namespace constrained_ik */
#endif /* JLA_IK_H_ */
