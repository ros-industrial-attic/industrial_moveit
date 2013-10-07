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

#include "constrained_ik/constrained_ik.h"
#include "constrained_ik/constraints/avoid_singularities.h"
#include "ros/ros.h"

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;

// initialize limits/tolerances to default values
AvoidSingularities::AvoidSingularities() : Constraint(),
                                            weight_(1.0),
                                            enable_threshold_(.01),
                                            ignore_threshold_(1e-5),
                                            avoidance_enabled_(false)
{
}

Eigen::VectorXd AvoidSingularities::calcError()
{
//    ROS_INFO_STREAM("Starting calcError"); //TODO delete
    size_t n(avoidance_enabled_? numJoints():0);    // number of columns = joints
    VectorXd err(n);
    if (avoidance_enabled_)
    {
        for (size_t jntIdx=0; jntIdx<n; ++jntIdx)
        {
            err(jntIdx) = (Ui_.transpose() * jacobianPartialDerivative(jntIdx) * Vi_)(0);
         }
        err *= weight_;
    }
//    ROS_INFO_STREAM("Finished calcError"); //TODO delete
    return err;
}

Eigen::MatrixXd AvoidSingularities::calcJacobian()
{
//    ROS_INFO_STREAM("Starting calcJacobian"); //TODO delete
    size_t n(avoidance_enabled_? numJoints():0);    // number of columns = joints
    MatrixXd  J = MatrixXd::Identity(n,n);
    J *= weight_;
//    ROS_INFO_STREAM("Finished calcJacobian"); //TODO delete
    return J;
}

Eigen::MatrixXd AvoidSingularities::jacobianPartialDerivative(size_t jntIdx, double eps)
{
//    ROS_INFO_STREAM("Starting jpd"); //TODO delete
    MatrixXd jacobian_increment;
    Eigen::VectorXd joints = state_.joints;
    joints(jntIdx) += eps;

    //TODO fix this by overriding calcJacobian error?
    if (!ik_->getKin().calcJacobian(joints, jacobian_increment))
    {
        eps = -eps;
        joints(jntIdx) += 2*eps;
        ik_->getKin().calcJacobian(joints, jacobian_increment);
    }
//    ROS_INFO_STREAM("Finished jpd"); //TODO delete
    return (jacobian_increment-jacobian_orig_)/eps;
}

void AvoidSingularities::update(const SolverState &state)
{
    Constraint::update(state);
    size_t n = numJoints();

    ik_->getKin().calcJacobian(state_.joints, jacobian_orig_);
    Eigen::JacobiSVD<MatrixXd> svd(jacobian_orig_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Ui_ = svd.matrixU().col(n-1);
    Vi_ = svd.matrixV().col(n-1);
    double smallest_sv = svd.singularValues().tail(1)(0);
    avoidance_enabled_ =  smallest_sv < enable_threshold_ && smallest_sv > ignore_threshold_;
    if (avoidance_enabled_)
    {
        ROS_INFO_STREAM("Sing. avoidance with s=" << svd.singularValues().tail(1));
    }
    ROS_INFO_STREAM(smallest_sv);
//    ROS_INFO_STREAM("Finished update"); //TODO delete
}

} // namespace constraints
} // namespace constrained_ik

