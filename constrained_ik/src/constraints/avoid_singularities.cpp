/**
 * @file avoid_singularities.cpp
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
                                            smallest_sv_(0.0),
                                            avoidance_enabled_(false)
{
}

Eigen::VectorXd AvoidSingularities::calcError()
{
    size_t n(avoidance_enabled_? numJoints():0);    // number of columns = joints
    VectorXd err(n);
    if (avoidance_enabled_)
    {
        for (size_t jntIdx=0; jntIdx<n; ++jntIdx)
        {
            err(jntIdx) = (Ui_.transpose() * jacobianPartialDerivative(jntIdx) * Vi_)(0);
        }
        err *= (weight_*smallest_sv_);
        err = err.cwiseMax(VectorXd::Constant(n,.25));
    }
    return err;
}

Eigen::MatrixXd AvoidSingularities::calcJacobian()
{
    size_t n(avoidance_enabled_? numJoints():0);    // number of columns = joints
    MatrixXd  J = MatrixXd::Identity(n,n);
    J *= (weight_*smallest_sv_);
    return J;
}

Eigen::MatrixXd AvoidSingularities::jacobianPartialDerivative(size_t jntIdx, double eps)
{
    MatrixXd jacobian_increment;
    Eigen::VectorXd joints = state_.joints;
    joints(jntIdx) += eps;

    if (!ik_->getKin().checkJoints(joints))
    {
        eps = -eps;
        joints(jntIdx) += 2*eps;
    }
    if (!ik_->getKin().calcJacobian(joints, jacobian_increment))
        ROS_WARN("Could not calculate jacobian in AvoidSingularities");
    return (jacobian_increment-jacobian_orig_)/eps;
}

void AvoidSingularities::update(const SolverState &state)
{
    Constraint::update(state);
    size_t n = numJoints();

    ik_->getKin().calcJacobian(state_.joints, jacobian_orig_);
    Eigen::JacobiSVD<MatrixXd> svd(jacobian_orig_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Ui_ = svd.matrixU().rightCols(1);
    Vi_ = svd.matrixV().rightCols(1);
    smallest_sv_ = svd.singularValues().tail(1)(0);
    avoidance_enabled_ =  smallest_sv_ < enable_threshold_ && smallest_sv_ > ignore_threshold_;
    if (avoidance_enabled_)
    {
        ROS_INFO_STREAM("Sing. avoidance with s=" << svd.singularValues().tail(1));
    }
//    ROS_INFO_STREAM(smallest_sv_);
}

} // namespace constraints
} // namespace constrained_ik

