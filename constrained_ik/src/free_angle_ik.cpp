/*
 * free_angle_ik.cpp
 *
 *  Created on: Sep 22, 2013
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

#include <constrained_ik/free_angle_ik.h>
#include <ros/ros.h>

namespace constrained_ik
{

namespace free_angle_ik
{

using Eigen::VectorXd;
using Eigen::MatrixXd;

FreeAngleIK::FreeAngleIK(): jla_ik::JLA_IK()
{
    weight_position_ = 10.;
    weight_rotx_ = weight_roty_ = 8.;
    weight_rotz_ = .05;

    pos_err_tol_ = .005;
    rot_err_tol_ = .09;
    debug_ = true;
}

VectorXd FreeAngleIK::calcConstraintError()
{
    VectorXd err(6);
    err = Basic_IK::calcConstraintError();
    err.head(3) *= weight_position_;
    err(3) *= weight_rotx_;
    err(4) *= weight_roty_;
    err(5) *= weight_rotz_;
    return err;
}

MatrixXd FreeAngleIK::calcConstraintJacobian()
{
    MatrixXd J;
    J = Basic_IK::calcConstraintJacobian();
    J.topRows(3) *= weight_position_;
    J.row(3) *= weight_rotx_;
    J.row(4) *= weight_roty_;
    J.row(5) *= weight_rotz_;
    return J;
}

bool FreeAngleIK::checkStatus() const
{
    // check to see if we've reached the goal pose
    if ( (pos_err_ < pos_err_tol_) && (rot_err_ < rot_err_tol_) )
    {
        ROS_DEBUG_STREAM("Basic_IK solution found: pos_err " << pos_err_ << ", rot_err " << rot_err_);
        return true;
    }

    // check maximum iterations
    if (iter_ > max_iter_)
      throw std::runtime_error("Maximum iterations reached.  IK solution may be invalid.");

    // check for joint convergence
    //   - this is an error: joints stabilize, but goal pose not reached
    if (joints_delta_.cwiseAbs().maxCoeff() < joint_convergence_tol_)
    {
        ROS_ERROR_STREAM("Reached " << iter_ << "/" << max_iter_ << " iterations before convergence.");
        ROS_ERROR_STREAM("Position/Rotation Error " << pos_err_tol_/pos_err_*100 << "/" << rot_err_tol_/rot_err_*100 << " %");
//        throw std::runtime_error("Iteration converged before goal reached.  IK solution may be invalid");
        return true;
    }

    return false;
}

} /* namespace jla_ik */
} /* namespace constrained_ik */
