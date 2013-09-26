/*
 * jla_ik.cpp
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

#include "constrained_ik/jla_ik.h"
#include <ros/ros.h>

namespace constrained_ik
{

namespace jla_ik
{

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Affine3d;
using basic_ik::Basic_IK;

JLA_IK::JLA_IK(): basic_ik::Basic_IK()
{
    weight_jla_ = 5;
    pos_err_tol_ = .005;
    rot_err_tol_ = .09;
    debug_ = true;
}

void JLA_IK::augmentJErr(MatrixXd &J, VectorXd &err)
{
    const size_t n(kin_.numJoints()), orig_rows(J.rows()), orig_cols(J.cols());
    MatrixXd augJ(n, orig_cols);
    VectorXd augErr(n);
    Eigen::VectorXi limited_joints(n);

    size_t k = 0;   //number of additional rows
    for (size_t ii=0; ii<n; ++ii)
    {
        if (joints_(ii) < threshold_lower_(ii) )
        {
            // lower limit: add positive velocity
            augJ(k, ii) = 1;
            augErr(k)= jla_VelCubic(joints_(ii),
                                    joint_limits_(ii,0), max_vel_(ii),
                                    joint_mid_(ii), 0.0);
            limited_joints(k) = ii;
            ++k;
            continue;
        }

        if (joints_(ii) > threshold_upper_(ii))
        {
            // upper limit: add negative velocity
            augJ(k, ii) = 1;
            augErr(k) = -jla_VelCubic(joints_(ii),
                                      joint_limits_(ii,0), max_vel_(ii),
                                      joint_mid_(ii), 0.0);
            limited_joints(k) = ii;
            ++k;
            continue;
        }
    }
    if (debug_ && k>0)
    {
        Eigen::RowVectorXi limited = limited_joints.head(k);
        ROS_ERROR_STREAM(k << " joint limits active: " << limited);
    }

    //resize J and err and add augmented data
    J.conservativeResize(orig_rows + k, orig_cols);
    err.conservativeResize(orig_rows + k);
    J.block(orig_rows, 0, k, orig_cols) = augJ.block(0, 0, k, orig_cols) * weight_jla_;
    err.tail(k) = augErr.head(k) * weight_jla_;
}

void JLA_IK::augmentJErr2(MatrixXd &J)
{
    const size_t n(kin_.numJoints()), rows = J.rows();
    Eigen::VectorXi limited_joints(n);

    size_t k = 0;   //number of additional rows
    for (size_t ii=0; ii<n; ++ii)
    {
        if (joints_(ii) < threshold_lower_(ii) || joints_(ii) > threshold_upper_(ii))
        {
            J.col(ii) = VectorXd::Zero(rows);
            limited_joints(k) = ii;
            ++k;
        }
    }
}

// continuous jl function (NOT well-behaved)
void JLA_IK::augmentJErr3(MatrixXd &J, VectorXd &err)
{
    const size_t n(kin_.numJoints()), orig_rows(J.rows()), orig_cols(J.cols());
    MatrixXd augJ(n, orig_cols);
    VectorXd augErr(n);
//    Eigen::VectorXi limited_joints(n);

    size_t k = 0;   //number of additional rows
    for (size_t ii=0; ii<n; ++ii)
    {
        if (joints_(ii) < joint_mid_(ii) )
        {
            augJ(k, ii) = 1;
            augErr(k) = jla_VelCubic(joints_(ii),
                                     joint_limits_(ii,0), max_vel_(ii),
                                     joint_mid_(ii), 0.0);
//            limited_joints(k) = ii;
            ++k;
            continue;
        }

        if (joints_(ii) > joint_mid_(ii))
        {
            // upper limit: add negative velocity
            augJ(k, ii) = 1;
            augErr(k) = -jla_VelCubic(joints_(ii),
                                      joint_limits_(ii,0), max_vel_(ii),
                                      joint_mid_(ii), 0.0);
//            limited_joints(k) = ii;
            ++k;
            continue;
        }
    }
//    if (debug_ && k>0)
//    {
//        Eigen::VectorXi limited = limited_joints.head(k).transpose();
//        ROS_ERROR_STREAM(k << " joint limits active.");
//    }

    //resize J and err and add augmented data
    J.conservativeResize(orig_rows + k, orig_cols);
    err.conservativeResize(orig_rows + k);
    J.block(orig_rows, 0, k, orig_cols) = augJ.block(0, 0, k, orig_cols);
    err.tail(k) = augErr.head(k);
}

void JLA_IK::calcInvKin(const Affine3d &goal, const VectorXd &joint_seed, VectorXd &joint_angles)
{
    if (!checkInitialized())
        throw std::runtime_error("Must call init() before using Constrained_IK");

    // initialize state
    joint_angles = joint_seed;  // initialize result to seed value
    reset(goal, joint_seed);    // reset state vars for this IK solve
    update(joint_angles);       // update current state

    // iterate until solution converges (or aborted)
    while (!checkStatus())
    {
        // calculate a Jacobian (relating joint-space updates/deltas to cartesian-space errors/deltas)
        // and the associated cartesian-space error/delta vector
        MatrixXd J  = calcConstraintJacobian();
        VectorXd err = calcConstraintError();

        // Method 1: Add velocity to push joint back into range
        augmentJErr(J, err);	// add augmented rows to J and err to avoid joint limits

        //Method 3: Always add some velocity to push joint to center of range
        //Warning - unstable kinematic behavior
//        augmentJErr3(J, err);    // add augmented rows to J and err to avoid joint limits

        // solve for the resulting joint-space update
        VectorXd dJoint;
        kin_.solvePInv(J, err, dJoint);

        //Method 2: Zero-out offending row in Jacobian
//        if (!kin_.checkJoints(joint_angles + dJoint * joint_update_gain_))
//        {
//            ROS_WARN("Recalculating jacobian with joint lock");
//            augmentJErr2(J, err);           // zero out jl rows in J
//            kin_.solvePInv(J, err, dJoint); // re-solve for new joint update
//        }

        // update joint solution by the calculated update (or a partial fraction)
        joint_angles += dJoint * joint_update_gain_;

        clipToJointLimits(joint_angles);
        // re-update internal state variables
        update(joint_angles);
    }

    ROS_INFO_STREAM("IK solution: " << joint_angles.transpose());
}

bool JLA_IK::checkStatus() const
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

void JLA_IK::init(const basic_kin::BasicKin &kin)
{
    if (!kin.checkInitialized())
        throw std::invalid_argument("Input argument 'BasicKin' must be initialized");

    kin_ = kin;

    joint_limits_ = kin_.getLimits();

    const size_t n = joint_limits_.rows();
    joint_ranges_.resize(n);
    joint_mid_.resize(n);
    for (size_t ii=0; ii<n; ++ii)
    {
        joint_ranges_(ii) = joint_limits_(ii,1) - joint_limits_(ii,0);
        joint_mid_(ii) = (joint_limits_(ii,1) + joint_limits_(ii,0)) / 2.0;
    }
    jl_threshold_ = VectorXd::Constant(n, 0.05);  //default threshold is 5% of each joint range
    threshold_upper_ = joint_limits_.col(1) - (joint_ranges_.cwiseProduct(jl_threshold_));
    threshold_lower_ = joint_limits_.col(0) + (joint_ranges_.cwiseProduct(jl_threshold_));
    max_vel_ = 2.0 * joint_ranges_.cwiseProduct(jl_threshold_); //max velocity is 2*(threshold % of range)

    initialized_ = true;
}

double JLA_IK::jla_VelCubic(double angle,
                            const double &max_angle, const double &max_vel,
                            const double &min_angle, const double &min_vel) const
{
    // (y-y0) = k(x-x0)^3
    double k = (max_vel - min_vel) / std::pow(max_angle-min_angle, 3);  // k=(y1-y0)/(x1-x0)^3
    angle = std::abs(angle - min_angle);
    return min_vel + k*std::pow(angle-min_angle, 3);                    // y = y0 + k(x-x0)^3
}

void JLA_IK::update(const VectorXd &joints)
{
  Constrained_IK::update(joints);

  kin_.calcFwdKin(joints, pose_);
  pos_err_ = calcDistance(goal_, pose_);
  rot_err_ = calcAngle(goal_, pose_);
}

} /* namespace jla_ik */
} /* namespace constrained_ik */
