/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "constrained_ik/basic_ik.h"
#include <ros/ros.h>

namespace constrained_ik
{
namespace basic_ik
{

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Affine3d;

Basic_IK::Basic_IK() : Constrained_IK()
{
  // initialize limits/tolerances to default values
  pos_err_tol_ = 0.001;
  rot_err_tol_ = 0.009;
  pos_err_ = 0.0;
  rot_err_ = 0.0;
}

void Basic_IK::reset(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed)
{
  Constrained_IK::reset(goal, joint_seed);

  pos_err_ = rot_err_ = 0;
}

void Basic_IK::update(const Eigen::VectorXd &joints)
{
  Constrained_IK::update(joints);

  kin_.calcFwdKin(joints, pose_);
  pos_err_ = calcDistance(goal_, pose_);
  rot_err_ = calcAngle(goal_, pose_);
}

bool Basic_IK::checkStatus() const
{
  ROS_DEBUG_STREAM("pos_err: " << pos_err_ << ",  rot_err: " << rot_err_ );
  // check to see if we've reached the goal pose
  if ( (pos_err_ < pos_err_tol_) && (rot_err_ < rot_err_tol_) )
  {
    ROS_DEBUG_STREAM("Basic_IK solution found: pos_err " << pos_err_ << ", rot_err " << rot_err_);
    return true;
  }

  return Constrained_IK::checkStatus();
}

double Basic_IK::calcDistance(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2)
{
  return (p2.translation() - p1.translation()).norm();
}

double Basic_IK::calcAngle(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2)
{
  Eigen::Quaterniond q1(p1.rotation()), q2(p2.rotation());
  return q1.angularDistance(q2);
}

// For the basic solution, translate XYZWPR error into JointError
Eigen::MatrixXd Basic_IK::calcConstraintJacobian()
{
  MatrixXd J;
  if (!kin_.calcJacobian(joints_, J))
    throw std::runtime_error("Failed to calculate Jacobian");

  return J;
}

// For the basic solution, calculate XYZWPR error
Eigen::VectorXd Basic_IK::calcConstraintError()
{
  VectorXd err(6);

  Affine3d new_pose;
  kin_.calcFwdKin(joints_, new_pose);

  err << goal_.translation() - pose_.translation(), calcAngleError(pose_, goal_);
  return err;
}

Eigen::Vector3d Basic_IK::calcAngleError(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2)
{
  Eigen::AngleAxisd r12(p1.rotation().transpose()*p2.rotation());   // rotation from p1 -> p2
  return p1.rotation() * r12.axis() * rangedAngle(r12.angle());     // axis k * theta expressed in frame0
}

Eigen::Vector3d Basic_IK::calcAngleError2(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2)
{
  Eigen::Matrix3d R12 = p1.rotation().transpose() * p2.rotation();  // rotation from p1->p2 (typically used for pose-goal)

  //R12 can be expressed as rotation about axis k by angle theta
  //Axis k described in frame p1
  Eigen::Vector3d k (R12(2,1)-R12(1,2), R12(0,2)-R12(2,0), R12(1,0)-R12(0,1));  // =k*2sin(theta) (k not normalized here)
  double theta = atan2(k.norm(), R12.trace()-1);    // -pi < theta < pi (atan2(2sin(theta),2cos(theta))

  // solution is undefined at theta = 0
  if (fabs(theta) < 1e-6)
      return Eigen::Vector3d::Zero();

  // return 3 component terms of theta in frame0:
  // normalize k, express in frame0 by premultiplying by R0p1, multiply through by theta
  return p1.rotation() * k.normalized() * theta;
}


} // namespace basic_ik
} // namespace constrained_ik

