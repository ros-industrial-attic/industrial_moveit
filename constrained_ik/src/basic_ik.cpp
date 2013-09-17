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

double Basic_IK::calcAngle(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2)
{
  Eigen::Quaterniond q1(p1.rotation()), q2(p2.rotation());
  return q1.angularDistance(q2);
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

// For the basic solution, calculate XYZWPR error
Eigen::VectorXd Basic_IK::calcConstraintError()
{
  VectorXd err(6);

  Affine3d new_pose;
  kin_.calcFwdKin(joints_, new_pose);

  err << goal_.translation() - pose_.translation(), calcAngleError(pose_, goal_);
  return err;
}

// For the basic solution, translate XYZWPR error into JointError
Eigen::MatrixXd Basic_IK::calcConstraintJacobian()
{
  MatrixXd J;
  if (!kin_.calcJacobian(joints_, J))
    throw std::runtime_error("Failed to calculate Jacobian");

  return J;
}

double Basic_IK::calcDistance(const Eigen::Affine3d &p1, const Eigen::Affine3d &p2)
{
  return (p2.translation() - p1.translation()).norm();
}

bool Basic_IK::checkStatus() const
{
  // check to see if we've reached the goal pose
//  ROS_DEBUG_STREAM("pos_err: " << pos_err_ << ",  rot_err: " << rot_err_ );
  if ( (pos_err_ < pos_err_tol_) && (rot_err_ < rot_err_tol_) )
  {
    ROS_DEBUG_STREAM("Basic_IK solution found: pos_err " << pos_err_ << ", rot_err " << rot_err_);
    return true;
  }

  return Constrained_IK::checkStatus();
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

} // namespace basic_ik
} // namespace constrained_ik

