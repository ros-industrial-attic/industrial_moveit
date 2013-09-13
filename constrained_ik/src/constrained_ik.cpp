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

#include "constrained_ik/constrained_ik.h"
#include <limits>
#include <ros/ros.h>

namespace constrained_ik
{

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Affine3d;

Constrained_IK::Constrained_IK()
{
  initialized_ = false;
  joint_update_gain_ = 0.09;        //default joint update gain
  iter_ = 0;
  max_iter_ = 500;                  //default max_iter
  joint_convergence_tol_ = 0.0001;   //default convergence tolerance
  debug_ = false;
}

void Constrained_IK::init(const urdf::Model &robot, const std::string &base_name, const std::string &tip_name)
{
  basic_kin::BasicKin kin;
  if (! kin.init(robot, base_name, tip_name))
    throw std::runtime_error("Failed to initialize BasicKin");

  init(kin);
}

void Constrained_IK::init(const basic_kin::BasicKin &kin)
{
  if (!kin.checkInitialized())
    throw std::invalid_argument("Input argument 'BasicKin' must be initialized");

  kin_ = kin;
  initialized_ = true;
}

void Constrained_IK::calcInvKin(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed, Eigen::VectorXd &joint_angles)
{
  if (!checkInitialized())
    throw std::runtime_error("Must call init() before using Constrained_IK");
  //TODO should goal be checked here instead of in reset()?

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

    // solve for the resulting joint-space update
    VectorXd dJoint;
    kin_.solvePInv(J, err, dJoint);

    // update joint solution by the calculated update (or a partial fraction)
    joint_angles += dJoint * joint_update_gain_;
    clipToJointLimits(joint_angles);

    // re-update internal state variables
    update(joint_angles);
//    std::cout << "manipulability: " << J.determinant()*1000. << std::endl;
//    std::cout << dJoint.transpose() << std::endl;
//    std::cout << joints_.transpose() << std::endl;
  }

  ROS_INFO_STREAM("IK solution: " << joint_angles.transpose());
}

void Constrained_IK::reset(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed)
{
  if (!kin_.checkJoints(joint_seed))
    throw std::invalid_argument("Seed doesn't match kinematic model");

  if (!goal.matrix().block(0,0,3,3).isUnitary(1e-6)) {
        throw std::invalid_argument("Goal pose not proper affine");
    }

  goal_ = goal;
  joint_seed_ = joint_seed;

  iter_ = 0;
  joints_ = VectorXd::Constant(joint_seed.size(), std::numeric_limits<double>::max());
  joints_delta_ = VectorXd::Zero(joint_seed.size());
}

void Constrained_IK::update(const Eigen::VectorXd &joints)
{
  // update maximum iterations
  iter_++;

  // update joint convergence
  joints_delta_ = joints - joints_;
  joints_ = joints;

  if (debug_)
      iteration_path_.push_back(joints);
}

// NOTE: the default status() method will never return SUCCESS (true)
bool Constrained_IK::checkStatus() const
{
  // check maximum iterations
  if (iter_ > max_iter_)
    throw std::runtime_error("Maximum iterations reached.  IK solution may be invalid.");

  // check for joint convergence
  //   - this is an error: joints stabilize, but goal pose not reached
  if (joints_delta_.cwiseAbs().maxCoeff() < joint_convergence_tol_)
    throw std::runtime_error("Iteration converged before goal reached.  IK solution may be invalid");

  return false;
}

void Constrained_IK::clipToJointLimits(Eigen::VectorXd &joints)
{
  MatrixXd limits = kin_.getLimits();

  if (joints.size() != limits.rows())
    throw std::invalid_argument("clipToJointLimits: Unexpected number of joints");

  for (size_t i=0; i<limits.rows(); ++i)
  {
    joints[i] = std::max(limits(i,0), std::min(limits(i,1), joints[i]));
  }
}

double Constrained_IK::rangedAngle(double angle)
{
    angle = copysign(fmod(fabs(angle),2.0*M_PI), angle);
    if (angle < -M_PI) return angle+2.*M_PI;
    if (angle > M_PI)  return angle-2.*M_PI;
    return angle;
}
} // namespace constrained_ik

