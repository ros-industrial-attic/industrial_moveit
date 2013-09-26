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
  }

  ROS_INFO_STREAM("IK solution: " << joint_angles.transpose());
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
  {
	  ROS_ERROR_STREAM("Reached " << iter_ << " / " << max_iter_ << " iterations before convergence.");
	  throw std::runtime_error("Iteration converged before goal reached.  IK solution may be invalid");
  }

  return false;
}

void Constrained_IK::clipToJointLimits(VectorXd &joints)
{
  const MatrixXd limits = kin_.getLimits();
  const VectorXd orig_joints(joints);

  if (joints.size() != limits.rows())
    throw std::invalid_argument("clipToJointLimits: Unexpected number of joints");

  for (size_t i=0; i<limits.rows(); ++i)
  {
    joints[i] = std::max(limits(i,0), std::min(limits(i,1), joints[i]));
  }
  if (debug_ && !joints.isApprox(orig_joints))
      ROS_WARN("Joints have been clipped");
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

double Constrained_IK::rangedAngle(double angle)
{
    angle = copysign(fmod(fabs(angle),2.0*M_PI), angle);
    if (angle < -M_PI) return angle+2.*M_PI;
    if (angle > M_PI)  return angle-2.*M_PI;
    return angle;
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

} // namespace constrained_ik

