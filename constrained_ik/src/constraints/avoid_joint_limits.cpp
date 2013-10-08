/*
 * avoid_joint_limits.cpp
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

#include "constrained_ik/constraints/avoid_joint_limits.h"
#include "constrained_ik/constrained_ik.h"
#include <ros/ros.h>

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;
using namespace std;

// TODO: constraint-weights, calcJacobianRow/2  ??

AvoidJointLimits::AvoidJointLimits(): Constraint(), weight_(1.0), threshold_(0.05)
{
  debug_ = true;
}

Eigen::VectorXd AvoidJointLimits::calcError()
{
  size_t nRows = limited_joints_.size();
  VectorXd error(nRows);

  for (int ii=0; ii<nRows; ++ii)
  {
    size_t jntIdx = limited_joints_[ii];
    int velSign = nearLowerLimit(jntIdx) ? 1 : -1;  // lower limit: positive velocity, upper limit: negative velocity

    const LimitsT &lim = limits_[jntIdx];
    error(ii) = velSign * weight_ * lim.cubicVelRamp(state_.joints[jntIdx]);
  }

  if (debug_ && nRows)
  {
      ROS_ERROR_STREAM("iteration " << state_.iter);
      ROS_ERROR_STREAM("Joint position: " << state_.joints(limited_joints_[0]) << " / " << limits_[limited_joints_[0]].min_pos);
      ROS_ERROR_STREAM("velocity error: " << error(0) << " / " << 2.0*threshold_ * limits_[limited_joints_[0]].range);
  }

  return error;
}

Eigen::MatrixXd AvoidJointLimits::calcJacobian()
{
  size_t nRows = limited_joints_.size();
  MatrixXd jacobian(nRows, numJoints());

  for (int ii=0; ii<nRows; ++ii)
  {
    size_t jntIdx = limited_joints_[ii];

    VectorXd tmpRow = VectorXd::Zero(numJoints());
    tmpRow(jntIdx) = 1.0;

    jacobian.row(ii) = tmpRow * weight_;
  }

  return jacobian;
}

double AvoidJointLimits::cubicVelRamp(double angle, double max_angle, double max_vel, double min_angle, double min_vel)
{
    // WARNING: DEPRECATED
    // fit a cubic function to (y-y0) = k(x-x0)^3
    // where y is desired speed, y0 is minimum vel, x is current angle, x0 is angle at which min vel is introduced
    // x1,y1 used to establish k, where x1 is max allowable angle, y1 is max allowable velocity
    // Requirements on inputs: max_vel > 0, min_vel >= 0, max_angle > min_angle, |angle| > min_angle
    // Note: Speed is always positive, velocity (vel) may be +/-, return should always be positive
    double k = (max_vel - min_vel) / std::pow(max_angle-min_angle, 3);  // k=(y1-y0)/(x1-x0)^3
    angle = std::abs(angle - min_angle);
    return min_vel + k*std::pow(angle-min_angle, 3);                    // y = y0 + k(x-x0)^3
}

void AvoidJointLimits::init(const Constrained_IK *ik)
{
  Constraint::init(ik);

  // initialize joint/thresholding limits
  MatrixXd joint_limits = ik->getKin().getLimits();
  for (size_t ii=0; ii<numJoints(); ++ii)
    limits_.push_back( LimitsT(joint_limits(ii,0), joint_limits(ii,1), threshold_ ) );
}

bool AvoidJointLimits::nearLowerLimit(size_t idx)
{
  if (idx >= state_.joints.size() || idx >= limits_.size() )
    return false;

  return state_.joints(idx) < limits_[idx].lower_thresh;
}
bool AvoidJointLimits::nearUpperLimit(size_t idx)
{
  if (idx >= state_.joints.size() || idx >= limits_.size() )
    return false;

  return state_.joints(idx) > limits_[idx].upper_thresh;
}

void AvoidJointLimits::reset()
{
  limited_joints_.clear();
}

// TODO: Move this to a common "utils" file
template<class T>
std::ostream& operator<< (std::ostream& os, const std::vector<T>& v)
{
  if (!v.empty())
  {
    copy(v.begin(), v.end()-1, std::ostream_iterator<T>(os, ", "));
    os << v.back();  // no comma after last element
  }
  return os;
}

void AvoidJointLimits::update(const SolverState &state)
{
  if (!initialized_) return;
  Constraint::update(state);
  limited_joints_.clear();

  // check for limited joints
  for (size_t ii=0; ii<numJoints(); ++ii)
    if ( nearLowerLimit(ii) || nearUpperLimit(ii) )
      limited_joints_.push_back(ii);

  // print debug message, if enabled
  if (debug_ && !limited_joints_.empty())
    ROS_ERROR_STREAM(limited_joints_.size() << " joint limits active: " << limited_joints_);
}

AvoidJointLimits::LimitsT::LimitsT(double minPos, double maxPos, double threshold)
{
  min_pos = minPos;
  max_pos = maxPos;

  range = maxPos - minPos;
  mid_pos = (minPos + maxPos) / 2.0;

  lower_thresh = minPos + threshold * range;
  upper_thresh = maxPos - threshold * range;
  double max_vel = 2.0 * threshold * range;  // max velocity is 2*(threshold % of range) - hopefully enough to push joint past threshold
  double min_vel = 0.0;
  k3 = (max_vel - min_vel)/std::pow(0.5*range, 3);  // (vel = minVel*k(pos-minPos)^3 where k=(maxVel-minVel)/(maxPos-midPos)^3 : 1/2 range used for cubic function
}

double AvoidJointLimits::LimitsT::cubicVelRamp(double angle) const
{
    double x = std::abs(angle - mid_pos);
    return k3 * std::pow(x,3);
}

} /* namespace jla_ik */
} /* namespace constrained_ik */
