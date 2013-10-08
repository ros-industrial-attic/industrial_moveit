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

#include "constrained_ik/constraints/joint_vel_limits.h"
#include "constrained_ik/constrained_ik.h"
#include <ros/ros.h>

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;
using namespace std;

JointVelLimits::JointVelLimits(): Constraint(), weight_(1.0), timestep_(.1)
{
  debug_ = true;
}

Eigen::VectorXd JointVelLimits::calcError()
{
  size_t nRows = limited_joints_.size();
  VectorXd error(nRows);
  VectorXd vel_error = jvel_.cwiseAbs() - vel_limits_;
  for (int ii=0; ii<nRows; ++ii)
  {
    size_t jntIdx = limited_joints_[ii];
    int velSign = jvel_(jntIdx)<0 ? 1 : -1;  // negative jvel requires positive velocity correction, and vice versa

    error(ii) = velSign * weight_ * vel_error(jntIdx) * 1.25;
  }

  if (debug_ && nRows)
  {
      ROS_ERROR_STREAM("iteration " << state_.iter);
      ROS_ERROR_STREAM("Joint velocity: " << jvel_.transpose());
      ROS_ERROR_STREAM("velocity error: " << error.transpose());
  }

  return error;
}

Eigen::MatrixXd JointVelLimits::calcJacobian()
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

void JointVelLimits::init(const Constrained_IK *ik)
{
  Constraint::init(ik);

  timestep_ = .08;  //TODO this should come from somewhere
  // initialize velocity limits
  vel_limits_.resize(numJoints());
  for (size_t ii=0; ii<numJoints(); ++ii)
      vel_limits_(ii) = 2*M_PI;  //TODO this should come from somewhere
}

void JointVelLimits::reset()
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

void JointVelLimits::update(const SolverState &state)
{
  if (!initialized_) return;
  Constraint::update(state);
  limited_joints_.clear();

  jvel_ = (state_.joints - state_.joint_seed)/timestep_;
  // check for limited joints
  for (size_t ii=0; ii<numJoints(); ++ii)
    if ( std::abs(jvel_(ii)) > vel_limits_(ii) )
      limited_joints_.push_back(ii);

  // print debug message, if enabled
  if (debug_ && !limited_joints_.empty())
    ROS_ERROR_STREAM(limited_joints_.size() << " joint vel limits active: " << limited_joints_);
}

} /* namespace constraints */
} /* namespace constrained_ik */
