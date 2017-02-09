/**
 * @file joint_vel_limits.cpp
 * @brief Constraint to avoid joint velocity limits.
 *
 * @author dsolomon
 * @date Sep 23, 2013
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "constrained_ik/constraints/joint_vel_limits.h"
#include "constrained_ik/constrained_ik.h"
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(constrained_ik::constraints::JointVelLimits, constrained_ik::Constraint)

const double DEFAULT_WEIGHT = 1.0; /**< Default weight */
const double DEFAULT_TIMESTEP = 0.1; /**< Default timestep used to calculate joint velocities */

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;
using namespace std;

JointVelLimits::JointVelLimits(): Constraint(), weight_(DEFAULT_WEIGHT), timestep_(DEFAULT_TIMESTEP)
{
}

constrained_ik::ConstraintResults JointVelLimits::evalConstraint(const SolverState &state) const
{
  constrained_ik::ConstraintResults output;
  JointVelLimits::JointVelLimitsData cdata(state, this);

  output.error = calcError(cdata);
  output.jacobian = calcJacobian(cdata);
  output.status = checkStatus(cdata);

  return output;
}

Eigen::VectorXd JointVelLimits::calcError(const JointVelLimits::JointVelLimitsData &cdata) const
{
  size_t nRows = cdata.limited_joints_.size();
  VectorXd error(nRows);
  VectorXd vel_error = cdata.jvel_.cwiseAbs() - vel_limits_;
  for (int ii=0; ii<nRows; ++ii)
  {
    size_t jntIdx = cdata.limited_joints_[ii];
    int velSign = cdata.jvel_(jntIdx)<0 ? 1 : -1;  // negative jvel requires positive velocity correction, and vice versa

    error(ii) = velSign * weight_ * vel_error(jntIdx) * 1.25;
  }

  if (debug_ && nRows)
  {
      ROS_ERROR_STREAM("iteration " << cdata.state_.iter);
      ROS_ERROR_STREAM("Joint velocity: " << cdata.jvel_.transpose());
      ROS_ERROR_STREAM("velocity error: " << error.transpose());
  }

  return error;
}

Eigen::MatrixXd JointVelLimits::calcJacobian(const JointVelLimits::JointVelLimitsData &cdata) const
{
  size_t nRows = cdata.limited_joints_.size();
  MatrixXd jacobian(nRows, numJoints());

  for (int ii=0; ii<nRows; ++ii)
  {
    size_t jntIdx = cdata.limited_joints_[ii];

    VectorXd tmpRow = VectorXd::Zero(numJoints());
    tmpRow(jntIdx) = 1.0;

    jacobian.row(ii) = tmpRow * weight_;
  }

  return jacobian;
}

void JointVelLimits::init(const Constrained_IK *ik)
{
  Constraint::init(ik);

  // initialize velocity limits
  vel_limits_.resize(numJoints());
  for (size_t ii=0; ii<numJoints(); ++ii)
      vel_limits_(ii) = 2*M_PI;  //TODO this should come from somewhere
}

void JointVelLimits::loadParameters(const XmlRpc::XmlRpcValue &constraint_xml)
{
  XmlRpc::XmlRpcValue local_xml = constraint_xml;
  if (!getParam(local_xml, "weights", weight_))
  {
    ROS_WARN("Avoid Joint Velocity Limits: Unable to retrieve weights member, default parameter will be used.");
  }

  if (!getParam(local_xml, "timestep", timestep_))
  {
    ROS_WARN("Avoid Joint Velocity Limits: Unable to retrieve timestep member, default parameter will be used.");
  }

  if (!getParam(local_xml, "debug", debug_))
  {
    ROS_WARN("Avoid Joint Velocity Limits: Unable to retrieve debug member, default parameter will be used.");
  }
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

JointVelLimits::JointVelLimitsData::JointVelLimitsData(const SolverState &state, const constraints::JointVelLimits *parent): ConstraintData(state)
{
  if (!parent->initialized_) return;
  parent_ = parent;

  jvel_ = (state_.joints - state_.joint_seed)/parent_->timestep_;
  // check for limited joints
  for (size_t ii=0; ii<parent_->numJoints(); ++ii)
    if ( std::abs(jvel_(ii)) > parent_->vel_limits_(ii) )
      limited_joints_.push_back(ii);

  // print debug message, if enabled
  if (parent_->debug_ && !limited_joints_.empty())
    ROS_ERROR_STREAM(limited_joints_.size() << " joint vel limits active: " << limited_joints_);
}

} /* namespace constraints */
} /* namespace constrained_ik */
