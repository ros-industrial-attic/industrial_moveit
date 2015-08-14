/**
 * @file avoid_joint_limits.cpp
 * @brief Constraint to avoid joint position limits.
 *
 * Using cubic velocity ramp, it pushes each joint away from its limits,
 * with a maximimum velocity of 2*threshold*(joint range).
 * Only affects joints that are within theshold of joint limit.
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
#include "constrained_ik/constraints/avoid_joint_limits.h"
#include "constrained_ik/constrained_ik.h"
#include <ros/ros.h>

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;
using namespace std;

constrained_ik::ConstraintResults AvoidJointLimits::evalConstraint(const SolverState &state) const
{
  constrained_ik::ConstraintResults output;
  AvoidJointLimits::AvoidJointLimitsData cdata(state, this);

  output.error = calcError(cdata);
  output.jacobian = calcJacobian(cdata);
  output.status = checkStatus(cdata);

  return output;
}

Eigen::VectorXd AvoidJointLimits::calcError(const AvoidJointLimits::AvoidJointLimitsData &cdata) const
{
  size_t nRows = cdata.limited_joints_.size();
  VectorXd error(nRows);

  for (int ii=0; ii<nRows; ++ii)
  {
    size_t jntIdx = cdata.limited_joints_[ii];
    int velSign;
    const LimitsT &lim = limits_[jntIdx];
    double limit;
    if (cdata.nearLowerLimit(jntIdx))
    {
        velSign = 1; // lower limit: positive velocity
        limit = lim.min_pos;
    }
    else
    {
        velSign = -1;   //upper limit, negative velocity
        limit = lim.max_pos;
    }

    error(ii) = velSign * weight_ * lim.cubicVelRamp(cdata.state_.joints[jntIdx], limit);

    if (debug_)
    {
        ROS_WARN_STREAM("iteration " << cdata.state_.iter << std::endl <<
                         "Joint position: " << cdata.state_.joints(jntIdx) << " / " << limit << std::endl <<
                         "velocity error: " << error(ii) << " / " << lim.e/2.0);
    }
  }
  return error;
}

Eigen::MatrixXd AvoidJointLimits::calcJacobian(const AvoidJointLimits::AvoidJointLimitsData &cdata) const
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

bool AvoidJointLimits::checkStatus(const AvoidJointLimits::AvoidJointLimitsData &cdata) const
{
    size_t n = cdata.state_.joints.size();
    for (size_t ii = 0; ii<n; ++ii)
        if (cdata.state_.joints[ii] > limits_[ii].max_pos || cdata.state_.joints[ii] < limits_[ii].min_pos)
            return false;
    return true;
}

void AvoidJointLimits::init(const Constrained_IK *ik)
{
  Constraint::init(ik);

  // initialize joint/thresholding limits
  MatrixXd joint_limits = ik->getKin().getLimits();
  for (size_t ii=0; ii<numJoints(); ++ii)
    limits_.push_back( LimitsT(joint_limits(ii,0), joint_limits(ii,1), threshold_ ) );
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

AvoidJointLimits::AvoidJointLimitsData::AvoidJointLimitsData(const SolverState &state, const constraints::AvoidJointLimits *parent): ConstraintData(state)
{
  if (!parent->initialized_) return;
  parent_ = parent;

  // check for limited joints
  for (size_t ii=0; ii<parent_->numJoints(); ++ii)
    if ( nearLowerLimit(ii) || nearUpperLimit(ii) )
      limited_joints_.push_back(ii);

  // print debug message, if enabled
  if (parent_->debug_ && !limited_joints_.empty())
    ROS_WARN_STREAM(limited_joints_.size() << " joint limits active: " << limited_joints_);
}

bool AvoidJointLimits::AvoidJointLimitsData::nearLowerLimit(size_t idx) const
{
  if (idx >= state_.joints.size() || idx >= parent_->limits_.size() )
    return false;

  return state_.joints(idx) < parent_->limits_[idx].lower_thresh;
}

bool AvoidJointLimits::AvoidJointLimitsData::nearUpperLimit(size_t idx)
{
  if (idx >= state_.joints.size() || idx >= parent_->limits_.size() )
    return false;

  return state_.joints(idx) > parent_->limits_[idx].upper_thresh;
}


AvoidJointLimits::LimitsT::LimitsT(double minPos, double maxPos, double threshold)
{
  min_pos = minPos;
  max_pos = maxPos;

  //threshold given as a percentage of range. Translate to actual joint distance
  e = threshold * (maxPos - minPos); //range;
  lower_thresh = minPos + e;
  upper_thresh = maxPos - e;

  /* For d = distance from limit: velocity v = k(d-e)^3, where e is distance from threshold to limit.
   * k is chosen such that v_max = v[d=0] = e/2 --> k = 1/(2e^2)
   */
  k3 = 1.0/(2 * e * e);
}

double AvoidJointLimits::LimitsT::cubicVelRamp(double angle, double limit) const
{
    double d = std::abs(angle - limit); // distance from limit
    return k3 * std::pow(e-d,3);
}

} /* namespace constraint */
} /* namespace constrained_ik */
