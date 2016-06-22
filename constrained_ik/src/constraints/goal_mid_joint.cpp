/**
 * @file goal_mid_joint.cpp
 * @brief Constraint to push joint to center of its range
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
#include "constrained_ik/constrained_ik.h"
#include "constrained_ik/constraints/goal_mid_joint.h"
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(constrained_ik::constraints::GoalMidJoint, constrained_ik::Constraint)

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;

// initialize limits/tolerances to default values
GoalMidJoint::GoalMidJoint() : Constraint(), weight_(1.0)
{
}

constrained_ik::ConstraintResults GoalMidJoint::evalConstraint(const SolverState &state) const
{
  constrained_ik::ConstraintResults output;
  GoalMidJoint::ConstraintData cdata(state);

  output.error = calcError(cdata);
  output.jacobian = calcJacobian(cdata);
  output.status = checkStatus(cdata);

  return output;
}

Eigen::VectorXd GoalMidJoint::calcError(const GoalMidJoint::ConstraintData &cdata) const
{
    VectorXd err = mid_range_ - cdata.state_.joints;
    err *= weight_;
    return err;
}

Eigen::MatrixXd GoalMidJoint::calcJacobian(const GoalMidJoint::ConstraintData &cdata) const
{
    size_t n = cdata.state_.joints.size();    // number of joints
    MatrixXd  J = MatrixXd::Identity(n,n) * weight_;
    return J;
}

void GoalMidJoint::init(const Constrained_IK *ik)
{
  Constraint::init(ik);

  // initialize joint/thresholding limits
  MatrixXd joint_limits = ik->getKin().getLimits();
  mid_range_ = joint_limits.col(1) - joint_limits.col(0);
}

void GoalMidJoint::loadParameters(const XmlRpc::XmlRpcValue &constraint_xml)
{
  XmlRpc::XmlRpcValue local_xml = constraint_xml;
  if (!getParam(local_xml, "weight", weight_))
  {
    ROS_WARN("Goal Mid Joint: Unable to retrieving weight member, default parameter will be used.");
  }
}

} // namespace constraints
} // namespace constrained_ik

