/**
 * @file avoid_singularities.cpp
 * @brief Constraint to increases dexterity when manipulator is close to singularity
 *
 * Joint velocity is determined by gradient of smallest singular value
 * Constraint is only active when smallest SV is below theshold
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
#include "constrained_ik/constrained_ik.h"
#include "constrained_ik/constraints/avoid_singularities.h"
#include "ros/ros.h"
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(constrained_ik::constraints::AvoidSingularities, constrained_ik::Constraint)

const double DEFAULT_WEIGHT = 1.0; /**< Default weight */
const double DEFAULT_ENABLE_THRESHOLD = 0.01; /**< Default for how small singular value must be to trigger avoidance */
const double DEFAULT_IGNORE_THRESHOLD = 1e-5; /**< Default for how small is too small */

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;

// initialize limits/tolerances to default values
AvoidSingularities::AvoidSingularities() : Constraint(),
                                            weight_(DEFAULT_WEIGHT),
                                            enable_threshold_(DEFAULT_ENABLE_THRESHOLD),
                                            ignore_threshold_(DEFAULT_IGNORE_THRESHOLD)
{
}

constrained_ik::ConstraintResults AvoidSingularities::evalConstraint(const SolverState &state) const
{
  constrained_ik::ConstraintResults output;
  AvoidSingularities::AvoidSingularitiesData cdata(state, this);

  output.error = calcError(cdata);
  output.jacobian = calcJacobian(cdata);
  output.status = checkStatus(cdata);

  return output;
}

Eigen::VectorXd AvoidSingularities::calcError(const AvoidSingularities::AvoidSingularitiesData &cdata) const
{
    size_t n(cdata.avoidance_enabled_? numJoints():0);    // number of columns = joints
    VectorXd err(n);
    if (cdata.avoidance_enabled_)
    {
        for (size_t jntIdx=0; jntIdx<n; ++jntIdx)
        {
            err(jntIdx) = (cdata.Ui_.transpose() * jacobianPartialDerivative(cdata, jntIdx) * cdata.Vi_)(0);
        }
        err *= (weight_*cdata.smallest_sv_);
        err = err.cwiseMax(VectorXd::Constant(n,.25));
    }
    return err;
}

Eigen::MatrixXd AvoidSingularities::calcJacobian(const AvoidSingularities::AvoidSingularitiesData &cdata) const
{
    size_t n(cdata.avoidance_enabled_? numJoints():0);    // number of columns = joints
    MatrixXd  J = MatrixXd::Identity(n,n);
    J *= (weight_*cdata.smallest_sv_);
    return J;
}

Eigen::MatrixXd AvoidSingularities::jacobianPartialDerivative(const AvoidSingularities::AvoidSingularitiesData &cdata, size_t jntIdx, double eps) const
{
    MatrixXd jacobian_increment;
    Eigen::VectorXd joints = cdata.state_.joints;
    joints(jntIdx) += eps;

    if (!ik_->getKin().checkJoints(joints))
    {
        eps = -eps;
        joints(jntIdx) += 2*eps;
    }
    if (!ik_->getKin().calcJacobian(joints, jacobian_increment))
        ROS_WARN("Could not calculate jacobian in AvoidSingularities");
    return (jacobian_increment-cdata.jacobian_orig_)/eps;
}

void AvoidSingularities::loadParameters(const XmlRpc::XmlRpcValue &constraint_xml)
{
  XmlRpc::XmlRpcValue local_xml = constraint_xml;
  if (!getParam(local_xml, "enable_threshold", enable_threshold_))
  {
    ROS_WARN("Avoid Singularities: Unable to retrieve enable_threshold member, default parameter will be used.");
  }

  if (!getParam(local_xml, "ignore_threshold", ignore_threshold_))
  {
    ROS_WARN("Avoid Singularities: Unable to retrieve ignore_threshold member, default parameter will be used.");
  }

  if (!getParam(local_xml, "weights", weight_))
  {
    ROS_WARN("Avoid Singularities: Unable to retrieve weights member, default parameter will be used.");
  }

  if (!getParam(local_xml, "debug", debug_))
  {
    ROS_WARN("Avoid Singularities: Unable to retrieve debug member, default parameter will be used.");
  }
}

AvoidSingularities::AvoidSingularitiesData::AvoidSingularitiesData(const SolverState &state, const constraints::AvoidSingularities *parent): ConstraintData(state)
{
  if (!parent->initialized_) return;
  parent_ = parent;

  parent_->ik_->getKin().calcJacobian(state_.joints, jacobian_orig_);
  Eigen::JacobiSVD<MatrixXd> svd(jacobian_orig_, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Ui_ = svd.matrixU().rightCols(1);
  Vi_ = svd.matrixV().rightCols(1);
  smallest_sv_ = svd.singularValues().tail(1)(0);
  avoidance_enabled_ =  smallest_sv_ < parent_->enable_threshold_ && smallest_sv_ > parent_->ignore_threshold_;
  if (avoidance_enabled_)
    ROS_INFO_STREAM("Sing. avoidance with s=" << svd.singularValues().tail(1));
}

} // namespace constraints
} // namespace constrained_ik

