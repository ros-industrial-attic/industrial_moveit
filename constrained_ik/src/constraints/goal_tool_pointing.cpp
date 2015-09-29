/**
* @file goal_tool_pointing.cpp
* @brief Constraint to specify cartesian tool goal pointing (XYZRP)
* @author Levi Armstrong
* @date September 18, 2015
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
#include "constrained_ik/constraints/goal_tool_pointing.h"
#include "constrained_ik/constraints/goal_orientation.h"

namespace constrained_ik
{
namespace constraints
{

using namespace Eigen;

// initialize limits/tolerances to default values
GoalToolPointing::GoalToolPointing() : Constraint(), pos_err_tol_(0.001), rot_err_tol_(0.009), weight_(VectorXd::Ones(5).asDiagonal())
{
}

constrained_ik::ConstraintResults GoalToolPointing::evalConstraint(const SolverState &state) const
{
  constrained_ik::ConstraintResults output;
  GoalToolPointing::GoalToolPointingData cdata(state);

  output.error = calcError(cdata);
  output.jacobian = calcJacobian(cdata);
  output.status = checkStatus(cdata);

  return output;
}

Eigen::VectorXd GoalToolPointing::calcError(const GoalToolPointing::GoalToolPointingData &cdata) const
{
  Matrix<double, 5, 1> err, tmp_err;
  MatrixXd R = cdata.state_.pose_estimate.rotation().transpose();
  
  tmp_err << (R * (cdata.state_.goal.translation() - cdata.state_.pose_estimate.translation())),
             (R * GoalOrientation::calcAngleError(cdata.state_.pose_estimate, cdata.state_.goal)).topRows(2);
  
  err = weight_ * tmp_err;

  return err;
}

Eigen::MatrixXd GoalToolPointing::calcJacobian(const GoalToolPointing::GoalToolPointingData &cdata) const
{
  MatrixXd tmpJ, J, Jt, R;
  
  if (!ik_->getKin().calcJacobian(cdata.state_.joints, tmpJ))
    throw std::runtime_error("Failed to calculate Jacobian");

  //rotate jacobian into tool frame by premultiplying by otR.transpose()
  R = cdata.state_.pose_estimate.rotation().transpose();
  Jt.resize(5, ik_->numJoints());
  Jt << (R * tmpJ.topRows(3)),
        (R * tmpJ.bottomRows(3)).topRows(2);

  // weight each row of J
  J = weight_ * Jt;

  return J;
}

bool GoalToolPointing::checkStatus(const GoalToolPointing::GoalToolPointingData &cdata) const
{
  // check to see if we've reached the goal position
  if (cdata.pos_err_ < pos_err_tol_ && cdata.rot_err_ < rot_err_tol_)
    return true;

  return false;
}

GoalToolPointing::GoalToolPointingData::GoalToolPointingData(const SolverState &state): ConstraintData(state)
{
  MatrixXd R = state_.pose_estimate.rotation().transpose();
  pos_err_ = (R * (state_.pose_estimate.translation() - state_.goal.translation())).norm();
  rot_err_ = (R * GoalOrientation::calcAngleError(state_.pose_estimate, state_.goal)).topRows(2).norm();
}

} // namespace constraints
} // namespace constrained_ik


