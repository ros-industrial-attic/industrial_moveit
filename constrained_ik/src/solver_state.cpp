/**
 * @file solver_state.cpp
 * @brief Internal state of Constrained_IK solver
 *
 * @author dsolomon
 * @date Sep 15, 2013
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
#include "constrained_ik/solver_state.h"
#include <limits>
#include <ros/ros.h>

using namespace Eigen;

namespace constrained_ik
{

SolverState::SolverState(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed)
{
  reset(goal, joint_seed);
}

void SolverState::reset(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed)
{
  this->goal = goal;
  this->joint_seed = joint_seed;
  this->iter = 0;
  this->joints = VectorXd::Constant(joint_seed.size(), std::numeric_limits<double>::max());
  this->joints_delta = VectorXd::Zero(joint_seed.size());
  this->primary_sum = 0.0;
  this->auxiliary_sum = 0.0;
  this->auxiliary_at_limit = false;
  this->pose_estimate = Affine3d::Identity();
  this->condition = initialization_state::NothingInitialized;
}

} // namespace constrained_ik

