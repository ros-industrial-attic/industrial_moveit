/**
 * @file solver_state.h
 * @brief Internal state of Constrained_IK solver
 * @author dsolomon
 * @date Sep 15, 2013
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
#ifndef SOLVER_STATE_H
#define SOLVER_STATE_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <constrained_ik/enum_types.h>

namespace constrained_ik
{

/** @brief Internal state of Constrained_IK solver */
struct SolverState
{
  Eigen::Affine3d goal;
  Eigen::VectorXd joint_seed;

  int iter;
  Eigen::VectorXd joints;
  Eigen::VectorXd joints_delta;
  Eigen::Affine3d pose_estimate;
  std::vector<Eigen::VectorXd> iteration_path;
  initialization_state::InitializationState condition;

  SolverState(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed);
  SolverState(){}

  void reset(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // namespace constrained_ik

#endif // SOLVER_STATE_H

