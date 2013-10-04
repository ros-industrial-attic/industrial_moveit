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


#ifndef SOLVER_STATE_H
#define SOLVER_STATE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace constrained_ik
{

/**
 * \brief Internal state of Constrained_IK solver
  */
struct SolverState
{
  Eigen::Affine3d goal;
  Eigen::VectorXd joint_seed;

  int iter;
  Eigen::VectorXd joints;
  Eigen::VectorXd joints_delta;
  Eigen::Affine3d pose_estimate;

  void reset(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed);
};

} // namespace constrained_ik

#endif // SOLVER_STATE_H

