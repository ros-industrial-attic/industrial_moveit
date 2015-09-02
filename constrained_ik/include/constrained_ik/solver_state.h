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
#include <moveit/planning_scene/planning_scene.h>
#include <constrained_ik/collision_robot_fcl_detailed.h>

namespace constrained_ik
{

/** @brief Internal state of Constrained_IK solver */
struct SolverState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Affine3d goal;
  Eigen::VectorXd joint_seed;

  int iter;
  Eigen::VectorXd joints;
  Eigen::VectorXd joints_delta;
  Eigen::Affine3d pose_estimate;
  std::vector<Eigen::VectorXd> iteration_path;
  double primary_sum; /**< The absolute sum of the cumulative primary motion */
  double auxiliary_sum; /**< The absolute sum of the cumulative auxiliary motion */
  bool auxiliary_at_limit; /**< This is set if auxiliary reached motion or iteration limit. */
  initialization_state::InitializationState condition;
  planning_scene::PlanningSceneConstPtr planning_scene;
  constrained_ik::CollisionRobotFCLDetailed::CollisionRobotFCLDetailedPtr collision_robot;
  moveit::core::RobotStatePtr robot_state;

  SolverState(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed);
  SolverState(){}

  void reset(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed);

};

} // namespace constrained_ik

#endif // SOLVER_STATE_H

