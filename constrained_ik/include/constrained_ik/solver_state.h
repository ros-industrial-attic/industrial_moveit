/**
 * @file solver_state.h
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
#ifndef SOLVER_STATE_H
#define SOLVER_STATE_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <constrained_ik/enum_types.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection/collision_world.h>

namespace constrained_ik
{

/** @brief Internal state of Constrained_IK solver */
struct SolverState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Affine3d goal;                                                  /**< Desired goal position to solve IK about */
  Eigen::VectorXd joint_seed;                                            /**< Joint position (initial guess) to seed the IK solver with */

  int iter;                                                              /**< Current number of solver iterations */
  Eigen::VectorXd joints;                                                /**< Updated joint positions */
  Eigen::VectorXd joints_delta;                                          /**< The joint delta between this state and the previous state */
  Eigen::Affine3d pose_estimate;                                         /**< The pose for the updated joint position from the solver */
  std::vector<Eigen::VectorXd> iteration_path;                           /**< Store the joint position for each iteration of the solver */
  double primary_sum;                                                    /**< The absolute sum of the cumulative primary motion */
  double auxiliary_sum;                                                  /**< The absolute sum of the cumulative auxiliary motion */
  bool auxiliary_at_limit;                                               /**< This is set if auxiliary reached motion or iteration limit. */
  initialization_state::InitializationState condition;                   /**< State of the IK Solver */
  planning_scene::PlanningSceneConstPtr planning_scene;                  /**< Pointer to the planning scene, some constraints require it */
  collision_detection::CollisionRobotConstPtr collision_robot;           /**< Pointer to the collision robot, some constraints require it */
  collision_detection::CollisionWorldConstPtr collision_world;            /**< Pointer to the collision world, some constraints require it */
  moveit::core::RobotStatePtr robot_state;                               /**< Pointer to the current robot state */
  std::string group_name;                                                /**< Move group name */

  /**
   * @brief SolverState Constructor
   * @param goal desired goal position to solve IK about
   * @param joint_seed joint position (initial guess) to seed the IK solver with
   */
  SolverState(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed);
  SolverState(){}

  /**
   * @brief Reset the current state to default state
   * @param goal desired goal position to solve IK about
   * @param joint_seed joint position (initial guess) to seed the IK solver with
   */
  void reset(const Eigen::Affine3d &goal, const Eigen::VectorXd &joint_seed);

};

} // namespace constrained_ik

#endif // SOLVER_STATE_H

