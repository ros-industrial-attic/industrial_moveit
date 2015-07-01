/**
 * @file avoid_obstacles.h
 * @brief Constraint to avoid obstacles
 *
 *
 * @author clewis
 * @date June 1 2015
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
#ifndef AVOID_OBSTACLES_H_
#define AVOID_OBSTACLES_H_

#include "constrained_ik/constraint.h"
#include "constrained_ik/constrained_ik.h"
#include <constrained_ik/collision_robot_fcl_detailed.h>
#include <vector>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>

namespace constrained_ik
{
namespace constraints
{

/**
 * @brief Constraint to avoid collisions
 *
 */
class AvoidObstacles: public Constraint
{
public:
  struct AvoidObstaclesData: public ConstraintData
  {
    constrained_ik::CollisionRobotFCLDetailed::DistanceDetailedMap distance_map_;

    AvoidObstaclesData(const constrained_ik::SolverState &state);
  };

  /**
   * @brief constructor
   * @param link_name name of link which should avoid obstacles
   */
  AvoidObstacles(std::string link_name): Constraint(), weight_(1.0), min_distance_(0.006), link_name_(link_name)
  {
    requires_collision_checks_ = true;
  }

  /**
   * @brief Destructor
   */
  virtual ~AvoidObstacles()
  {
    delete jac_solver_;
  }

  virtual constrained_ik::ConstraintResults evalConstraint(const SolverState &state) const;

  /**
   * @brief Creates Jacobian for avoiding a collision with link closest to a collision
   * @param cdata, The constraint specific data.
   * @return Jacobian scaled by weight
   */
  virtual Eigen::MatrixXd calcJacobian(const AvoidObstaclesData &cdata) const;

  /**
   * @brief Creates vector representing velocity error term
   * corresponding to calcJacobian()
   * @param cdata, The constraint specific data.
   * @return VectorXd of joint velocities for obstacle avoidance
   */
  virtual Eigen::VectorXd calcError(const AvoidObstaclesData &cdata) const;

  /**
   * @brief Checks termination criteria
   * There are no termination criteria for this constraint
   * @param cdata, The constraint specific data.
   * @return True
   */
  virtual bool checkStatus(const AvoidObstaclesData &cdata) const;

  /**
   * @brief Initialize constraint (overrides Constraint::init)
   * Should be called before using class.
   * @param ik Pointer to Constrained_IK used for base-class init
   */
  virtual void init(const Constrained_IK * ik);

  /**
   * @brief getter for weight_
   * @return weight_
   */
  double getWeight() {return weight_;}

  /**
   * @brief setter for weight_
   * @param weight Value to set weight_ to
   */
  void setWeight(const double &weight) {weight_ = weight;}

protected:
  double weight_;/**< importance weight applied to this avoidance constraint */
  double min_distance_;         /**< minimum obstacle distance allowed */
  int num_robot_joints_; /**< number of joints in the whole robot*/
  int num_obstacle_joints_; /**< number of joints inboard to the obstacle link */
  std::string link_name_; /**< the name of the link that is to avoid obstacles */
  KDL::Chain avoid_chain_; /**< the kinematic chain from base to the obstacle avoidance link */
  int num_inboard_joints_; /**< number of joints in the inboard chain */
  KDL::Vector link_point_;/**< vector to point on link closest to an obstacle */
  KDL::ChainJntToJacSolver * jac_solver_; /**< a KDL object for computing jacobians */
  KDL::Vector obstacle_point_;/**< vector to point on link closest to an obstacle */
};

} /* namespace constraints */
} /* namespace constrained_ik */
#endif /* AVOID_OBSTACLES_H_ */
