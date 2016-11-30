/**
 * @file cartesian_planner.h
 * @brief Cartesian path planner for moveit.
 *
 * This class is used to represent a cartesian path planner for moveit.
 * It finds a straight line path between the start and goal position. This
 * planner does not have the inherent ability to avoid collision. It does
 * check if the path created is collision free before it returns a trajectory.
 * If a collision is found it returns an empty trajectory and moveit error.
 *
 * @author Levi Armstrong
 * @date May 4, 2015
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2015, Southwest Research Institute
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
#ifndef CARTESIAN_PLANNER_H
#define CARTESIAN_PLANNER_H

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <constrained_ik/moveit_interface/constrained_ik_planning_context.h>
#include <boost/atomic.hpp>
#include <constrained_ik/ik/master_ik.h>

namespace constrained_ik
{
  /**
  * @brief Cartesian path planner for moveit.
  *
  * This class is used to represent a cartesian path planner for moveit.
  * It finds a straight line path between the start and goal position. This
  * planner does not have the inherent ability to avoid collision. It does
  * check if the path created is collision free before it returns a trajectory.
  * If a collision is found it returns an empty trajectory and moveit error.
   */
  class CartesianPlanner : public constrained_ik::CLIKPlanningContext
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief CartesianPlanner Constructor
     * @param name of planner
     * @param group of the planner
     */
    CartesianPlanner(const std::string &name, const std::string &group) : constrained_ik::CLIKPlanningContext(name, group), terminate_(false), robot_description_("robot_description") {}

    /**
     * @brief CartesianPlanner Copy Constructor
     * @param other cartesian planner
     */
    CartesianPlanner(const CartesianPlanner &other) : constrained_ik::CLIKPlanningContext(other), terminate_(false), robot_description_("robot_description") {}

    /** @brief Clear planner data */
    void clear() override { terminate_ = false; }

    /**
     * @brief Terminate the active planner solve
     * @return True if successfully terminated, otherwise false
     */
    bool terminate() override
    {
      terminate_ = true;
      return true;
    }

    /**
     * @brief Inverse Kinematic Solve
     * @param res planner response
     * @return
     */
    bool solve(planning_interface::MotionPlanResponse &res) override;

    /**
     * @brief Inverse Kinematic Solve
     * @param res planner detailed response
     * @return
     */
    bool solve(planning_interface::MotionPlanDetailedResponse &res) override;

    /**
     * @brief Get a solver for a given group
     * @param group_name Name of the request group
     * @return Constrained IK Solver
     */
    virtual boost::shared_ptr<constrained_ik::MasterIK> getSolver(std::string group_name);

  private:
    /**
     * @brief Preform position and orientation interpolation between start and stop.
     * @param start begining pose of trajectory
     * @param stop end pose of trajectory
     * @param ds max cartesian translation interpolation step
     * @param dt max cartesian orientation interpolation step
     * @return std::vector<Eigen::Affine3d>
     */
    std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> >
    interpolateCartesian(const Eigen::Affine3d& start, const Eigen::Affine3d& stop, double ds, double dt) const;

    boost::atomic<bool> terminate_; /**< Termination flag */
    std::string robot_description_; /**< robot description value from ros param server */
    robot_model::RobotModelConstPtr robot_model_; /**< Robot model object */
    std::map<std::string, robot_model::JointModelGroup*> joint_model_groups_; /**< Joint model group object */
    std::vector<std::string> groups_; /**< List of available move groups */
    std::map<std::string, boost::shared_ptr<constrained_ik::MasterIK> > solvers_; /**< Constrained IK Solver */
    boost::mutex mutex_; /**< Mutex */
  };
} //namespace constrained_ik

#endif // CARTESIAN_PLANNER_H
