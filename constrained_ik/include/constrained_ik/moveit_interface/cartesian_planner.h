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
#ifndef CARTESIAN_PLANNER_H
#define CARTESIAN_PLANNER_H

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <constrained_ik/moveit_interface/constrained_ik_planning_context.h>
#include <boost/atomic.hpp>
#include <pluginlib/class_loader.h>

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

    CartesianPlanner(const std::string &name, const std::string &group) : constrained_ik::CLIKPlanningContext(name, group), terminate_(false), robot_description_("robot_description"), initialized_(false) {}

    CartesianPlanner(const CartesianPlanner &other) : constrained_ik::CLIKPlanningContext(other), terminate_(false), robot_description_("robot_description"), initialized_(false) {}

    void initialize();

    void clear() { terminate_ = false; }

    bool terminate()
    {
      terminate_ = true;
      return true;
    }

    bool solve(planning_interface::MotionPlanResponse &res);

    /**
     * \brief Helper function to decide which, and how many, tip frames a planning group has
     * \param jmg - joint model group pointer
     * \return tips - list of valid links in a planning group to plan for
     */
    std::vector<std::string> chooseTipFrames(const robot_model::JointModelGroup *jmg);

    boost::shared_ptr<kinematics::KinematicsBase> allocKinematicsSolver(const robot_model::JointModelGroup *jmg);

    bool getConstrainedIKSolverData();

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

    bool initialized_;
    boost::atomic<bool> terminate_;
    std::string robot_description_;
    boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase> > kinematics_loader_;
    robot_model::RobotModelConstPtr robot_model_;
    robot_model::SolverAllocatorFn solver_allocator_;
    std::map<std::string, robot_model::JointModelGroup*> joint_model_groups_;
    std::vector<std::string> groups_;
    std::map<std::string, double> ik_timeout_;
    std::map<std::string, unsigned int> ik_attempts_;
    std::map<std::string, std::vector<std::string> > possible_kinematics_solvers_;
    std::map<std::string, std::vector<double> > search_res_;
    std::map<std::string, std::vector<std::string> > iksolver_to_tip_links_;  // a map between each ik solver and a vector of custom-specified tip link(s)
    boost::mutex mutex_;
  };
} //namespace constrained_ik

#endif // CARTESIAN_PLANNER_H
