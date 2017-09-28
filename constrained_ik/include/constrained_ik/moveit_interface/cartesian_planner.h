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
#include <boost/atomic.hpp>
#include <constrained_ik/constrained_ik.h>

namespace constrained_ik
{
  const double DEFAULT_TRANSLATIONAL_DISCRETIZATION_STEP = 0.01;
  const double DEFAULT_ORIENTATIONAL_DISCRETIZATION_STEP = 0.01;

  /**
  * @brief Cartesian path planner for moveit.
  *
  * This class is used to represent a cartesian path planner for moveit.
  * It finds a straight line path between the start and goal position. This
  * planner does not have the inherent ability to avoid collision. It does
  * check if the path created is collision free before it returns a trajectory.
  * If a collision is found it returns an empty trajectory and moveit error.
   */
  class CartesianPlanner : public planning_interface::PlanningContext
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief CartesianPlanner Constructor
     * @param name of planner
     * @param group of the planner
     */
    CartesianPlanner(const std::string &name, const std::string &group, const ros::NodeHandle &nh);

    /**
     * @brief CartesianPlanner Copy Constructor
     * @param other cartesian planner
     */
    CartesianPlanner(const CartesianPlanner &other) : planning_interface::PlanningContext(other),
      terminate_(false),
      robot_description_(robot_description_),
      solver_(solver_) {}

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
     * @brief This will initialize the solver if it has not all ready
     * @return True if successful, otherwise false
     */
    bool initializeSolver();

    /**
     * @brief Set the planners configuration
     * @param translational_discretization_step Max translational discretization step
     * @param orientational_discretization_step Max orientational discretization step
     * @param debug_mode Set debug state
     */
    void setPlannerConfiguration(double translational_discretization_step, double orientational_discretization_step, bool debug_mode = false);

    /** @brief Reset the planners configuration to it default settings */
    void resetPlannerConfiguration();

    /**
     * @brief Set the planners IK solver configuration
     * @param config Constrained IK solver configuration
     */
    void setSolverConfiguration(const ConstrainedIKConfiguration &config);

    /** @brief Reset the planners IK solver configuration to it default settings */
    void resetSolverConfiguration();

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

    double translational_discretization_step_;    /**< Max translational discretization step */
    double orientational_discretization_step_;    /**< Max orientational discretization step */
    bool debug_mode_;                             /**< Debug state */
    boost::atomic<bool> terminate_;               /**< Termination flag */
    std::string robot_description_;               /**< robot description value from ros param server */
    robot_model::RobotModelConstPtr robot_model_; /**< Robot model object */
    boost::shared_ptr<Constrained_IK> solver_;    /**< Constrained IK Solver */
    boost::mutex mutex_;                          /**< Mutex */
  };
} //namespace constrained_ik

#endif // CARTESIAN_PLANNER_H
