/**
 * @file joint_interpolation_planner.h
 * @brief Joint interpolation planner for moveit.
 *
 * This class is used to represent a joint interpolated path planner for
 * moveit.  This planner does not have the inherent ability to avoid
 * collision. It does check if the path created is collision free before it
 * returns a trajectory.  If a collision is found it returns an empty
 * trajectory and moveit error.
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
#ifndef JOINT_INTERPOLATION_PLANNER_H
#define JOINT_INTERPOLATION_PLANNER_H

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <boost/atomic.hpp>

namespace constrained_ik
{

  const double DEFAULT_JOINT_DISCRETIZATION_STEP = 0.02;

  /**
   * @brief Joint interpolation planner for moveit.
   *
   * This class is used to represent a joint interpolated path planner for
   * moveit.  This planner does not have the inherent ability to avoid
   * collision. It does check if the path created is collision free before it
   * returns a trajectory.  If a collision is found it returns an empty
   * trajectory and moveit error.
   */
  class JointInterpolationPlanner : public planning_interface::PlanningContext
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief JointInterpolationPlanner Constructor
     * @param name of planner
     * @param group of the planner
     */
    JointInterpolationPlanner(const std::string &name, const std::string &group) : planning_interface::PlanningContext(name, group), terminate_(false) { resetPlannerConfiguration(); }

    /**
     * @brief JointInterpolationPlanner Copy Constructor
     * @param other joint interpolation planner
     */
    JointInterpolationPlanner(const JointInterpolationPlanner &other) : planning_interface::PlanningContext(other), terminate_(false) {}

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
     * @brief Generate a joint interpolated trajectory
     * @param res planner response
     * @return
     */
    bool solve(planning_interface::MotionPlanResponse &res) override;

    /**
     * @brief Generate a joint interpolated trajectory
     * @param res planner detailed response
     * @return
     */
    bool solve(planning_interface::MotionPlanDetailedResponse &res) override;

    /**
     * @brief Set the planners configuration
     * @param joint_discretization_step Max joint discretization step
     * @param debug_mode Set debug state
     */
    void setPlannerConfiguration(double joint_discretization_step, bool debug_mode = false);

    /** @brief Reset the planners configuration to it default settings */
    void resetPlannerConfiguration();

  private:
    double joint_discretization_step_; /**< Joint discretization step */
    bool debug_mode_;                  /**< Debug state */
    boost::atomic<bool> terminate_;    /**< Termination flag */
  };
} //namespace constrained_ik

#endif // JOINT_INTERPOLATION_PLANNER_H
