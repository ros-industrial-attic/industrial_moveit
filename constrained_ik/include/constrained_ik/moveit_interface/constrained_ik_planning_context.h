/**
 * @file constrained_ik_planning_context.h
 * @brief This is a virtual class that is inherited by all of the CLIK planner.
 *
 * It containes CLIK specific functions used by each of the CLIK planners.
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
#ifndef CONSTRAINED_IK_PLANNING_CONTEXT_H
#define CONSTRAINED_IK_PLANNING_CONTEXT_H

#include <moveit/planning_interface/planning_interface.h>
#include <constrained_ik/moveit_interface/constrained_ik_planner_parameters.h>
#include <constrained_ik/CLIKPlannerDynamicConfig.h>

namespace constrained_ik
{
  /**
   * @brief This is a virtual class that is inherited by all of the CLIK planner. It
   * containes CLIK specific functions used by each of the CLIK planners.
   */
  class CLIKPlanningContext : public planning_interface::PlanningContext
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief CartesianPlanner Constructor
     * @param name of planner
     * @param group of the planner
     */
    CLIKPlanningContext(const std::string &name, const std::string &group) : planning_interface::PlanningContext(name, group)  {}

    /** @brief Clear the planner data */
    void clear() override = 0;

    /**
     * @brief Terminate the active planner solve
     * @return wether successfull
     */
    bool terminate() override = 0;

    /**
     * @brief Solve for the provided request
     * @param res planner response
     * @return wether successfull
     */
    bool solve(planning_interface::MotionPlanResponse &res) override = 0;

    /**
     * @brief Solve for the provided request
     * @param res planner detailed response
     * @return wether successfull
     */
    bool solve(planning_interface::MotionPlanDetailedResponse &res) override = 0;

    /**
     * @brief setConfiguration - Sets/Updates planner parameters.
     * @param config - Parameters used by the CLIK planners
     */
    virtual void setPlannerConfiguration(const CLIKPlannerDynamicConfig &config) { config_ = config; }

    /** @brief resetConfiguration - Resets configuration parameters to their default values. */
    virtual void resetPlannerConfiguration() { config_.__getDefault__(); }

  protected:
    CLIKPlannerDynamicConfig config_; /**< Store the parameters for the CLIK planners. */


  };
typedef boost::shared_ptr<CLIKPlanningContext> CLIKPlanningContextPtr; /**< Typedef for CLIKPlanning Context boost shared ptr */
} //namespace constrained_ik

#endif // CONSTRAINED_IK_PLANNING_CONTEXT_H
