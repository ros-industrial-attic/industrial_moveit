/**
 * @file constrained_ik_planning_context.h
 * @brief This is a virtual class that is inherited by all of the CLIK planner.
 * It containes CLIK specific functions used by each of the CLIK planners.
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
#ifndef CONSTRAINED_IK_PLANNING_CONTEXT_H
#define CONSTRAINED_IK_PLANNING_CONTEXT_H

#include <moveit/planning_interface/planning_interface.h>
#include <constrained_ik/constrained_ik_planner_parameters.h>

namespace constrained_ik
{
  /**
   * @brief This is a virtual class that is inherited by all of the CLIK planner. It
   * containes CLIK specific functions used by each of the CLIK planners.
   */
  class CLIKPlanningContext : public planning_interface::PlanningContext
  {
  public:
    CLIKPlanningContext(const std::string &name, const std::string &group) : planning_interface::PlanningContext(name, group) {}

    virtual void clear() = 0;

    virtual bool terminate() = 0;

    virtual bool solve(planning_interface::MotionPlanResponse &res) = 0;

    bool solve(planning_interface::MotionPlanDetailedResponse &res)
    {
      std::cout << "Entered TESTPlanner.solve detailed" << std::endl;
      return false;
    }
    /**
     * @brief setParameters - Sets/Updates planner parameters.
     * @param params - Parameters used by the CLIK planners
     */
    void setParameters(const CLIKParameters &params) { params_ = params; }

  protected:
    /** Store the parameters for the CLIK planners. */
    CLIKParameters params_;

  };
typedef boost::shared_ptr<CLIKPlanningContext> CLIKPlanningContextPtr;
} //namespace constrained_ik

#endif // CONSTRAINED_IK_PLANNING_CONTEXT_H
