/*
 * constrained_ik_planner_plugin.h
 *
 *  Created on: May 4, 2015
 *      Author: Levi Armstrong
 */
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

#ifndef CONSTRAINED_IK_PLANNER_PLUGIN_H
#define CONSTRAINED_IK_PLANNER_PLUGIN_H
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <class_loader/class_loader.h>
#include <ros/ros.h>
#include <constrained_ik/joint_interpolation_planner.h>
#include <constrained_ik/cartesian_planner.h>
#include <constrained_ik/constrained_ik_planner_parameters.h>

namespace constrained_ik
{
  /**
   * @brief This class represents the CLIK planner plugin for moveit.  It
   * manages all of the CLIK planners.
   */
  class CLIKPlannerManager : public planning_interface::PlannerManager
  {
  public:
    CLIKPlannerManager() : planning_interface::PlannerManager(),
                           nh_("~")
    {
    }

    bool initialize(const robot_model::RobotModelConstPtr &model, const std::string &ns);

    bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const {return req.trajectory_constraints.constraints.empty();}

    std::string getDescription() const { return "CLIK"; }

    void getPlanningAlgorithms(std::vector<std::string> &algs) const;

    void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pcs);

    planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr &planning_scene, const planning_interface::MotionPlanRequest &req, moveit_msgs::MoveItErrorCodes &error_code) const;

    /**
     * @brief This loads the planner parameters from the ROS paramter server.
     * @return CLIKParameters
     */
    CLIKParameters loadParameters() const
    {
      CLIKParameters param;
      param.load(nh_);
      return param;
    }

  protected:
    ros::NodeHandle nh_;

    /**
     * @brief Containes all the availble CLIK planners
     */
    std::map< std::string, constrained_ik::CLIKPlanningContextPtr> planners_;
  };
} //namespace constrained_ik
#endif //CONSTRAINED_IK_PLANNER_PLUGIN_H
