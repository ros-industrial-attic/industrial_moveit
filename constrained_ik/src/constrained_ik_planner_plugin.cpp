/*
 * constrained_ik_planner_plugin.cpp
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
#include <constrained_ik/constrained_ik_planner_plugin.h>
#include <class_loader/class_loader.h>

namespace constrained_ik
{
  bool CLIKPlannerManager::initialize(const robot_model::RobotModelConstPtr &model, const std::string &ns)
  {
    if (!ns.empty())
      nh_ = ros::NodeHandle(ns);

    // Create map of planners
    planners_.insert(std::make_pair("Default", new CartesianPlanner("", "")));
    planners_.insert(std::make_pair("JointInterpolation", new JointInterpolationPlanner("", "")));
    planners_.insert(std::make_pair("Cartesian", new CartesianPlanner("", "")));

    return true;
  }

  void CLIKPlannerManager::getPlanningAlgorithms(std::vector<std::string> &algs) const
  {
    algs.clear();

    for(std::map<std::string, constrained_ik::CLIKPlanningContextPtr>::const_iterator iter = planners_.begin(); iter != planners_.end(); ++iter)
    {
      if (iter->first != "Default"){ algs.push_back(iter->first); }
    }

  }

  void CLIKPlannerManager::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pcs)
  {
    std::cout << "Entered setPlannerConfigurations" << std::endl;
  }

  planning_interface::PlanningContextPtr CLIKPlannerManager::getPlanningContext(const planning_scene::PlanningSceneConstPtr &planning_scene, const planning_interface::MotionPlanRequest &req, moveit_msgs::MoveItErrorCodes &error_code) const
  {
    constrained_ik::CLIKPlanningContextPtr planner;

    if (req.group_name.empty())
    {
      logError("No group specified to plan for");
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return planning_interface::PlanningContextPtr();
    }

    error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;

    if (!planning_scene)
    {
      logError("No planning scene supplied as input");
      return planning_interface::PlanningContextPtr();
    }

    // Load Parameters from ROS Parameter Server
    CLIKParameters params = loadParameters();

    // Get planner
    std::map<std::string, constrained_ik::CLIKPlanningContextPtr>::const_iterator it = planners_.find(req.planner_id);
    if (it != planners_.end()) { planner = it->second; }
    else { planner = planners_.find("Default")->second; }

    // Setup Planner
    planner->clear();
    planner->setPlanningScene(planning_scene);
    planner->setMotionPlanRequest(req);
    planner->setParameters(params);

    // Return Planner
    return planner;
  }
} //namespace constrained_ik
CLASS_LOADER_REGISTER_CLASS(constrained_ik::CLIKPlannerManager, planning_interface::PlannerManager)
