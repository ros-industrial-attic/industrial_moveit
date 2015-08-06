/**
* @file constrained_ik_planner_plugin.cpp
* @brief This class represents the CLIK planner plugin for moveit.  It manages all of the CLIK planners.
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
#include <constrained_ik/moveit_interface/constrained_ik_planner_plugin.h>
#include <class_loader/class_loader.h>

const std::string JOINT_INTERP_PLANNER = "JointInterpolation";
const std::string CARTESIAN_PLANNER = "Cartesian";

namespace constrained_ik
{
  bool CLIKPlannerManager::initialize(const robot_model::RobotModelConstPtr &model, const std::string &ns)
  {
    if (!ns.empty())
      nh_ = ros::NodeHandle(ns);

    // Create map of planners
    planners_.insert(std::make_pair(JOINT_INTERP_PLANNER, new JointInterpolationPlanner("", "")));
    planners_.insert(std::make_pair(CARTESIAN_PLANNER, new CartesianPlanner("", "")));

    dynamic_reconfigure_server_.reset(new dynamic_reconfigure::Server<ConstrainedIKPlannerDynamicReconfigureConfig>(mutex_, ros::NodeHandle(nh_, "constrained_ik_planner")));
    dynamic_reconfigure_server_->setCallback(boost::bind(&CLIKPlannerManager::dynamicReconfigureCallback, this, _1, _2));

    return true;
  }

  void CLIKPlannerManager::getPlanningAlgorithms(std::vector<std::string> &algs) const
  {
    algs.clear();

    for(std::map<std::string, constrained_ik::CLIKPlanningContextPtr>::const_iterator iter = planners_.begin(); iter != planners_.end(); ++iter)
    {
      algs.push_back(iter->first);
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

    // Get planner
    std::map<std::string, constrained_ik::CLIKPlanningContextPtr>::const_iterator it = planners_.find(req.planner_id);
    if (it != planners_.end())
    {
      planner = it->second;
    }
    else
    {
      planner = planners_.find(CARTESIAN_PLANNER)->second;
    }

    // Setup Planner
    planner->clear();
    planner->setPlanningScene(planning_scene);
    planner->setMotionPlanRequest(req);

    // Return Planner
    return planner;
  }

  void CLIKPlannerManager::dynamicReconfigureCallback(ConstrainedIKPlannerDynamicReconfigureConfig &config, uint32_t level)
  {
    typedef std::map< std::string, constrained_ik::CLIKPlanningContextPtr>::const_iterator it_type;

    config_ = config;
    for (it_type it = planners_.begin(); it != planners_.end(); it++)
    {
      it->second->setConfiguration(config_);
    }

  }

} //namespace constrained_ik
CLASS_LOADER_REGISTER_CLASS(constrained_ik::CLIKPlannerManager, planning_interface::PlannerManager)
