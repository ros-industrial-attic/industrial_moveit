/**
 * @file constrained_ik_planner_plugin.cpp
 * @brief This class represents the CLIK planner plugin for moveit.  It manages all of the CLIK planners.
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
#include <constrained_ik/moveit_interface/constrained_ik_planner_plugin.h>
#include <class_loader/class_loader.h>

const std::string JOINT_INTERP_PLANNER = "JointInterpolation"; /**< Joint interpolation planner name */
const std::string CARTESIAN_PLANNER = "Cartesian"; /**< Cartesian plannner name */

namespace constrained_ik
{
  bool CLIKPlannerManager::initialize(const robot_model::RobotModelConstPtr &model, const std::string &ns)
  {
    if (!ns.empty())
      nh_ = ros::NodeHandle(ns);

    std::string clik_mannager_param = "constrained_ik_planner";
    std::string clik_solver_param = "constrained_ik_solver";

    for (auto & group_name : model->getJointModelGroupNames())
    {
      //Create a cartesian planner for group
      if (model->getJointModelGroup(group_name)->isChain())
      {
        planners_.insert(std::make_pair(std::make_pair(CARTESIAN_PLANNER, group_name), planning_interface::PlanningContextPtr(new CartesianPlanner(CARTESIAN_PLANNER, group_name, nh_))));

        CartesianDynReconfigServerPtr drs(new CartesianDynReconfigServer(mutex_, ros::NodeHandle(nh_, clik_solver_param + "/" + group_name)));
        drs->setCallback(boost::bind(&CLIKPlannerManager::cartesianDynamicReconfigureCallback, this, _1, _2, group_name));
        cartesian_dynamic_reconfigure_server_.insert(std::make_pair(group_name, drs));

        //Create a joint interpolated planner for group
        planners_.insert(std::make_pair(std::make_pair(JOINT_INTERP_PLANNER, group_name), planning_interface::PlanningContextPtr(new JointInterpolationPlanner(JOINT_INTERP_PLANNER, group_name))));
      }
    }

    dynamic_reconfigure_server_.reset(new ManagerDynReconfigServer(mutex_, ros::NodeHandle(nh_, clik_mannager_param)));
    dynamic_reconfigure_server_->setCallback(boost::bind(&CLIKPlannerManager::managerDynamicReconfigureCallback, this, _1, _2));

    return true;
  }

  void CLIKPlannerManager::getPlanningAlgorithms(std::vector<std::string> &algs) const
  {
    algs.clear();
    algs.push_back(JOINT_INTERP_PLANNER);
    algs.push_back(CARTESIAN_PLANNER);
  }

  void CLIKPlannerManager::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pcs)
  {
    std::cout << "Entered setPlannerConfigurations" << std::endl;
  }

  planning_interface::PlanningContextPtr CLIKPlannerManager::getPlanningContext(const planning_scene::PlanningSceneConstPtr &planning_scene, const planning_interface::MotionPlanRequest &req, moveit_msgs::MoveItErrorCodes &error_code) const
  {
    planning_interface::PlanningContextPtr planner;

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
    std::map<std::pair<std::string, std::string>, planning_interface::PlanningContextPtr>::const_iterator it;
    if (req.planner_id.empty())
      it = planners_.find(std::make_pair(JOINT_INTERP_PLANNER, req.group_name));
    else
      it = planners_.find(std::make_pair(req.planner_id, req.group_name));

    if (it == planners_.end())
    {
      logError("No planner for specified group");
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return planning_interface::PlanningContextPtr();
    }
    planner = it->second;

    // Setup Planner
    planner->clear();
    planner->setPlanningScene(planning_scene);
    planner->setMotionPlanRequest(req);

    // Return Planner
    return planner;
  }

  void CLIKPlannerManager::managerDynamicReconfigureCallback(CLIKPlannerDynamicConfig &config, uint32_t level)
  {
    typedef std::map<std::pair<std::string, std::string>, planning_interface::PlanningContextPtr>::const_iterator it_type;

    config_ = config;
    for (it_type it = planners_.begin(); it != planners_.end(); it++)
    {
      if (it->second->getName() == JOINT_INTERP_PLANNER)
      {
        std::shared_ptr<JointInterpolationPlanner> planner = std::static_pointer_cast<JointInterpolationPlanner>(it->second);
        planner->setPlannerConfiguration(config_.joint_discretization_step, config_.debug_mode);
      }

      if (it->second->getName() == CARTESIAN_PLANNER)
      {
        std::shared_ptr<CartesianPlanner> planner = std::static_pointer_cast<CartesianPlanner>(it->second);
        planner->setPlannerConfiguration(config_.translational_discretization_step, config_.orientational_discretization_step, config_.debug_mode);
      }

    }

  }

  void CLIKPlannerManager::cartesianDynamicReconfigureCallback(CLIKDynamicConfig &config, uint32_t level, std::string group_name)
  {
    // process configuration parameters and fix errors/limits
    validateConstrainedIKConfiguration<CLIKDynamicConfig>(config);

    typedef std::map<std::pair<std::string, std::string>, planning_interface::PlanningContextPtr>::const_iterator it_type;
    it_type it = planners_.find(std::make_pair(CARTESIAN_PLANNER, group_name));
    if (it != planners_.end())
    {
      ConstrainedIKConfiguration new_config = convertToConstrainedIKConfiguration(config);
      std::shared_ptr<CartesianPlanner> planner = std::static_pointer_cast<CartesianPlanner>(it->second);
      planner->setSolverConfiguration(new_config);
    }
  }

} //namespace constrained_ik
CLASS_LOADER_REGISTER_CLASS(constrained_ik::CLIKPlannerManager, planning_interface::PlannerManager)
