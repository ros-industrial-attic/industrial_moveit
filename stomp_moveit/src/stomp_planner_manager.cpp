/*
 * stomp_planner_manager.cpp
 *
 *  Created on: April 5, 2016
 *      Author: Jorge Nicho
 */

#include <class_loader/class_loader.h>
#include <stomp_moveit/stomp_planner_manager.h>
#include <stomp_moveit/stomp_planner.h>

namespace stomp_moveit
{

StompPlannerManager::StompPlannerManager():
    planning_interface::PlannerManager(),
    nh_("~")
{

}

StompPlannerManager::~StompPlannerManager()
{

}

bool StompPlannerManager::initialize(const robot_model::RobotModelConstPtr &model, const std::string &ns)
{
  if (!ns.empty())
  {
    nh_ = ros::NodeHandle(ns);
  }

  // each element under 'stomp' should be a group name
  std::map<std::string, XmlRpc::XmlRpcValue> group_config;

  if (!StompPlanner::getConfigData(nh_, group_config))
    return false;

  for(std::map<std::string, XmlRpc::XmlRpcValue>::iterator v = group_config.begin(); v != group_config.end(); v++)
  {
    if(!model->hasJointModelGroup(v->first))
    {
      ROS_ERROR("The robot model does not support the planning group '%s' listed by in the stomp configuration",
                v->first.c_str());
      return false;
    }

    boost::shared_ptr<StompPlanner> planner(new StompPlanner(v->first, v->second, model));
    planners_.insert(std::make_pair(v->first, planner));
  }

  return true;
}

bool StompPlannerManager::canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
{
  if(planners_.count(req.group_name) == 0)
  {
    return false;
  }

  // Get planner
  boost::shared_ptr<StompPlanner> planner = boost::static_pointer_cast<StompPlanner>(planners_.at(req.group_name));
  return planner->canServiceRequest(req);
}

void StompPlannerManager::getPlanningAlgorithms(std::vector<std::string> &algs) const
{
  algs.clear();
  if(!planners_.empty())
  {
    algs.push_back(planners_.begin()->second->getName());
  }
}

void StompPlannerManager::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pcs)
{
  ROS_WARN_STREAM("The "<<__FUNCTION__<<" method is not applicable");
}

planning_interface::PlanningContextPtr StompPlannerManager::getPlanningContext(const planning_scene::PlanningSceneConstPtr &planning_scene,
                                                                               const planning_interface::MotionPlanRequest &req,
                                                                               moveit_msgs::MoveItErrorCodes &error_code) const
{
  error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  if (req.group_name.empty())
  {
    ROS_ERROR("No group specified to plan for");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return planning_interface::PlanningContextPtr();
  }

  if (!planning_scene)
  {
    ROS_ERROR("No planning scene supplied as input");
    error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return planning_interface::PlanningContextPtr();
  }

  if(planners_.count(req.group_name) <=0)
  {
    ROS_ERROR("STOMP does not have a planning context for group %s",req.group_name.c_str());
    error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return planning_interface::PlanningContextPtr();
  }

  // Get planner
  boost::shared_ptr<StompPlanner> planner = boost::static_pointer_cast<StompPlanner>(planners_.at(req.group_name));

  if(!planner->canServiceRequest(req))
  {
    error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return planning_interface::PlanningContextPtr();
  }

  // Setup Planner
  planner->clear();
  planner->setPlanningScene(planning_scene);
  planner->setMotionPlanRequest(req);

  // Return Planner
  return planner;
}


} /* namespace stomp_moveit_interface */
CLASS_LOADER_REGISTER_CLASS(stomp_moveit::StompPlannerManager, planning_interface::PlannerManager)
