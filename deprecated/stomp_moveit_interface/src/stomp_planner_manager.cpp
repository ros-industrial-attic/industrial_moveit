/*
 * stomp_planner_manager.cpp
 *
 *  Created on: Jun 10, 2015
 *      Author: rosindustrial
 */

#include <stomp_moveit_interface/stomp_planner_manager.h>
#include <stomp_moveit_interface/stomp_planner.h>

namespace stomp_moveit_interface
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
    nh_ = ros::NodeHandle(ns);

  // Create map of planners
  boost::shared_ptr<StompPlanner> stomp(new StompPlanner(std::string(""),model));

  planners_.insert(std::make_pair("STOMP", stomp));

  return true;
}

bool StompPlannerManager::canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
{
  return req.trajectory_constraints.constraints.empty() ;
}

void StompPlannerManager::getPlanningAlgorithms(std::vector<std::string> &algs) const
{
  algs.clear();
  std::map< std::string, planning_interface::PlanningContextPtr>::const_iterator plnr_iter;
  for(plnr_iter = planners_.begin(); plnr_iter != planners_.end(); plnr_iter++)
  {
    algs.push_back(plnr_iter->first);
  }
}

void StompPlannerManager::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pcs)
{
  ROS_WARN_STREAM("The "<<__FUNCTION__<<" is not implemented");
}

planning_interface::PlanningContextPtr StompPlannerManager::getPlanningContext(const planning_scene::PlanningSceneConstPtr &planning_scene,
                                                                               const planning_interface::MotionPlanRequest &req,
                                                                               moveit_msgs::MoveItErrorCodes &error_code) const
{

  if (req.group_name.empty())
  {
    ROS_ERROR("No group specified to plan for");
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
  planning_interface::PlanningContextPtr planner = planners_.begin()->second;

  // Setup Planner
  planner->clear();
  planner->setPlanningScene(planning_scene);
  planner->setMotionPlanRequest(req);

  // Return Planner
  return planner;
}


} /* namespace stomp_moveit_interface */
CLASS_LOADER_REGISTER_CLASS(stomp_moveit_interface::StompPlannerManager, planning_interface::PlannerManager)
