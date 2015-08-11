/*
 * stomp_planner_manager.h
 *
 *  Created on: Jun 10, 2015
 *      Author: Jorge Nicho
 */

#ifndef STOMP_MOVEIT_INTERFACE_STOMP_PLANNER_MANAGER_H_
#define STOMP_MOVEIT_INTERFACE_STOMP_PLANNER_MANAGER_H_

#include <moveit/planning_interface/planning_interface.h>
#include <ros/node_handle.h>
#include <class_loader/class_loader.h>

namespace stomp_moveit_interface
{

class StompPlannerManager : public planning_interface::PlannerManager
{
public:
  StompPlannerManager();
  virtual ~StompPlannerManager();

  bool initialize(const robot_model::RobotModelConstPtr& model, const std::string &ns);

  bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const ;

  std::string getDescription() const
  {
    return "STOMP_PLANNER_MANAGER";
  }

  void getPlanningAlgorithms(std::vector<std::string> &algs) const;

  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pcs);

  planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr &planning_scene,
                                                            const planning_interface::MotionPlanRequest &req, moveit_msgs::MoveItErrorCodes &error_code) const;



protected:
  ros::NodeHandle nh_;

  /** Containes all the availble CLIK planners */
  std::map< std::string, planning_interface::PlanningContextPtr> planners_;
};

} /* namespace stomp_moveit_interface */

#endif /* STOMP_MOVEIT_INTERFACE_STOMP_PLANNER_MANAGER_H_ */
