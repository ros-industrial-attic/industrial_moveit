/*
 * stomp_planner_manager.h
 *
 *  Created on: April 5, 2016
 *      Author: Jorge Nicho
 */

#ifndef STOMP_MOVEIT_STOMP_PLANNER_MANAGER_H_
#define STOMP_MOVEIT_STOMP_PLANNER_MANAGER_H_

#include <moveit/planning_interface/planning_interface.h>
#include <ros/node_handle.h>

namespace stomp_moveit
{

class StompPlannerManager : public planning_interface::PlannerManager
{
public:
  StompPlannerManager();
  virtual ~StompPlannerManager();

  bool initialize(const robot_model::RobotModelConstPtr& model, const std::string &ns) override;

  bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const override;

  std::string getDescription() const override
  {
    return "Stomp Planner";
  }

  void getPlanningAlgorithms(std::vector<std::string> &algs) const override;

  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pcs) override;

  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr &planning_scene,
      const planning_interface::MotionPlanRequest &req,
      moveit_msgs::MoveItErrorCodes &error_code) const override;


protected:
  ros::NodeHandle nh_;

  /** Contains the Stomp planners for each planning group */
  std::map< std::string, planning_interface::PlanningContextPtr> planners_;
};

} /* namespace stomp_moveit */

#endif /* STOMP_MOVEIT_STOMP_PLANNER_MANAGER_H_ */
