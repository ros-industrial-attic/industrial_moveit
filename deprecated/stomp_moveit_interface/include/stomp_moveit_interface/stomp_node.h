/*
 * stomp_node.h
 *
 *  Created on: Jul 21, 2012
 *      Author: kalakris
 */

#ifndef STOMP_NODE_H_
#define STOMP_NODE_H_

#include <arm_navigation_msgs/GetMotionPlan.h>
#include <planning_environment/models/collision_models_interface.h>
#include <stomp/stomp.h>
#include <stomp_ros_interface/stomp_optimization_task.h>

namespace stomp_ros_interface
{

class StompNode
{
public:
  StompNode();
  virtual ~StompNode();

  bool run();

  bool plan(arm_navigation_msgs::GetMotionPlan::Request& request,
            arm_navigation_msgs::GetMotionPlan::Response& response);

private:
  ros::NodeHandle node_handle_;
  boost::shared_ptr<planning_environment::CollisionModelsInterface> collision_models_interface_;

  ros::ServiceServer plan_path_service_;

  boost::shared_ptr<stomp::STOMP> stomp_;
  std::map<std::string, boost::shared_ptr<StompOptimizationTask> > stomp_tasks_;
  ros::Publisher rviz_trajectory_pub_;

};

} /* namespace stomp */
#endif /* STOMP_NODE_H_ */
