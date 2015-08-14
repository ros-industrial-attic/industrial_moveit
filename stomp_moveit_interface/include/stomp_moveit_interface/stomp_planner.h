/*
 * stomp_planner.h
 *
 *  Created on: Jan 22, 2013
 *      Author: kalakris
 */

#ifndef STOMP_PLANNER_H_
#define STOMP_PLANNER_H_

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_distance_field/collision_robot_distance_field.h>
#include <moveit/collision_distance_field/collision_world_distance_field.h>
#include <stomp/stomp.h>
#include <boost/thread.hpp>
#include <ros/ros.h>

namespace stomp_moveit_interface
{

class StompPlanner: public planning_interface::PlanningContext
{
public:
  StompPlanner(const std::string& group,const moveit::core::RobotModelConstPtr& model);
  virtual ~StompPlanner();

  /// Get a short string that identifies the planning interface
  virtual std::string getDescription(void) const { return "STOMP"; }

  /// Subclass must implement methods below
  virtual bool solve(planning_interface::MotionPlanResponse &res);

  virtual bool solve(planning_interface::MotionPlanDetailedResponse &res);

  /// Determine whether this plugin instance is able to represent this planning request
  virtual bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req)  const;

  /// Request termination, if a solve() function is currently computing plans
  virtual bool terminate() ;

  virtual void clear();

protected:

  virtual void init(const moveit::core::RobotModelConstPtr& model);

  // thread save methods
  bool getSolving();
  void setSolving(bool solve);

  void updateCollisionModels(planning_scene::PlanningSceneConstPtr& current_planning_scene);

protected:
  ros::NodeHandle node_handle_;

  boost::shared_ptr<stomp::STOMP> stomp_;

  bool solving_;
  boost::mutex solving_mutex_;

  ros::Publisher trajectory_viz_pub_;
  ros::Publisher robot_body_viz_pub_;
  moveit::core::RobotModelConstPtr kinematic_model_;

  // distance field params
  Eigen::Vector3d df_size_;
  Eigen::Vector3d df_origin_;
  double df_resolution_;
  double df_collision_tolerance_;
  double df_max_propagation_distance_;

  // planning scene management
  planning_scene::PlanningScenePtr last_planning_scene_;

  // collision models
  boost::shared_ptr<collision_detection::CollisionWorldDistanceField> collision_world_df_;
  boost::shared_ptr<collision_detection::CollisionRobotDistanceField> collision_robot_df_;

  void copyObjects(const boost::shared_ptr<const collision_detection::CollisionWorld>& from_world,
                   const boost::shared_ptr<collision_detection::CollisionWorld>& to_world) const;

};

} /* namespace stomp_moveit_interface */
#endif /* STOMP_PLANNER_H_ */
