/*
 * stomp_cost_feature.h
 *
 *  Created on: Aug 31, 2012
 *      Author: kalakris
 */

#ifndef STOMP_COST_FEATURE_H_
#define STOMP_COST_FEATURE_H_

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection/collision_world.h>
#include <stomp_moveit_interface/stomp_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/MotionPlanRequest.h>

namespace stomp_moveit_interface
{

class StompCostFeature
{
public:
  StompCostFeature(){};
  virtual ~StompCostFeature(){};

  virtual bool initialize(XmlRpc::XmlRpcValue& config,
                  int num_threads,
                  const std::string& group_name,
                  planning_scene::PlanningSceneConstPtr planning_scene);

  virtual void setPlanningScene(planning_scene::PlanningSceneConstPtr planning_scene);

  virtual int getNumValues() const = 0;
  virtual void computeValuesAndGradients(const boost::shared_ptr<StompTrajectory const>& trajectory,
                                         Eigen::MatrixXd& feature_values,         // num_time_steps x num_features
                                         bool compute_gradients,
                                         std::vector<Eigen::MatrixXd>& gradients, // [num_features] num_joints x num_time_steps
                                         std::vector<int>& validities,             // [num_time_steps] each state valid or not
                                         int thread_id,
                                         int start_timestep,                      // start timestep
                                         int num_time_steps) const = 0;
  virtual std::string getName() const = 0;
  virtual void getNames(std::vector<std::string>& names) const = 0;



protected:

  void initOutputs(const boost::shared_ptr<StompTrajectory const>& trajectory,
                   Eigen::MatrixXd& feature_values,
                   bool compute_gradients,
                   std::vector<Eigen::MatrixXd>& gradients,
                   std::vector<int>& validities) const;

  moveit::core::RobotModelConstPtr kinematic_model_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  std::string group_name_;
  int num_threads_;

};

} /* namespace stomp_moveit_interface */
#endif /* STOMP_COST_FEATURE_H_ */
