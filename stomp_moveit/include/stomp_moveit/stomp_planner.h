/*
 * stomp_planner.h
 *
 *  Created on: April 4, 2016
 *      Author: Jorge Nicho
 */

#ifndef STOMP_MOVEIT_STOMP_PLANNER_H_
#define STOMP_MOVEIT_STOMP_PLANNER_H_

#include <moveit/planning_interface/planning_interface.h>
#include <stomp_core/stomp.h>
#include <stomp_moveit/stomp_optimization_task.h>
#include <boost/thread.hpp>
#include <ros/ros.h>

namespace stomp_moveit
{

using StompOptimizationTaskPtr = boost::shared_ptr<StompOptimizationTask>;

class StompPlanner: public planning_interface::PlanningContext
{
public:
  StompPlanner(const std::string& group,const XmlRpc::XmlRpcValue& config,const moveit::core::RobotModelConstPtr& model);
  virtual ~StompPlanner();

  /**
   * @brief Solve the motion planning problem and store the result in \e res
   */
  virtual bool solve(planning_interface::MotionPlanResponse &res) override;

  /**
   * @brief Solve the motion planning problem and store the detailed result in \e res
   */
  virtual bool solve(planning_interface::MotionPlanDetailedResponse &res) override;

  /**
   * @brief Request termination, if a solve() function is currently computing plans
   */
  virtual bool terminate() override;

  /**
   * @brief Clears results from previous plan
   */
  virtual void clear() override;

  static bool getConfigData(ros::NodeHandle &nh, std::map<std::string, XmlRpc::XmlRpcValue> &config, std::string param = std::string("stomp"));

  /**
   * @brief Determine whether this plugin instance is able to represent this planning request
   */
  bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req)  const;

  /**
   * @brief On the next call to 'solve()', the trajectory provided here will be passed
   *        along to the underlying planner and then will be cleared.
   */
  bool setInitialTrajectory(const trajectory_msgs::JointTrajectory& initial_traj);

protected:

  void setup();
  bool getStartAndGoal(std::vector<double>& start, std::vector<double>& goal);

  bool parametersToJointTrajectory(Eigen::MatrixXd& parameters, trajectory_msgs::JointTrajectory& traj);
  bool jointTrajectorytoParameters(const trajectory_msgs::JointTrajectory& traj, Eigen::MatrixXd& parameters) const;

  trajectory_msgs::JointTrajectory resample(const trajectory_msgs::JointTrajectory& other) const;

protected:

  // stomp optimization
  boost::shared_ptr< stomp_core::Stomp> stomp_;
  StompOptimizationTaskPtr task_;
  XmlRpc::XmlRpcValue config_;
  stomp_core::StompConfiguration stomp_config_;

  // robot environment
  moveit::core::RobotModelConstPtr robot_model_;

  // For seeding the optimization
  bool seed_provided_;
  trajectory_msgs::JointTrajectory seed_traj_;

};

} /* namespace stomp_moveit */
#endif /* STOMP_MOVEIT_STOMP_PLANNER_H_ */
