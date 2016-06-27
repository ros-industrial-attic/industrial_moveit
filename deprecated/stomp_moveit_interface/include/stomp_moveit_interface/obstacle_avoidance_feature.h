/*
 * obstacle_avoidance_feature.h
 *
 *  Created on: Oct 19, 2015
 *      Author: Jorge Nicho
 */

#ifndef STOMP_MOVEIT_INTERFACE_OBSTACLE_AVOIDANCE_FEATURE_H_
#define STOMP_MOVEIT_INTERFACE_OBSTACLE_AVOIDANCE_FEATURE_H_

#include <stomp_moveit_interface/cost_features/stomp_cost_feature.h>
#include <industrial_collision_detection/collision_robot_industrial.h>
#include <industrial_collision_detection/collision_world_industrial.h>

namespace stomp_moveit_interface
{

class ObstacleAvoidanceFeature : public StompCostFeature
{
public:
  ObstacleAvoidanceFeature();
  virtual ~ObstacleAvoidanceFeature();

  virtual bool initialize(XmlRpc::XmlRpcValue& config,
                          int num_threads,
                          const std::string& group_name,
                          planning_scene::PlanningSceneConstPtr planning_scene);

  virtual void setPlanningScene(planning_scene::PlanningSceneConstPtr planning_scene);

  virtual int getNumValues() const;
  virtual void computeValuesAndGradients(const boost::shared_ptr<StompTrajectory const>& trajectory,
                                         Eigen::MatrixXd& feature_values,         // num_time_steps x num_features
                                         bool compute_gradients,
                                         std::vector<Eigen::MatrixXd>& gradients, // [num_features] num_joints x num_time_steps
                                         std::vector<int>& validities,             // [num_time_steps] each state valid or not
                                         int thread_id,
                                         int start_timestep,                      // start timestep
                                         int num_time_steps) const;
  virtual std::string getName() const;
  virtual void getNames(std::vector<std::string>& names) const;

protected:

  bool loadParameters(XmlRpc::XmlRpcValue& config);

protected:

  collision_detection::CollisionRequest collision_request_;
  collision_detection::CollisionRobotIndustrialConstPtr collision_robot_;
  collision_detection::CollisionWorldIndustrialConstPtr collision_world_;

  // parameters
  double clearance_;


};

} /* namespace stomp_moveit_interface */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INTERFACE_INCLUDE_STOMP_MOVEIT_INTERFACE_OBSTACLE_AVOIDANCE_FEATURE_H_ */
