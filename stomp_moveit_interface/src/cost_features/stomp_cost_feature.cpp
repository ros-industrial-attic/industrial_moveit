/*
 * stomp_cost_feature.cpp
 *
 *  Created on: Jan 24, 2013
 *      Author: kalakris
 */

#include <stomp_moveit_interface/cost_features/stomp_cost_feature.h>

namespace stomp_moveit_interface
{

bool StompCostFeature::initialize(XmlRpc::XmlRpcValue& config,
                                  int num_threads,
                                  const std::string& group_name,
                                  moveit::core::RobotModelConstPtr kinematic_model,
                                  boost::shared_ptr<const collision_detection::CollisionRobot> collision_robot,
                                  boost::shared_ptr<const collision_detection::CollisionWorld> collision_world,
                                  boost::shared_ptr<const collision_detection::CollisionRobotDistanceField> collision_robot_df,
                                  boost::shared_ptr<const collision_detection::CollisionWorldDistanceField> collision_world_df)
{
  num_threads_ = num_threads;
  group_name_ = group_name;
  collision_robot_ = collision_robot;
  collision_world_ = collision_world;
  collision_robot_df_ = collision_robot_df;
  collision_world_df_ = collision_world_df;
  kinematic_model_ = kinematic_model;
  return this->initialize(config);
}

void StompCostFeature::setPlanningScene(planning_scene::PlanningSceneConstPtr planning_scene)
{
  planning_scene_ = planning_scene;
}

void StompCostFeature::initOutputs(const boost::shared_ptr<StompTrajectory const>& trajectory,
                 Eigen::MatrixXd& feature_values,
                 bool compute_gradients,
                 std::vector<Eigen::MatrixXd>& gradients,
                 std::vector<int>& validities) const
{
  int num_features = getNumValues();
  feature_values = Eigen::MatrixXd::Zero(trajectory->num_time_steps_, num_features);
  validities.clear();
  validities.resize(trajectory->num_time_steps_, 1);
  if (compute_gradients)
  {
    gradients.resize(num_features);
    for (int i=0; i<num_features; ++i)
    {
      gradients[i] = Eigen::MatrixXd::Zero(trajectory->num_joints_, trajectory->num_time_steps_);
    }
  }
}

} // namespace stomp_moveit_interface
