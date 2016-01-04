/*
 * collision_feature.h
 *
 *  Created on: May 25, 2012
 *      Author: kalakris
 */

#ifndef COLLISION_FEATURE_H_
#define COLLISION_FEATURE_H_

#include <stomp_moveit_interface/cost_features/stomp_cost_feature.h>
#include <moveit/collision_distance_field/collision_common_distance_field.h>
#include <moveit/collision_distance_field/collision_world_distance_field.h>
#include <moveit/collision_distance_field/collision_robot_distance_field.h>
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>

namespace stomp_moveit_interface
{

class CollisionFeature: public StompCostFeature
{
public:
  CollisionFeature();
  virtual ~CollisionFeature();

  virtual bool initialize(XmlRpc::XmlRpcValue& config,
                  int num_threads,
                  const std::string& group_name,
                  planning_scene::PlanningSceneConstPtr planning_scene);

  virtual int getNumValues() const;
  virtual void computeValuesAndGradients(const boost::shared_ptr<StompTrajectory const>& trajectory,
                                         Eigen::MatrixXd& feature_values,         // num_time_steps x num_features
                                         bool compute_gradients,
                                         std::vector<Eigen::MatrixXd>& gradients, // [num_features] num_joints x num_time_steps
                                         std::vector<int>& validities,             // [num_time_steps] each state valid or not
                                         int thread_id,
                                         int start_timestep,                      // start timestep
                                         int num_time_steps) const;
  virtual void setPlanningScene(planning_scene::PlanningSceneConstPtr planning_scene);
  virtual std::string getName() const;
  void getNames(std::vector<std::string>& names) const;

protected:
  bool loadParameters(XmlRpc::XmlRpcValue& config);
  void copyObjects(const boost::shared_ptr<const collision_detection::CollisionWorld>& from_world,
                   const boost::shared_ptr<collision_detection::CollisionWorld>& to_world) const;
  void updateCollisionModels();

protected:
  collision_detection::CollisionRequest collision_request_;

  mutable std::vector<boost::shared_ptr<collision_detection::GroupStateRepresentation> > group_state_representations_;
  planning_scene::PlanningSceneConstPtr previous_planning_scene_;
  Eigen::Vector3d df_size_;
  Eigen::Vector3d df_origin_;
  double df_resolution_;
  double df_collision_tolerance_;
  double df_max_propagation_distance_;
  boost::shared_ptr<collision_detection::CollisionRobotDistanceField> collision_robot_df_;    /**< distance field robot collision checker */
  boost::shared_ptr<collision_detection::CollisionWorldDistanceField> collision_world_df_;

  double clearance_;
  bool report_validity_;
  bool debug_collisions_;

  double num_sigmoids_;
  std::vector<double> sigmoid_centers_;
  std::vector<double> sigmoid_slopes_;
};

} /* namespace stomp_ros_interface */
#endif /* COLLISION_FEATURE_H_ */
