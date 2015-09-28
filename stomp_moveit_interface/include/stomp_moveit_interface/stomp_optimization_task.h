/*
 * stomp_optimization_task.h
 *
 *  Created on: May 24, 2012
 *      Author: kalakris
 */

#ifndef STOMP_OPTIMIZATION_TASK_H_
#define STOMP_OPTIMIZATION_TASK_H_

#include <stomp/task.h>
#include <pluginlib/class_loader.h>
#include <stomp_moveit_interface/cost_features/stomp_cost_feature.h>
#include <stomp_moveit_interface/stomp_trajectory.h>
#include <stomp_moveit_interface/feature_set.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/MotionPlanRequest.h>

namespace stomp_moveit_interface
{

class StompOptimizationTask: public stomp::Task
{

public:
  StompOptimizationTask(ros::NodeHandle node_handle,
                        const std::string& planning_group,
                        moveit::core::RobotModelConstPtr kinematic_model,
                        boost::shared_ptr<const collision_detection::CollisionRobot> collision_robot,
                        boost::shared_ptr<const collision_detection::CollisionWorld> collision_world,
                        boost::shared_ptr<const collision_detection::CollisionRobotDistanceField> collision_robot_df,
                        boost::shared_ptr<const collision_detection::CollisionWorldDistanceField> collision_world_df);
  virtual ~StompOptimizationTask();

  virtual bool initialize(int num_threads, int num_rollouts);

  void setFeatures(std::vector<boost::shared_ptr<StompCostFeature> >& features);
  void setFeaturesFromXml(const XmlRpc::XmlRpcValue& config);

  virtual bool execute(std::vector<Eigen::VectorXd>& parameters,
                       std::vector<Eigen::VectorXd>& projected_parameters,
                       Eigen::VectorXd& costs,
                       Eigen::MatrixXd& weighted_feature_values,
                       const int iteration_number,
                       const int rollout_number,
                       int thread_id,
                       bool compute_gradients,
                       std::vector<Eigen::VectorXd>& gradients,
                       bool& validity);

  virtual bool filter(std::vector<Eigen::VectorXd>& parameters, int rollout_id, int thread_id);

  void computeFeatures(std::vector<Eigen::VectorXd>& parameters,
                       int rollout_id,
                       bool& validity);

  void computeCosts(const Eigen::MatrixXd& features, Eigen::VectorXd& costs, Eigen::MatrixXd& weighted_feature_values) const;

  bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   moveit_msgs::MoveItErrorCodes& error_code);

  void setInitialTrajectory(const std::vector<sensor_msgs::JointState>& joint_states);
  //void getTrajectory(std::vector<sensor_msgs::JointState>& joint_states);
  void setToMinControlCostTrajectory();

  void setFeatureWeights(const std::vector<double>& weights);
  void setFeatureWeights(const Eigen::VectorXd& weights);
  void setFeatureWeightsFromFile(const std::string& abs_file_name);
  void setFeatureScaling(const std::vector<double>& means, const std::vector<double>& variances);
  void setFeatureScalingFromFile(const std::string& abs_means_file,
                                 const std::string& abs_variance_file);

  bool parametersToJointTrajectory(const std::vector<Eigen::VectorXd>& parameters, trajectory_msgs::JointTrajectory& trajectory);

  int getNumFeatures();

  void getNoisyRolloutData(std::vector<boost::shared_ptr<const StompTrajectory> >& noisy_rollouts);
  void getNoiselessRolloutData(boost::shared_ptr<const StompTrajectory>& noiseless_rollout);

  virtual bool getPolicy(boost::shared_ptr<stomp::CovariantMovementPrimitive>& policy);
  virtual bool setPolicy(const boost::shared_ptr<stomp::CovariantMovementPrimitive> policy);
  virtual double getControlCostWeight();
  void setControlCostWeight(double w);

  virtual void onEveryIteration();

  void setTrajectoryVizPublisher(ros::Publisher& viz_trajectory_pub);
  void setCollisionRobotMarkerPublisher(ros::Publisher& marker_array_pub);
  void setDistanceFieldMarkerPublisher(ros::Publisher& marker_pub);

  void publishResultsMarkers(const std::vector<Eigen::VectorXd>& best_parameters);
  void publishTrajectoryMarkers(ros::Publisher& viz_pub);
  void publishTrajectoryMarkers(ros::Publisher& viz_pub, const std::vector<Eigen::VectorXd>& parameters);  // this function overwrites the last noiseless rollout!
  void publishCollisionModelMarkers(ros::Publisher& viz_robot_body_pub)const ;
  void publishDistanceFieldMarker(ros::Publisher& viz_pub);


private:
  boost::shared_ptr<stomp::CovariantMovementPrimitive> policy_;
  boost::shared_ptr<FeatureSet> feature_set_;
  double control_cost_weight_;
  std::vector<boost::shared_ptr<StompTrajectory> > trajectories_;
  ros::NodeHandle node_handle_;

  Eigen::VectorXd feature_weights_;
  Eigen::VectorXd feature_means_;
  Eigen::VectorXd feature_variances_;

  int num_threads_;
  int num_rollouts_;
  int num_time_steps_;
  int num_time_steps_all_; // includes padding at the start and end
  int num_dimensions_;
  double movement_duration_;
  double dt_;
  std::string reference_frame_;
  std::string planning_group_name_;

  std::vector<double> start_joints_;
  std::vector<double> goal_joints_;

  ros::Publisher viz_distance_field_pub_;
  ros::Publisher viz_robot_body_pub_;
  ros::Publisher viz_trajectory_pub_;

  bool publish_trajectory_markers_;
  bool publish_distance_fields_;
  bool publish_collision_models_;
  bool publish_best_trajectory_marker_;

  int max_rollout_markers_published_;
  int last_executed_rollout_;

  // variables to handle splitting features based on time
  int num_feature_basis_functions_;
  std::vector<double> feature_basis_centers_;
  std::vector<double> feature_basis_stddev_;
  int num_features_;            // original number of features
  int num_split_features_;      // number of features after "time-split"
  Eigen::MatrixXd feature_basis_functions_; // num_time x num_basis_functions

  pluginlib::ClassLoader<StompCostFeature> feature_loader_;

  moveit::core::RobotModelConstPtr kinematic_model_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  const moveit_msgs::MotionPlanRequest* motion_plan_request_;
  const moveit::core::JointModelGroup* joint_model_group_;

  boost::shared_ptr<const collision_detection::CollisionRobot> collision_robot_; /**< standard robot collision checker */
  boost::shared_ptr<const collision_detection::CollisionWorld> collision_world_; /**< standard robot -> world collision checker */
  boost::shared_ptr<const collision_detection::CollisionRobotDistanceField> collision_robot_df_;    /**< distance field robot collision checker */
  boost::shared_ptr<const collision_detection::CollisionWorldDistanceField> collision_world_df_;    /**< distance field robot -> world collision checker */

};

} /* namespace stomp_moveit_interface */
#endif /* STOMP_OPTIMIZATION_TASK_H_ */
