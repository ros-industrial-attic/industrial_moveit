/*
 * stomp_cost_function_input.h
 *
 *  Created on: May 24, 2012
 *      Author: kalakris
 */

#ifndef STOMP_TRAJECTORY_H_
#define STOMP_TRAJECTORY_H_

//#include <moveit/kinematic_state/kinematic_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <stomp/covariant_movement_primitive.h>

namespace stomp_moveit_interface
{

class StompTrajectory
{
public:
  StompTrajectory(int num_time_steps, const moveit::core::RobotModelConstPtr &kinematic_model,
                  const std::string& group_name, const boost::shared_ptr<stomp::CovariantMovementPrimitive>& primitive);
  virtual ~StompTrajectory();

  void setJointPositions(const std::vector<Eigen::VectorXd>& joint_positions, int start_index=0);
  bool filterJoints(std::vector<Eigen::VectorXd>& joint_positions);
  void update();

  int num_time_steps_;
  int num_joints_;
  std::string group_name_;

  // classes for computing kinematic info
  moveit::core::RobotModelConstPtr kinematic_model_;
  std::vector<moveit::core::RobotState> kinematic_states_;     /**< [num_time_steps] */
  std::vector<const moveit::core::JointModelGroup*> joint_state_groups_; /**< [num_time_steps] */
  std::vector<moveit::core::LinkModel*> endeffector_link_states_; /**< [num_time_steps] */
  std::vector<const moveit::core::JointModel*> joint_models_; /**< [num_joints] */

  // joint trajectories - pos, vel, acc
  Eigen::MatrixXd joint_pos_;               /**< num_joints x num_time_steps */
  Eigen::MatrixXd joint_vel_;               /**< num_joints x num_time_steps */
  Eigen::MatrixXd joint_acc_;               /**< num_joints x num_time_steps */

  // trajectories of collision spheres
  std::vector<std::vector<std::vector<Eigen::VectorXd> > > sphere_pos_;  /**< [num_links] [sphere_index] [x/y/z] [num_time_steps] */
  std::vector<std::vector<std::vector<Eigen::VectorXd> > > sphere_vel_;  /**< [num_links] [sphere_index] [x/y/z] [num_time_steps] */
  //std::vector<std::vector<std::vector<Eigen::VectorXd> > > sphere_acc_;  /**< [num_links] [sphere_index] [x/y/z] [num_time_steps] */

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EigenSTL::vector_Affine3d endeffector_pos_;
//  EigenSTL::vector_Vector3d endeffector_vel_;
//  EigenSTL::vector_Vector3d endeffector_acc_;

  Eigen::MatrixXd features_; // num_time x num_features
  Eigen::MatrixXd weighted_features_; // num_time x num_features
  Eigen::VectorXd costs_; // num_time
  std::vector<Eigen::MatrixXd> gradients_; // [num_features] num_joints x num_time_steps
  std::vector<int> validities_; // [num_time_steps]

  boost::shared_ptr<stomp::CovariantMovementPrimitive> covariant_movement_primitive_;

  std::vector<double> tmp_joint_angles_; /**< [num_joints] temporary storage for joint angles */
  std::vector<double> tmp_joint_filter_; /**< [1] tmp storage to filter each joint at limits */

  void getVisualizationMarker(visualization_msgs::Marker& marker,
                             const std_msgs::ColorRGBA& good_color,
                             const std_msgs::ColorRGBA& bad_color);

//  void differentiate(double dt);
//  void publishMarkers(ros::Publisher& viz_pub, int id, bool noiseless, const std::string& reference_frame);
//  void publishMarkers(ros::Publisher& viz_pub, int id, const std::string& ns,
//                      const std_msgs::ColorRGBA& color, double size, const std::string& reference_frame);

//  KDL::JntArray joint_angles_;
//  KDL::JntArray joint_angles_vel_;
//  KDL::JntArray joint_angles_acc_;
//  KDL::JntArray all_joint_angles_;
//  std::vector<KDL::Vector> joint_axis_;
//  std::vector<KDL::Vector> joint_pos_;
//  std::vector<KDL::Frame> segment_frames_;
//  std::vector<KDL::Vector> collision_point_pos_;
//  std::vector<KDL::Vector> collision_point_vel_;
//  std::vector<KDL::Vector> collision_point_acc_;
//  KDL::Frame endeffector_frame_;
//  KDL::Twist endeffector_vel_;
//  KDL::Twist endeffector_acc_;
//  double time_;
//  int time_index_;

//  boost::shared_ptr<StompCollisionSpace const> collision_space_;
//  boost::shared_ptr<StompRobotModel const> robot_model_;
//  const StompRobotModel::StompPlanningGroup* planning_group_;
//
//  void doFK(boost::shared_ptr<KDL::TreeFkSolverJointPosAxisPartial> fk_solver);
//
//  void publishVizMarkers(const ros::Time& stamp, ros::Publisher& publisher);
//
//  StompOptimizationTask::PerRolloutData* per_rollout_data_;
//
//  boost::shared_ptr<StompTrajectory> clone();

//private:
//  bool full_fk_done_;
};

} /* namespace stomp_moveit_interface */
#endif /* STOMP_TRAJECTORY_H_ */
