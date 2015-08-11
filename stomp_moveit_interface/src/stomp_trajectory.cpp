/*
 * stomp_cost_function_input.cpp
 *
 *  Created on: May 24, 2012
 *      Author: kalakris
 */

#include <stomp_moveit_interface/stomp_trajectory.h>
#include <ros/console.h>
#include <ros/assert.h>

namespace stomp_moveit_interface
{

StompTrajectory::StompTrajectory(int num_time_steps, const moveit::core::RobotModelConstPtr &kinematic_model,
                                 const std::string& group_name, const boost::shared_ptr<stomp::CovariantMovementPrimitive>& primitive)
{
  num_time_steps_ = num_time_steps;
  group_name_ = group_name;
  kinematic_model_ = kinematic_model;
  covariant_movement_primitive_ = primitive;

  // get the end-effector name
  const moveit::core::JointModelGroup* joint_group = kinematic_model->getJointModelGroup(group_name_);
  ROS_ASSERT(joint_group != NULL);

  std::vector<std::string> endeffector_group_names = joint_group->getAttachedEndEffectorNames();
  ROS_ASSERT_MSG(endeffector_group_names.size() == 1, "STOMP: We only handle groups with one endeffector for now");
  const moveit::core::JointModelGroup* endeff_joint_group = kinematic_model->getEndEffector(endeffector_group_names[0]);
  ROS_ASSERT(endeff_joint_group != NULL);
  std::string endeffector_name = endeff_joint_group->getEndEffectorParentGroup().second;

  //ROS_INFO("StompTrajectory: Group %s has endeffector %s", group_name_.c_str(), endeffector_name.c_str());

  // pre allocate all memory
  kinematic_states_.resize(num_time_steps, moveit::core::RobotState(kinematic_model));
  joint_state_groups_.resize(num_time_steps);
  endeffector_link_states_.resize(num_time_steps);
  for (int i=0; i<num_time_steps; ++i)
  {
    joint_state_groups_[i] = kinematic_states_[i].getJointModelGroup(group_name);
    endeffector_link_states_[i] = const_cast<moveit::core::LinkModel*>( kinematic_states_[i].getLinkModel(endeffector_name) );
    ROS_ASSERT(joint_state_groups_[i] != NULL);
    ROS_ASSERT(endeffector_link_states_[i] != NULL);
  }
  num_joints_ = joint_state_groups_[0]->getVariableCount();

  joint_models_ = joint_group->getJointModels();

  joint_pos_ = Eigen::MatrixXd::Zero(num_joints_, num_time_steps_);
  joint_vel_ = Eigen::MatrixXd::Zero(num_joints_, num_time_steps_);
  joint_acc_ = Eigen::MatrixXd::Zero(num_joints_, num_time_steps_);

  // TODO allocate collision spheres

  endeffector_pos_.resize(num_time_steps_);

  tmp_joint_angles_.resize(num_joints_);
  tmp_joint_filter_.resize(1);

}

StompTrajectory::~StompTrajectory()
{
}

bool StompTrajectory::filterJoints(std::vector<Eigen::VectorXd>& joint_positions)
{
  bool filtered = false;
  ROS_ASSERT(joint_positions.size() == num_joints_);
  for (int j = 0; j < num_joints_; ++j)
  {
    for (int t=0; t< joint_positions[j].rows(); ++t)
    {
      tmp_joint_filter_[0] = joint_positions[j](t);
      joint_models_[j]->enforcePositionBounds(tmp_joint_filter_.data());
      if (tmp_joint_filter_[0] != joint_positions[j](t))
        filtered = true;
    }
  }
  return filtered;
}

void StompTrajectory::setJointPositions(const std::vector<Eigen::VectorXd>& joint_positions, int start_index)
{
  //ROS_ASSERT(joint_positions.size() == num_joints_);
  if(joint_positions.size() != num_joints_)
  {
    ROS_DEBUG_STREAM("Number of expected joints ["<< num_joints_ <<" ] differs from number of passed joints ["<<
                     joint_positions.size() <<"]");
  }

  int num_joints = num_joints_ > joint_positions.size() ? joint_positions.size() : num_joints_;
  int max_input_length = 0;
  for (int i = 0; i < num_joints; ++i)
  {
    int input_length = joint_positions[i].rows();
    joint_pos_.row(i).segment(start_index, input_length) = joint_positions[i];
    if (max_input_length < input_length)
      max_input_length = input_length;
  }

  // compute FK
  for (int t=start_index; t<start_index+max_input_length; ++t)
  {
    for (int j=0; j<num_joints_; ++j)
    {
      tmp_joint_angles_[j] = joint_pos_(j,t);
    }
    //joint_state_groups_[t]->setVariableValues(tmp_joint_angles_);
    kinematic_states_[t].setJointGroupPositions(joint_state_groups_[t],tmp_joint_angles_);

    // get end-effector positions
    //endeffector_pos_[t] = endeffector_link_states_[t]->getGlobalLinkTransform();
    endeffector_pos_[t] = kinematic_states_[t].getFrameTransform(endeffector_link_states_[t]->getName());
  }

  // differentiation
  joint_vel_ = (covariant_movement_primitive_->getDifferentiationMatrix(stomp::STOMP_VELOCITY) * joint_pos_.transpose()).transpose();
  joint_acc_ = (covariant_movement_primitive_->getDifferentiationMatrix(stomp::STOMP_ACCELERATION) * joint_pos_.transpose()).transpose();


  update();
  // TODO: compute endeffector vel and acc

  // TODO: update collision spheres

}

void StompTrajectory::update()
{
  for(unsigned int i = 0; i < kinematic_states_.size(); i++)
  {
    kinematic_states_[i].updateLinkTransforms();
  }
}

void StompTrajectory::getVisualizationMarker(visualization_msgs::Marker& marker,
                                            const std_msgs::ColorRGBA& good_color,
                                            const std_msgs::ColorRGBA& bad_color)
{
  marker.header.frame_id = kinematic_model_->getModelFrame();
  marker.header.stamp = ros::Time();
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.points.resize(num_time_steps_);
  marker.colors.resize(num_time_steps_);
  for (int t=0; t<num_time_steps_; ++t)
  {
    Eigen::Affine3d test;
    marker.points[t].x = endeffector_pos_[t].translation()(0);
    marker.points[t].y = endeffector_pos_[t].translation()(1);
    marker.points[t].z = endeffector_pos_[t].translation()(2);
    if (validities_[t])
      marker.colors[t] = good_color;
    else
      marker.colors[t] = bad_color;
  }
  marker.scale.x = 0.02;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
}

//void StompTrajectory::publishVizMarkers(const ros::Time& stamp, ros::Publisher& publisher)
//{
//  visualization_msgs::MarkerArray marker_array;
//  marker_array.markers.resize(collision_point_pos_.size());
//  for (unsigned int i=0; i<collision_point_pos_.size(); ++i)
//  {
//    visualization_msgs::Marker& marker = marker_array.markers[i];
//    marker.header.frame_id = robot_model_->getReferenceFrame();
//    marker.header.stamp = stamp;
//    marker.ns=planning_group_->name_;
//    marker.id=i;
//    marker.type=visualization_msgs::Marker::SPHERE;
//    marker.action=visualization_msgs::Marker::ADD;
//    //marker.points.resize(group_->collision_points_.size());
//    marker.color.a=1.0;
//    marker.color.r=0.0;
//    marker.color.g=1.0;
//    marker.color.b=0.0;
//    marker.pose.orientation.w = 1.0;
//    marker.pose.orientation.x = 0.0;
//    marker.pose.orientation.y = 0.0;
//    marker.pose.orientation.z = 0.0;
//    marker.pose.position.x = collision_point_pos_[i].x();
//    marker.pose.position.y = collision_point_pos_[i].y();
//    marker.pose.position.z = collision_point_pos_[i].z();
//    marker.scale.x=2*planning_group_->collision_points_[i].getRadius();
//    marker.scale.y=2*planning_group_->collision_points_[i].getRadius();
//    marker.scale.z=2*planning_group_->collision_points_[i].getRadius();
//  }
//  publisher.publish(marker_array);
//}

} /* namespace stomp_ros_interface */
