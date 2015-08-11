/*
 * DisplayRobotModel.cpp
 *
 *  Created on: May 23, 2012
 *      Author: kalakris
 */

#include <stomp_ros_interface/display_robot_model.h>
#include <visualization_msgs/Marker.h>

namespace stomp_ros_interface
{

DisplayRobotModel::DisplayRobotModel():
    node_handle_("~")
{
  std::string reference_frame = "BASE";
  std::string planning_group = "R_ARM";
  robot_model_.reset(new StompRobotModel(node_handle_));
  robot_model_->init(reference_frame);

  std::string group_name;
  node_handle_.param("group", group_name, std::string(planning_group));
  group_ = robot_model_->getPlanningGroup(group_name);
//  double clearance = robot_model_->getMaxRadiusClearance();

//  collision_space_.reset(new StompCollisionSpace(node_handle_));
//  collision_space_->init(clearance, reference_frame);

  joint_states_sub_ = node_handle_.subscribe<sensor_msgs::JointState>("/joint_states", 1,
    &DisplayRobotModel::jointStateCallback, this);
  marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("markers", 100, true);
}

DisplayRobotModel::~DisplayRobotModel()
{
}

void DisplayRobotModel::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  KDL::JntArray kdl_jnt_array(robot_model_->getNumKDLJoints());
  std::vector<KDL::Vector> joint_axis;
  std::vector<KDL::Vector> joint_pos;
  std::vector<KDL::Frame> segment_frames;

  for (unsigned int i=0; i<msg->name.size(); ++i)
  {
    int index = robot_model_->urdfNameToKdlNumber(msg->name[i]);
    kdl_jnt_array(index) = msg->position[i];
  }

  group_->fk_solver_->JntToCartFull(kdl_jnt_array, joint_pos, joint_axis, segment_frames);

  visualization_msgs::Marker marker;
  marker.header.frame_id = robot_model_->getReferenceFrame();
  marker.header.stamp = msg->header.stamp;
  marker.ns=group_->name_;
  marker.id=0;
  marker.type=visualization_msgs::Marker::SPHERE;
  marker.action=visualization_msgs::Marker::ADD;
  //marker.points.resize(group_->collision_points_.size());
  marker.color.a=1.0;
  marker.color.r=0.0;
  marker.color.g=1.0;
  marker.color.b=0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;

  for (unsigned int i=0; i<group_->collision_points_.size(); ++i)
  {
    KDL::Vector point;
    group_->collision_points_[i].getTransformedPosition(segment_frames, point);
    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = point.z();
    marker.scale.x=2*group_->collision_points_[i].getRadius();
    marker.scale.y=2*group_->collision_points_[i].getRadius();
    marker.scale.z=2*group_->collision_points_[i].getRadius();
    marker.id = i;
    marker_pub_.publish(marker);
  }

}

} /* namespace stomp_ros_interface */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "display_robot_model");

  stomp_ros_interface::DisplayRobotModel drm;
  ros::spin();
}
