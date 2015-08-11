/*
 * exact_collision_feature.cpp
 *
 *  Created on: Jul 22, 2012
 *      Author: kalakris
 */

#include <stomp_moveit_interface/cost_features/exact_collision_feature.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit_interface::ExactCollisionFeature,stomp_moveit_interface::StompCostFeature);

namespace stomp_moveit_interface
{

ExactCollisionFeature::ExactCollisionFeature():
    node_handle_("~")
{
  collision_viz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("exact_collision_markers", 128);
  collision_array_viz_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>("exact_collision_markers_array", 128);
  collision_color.a = 1.0;
  collision_color.r = 1.0;
  collision_color.g = 0.0;
  collision_color.b = 0.0;

  node_handle_.param("debug_collisions", debug_collisions_, false);
  ROS_INFO("ECF created");
}

ExactCollisionFeature::~ExactCollisionFeature()
{
  ROS_INFO("ECF deleted");
}

bool ExactCollisionFeature::initialize(XmlRpc::XmlRpcValue& config)
{
  // initialize collision request
  collision_request_.group_name = group_name_;
  collision_request_.cost = false;
  collision_request_.distance = false;
  collision_request_.max_contacts = 1;
  collision_request_.max_contacts_per_pair = 1;
  collision_request_.contacts = false;
  collision_request_.verbose = false;
  if (debug_collisions_)
  {
    collision_request_.contacts = true;
    collision_request_.verbose = true;
  }
  return true;
}

int ExactCollisionFeature::getNumValues() const
{
  return 1;
}

void ExactCollisionFeature::getNames(std::vector<std::string>& names) const
{
  names.clear();
  names.push_back(getName());
}

void ExactCollisionFeature::computeValuesAndGradients(const boost::shared_ptr<StompTrajectory const>& trajectory,
                                       Eigen::MatrixXd& feature_values,         // num_time_steps x num_features
                                       bool compute_gradients,
                                       std::vector<Eigen::MatrixXd>& gradients, // [num_features] num_joints x num_time_steps
                                       std::vector<int>& validities,             // [num_time_steps] each state valid or not
                                       int thread_id,
                                       int start_timestep,                      // start timestep
                                       int num_time_steps) const
{
  initOutputs(trajectory, feature_values, compute_gradients, gradients, validities);

  collision_detection::CollisionResult result;

  for (int t=start_timestep; t<start_timestep + num_time_steps; ++t)
  {
    collision_world_->checkCollision(collision_request_, result, *collision_robot_,
                                         trajectory->kinematic_states_[t], planning_scene_->getAllowedCollisionMatrix());
    if (result.collision)
    {
      validities[t] = 0;
      feature_values(t, 0) = 1.0;
      if (debug_collisions_)
      {
        collision_detection::CollisionResult::ContactMap::iterator it;
        for (it = result.contacts.begin(); it!= result.contacts.end(); ++it)
        {
          ROS_ERROR("Collision between %s and %s", it->first.first.c_str(), it->first.second.c_str());
        }
      }
    }
  }

//  if (input->per_rollout_data_->collision_models_->isKinematicStateInCollision(*input->per_rollout_data_->kinematic_state_))
//  {
//    state_validity = false;
//    feature_values[0] = 1.0;
//    if (debug_collisions_)
//    {
//      visualization_msgs::MarkerArray arr;
//      std::vector<arm_navigation_msgs::ContactInformation> contact_info;
//      input->per_rollout_data_->collision_models_->getAllCollisionPointMarkers(*input->per_rollout_data_->kinematic_state_,
//                                                                              arr, collision_color, ros::Duration(1.0));
//      input->per_rollout_data_->collision_models_->getAllCollisionsForState(*input->per_rollout_data_->kinematic_state_,
//                                                                           contact_info, 1);
//      for (unsigned int i=0; i<contact_info.size(); ++i)
//      {
//        ROS_INFO("t %02d, Collision between %s and %s",
//                 input->time_index_,
//                 contact_info[i].contact_body_1.c_str(),
//                 contact_info[i].contact_body_2.c_str());
//      }
//      collision_array_viz_pub_.publish(arr);
//    }
//  }
//  else
//  {
//    state_validity = true;
//  }

}

std::string ExactCollisionFeature::getName() const
{
  return "ExactCollisionFeature";
}

} /* namespace stomp_ros_interface */
