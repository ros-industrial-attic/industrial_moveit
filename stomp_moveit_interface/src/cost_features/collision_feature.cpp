/*
 * collision_feature.cpp
 *
 *  Created on: May 25, 2012
 *      Author: kalakris
 */

#include <stomp_moveit_interface/cost_features/collision_feature.h>
#include <stomp_moveit_interface/sigmoid.h>
#include <sstream>

PLUGINLIB_EXPORT_CLASS(stomp_moveit_interface::CollisionFeature,stomp_moveit_interface::StompCostFeature);

namespace stomp_moveit_interface
{

CollisionFeature::CollisionFeature()
{
  sigmoid_centers_.push_back(-0.025);
  sigmoid_slopes_.push_back(200.0);

  sigmoid_centers_.push_back(0.0);
  sigmoid_slopes_.push_back(200.0);

  sigmoid_centers_.push_back(0.025);
  sigmoid_slopes_.push_back(200.0);

  sigmoid_centers_.push_back(0.05);
  sigmoid_slopes_.push_back(200.0);

  sigmoid_centers_.push_back(0.075);
  sigmoid_slopes_.push_back(200.0);

  sigmoid_centers_.push_back(0.1);
  sigmoid_slopes_.push_back(200.0);

  num_sigmoids_ = sigmoid_centers_.size();
  num_sigmoids_ = 0;

}

CollisionFeature::~CollisionFeature()
{
}

bool CollisionFeature::initialize(XmlRpc::XmlRpcValue& config)
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

  group_state_representations_.clear();
  group_state_representations_.resize(num_threads_);

  report_validity_ = false;
  debug_collisions_ = false;
  clearance_ = 0.2;
  stomp::getParam(config, "report_validity", report_validity_);
  stomp::getParam(config, "collision_clearance", clearance_);
  stomp::getParam(config, "debug_collisions", debug_collisions_);
  return true;
}

int CollisionFeature::getNumValues() const
{
  return 1 + num_sigmoids_; // 1 for smooth cost, rest for sigmoids
}

void CollisionFeature::getNames(std::vector<std::string>& names) const
{
  names.clear();
  names.push_back(getName()+"/DistanceFieldCost");
  for (int i=0; i<num_sigmoids_; ++i)
  {
    std::stringstream ss;
    ss << getName() << "/Sigmoid_" << sigmoid_centers_[i];
    names.push_back(ss.str());
  }
}

void CollisionFeature::computeValuesAndGradients(const boost::shared_ptr<StompTrajectory const>& trajectory,
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
  boost::shared_ptr<collision_detection::GroupStateRepresentation>& gsr = group_state_representations_[thread_id];

  for (int t=start_timestep; t<start_timestep + num_time_steps; ++t)
  {
    collision_world_df_->getCollisionGradients(collision_request_, result, *collision_robot_df_,
                                         trajectory->kinematic_states_[t], &(planning_scene_->getAllowedCollisionMatrix()),
                                         gsr);

    for (size_t i=0; i<gsr->gradients_.size(); ++i)
    {
      for (size_t j=0; j<gsr->gradients_[i].distances.size(); ++j)
      {
        double distance = gsr->gradients_[i].distances[j];

        double potential = 0.0;
        if (distance >= clearance_)
        {
          potential = 0.0;
        }
        else if (distance >= 0.0)
        {
          potential = 0.5 * (distance - clearance_) * (distance - clearance_) / clearance_;
        }
        else // distance < 0.0
        {
          potential = -distance + 0.5 * clearance_;
        }

        feature_values(t,0) += potential;


//        for (int i=0; i<num_sigmoids_; ++i)
//        {
//          double val = (1.0 - sigmoid(distance, sigmoid_centers_[i], sigmoid_slopes_[i]));
//          feature_values[i+1] += val * vel_mag;
//          //printf("distance = %f, sigmoid %d = %f\n", distance, i, val);
//        }

      }
    }

    if (report_validity_)
    {
      collision_world_df_->checkCollision(collision_request_, result, *collision_robot_df_,
                                           trajectory->kinematic_states_[t], planning_scene_->getAllowedCollisionMatrix(),
                                           gsr);
      if (result.collision)
      {
        validities[t] = 0;
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


  }

  // TODO gradients not computed yet!!!
}

std::string CollisionFeature::getName() const
{
  return "CollisionFeature";
}

} /* namespace stomp_ros_interface */
