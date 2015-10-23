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

const static bool USE_SIGNED_DISTANCE_FIELD = true;
const static double DEFAULT_PADDING = 0.01f;

namespace stomp_moveit_interface
{

CollisionFeature::CollisionFeature():
    previous_planning_scene_(),
    report_validity_(false)
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

bool CollisionFeature::initialize(XmlRpc::XmlRpcValue& config,
                                  int num_threads,
                                  const std::string& group_name,
                                  planning_scene::PlanningSceneConstPtr planning_scene)
{
  if(!StompCostFeature::initialize(config,num_threads,group_name,planning_scene))
  {
    return false;
  }

  // initialize collision request
  collision_request_.group_name = group_name_;
  collision_request_.cost = false;
  collision_request_.distance = false;
  collision_request_.max_contacts = 1;
  collision_request_.max_contacts_per_pair = 1;
  collision_request_.contacts = false;
  collision_request_.verbose = false;

  group_state_representations_.clear();
  group_state_representations_.resize(num_threads_);

  report_validity_ = false;
  debug_collisions_ = false;
  clearance_ = 0.2;

  if(!loadParameters(config))
  {
    return false;
  }

  // creating collision world representation
  updateCollisionModels();

  return true;
}

bool CollisionFeature::loadParameters(XmlRpc::XmlRpcValue& config)
{
  double sx, sy ,sz, orig_x,orig_y,orig_z;

  if(stomp::getParam(config, "report_validity", report_validity_)
    && stomp::getParam(config, "collision_clearance", clearance_)
    && stomp::getParam(config, "debug_collisions", debug_collisions_)
    && stomp::getParam(config, "collision_space/size_x", sx)
    && stomp::getParam(config, "collision_space/size_y", sy)
    && stomp::getParam(config, "collision_space/size_z", sz)
    && stomp::getParam(config, "collision_space/origin_x", orig_x)
    && stomp::getParam(config, "collision_space/origin_y", orig_y)
    && stomp::getParam(config, "collision_space/origin_z", orig_z)
    && stomp::getParam(config, "collision_space/resolution", df_resolution_)
    && stomp::getParam(config, "collision_space/collision_tolerance", df_collision_tolerance_)
    && stomp::getParam(config, "collision_space/max_propagation_distance", df_max_propagation_distance_))
  {
    ROS_DEBUG_STREAM("Collision Feature loaded parameters");
  }
  else
  {
    ROS_ERROR_STREAM("Collision Feature failed to load parameters");
    return false;
  }

  if (debug_collisions_)
  {
    collision_request_.contacts = true;
    collision_request_.verbose = true;
  }

  df_size_ = Eigen::Vector3d(sx,sy,sz);
  df_origin_ = Eigen::Vector3d(orig_x,orig_y,orig_z);

  return true;
}

void CollisionFeature::setPlanningScene(planning_scene::PlanningSceneConstPtr planning_scene)
{
  StompCostFeature::setPlanningScene(planning_scene);
  updateCollisionModels();
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

void CollisionFeature::copyObjects(const boost::shared_ptr<const collision_detection::CollisionWorld>& from_world,
                 const boost::shared_ptr<collision_detection::CollisionWorld>& to_world) const
{
  std::vector<std::string> object_ids = from_world->getWorld()->getObjectIds();
  for (size_t i=0; i<object_ids.size(); ++i)
  {
    collision_detection::CollisionWorld::ObjectConstPtr obj = from_world->getWorld()->getObject(object_ids[i]);
    to_world->getWorld()->addToObject(object_ids[i], obj->shapes_, obj->shape_poses_);
  }
}

void CollisionFeature::updateCollisionModels()
{
  bool reset_collision_robot = false;
  bool reset_collision_world = false;

  if(previous_planning_scene_)
  {
      // checking for changes in collision world
      std::vector<std::string> ids = planning_scene_->getWorld()->getObjectIds();

      // check for objects added to the world
      if(ids.empty())
      {
        // clean collision world
        reset_collision_world = !previous_planning_scene_->getWorld()->getObjectIds().empty();
      }
      else
      {
        reset_collision_world = true;
      }

      // check if new objects were attached to the robot
      std::vector<const robot_state::AttachedBody*> attached_bodies;
      planning_scene_->getCurrentState().getAttachedBodies(attached_bodies);
      reset_collision_robot = !attached_bodies.empty();
  }
  else
  {
    reset_collision_world = true;
    reset_collision_robot = true;
  }


  if(reset_collision_world)
  {
    ros::Time start_time = ros::Time::now();
    ROS_INFO_STREAM("Collision World Distance Field creation started");
    collision_world_df_.reset(new collision_detection::CollisionWorldDistanceField(df_size_,
                                                                                df_origin_,
                                                                                USE_SIGNED_DISTANCE_FIELD,
                                                                                df_resolution_, df_collision_tolerance_,
                                                                                df_max_propagation_distance_));
    copyObjects(planning_scene_->getCollisionWorld(), collision_world_df_);

    ROS_INFO_STREAM("Collision World Distance Field creation completed after "
        <<(ros::Time::now() - start_time).toSec()<<" seconds");
  }

  if(reset_collision_robot)
  {
    ros::Time start_time = ros::Time::now();
    ROS_INFO_STREAM("Collision Robot Distance Field creation started");
    collision_robot_df_.reset(
        new collision_detection::CollisionRobotDistanceField(*planning_scene_->getCollisionRobot(),
                                                            df_size_,
                                                            df_origin_,
                                                            USE_SIGNED_DISTANCE_FIELD,
                                                            df_resolution_,
                                                            df_collision_tolerance_,
                                                            df_max_propagation_distance_,
                                                            DEFAULT_PADDING));
    ROS_INFO_STREAM("Collision Robot Distance Field creation completed after "
        <<(ros::Time::now() - start_time).toSec()<<" seconds");

  }

  // saving last scene
  previous_planning_scene_ = planning_scene_->diff();

}

std::string CollisionFeature::getName() const
{
  return "CollisionFeature";
}

} /* namespace stomp_ros_interface */
