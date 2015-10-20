/*
 * obstacle_avoidance_feature.cpp
 *
 *  Created on: Oct 19, 2015
 *      Author: rosindustrial
 */

#include <stomp_moveit_interface/obstacle_avoidance_feature.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit_interface::ObstacleAvoidanceFeature,stomp_moveit_interface::StompCostFeature);

const int NUM_FEATURE_VALUES = 1;
const double DEFAULT_CLEARANCE = 0.01f;
const std::string DEFAULT_COLLISION_DETECTOR = "FCL";
const std::string FEATURE_NAME = "ObstacleAvoidance";

namespace stomp_moveit_interface
{

ObstacleAvoidanceFeature::ObstacleAvoidanceFeature():
    clearance_(DEFAULT_CLEARANCE)
{

}

ObstacleAvoidanceFeature::~ObstacleAvoidanceFeature()
{
  // TODO Auto-generated destructor stub
}

bool ObstacleAvoidanceFeature::initialize(XmlRpc::XmlRpcValue& config,
                int num_threads,
                const std::string& group_name,
                moveit::core::RobotModelConstPtr kinematic_model,
                boost::shared_ptr<const collision_detection::CollisionRobot> collision_robot,
                boost::shared_ptr<const collision_detection::CollisionWorld> collision_world,
                boost::shared_ptr<const collision_detection::CollisionRobotDistanceField> collision_robot_df,
                boost::shared_ptr<const collision_detection::CollisionWorldDistanceField> collision_world_df)
{

  return StompCostFeature::initialize(config,
                                      num_threads,
                                      group_name,
                                      kinematic_model,
                                      collision_robot,
                                      collision_world,
                                      collision_robot_df,
                                      collision_world_df);
}

bool ObstacleAvoidanceFeature::initialize(XmlRpc::XmlRpcValue& config)
{
  // initialize collision request
  collision_request_.group_name = group_name_;
  collision_request_.cost = false;
  collision_request_.distance = true;
  collision_request_.max_contacts = 1;
  collision_request_.max_contacts_per_pair = 1;
  collision_request_.contacts = false;
  collision_request_.verbose = false;

  if(config.hasMember("collision_clearance"))
  {
    clearance_ = static_cast<double>(config["collision_clearance"]);
    if(clearance_ == 0.0)
    {
      clearance_ = DEFAULT_CLEARANCE;
      ROS_WARN_STREAM("Clearance can not be 0, using default "<<clearance_);
    }
  }
  else
  {
    ROS_ERROR_STREAM(getName()<<" feature failed to load parameters");
    return false;
  }

  ROS_DEBUG_STREAM("Obstacle Avoidance feature initialized");

  return true;
}

int ObstacleAvoidanceFeature::getNumValues() const
{
  return NUM_FEATURE_VALUES;
}

void ObstacleAvoidanceFeature::computeValuesAndGradients(const boost::shared_ptr<StompTrajectory const>& trajectory,
                                       Eigen::MatrixXd& feature_values,         // num_time_steps x num_features
                                       bool compute_gradients,
                                       std::vector<Eigen::MatrixXd>& gradients, // [num_features] num_joints x num_time_steps
                                       std::vector<int>& validities,             // [num_time_steps] each state valid or not
                                       int thread_id,
                                       int start_timestep,                      // start timestep
                                       int num_time_steps) const
{

  // initializing result arrays (gradients are not used by stomp)
  feature_values = Eigen::MatrixXd::Zero(trajectory->num_time_steps_, getNumValues());
  validities.resize(trajectory->num_time_steps_, 1);

  std::vector<collision_detection::CollisionResult> results(2);
  moveit::core::RobotStatePtr state0(new moveit::core::RobotState(planning_scene_->getRobotModel()));

  for (int t=start_timestep; t<start_timestep + num_time_steps; ++t)
  {


    *state0 = trajectory->kinematic_states_[t] ;
    state0->update();

    results.resize(2);
    planning_scene_->getCollisionWorld(DEFAULT_COLLISION_DETECTOR)->checkRobotCollision(collision_request_,
                                                              results[0],
                                                              *planning_scene_->getCollisionRobot(),
                                                              *state0,
                                                              planning_scene_->getAllowedCollisionMatrix());

    planning_scene_->getCollisionRobot(DEFAULT_COLLISION_DETECTOR)->checkSelfCollision(collision_request_,
                                                             results[1],
                                                             *state0,
                                                             planning_scene_->getAllowedCollisionMatrix());


    for(std::vector<collision_detection::CollisionResult>::iterator i = results.begin(); i != results.end(); i++)
    {
      collision_detection::CollisionResult& result = *i;
      double potential = 0.0;
      if(result.collision)
      {
        //potential = -result.distance + 0.5 * clearance_;
        potential = 0.5 * clearance_;
        validities[t] = 0;
      }
      else
      {
        if( (result.distance > 0) &&  (result.distance < clearance_))
        {
          potential = 0.5 * (result.distance - clearance_) * (result.distance - clearance_) / clearance_;
        }
      }

      feature_values(t,0) += potential;
    }
  }
}

std::string ObstacleAvoidanceFeature::getName() const
{
  return FEATURE_NAME;
}

void ObstacleAvoidanceFeature::getNames(std::vector<std::string>& names) const
{
  names.push_back(FEATURE_NAME);
}

} /* namespace stomp_moveit_interface */
