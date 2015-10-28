/*
 * stomp_optimization_task.cpp
 *
 *  Created on: May 24, 2012
 *      Author: kalakris
 */

#include <stomp_moveit_interface/stomp_optimization_task.h>
#include <stomp/stomp_utils.h>
#include <iostream>
#include <fstream>

using namespace stomp;

namespace stomp_moveit_interface
{

StompOptimizationTask::StompOptimizationTask(const std::string& planning_group,
                                             planning_scene::PlanningSceneConstPtr planning_scene):
    Task(planning_group),
    feature_loader_("stomp_moveit_interface", "stomp_moveit_interface::StompCostFeature"),
    kinematic_model_(planning_scene->getRobotModel()),
    publish_trajectory_markers_(false),
    publish_best_trajectory_marker_(false),
    max_rollout_markers_published_(0),
    planning_scene_(planning_scene),
    node_handle_("~")
{

}

StompOptimizationTask::~StompOptimizationTask()
{
  feature_set_.reset(); // delete the features before their classloader
}

bool StompOptimizationTask::initialize(int num_rollouts)
{
  num_rollouts_ = num_rollouts;

  // read some params
  if(node_handle_.getParam("num_feature_basis_functions", num_feature_basis_functions_) &&
    node_handle_.getParam("trajectory_duration", movement_duration_) &&
    node_handle_.getParam("num_time_steps", num_time_steps_) &&
    node_handle_.getParam("publish_trajectory_markers",publish_trajectory_markers_) &&
    node_handle_.getParam("publish_best_trajectory_marker",publish_best_trajectory_marker_) )
  {
    ROS_DEBUG_STREAM("Stomp Optimization Task parameters loaded");
  }
  else
  {
    return false;
  }

  return true;
}

bool StompOptimizationTask::setFeaturesFromXml(const XmlRpc::XmlRpcValue& features_xml)
{
  ROS_ASSERT (features_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);

  std::vector<boost::shared_ptr<StompCostFeature> > features;

  for (int i=0; i<features_xml.size(); ++i)
  {
    XmlRpc::XmlRpcValue feature_xml = features_xml[i];
    std::string class_name;

    if(feature_xml.hasMember("class") &&
               (feature_xml["class"].getType() == XmlRpc::XmlRpcValue::TypeString))
    {
      class_name = static_cast<std::string>(feature_xml["class"]);
    }
    else
    {
      return false;
    }

    boost::shared_ptr<StompCostFeature> feature;
    try
    {
      feature = feature_loader_.createInstance(class_name);
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("Couldn't load feature named %s", class_name.c_str());
      ROS_ERROR("Error: %s", ex.what());
      return false;
    }

    STOMP_VERIFY(feature->initialize(feature_xml, num_rollouts_+1,
                                     group_name_,
                                     planning_scene_));
    features.push_back(feature);
  }

  setFeatures(features);

  return true;

}

void StompOptimizationTask::setFeatures(std::vector<boost::shared_ptr<StompCostFeature> >& features)
{
  // create the feature set
  feature_set_.reset(new FeatureSet(num_rollouts_+1));

  for (unsigned int i=0; i<features.size(); ++i)
  {
    feature_set_->addFeature(features[i]);
  }

  // init feature splits
  num_features_ = feature_set_->getNumValues();
  ROS_DEBUG("STOMP: Using %ld features classes with %d values", features.size(), num_features_);
  num_split_features_ = num_features_ * num_feature_basis_functions_;

  feature_basis_centers_.resize(num_feature_basis_functions_);
  feature_basis_stddev_.resize(num_feature_basis_functions_);
  double separation = 100.0;
  if (num_feature_basis_functions_ > 1)
  {
    separation = (1.0 / (num_feature_basis_functions_-1));
  }

  for (int i=0; i<num_feature_basis_functions_; ++i)
  {
    feature_basis_centers_[i] = i * separation;
    feature_basis_stddev_[i] = 0.5 * separation;
  }

  // TODO remove initial value hardcoding here
  feature_weights_ = Eigen::VectorXd::Ones(num_split_features_);
  feature_means_ = Eigen::VectorXd::Zero(num_split_features_);
  feature_variances_ = Eigen::VectorXd::Ones(num_split_features_);

  std::vector<std::string> feature_names;
  feature_set_->getNames(feature_names);
  ROS_DEBUG("Features loaded:");
  for (int i=0; i<num_features_; ++i)
  {
    ROS_DEBUG("%2d) %s", i, feature_names[i].c_str());
  }
}

bool StompOptimizationTask::filter(std::vector<Eigen::VectorXd>& parameters, int rollout_id, int thread_id)
{
  // clip at joint limits
  bool filtered = false;

  if (rollout_id < 0)
    rollout_id = num_rollouts_;

  return trajectories_[rollout_id]->filterJoints(parameters);
}

bool StompOptimizationTask::execute(std::vector<Eigen::VectorXd>& parameters,
                     std::vector<Eigen::VectorXd>& projected_parameters,
                     Eigen::VectorXd& costs,
                     Eigen::MatrixXd& weighted_feature_values,
                     const int iteration_number,
                     const int rollout_number,
                     int thread_id,
                     bool compute_gradients,
                     std::vector<Eigen::VectorXd>& gradients,
                     bool& validity)
{
  int rollout_id = num_rollouts_;
  if (rollout_number >= 0)
  {
    rollout_id = rollout_number;
    last_executed_rollout_ = rollout_number;
  }

  computeFeatures(parameters, rollout_id, validity);
  computeCosts(trajectories_[rollout_id]->features_, costs, weighted_feature_values);

  return true;
}

void StompOptimizationTask::computeFeatures(std::vector<Eigen::VectorXd>& parameters,
                     int rollout_id,
                     bool& validity)
{
  trajectories_[rollout_id]->setJointPositions(parameters, stomp::TRAJECTORY_PADDING);

  feature_set_->computeValuesAndGradients(trajectories_[rollout_id],
                                          trajectories_[rollout_id]->features_,
                                          false, // don't compute gradients
                                          trajectories_[rollout_id]->gradients_,
                                          trajectories_[rollout_id]->validities_,
                                          rollout_id,
                                          stomp::TRAJECTORY_PADDING,
                                          num_time_steps_);

  // compute validity
  validity = true;
  for (int t=0; t<num_time_steps_all_; ++t)
  {
    if (!trajectories_[rollout_id]->validities_[t])
      validity = false;

  }

}

void StompOptimizationTask::computeCosts(const Eigen::MatrixXd& features, Eigen::VectorXd& costs, Eigen::MatrixXd& weighted_feature_values) const
{
  weighted_feature_values = Eigen::MatrixXd::Zero(num_time_steps_, num_split_features_);
  for (int t=0; t<num_time_steps_; ++t)
  {
    weighted_feature_values.row(t) = (((features.row(t+stomp::TRAJECTORY_PADDING) - feature_means_.transpose()).array() /
        feature_variances_.array().transpose()) * feature_weights_.array().transpose()).matrix();
  }
  costs = weighted_feature_values.rowwise().sum();
}

bool StompOptimizationTask::getPolicy(boost::shared_ptr<stomp::CovariantMovementPrimitive>& policy)
{
  policy = policy_;
  return true;
}

bool StompOptimizationTask::setPolicy(const boost::shared_ptr<stomp::CovariantMovementPrimitive> policy)
{
  policy_ = policy;
  return true;
}

double StompOptimizationTask::getControlCostWeight()
{
  return control_cost_weight_;
}

void StompOptimizationTask::setControlCostWeight(double w)
{
  control_cost_weight_ = w;
}

//void StompOptimizationTask::setPlanningScene(const planning_scene::PlanningSceneConstPtr& scene)
//{
//  collision_space_->setPlanningScene(scene);
//  for (unsigned int i=0; i<per_rollout_data_.size(); ++i)
//  {
//    if (per_rollout_data_[i].collision_models_->isPlanningSceneSet())
//      per_rollout_data_[i].collision_models_->revertPlanningScene(per_rollout_data_[i].kinematic_state_);
//    planning_models::KinematicState* kin_state = per_rollout_data_[i].collision_models_->setPlanningScene(scene);
//    per_rollout_data_[i].kinematic_state_ = kin_state;
//    per_rollout_data_[i].joint_state_group_ = kin_state->getJointStateGroup(group_name_);
//  }
//}

bool StompOptimizationTask::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& scene,
                                                 const moveit_msgs::MotionPlanRequest& request,
                                                 moveit_msgs::MoveItErrorCodes& error_code)
{
  planning_scene_ = scene;
  feature_set_->setPlanningScene(planning_scene_);
  motion_plan_request_ = &request;
  control_cost_weight_ = 0.0;
  last_executed_rollout_ = -1;
  reference_frame_ = kinematic_model_->getModelFrame();
  group_name_ = request.group_name;

  dt_ = movement_duration_ / (num_time_steps_-1.0);
  num_time_steps_all_ = num_time_steps_ + 2*stomp::TRAJECTORY_PADDING;

  if (!kinematic_model_->hasJointModelGroup(group_name_))
  {
    ROS_ERROR("STOMP: Planning group %s doesn't exist!", group_name_.c_str());
    error_code.val = error_code.INVALID_GROUP_NAME;
    return false;
  }

  if(request.goal_constraints.empty() || request.goal_constraints.front().joint_constraints.empty())
  {
    ROS_ERROR("Invalid goal definition, STOMP can only accept joint goals");
    error_code.val = error_code.INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  joint_model_group_ = kinematic_model_->getJointModelGroup(group_name_);

  num_dimensions_ = joint_model_group_->getVariableCount();

  // get the start and goal positions from the message
  moveit::core::RobotState start_state(kinematic_model_);
  moveit::core::RobotState goal_state(kinematic_model_);

  // storing start position joint values
  const sensor_msgs::JointState &js = request.start_state.joint_state;
  start_state.setVariableValues(js);
  start_joints_.clear();
  start_state.copyJointGroupPositions(group_name_,start_joints_);

  // storing goal position joint values
  std::map<std::string, double> goal_joint_map;
  for (size_t i=0; i<request.goal_constraints[0].joint_constraints.size(); ++i)
  {
    goal_joint_map[request.goal_constraints[0].joint_constraints[i].joint_name] =
        request.goal_constraints[0].joint_constraints[i].position;
  }
  goal_state.setVariableValues(js);
  goal_state.setVariablePositions(goal_joint_map);
  goal_joints_.clear();
  goal_state.copyJointGroupPositions(group_name_,goal_joints_);

  // checking collision at start and end
  if(planning_scene_->isStateColliding(start_state,group_name_,true))
  {
    ROS_ERROR("Start state in collision!");
    error_code.val = error_code.START_STATE_IN_COLLISION;
    return false;
  }

  if(planning_scene_->isStateColliding(goal_state,group_name_,true))
  {
    ROS_ERROR("Goal state in collision!");
    error_code.val = error_code.GOAL_IN_COLLISION;
    return false;
  }

  // create the derivative costs
  std::vector<Eigen::MatrixXd> derivative_costs(num_dimensions_,
                                                Eigen::MatrixXd::Zero(num_time_steps_all_, stomp::NUM_DIFF_RULES));
  std::vector<Eigen::VectorXd> initial_trajectory(num_dimensions_,
                                                  Eigen::VectorXd::Zero(num_time_steps_all_));


  for (int d=0; d<num_dimensions_; ++d)
  {
    derivative_costs[d].col(stomp::STOMP_ACCELERATION) = Eigen::VectorXd::Ones(num_time_steps_all_);
    initial_trajectory[d] = Eigen::VectorXd::Zero(num_time_steps_all_);
    initial_trajectory[d].head(stomp::TRAJECTORY_PADDING) = Eigen::VectorXd::Ones(stomp::TRAJECTORY_PADDING) * start_joints_[d];
    initial_trajectory[d].tail(stomp::TRAJECTORY_PADDING) = Eigen::VectorXd::Ones(stomp::TRAJECTORY_PADDING) * goal_joints_[d];
  }

  policy_.reset(new stomp::CovariantMovementPrimitive());
  policy_->initialize(num_time_steps_,
                      num_dimensions_,
                      movement_duration_,
                      derivative_costs,
                      initial_trajectory);
  policy_->setToMinControlCost();
  std::vector<Eigen::VectorXd> params_all;
  policy_->getParametersAll(params_all);

  // initialize all trajectories
  trajectories_.resize(num_rollouts_+1);
  for (int i=0; i<num_rollouts_+1; ++i)
  {
    trajectories_[i].reset(new StompTrajectory(num_time_steps_all_, kinematic_model_, group_name_, policy_));

    trajectories_[i]->features_ = Eigen::MatrixXd(num_time_steps_all_, num_split_features_);
    trajectories_[i]->weighted_features_ = Eigen::MatrixXd(num_time_steps_all_, num_split_features_);
    trajectories_[i]->costs_ = Eigen::VectorXd(num_time_steps_all_);
    trajectories_[i]->gradients_.resize(num_split_features_, Eigen::MatrixXd::Zero(num_dimensions_, num_time_steps_all_));
    trajectories_[i]->validities_.resize(num_time_steps_all_, 1);

    // initializing all trajectory state variables to the start position
    for(int j = 0; j < trajectories_[i]->kinematic_states_.size(); j++)
    {
      moveit::core::RobotState& state = trajectories_[i]->kinematic_states_[j];
      state.setVariablePositions(start_state.getVariablePositions());
    }

    trajectories_[i]->setJointPositions(params_all, 0);
  }

  // set up feature basis functions
  std::vector<double> feature_split_magnitudes(num_feature_basis_functions_);
  feature_basis_functions_ = Eigen::MatrixXd::Zero(num_time_steps_, num_feature_basis_functions_);
  for (int t=0; t<num_time_steps_; ++t)
  {
    // compute feature splits and normalizations
    double p = double(t) / double(num_time_steps_-1);
    double sum = 0.0;
    for (int i=0; i<num_feature_basis_functions_; ++i)
    {
      feature_split_magnitudes[i] = (1.0 / (feature_basis_stddev_[i] * sqrt(2.0*M_PI))) *
          exp(-pow((p - feature_basis_centers_[i])/feature_basis_stddev_[i],2)/2.0);
      sum += feature_split_magnitudes[i];
    }
    for (int i=0; i<num_feature_basis_functions_; ++i)
    {
      feature_split_magnitudes[i] /= sum;
      feature_basis_functions_(t, i) = feature_split_magnitudes[i];
    }
  }

  error_code.val = error_code.SUCCESS;
  return true;
}

bool StompOptimizationTask::parametersToJointTrajectory(const std::vector<Eigen::VectorXd>& parameters,
                                                        trajectory_msgs::JointTrajectory& trajectory)
{

  if(parameters.empty() || (parameters.size() != num_dimensions_) ||(parameters.front().size() != num_time_steps_) )
  {
    ROS_ERROR_STREAM("Parameters contain no data, JointTrajectory message will not be created");
    return false;
  }

  if(parameters.size() != num_dimensions_)
  {
    ROS_ERROR_STREAM("Parameters array dimensions "<<parameters.size()<<"do not match the expected number of dimensions"
                     <<num_dimensions_<<" JointTrajectory message will not be created");
    return false;
  }

  if(parameters.front().size() != num_time_steps_)
  {
    ROS_ERROR_STREAM("Parameters array time steps "<<parameters.front().size()<<"do not match the expected number of points"
                     <<num_time_steps_<<" JointTrajectory message will not be created");
    return false;
  }

  trajectory.joint_names = joint_model_group_->getVariableNames();
  trajectory.points.clear();
  trajectory.points.resize(num_time_steps_ + 2);
  trajectory.points[0].positions = start_joints_;
  trajectory.points[num_time_steps_+1].positions = goal_joints_;

//  std::vector<Eigen::VectorXd> vels, accs;
//  vels.resize(num_dimensions_);
//  accs.resize(num_dimensions_);
//  for (int d=0; d<num_dimensions_; ++d)
//  {
//    stomp::differentiate(parameters[d], stomp::STOMP_VELOCITY, vels[d], dt_);
//    stomp::differentiate(parameters[d], stomp::STOMP_ACCELERATION, accs[d], dt_);
//  }


  for (int i=0; i<num_time_steps_; ++i)
  {
    int j=i+1;
    trajectory.points[j].positions.resize(num_dimensions_);
//    trajectory.points[j].velocities.resize(num_dimensions_);
//    trajectory.points[j].accelerations.resize(num_dimensions_);
    for (int d=0; d<num_dimensions_; ++d)
    {
      trajectory.points[j].positions[d] = parameters[d](i);
//      trajectory.points[j].velocities[d] = vels[d](i);
//      trajectory.points[j].accelerations[d] = accs[d](i);
    }
  }

  for (int i=0; i<num_time_steps_+2; ++i)
  {
    trajectory.points[i].time_from_start = ros::Duration(i*dt_);
  }

  return true;
}

void StompOptimizationTask::setFeatureWeights(const std::vector<double>& weights)
{
  ROS_ASSERT((int)weights.size() == num_split_features_);
  feature_weights_ = Eigen::VectorXd::Zero(weights.size());
  for (unsigned int i=0; i<weights.size(); ++i)
  {
    feature_weights_(i) = weights[i];
  }
}

void StompOptimizationTask::setFeatureWeights(const Eigen::VectorXd& weights)
{
  ROS_ASSERT(weights.rows() == feature_weights_.rows());
  feature_weights_ = weights;
}

//void StompOptimizationTask::setFeatureWeightsFromFile(const std::string& abs_file_name)
//{
//  std::vector<double> weights;
//  ROS_VERIFY(usc_utilities::readDoubleArrayFromFile(abs_file_name, weights));
//  setFeatureWeights(weights);
//}

//void StompOptimizationTask::setFeatureScaling(const std::vector<double>& means, const std::vector<double>& variances)
//{
//  ROS_ASSERT((int)means.size() == num_split_features_);
//  ROS_ASSERT((int)variances.size() == num_split_features_);
//  feature_means_ = Eigen::VectorXd::Zero(means.size());
//  feature_variances_ = Eigen::VectorXd::Zero(variances.size());
//  for (int i=0; i<num_split_features_; ++i)
//  {
//    feature_means_(i) = means[i];
//    feature_variances_(i) = variances[i];
//  }
//}
//
//void StompOptimizationTask::setFeatureScalingFromFile(const std::string& abs_means_file,
//                               const std::string& abs_variance_file)
//{
//  std::vector<double> means, variances;
//  ROS_VERIFY(usc_utilities::readDoubleArrayFromFile(abs_means_file, means));
//  ROS_VERIFY(usc_utilities::readDoubleArrayFromFile(abs_variance_file, variances));
//  setFeatureScaling(means, variances);
//}

/*void StompOptimizationTask::setInitialTrajectory(const std::vector<sensor_msgs::JointState>& joint_states)
{
  ROS_ASSERT((int)joint_states.size() == num_time_steps_);
  std::vector<Eigen::VectorXd> params(num_dimensions_, Eigen::VectorXd::Zero(num_time_steps_));
  for (int t=0; t<num_time_steps_; ++t)
  {
    for (unsigned int j=0; j<joint_states[t].name.size(); ++j)
    {
      for (unsigned int sj=0; sj<group_name_->stomp_joints_.size(); ++sj)
      {
        if (joint_states[t].name[j] == planning_group_->stomp_joints_[sj].joint_name_)
        {
          params[sj](t) = joint_states[t].position[j];
        }
      }
    }
  }
  policy_->setParameters(params);
}*/

/*void StompOptimizationTask::getTrajectory(std::vector<sensor_msgs::JointState>& joint_states)
{
  std::vector<Eigen::VectorXd> params(num_dimensions_, Eigen::VectorXd::Zero(num_time_steps_));
  policy_->getParameters(params);
  joint_states.resize(num_time_steps_);
  ros::Time start_time = ros::Time::now();

  for (int t=0; t<num_time_steps_; ++t)
  {
    joint_states[t].header.stamp = start_time + ros::Duration(t*dt_);
    joint_states[t].name.resize(planning_group_->num_joints_);
    joint_states[t].position.resize(planning_group_->num_joints_);
    for (unsigned int sj=0; sj<planning_group_->stomp_joints_.size(); ++sj)
    {
      joint_states[t].name[sj] = planning_group_->stomp_joints_[sj].joint_name_;
      joint_states[t].position[sj] = params[sj](t);
    }
  }
}*/

void StompOptimizationTask::setToMinControlCostTrajectory()
{
  policy_->setToMinControlCost();
}

void StompOptimizationTask::getNoisyRolloutData(std::vector<boost::shared_ptr<const StompTrajectory> >& noisy_rollouts)
{
  noisy_rollouts.resize(num_rollouts_);
  for (int i=0; i<num_rollouts_; ++i)
    noisy_rollouts[i] = trajectories_[i];
}

void StompOptimizationTask::getNoiselessRolloutData(boost::shared_ptr<const StompTrajectory>& noiseless_rollout)
{
  noiseless_rollout = trajectories_[num_rollouts_];
}

void StompOptimizationTask::publishResultsMarkers(const std::vector<Eigen::VectorXd>& best_parameters)
{

  if(publish_best_trajectory_marker_ && !best_parameters.empty())
  {
    publishTrajectoryMarkers(viz_trajectory_pub_,best_parameters);
  }

}


void StompOptimizationTask::publishTrajectoryMarkers(ros::Publisher& viz_pub, const std::vector<Eigen::VectorXd>& parameters)
{

  if(publish_best_trajectory_marker_)
  {
    std_msgs::ColorRGBA good_color, bad_color;
    good_color.a = 1.0;
    good_color.g = 1.0;
    good_color.r = 0.2;
    good_color.b = 0.5;
    bad_color.a = 1.0;
    bad_color.r = 1.0;
    bad_color.g = 0.2;
    bad_color.b = 0.5;

    visualization_msgs::Marker marker;
    trajectories_[num_rollouts_]->setJointPositions(parameters, stomp::TRAJECTORY_PADDING);
    trajectories_[num_rollouts_]->getVisualizationMarker(marker, good_color, bad_color);
    marker.scale.x = 0.02;
    marker.ns="noiseless";
    marker.id = 0;
    viz_pub.publish(marker);
  }
}

void StompOptimizationTask::publishTrajectoryMarkers(ros::Publisher& viz_pub)
{
  std_msgs::ColorRGBA good_color, bad_color;
  good_color.a = 0.5;
  good_color.g = 1.0;
  good_color.r = 0.2;
  good_color.b = 0.2;
  bad_color.a = 0.5;
  bad_color.r = 1.0;
  bad_color.g = 0.2;
  bad_color.b = 0.2;

  visualization_msgs::Marker marker;
  for (int i=0; i<=last_executed_rollout_; ++i)
  {
    trajectories_[i]->getVisualizationMarker(marker, good_color, bad_color);
    marker.scale.x = 0.003;
    marker.ns = "noisy";
    marker.id = i;
    viz_pub.publish(marker);
  }


  good_color.a = 1.0;
  bad_color.a = 1.0;
  trajectories_[num_rollouts_]->getVisualizationMarker(marker, good_color, bad_color);
  marker.scale.x = 0.02;
  marker.ns="noiseless";
  marker.id = 0;
  viz_pub.publish(marker);

  // publish empty markers to the remaining IDs
  for (int i=last_executed_rollout_+1; i<=max_rollout_markers_published_; ++i)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = reference_frame_;
    marker.header.stamp = ros::Time();
    marker.ns = "noisy";
    marker.id = i;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::DELETE;
    viz_pub.publish(marker);
  }

  if (max_rollout_markers_published_ < (int)last_executed_rollout_)
    max_rollout_markers_published_ = last_executed_rollout_;
}

//void StompOptimizationTask::PerRolloutData::publishMarkers(ros::Publisher& viz_pub, int id, bool noiseless, const std::string& reference_frame)
//{
//  std_msgs::ColorRGBA color;
//  double size=0.0;
//  std::string ns;
//  if (noiseless)
//  {
//    color.a = 1.0;
//    color.r = 0.0;
//    color.g = 1.0;
//    color.b = 1.0;
//    size = 0.02;
//    ns="noiseless";
//  }
//  else
//  {
//    color.a = 0.5;
//    color.r = 0.0;
//    color.g = 0.0;
//    color.b = 1.0;
//    size = 0.003;
//    ns = "noisy";
//  }
//
//  publishMarkers(viz_pub, id, ns, color, size, reference_frame);
//}

//void StompOptimizationTask::PerRolloutData::publishMarkers(ros::Publisher& viz_pub, int id, const std::string& ns,
//                                                           const std_msgs::ColorRGBA& color, double size,
//                                                           const std::string& reference_frame)
//{
//  visualization_msgs::Marker marker;
//  marker.header.frame_id = reference_frame;
//  marker.header.stamp = ros::Time();
//  marker.ns = ns;
//  marker.id = id;
//  marker.type = visualization_msgs::Marker::LINE_STRIP;
//  marker.action = visualization_msgs::Marker::ADD;
//  marker.points.resize(cost_function_input_.size());
//  marker.colors.resize(cost_function_input_.size());
//  for (unsigned int t=0; t<cost_function_input_.size(); ++t)
//  {
//    KDL::Frame& v = cost_function_input_[t]->segment_frames_
//        [task_->planning_group_->end_effector_segment_index_];
//    marker.points[t].x = v.p.x();
//    marker.points[t].y = v.p.y();
//    marker.points[t].z = v.p.z();
//    marker.colors[t] = color;
//  }
//  marker.scale.x = size;
//  marker.pose.position.x = 0;
//  marker.pose.position.y = 0;
//  marker.pose.position.z = 0;
//  marker.pose.orientation.x = 0.0;
//  marker.pose.orientation.y = 0.0;
//  marker.pose.orientation.z = 0.0;
//  marker.pose.orientation.w = 1.0;
//  marker.color.a = 1.0;
//  marker.color.r = 0.0;
//  marker.color.g = 1.0;
//  marker.color.b = 0.0;
//  viz_pub.publish(marker);
//}


void StompOptimizationTask::onEveryIteration()
{
  if (publish_trajectory_markers_)
    publishTrajectoryMarkers(viz_trajectory_pub_);
}

void StompOptimizationTask::setTrajectoryVizPublisher(ros::Publisher& viz_trajectory_pub)
{
  publish_trajectory_markers_ = publish_trajectory_markers_ && true;
  publish_best_trajectory_marker_ = publish_best_trajectory_marker_ && true;
  viz_trajectory_pub_ = viz_trajectory_pub;
}

int StompOptimizationTask::getNumFeatures()
{
  return num_features_;
}

} /* namespace stomp_ros_interface */
