/*
 * collision_check.cpp
 *
 *  Created on: April 7, 2016
 *      Author: Jorge Nicho
 */
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <moveit/robot_state/conversions.h>
#include "stomp_moveit/cost_functions/collision_check.h"

PLUGINLIB_EXPORT_CLASS(stomp_moveit::cost_functions::CollisionCheck,stomp_moveit::cost_functions::StompCostFunction)

const double DEFAULT_SCALE = 1.0;
const double DEFAULT_EXP_DECAY = 0.5;
const std::string DEFAULT_COLLISION_DETECTOR = "IndustrialFCL";
const int WINDOW_SIZE = 7;

void generateExponentialSmoothingMatrix(int window_size, int num_timesteps, double decay,
                                        Eigen::SparseMatrix<double,Eigen::RowMajor>& exp_matrix)
{
  using namespace Eigen;

  // generating decay coefficients
  VectorXd coeffs(window_size);
  int mid_index = window_size/2;
  coeffs(mid_index) = decay;
  double val;
  for(auto i = 1u; i <= mid_index; i++)
  {
    val = std::pow((1 - decay),i);
    coeffs(mid_index + i) = val;
    coeffs(mid_index - i) = val;
  }
  coeffs/=(coeffs.sum());

  // populating sparse matrix
  int size = num_timesteps + window_size - 1;
  exp_matrix = SparseMatrix<double,RowMajor>(size,size);
  exp_matrix.reserve(window_size);
  for(int m = 0; m < size; m++)
  {
    exp_matrix.insert(m,m) = coeffs(mid_index);
    for(int c = 1; c <= mid_index; c++)
    {
      if((m-c) > 0)
      {
        exp_matrix.insert(m,m-c) = coeffs(mid_index-c);
      }

      if((m+c) < size)
      {
        exp_matrix.insert(m,m+c) = coeffs(mid_index+c);
      }
    }
  }

  exp_matrix.makeCompressed();

}

namespace stomp_moveit
{
namespace cost_functions
{

CollisionCheck::CollisionCheck():
    name_("CollisionCheckPlugin"),
    robot_state_(),
    collision_penalty_(0.0),
    cost_decay_(DEFAULT_EXP_DECAY)
{
  // TODO Auto-generated constructor stub

}

CollisionCheck::~CollisionCheck()
{
  // TODO Auto-generated destructor stub
}

bool CollisionCheck::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,XmlRpc::XmlRpcValue& config)
{
  robot_model_ptr_ = robot_model_ptr;
  group_name_ = group_name;
  return configure(config);
}

bool CollisionCheck::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 const stomp_core::StompConfiguration &config,
                 moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace moveit::core;

  planning_scene_ = planning_scene;
  plan_request_ = req;
  error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  // initialize collision request
  collision_request_.group_name = group_name_;
  collision_request_.cost = false;
  collision_request_.distance = false;
  collision_request_.max_contacts = 1;
  collision_request_.max_contacts_per_pair = 1;
  collision_request_.contacts = true;
  collision_request_.verbose = false;

  //Check and make sure the correct collision detector is loaded.
  if (planning_scene->getActiveCollisionDetectorName() != DEFAULT_COLLISION_DETECTOR)
  {
    throw std::runtime_error("STOMP Moveit Interface requires the use of collision detector \"" + DEFAULT_COLLISION_DETECTOR + "\"\n"
                             "To resolve the issue add the ros parameter collision_detector = " + DEFAULT_COLLISION_DETECTOR +
                             ".\nIt is recommend to added it where the move_group node is launched, usually in the in the "
                             "(robot_name)_moveit_config/launch/move_group.launch");
  }
  collision_robot_ = boost::dynamic_pointer_cast<const collision_detection::CollisionRobotIndustrial>(planning_scene->getCollisionRobot());
  collision_world_ = boost::dynamic_pointer_cast<const collision_detection::CollisionWorldIndustrial>(planning_scene->getCollisionWorld());

  // storing robot state
  robot_state_.reset(new RobotState(robot_model_ptr_));
  if(!robotStateMsgToRobotState(req.start_state,*robot_state_,true))
  {
    ROS_ERROR("%s Failed to get current robot state from request",getName().c_str());
    return false;
  }

  // initializing decay square matrix of size = num_timesteps + WINDOW_SIZE -1
  generateExponentialSmoothingMatrix(WINDOW_SIZE,config.num_timesteps,cost_decay_,exp_smoothing_matrix_);
  int padded_timesteps = config.num_timesteps + WINDOW_SIZE - 1;
  costs_padded_ = Eigen::VectorXd::Zero(padded_timesteps);
  intermediate_costs_slots_ = Eigen::VectorXd::Zero(config.num_timesteps);
  min_costs_ = Eigen::VectorXd::Zero(config.num_timesteps);

  return true;
}

bool CollisionCheck::computeCosts(const Eigen::MatrixXd& parameters,
                          std::size_t start_timestep,
                          std::size_t num_timesteps,
                          int iteration_number,
                          int rollout_number,
                          Eigen::VectorXd& costs,
                          bool& validity)
{

  using namespace moveit::core;
  using namespace Eigen;

  if(!robot_state_)
  {
    ROS_ERROR("%s Robot State has not been updated",getName().c_str());
    return false;
  }

  typedef collision_detection::CollisionResult::ContactMap ContactMap;
  typedef ContactMap::iterator ContactMapIterator;
  typedef std::vector<collision_detection::Contact> ContactArray;

  // initializing result array
  costs = Eigen::VectorXd::Zero(num_timesteps);

  // collision
  collision_detection::CollisionRequest request = collision_request_;
  collision_detection::CollisionResult result_world_collision, result_robot_collision;
  std::vector<collision_detection::CollisionResult> results(2);
  validity = true;

  // robot state
  const JointModelGroup* joint_group = robot_model_ptr_->getJointModelGroup(group_name_);

  if(parameters.cols()<start_timestep + num_timesteps)
  {
    ROS_ERROR_STREAM("Size in the 'parameters' matrix is less than required");
    return false;
  }

  // iterating through collisions
  for (auto t=start_timestep; t<start_timestep + num_timesteps; ++t)
  {
    robot_state_->setJointGroupPositions(joint_group,parameters.col(t));
    robot_state_->update();

    // checking robot vs world (attached objects, octomap, not in urdf) collisions
    result_world_collision.distance = std::numeric_limits<double>::max();

    collision_world_->checkRobotCollision(request,
                                          result_world_collision,
                                          *collision_robot_,
                                          *robot_state_,
                                          planning_scene_->getAllowedCollisionMatrix());

    collision_robot_->checkSelfCollision(request,
                                         result_robot_collision,
                                         *robot_state_,
                                         planning_scene_->getAllowedCollisionMatrix());

    results[0]= result_world_collision;
    results[1] = result_robot_collision;
    for(std::vector<collision_detection::CollisionResult>::iterator i = results.begin(); i != results.end(); i++)
    {
      collision_detection::CollisionResult& result = *i;
      if(result.collision)
      {
        costs(t) = collision_penalty_;
        validity = false;
        break;
      }
    }
  }

  // applying exponential smoothing and minimum costs
  if(!validity)
  {
    double min_penalty = collision_penalty_*(double((costs.array() > 0).count())/num_timesteps);

    int padding = WINDOW_SIZE/2;
    int start_ind = WINDOW_SIZE/2;
    costs_padded_.segment(start_ind,num_timesteps) = costs;
    costs_padded_.head(padding).setConstant(costs(0));
    costs_padded_.tail(padding).setConstant(costs(num_timesteps-1));

    // smoothed costs
    costs = (exp_smoothing_matrix_ * costs_padded_).col(0).segment(start_ind,num_timesteps);

    // reducing elements costs[i] < min_penalty to zero
    intermediate_costs_slots_ = (costs.array() > min_penalty).matrix().cast<double>();
    costs = (costs.array() * intermediate_costs_slots_.array()).matrix(); // turn all values < min_penalty  to zero

    // computing array with min_penalty at the elements where costs[i] < min_penalty
    min_costs_ = min_penalty * (costs.array() < min_penalty).matrix().cast<double>();

    // adding minimum penalty array
    costs+=min_costs_;
  }

  return true;
}

bool CollisionCheck::configure(const XmlRpc::XmlRpcValue& config)
{

  // check parameter presence
  auto members = {"cost_weight","collision_penalty"};
  for(auto& m : members)
  {
    if(!config.hasMember(m))
    {
      ROS_ERROR("%s failed to find one or more required parameters",getName().c_str());
      return false;
    }
  }

  try
  {
    XmlRpc::XmlRpcValue c = config;
    cost_weight_ = static_cast<double>(c["cost_weight"]);
    collision_penalty_ = static_cast<double>(c["collision_penalty"]);
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to parse configuration parameters",name_.c_str());
    return false;
  }

  return true;
}

void CollisionCheck::done(bool success,int total_iterations,double final_cost)
{
  robot_state_.reset();
}

} /* namespace cost_functions */
} /* namespace stomp_moveit */
