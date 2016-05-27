/*
 * control_cost_projection.cpp
 *
 *  Created on: Apr 12, 2016
 *      Author: Jorge Nicho
 */

#include <stomp_moveit/update_filters/control_cost_projection.h>
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <stomp_core/utils.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::update_filters::ControlCostProjection,stomp_moveit::update_filters::StompUpdateFilter);


namespace stomp_moveit
{
namespace update_filters
{

double DEFAULT_TIME_STEP = 1.0;


ControlCostProjection::ControlCostProjection():
    name_("ControlCostProjectionMatrix"),
    num_timesteps_(0)

{

}

ControlCostProjection::~ControlCostProjection()
{
  // TODO Auto-generated destructor stub
}

bool ControlCostProjection::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,const XmlRpc::XmlRpcValue& config)
{
  robot_model_ = robot_model_ptr;
  group_name_ = group_name;

  if(!configure(config))
  {
    return false;
  }

  return true;
}

bool ControlCostProjection::configure(const XmlRpc::XmlRpcValue& config)
{
  return true;
}

bool ControlCostProjection::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 const stomp_core::StompConfiguration &config,
                 moveit_msgs::MoveItErrorCodes& error_code)
{

  num_timesteps_ = config.num_timesteps;
  stomp_core::generateSmoothingMatrix(num_timesteps_,DEFAULT_TIME_STEP,projection_matrix_M_);

  // zeroing out first and last rows
  projection_matrix_M_.topRows(1) = Eigen::VectorXd::Zero(num_timesteps_).transpose();
  projection_matrix_M_(0,0) = 1.0;
  projection_matrix_M_.bottomRows(1) = Eigen::VectorXd::Zero(num_timesteps_).transpose();
  projection_matrix_M_(num_timesteps_ -1 ,num_timesteps_ -1 ) = 1;

  error_code.val = error_code.SUCCESS;
  return true;
}

bool ControlCostProjection::filter(std::size_t start_timestep,std::size_t num_timesteps,int iteration_number,
                                   const Eigen::MatrixXd& parameters,
                                   Eigen::MatrixXd& updates,
                                   bool& filtered)
{

  for(auto d = 0u; d < updates.rows();d++)
  {
    updates.row(d).transpose() = projection_matrix_M_ * (updates.row(d).transpose());
  }

  filtered = true;

  return true;
}

} /* namespace update_filters */
} /* namespace stomp_moveit */
