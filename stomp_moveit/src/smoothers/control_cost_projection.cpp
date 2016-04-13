/*
 * control_cost_projection.cpp
 *
 *  Created on: Apr 12, 2016
 *      Author: ros-ubuntu
 */

#include <stomp_moveit/smoothers/control_cost_projection.h>
#include <stomp_core/stomp_core_utils.h>
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::smoothers::ControlCostProjection,stomp_moveit::smoothers::SmootherInterface);


namespace stomp_moveit
{
namespace smoothers
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
                        const std::string& group_name,XmlRpc::XmlRpcValue& config)
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
                 moveit_msgs::MoveItErrorCodes& error_code)
{

  error_code.val = error_code.SUCCESS;
  return true;
}

bool ControlCostProjection::smooth(std::size_t start_timestep,
                                    std::size_t num_timesteps,
                                    double dt,
                                    int iteration_number,
                                    Eigen::MatrixXd& updates)
{
  if(updates.cols() != num_timesteps_)
  {
    num_timesteps_ = updates.cols();
    stomp_core::generateSmoothingMatrix(num_timesteps_,DEFAULT_TIME_STEP,projection_matrix_M_);
  }

  for(auto d = 0u; d < updates.rows();d++)
  {
    updates.row(d).transpose() = projection_matrix_M_ * (updates.row(d).transpose());
  }

  return true;
}

} /* namespace smoothers */
} /* namespace stomp_moveit */
