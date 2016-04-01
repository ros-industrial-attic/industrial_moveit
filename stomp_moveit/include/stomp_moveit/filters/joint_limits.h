/*
 * joint_limits.h
 *
 *  Created on: Apr 1, 2016
 *      Author: ros-ubuntu
 */

#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_FILTERS_JOINT_LIMITS_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_FILTERS_JOINT_LIMITS_H_

#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/GetMotionPlan.h>
#include "stomp_moveit/filters/stomp_filter.h"

namespace stomp_moveit
{
namespace filters
{

class JointLimits : public StompFilter
{
public:
  JointLimits();
  virtual ~JointLimits();

  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,const XmlRpc::XmlRpcValue& config) override;

  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   moveit_msgs::MoveItErrorCodes& error_code) override;


  virtual std::string getGroupName()
  {
    return group_name_;
  }

  /**
   * @brief filters the parameters and modifies the original values
   * @param parameters [num_dimensions] x [num_timesteps]
   * @return false if no filtering was applied
   */
  virtual bool filter(Eigen::MatrixXd& parameters,bool& filtered) const override;

protected:

  moveit::core::RobotModelConstPtr robot_model_;
  std::string group_name_;

  // options
  bool lock_start_;
  bool lock_goal_;

  // start and goal
  moveit::core::RobotStatePtr start_state_;
  moveit::core::RobotStatePtr goal_state_;


};

} /* namespace filters */
} /* namespace stomp_moveit */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_FILTERS_JOINT_LIMITS_H_ */
