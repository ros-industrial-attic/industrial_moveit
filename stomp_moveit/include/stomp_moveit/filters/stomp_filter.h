/*
 * stomp_filter.h
 *
 *  Created on: Mar 23, 2016
 *      Author: ros-ubuntu
 */

#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_FILTER_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_FILTER_H_

#include <Eigen/Core>
#include <XmlRpc.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>

namespace stomp_moveit
{

namespace filters
{

class StompFilter
{
public:
  StompFilter();
  virtual ~StompFilter();

  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,const XmlRpc::XmlRpcValue& config) = 0;

  virtual bool configure(const XmlRpc::XmlRpcValue& config) = 0;

  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   moveit_msgs::MoveItErrorCodes& error_code) = 0;


  virtual std::string getGroupName()
  {
    return "";
  }

  /**
   * @brief filters the parameters and modifies the original values
   * @param parameters [num_dimensions] x [num_timesteps]
   * @return false if no filtering was applied
   */
  virtual bool filter(Eigen::MatrixXd& parameters,bool& filtered) const = 0 ;


};

typedef boost::shared_ptr<StompFilter> StompFilterPtr;

} /* namespace filters */

} /* namespace stomp_moveit */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_FILTER_H_ */
