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
                          const std::string& group_name,XmlRpc::XmlRpcValue& config) = 0;

  virtual bool updatePlanningScene(planning_scene::PlanningSceneConstPtr planning_scene_ptr) = 0;


  std::string getGroupName() = 0;

  /**
   * @brief filters the parameters and modifies the original values
   * @param parameters [num_dimensions] x [num_timesteps]
   * @return false if no filtering was applied
   */
  virtual bool filter(std::vector<Eigen::VectorXd>& parameters) = 0;


};

typedef boost::shared_ptr<StompFilter> StompFilterPtr;

} /* namespace filters */

} /* namespace stomp_moveit */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_FILTER_H_ */
