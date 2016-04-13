/*
 * update_smoother.h
 *
 *  Created on: Apr 12, 2016
 *      Author: Jorge Nicho
 */

#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_SMOOTHERS_SMOOTHER_INTERFACE_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_SMOOTHERS_SMOOTHER_INTERFACE_H_

#include <string>
#include <XmlRpc.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>

namespace stomp_moveit
{
namespace smoothers
{

class SmootherInterface;
using SmootherInterfacePtr = boost::shared_ptr<SmootherInterface>;

class SmootherInterface
{
public:
  SmootherInterface(){}
  virtual ~SmootherInterface(){}

  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,XmlRpc::XmlRpcValue& config) = 0;

  virtual bool configure(const XmlRpc::XmlRpcValue& config) = 0;

  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   int num_timesteps,
                   moveit_msgs::MoveItErrorCodes& error_code) = 0;

  /**
   * Applies a smoothing scheme to the parameter updates
   *
   * @param start_timestep      column index in at which to start the smoothing.
   * @param num_timestep        number of elements column-wise to which smoothing will be applied.
   * @param dt                  time step.
   * @param iteration_number    the current iteration count.
   * @param updates             the parameter updates.
   * @return false if there was a failure, true otherwise.
   */
  virtual bool smooth(std::size_t start_timestep,
                                      std::size_t num_timesteps,
                                      double dt,
                                      int iteration_number,
                                      Eigen::MatrixXd& updates) = 0;

  virtual std::string getGroupName() const
  {
    return "not implemented";
  }

  virtual std::string getName() const
  {
    return "not implemented";
  }

};

} /* namespace smoothers */
} /* namespace stomp_moveit */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_SMOOTHERS_SMOOTHER_INTERFACE_H_ */
