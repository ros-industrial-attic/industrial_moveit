/*
 * control_cost_projection.h
 *
 *  Created on: Apr 12, 2016
 *      Author: Jorge Nicho
 */

#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_SMOOTHERS_CONTROL_COST_PROJECTION_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_SMOOTHERS_CONTROL_COST_PROJECTION_H_

#include <stomp_moveit/smoothers/smoother_interface.h>
#include <Eigen/Core>

namespace stomp_moveit
{
namespace smoothers
{

class ControlCostProjection : public SmootherInterface
{
public:
  ControlCostProjection();
  virtual ~ControlCostProjection();

  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,XmlRpc::XmlRpcValue& config) override;

  virtual bool configure(const XmlRpc::XmlRpcValue& config) override;

  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   moveit_msgs::MoveItErrorCodes& error_code) override;

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
                                      Eigen::MatrixXd& updates) override;

  virtual std::string getGroupName() const
  {
    return group_name_;
  }

  virtual std::string getName() const
  {
    return name_ + "/" + group_name_;
  }

protected:

  // local
  std::string name_;

  // robot properties
  moveit::core::RobotModelConstPtr robot_model_;
  std::string group_name_;

  // smoothing matrix
  int num_timesteps_;
  Eigen::MatrixXd projection_matrix_M_;

};

} /* namespace smoothers */
} /* namespace stomp_moveit */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_SMOOTHERS_CONTROL_COST_PROJECTION_H_ */
