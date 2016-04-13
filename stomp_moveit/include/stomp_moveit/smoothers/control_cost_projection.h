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
                   int num_timesteps,
                   double dt,
                   moveit_msgs::MoveItErrorCodes& error_code) override;

  /**
   * @brief Applies a smoothing scheme to the parameter updates
   *
   * @param start_timestep      start column index in the 'updates' matrix.
   * @param num_timestep        number of column-wise elements to use from the 'updates' matrix.
   * @param iteration_number    the current iteration count.
   * @param updates             the parameter updates.
   * @return                    False if there was a failure, true otherwise.
   */
  virtual bool smooth(std::size_t start_timestep,
                      std::size_t num_timesteps,
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
