/*
 * constrained_cartesian_goal.h
 *
 *  Created on: Jun 3, 2016
 *      Author: Jorge Nicho
 */

#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_CONSTRAINED_CARTESIAN_GOAL_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_CONSTRAINED_CARTESIAN_GOAL_H_

#include <stomp_moveit/update_filters/stomp_update_filter.h>

namespace stomp_moveit
{
namespace update_filters
{

class ConstrainedCartesianGoal : public StompUpdateFilter
{
public:
  ConstrainedCartesianGoal();
  virtual ~ConstrainedCartesianGoal();


  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,const XmlRpc::XmlRpcValue& config) override;

  virtual bool configure(const XmlRpc::XmlRpcValue& config) override;

  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code) override;

  /**
   * @brief smoothes the updates array.  Uses the Control Cost Matrix projection.
   *
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param parameters        The parameters generated in the previous iteration [num_dimensions] x [num_timesteps]
   * @param updates           The updates to be applied to the parameters [num_dimensions] x [num_timesteps]
   * @param filtered          set ot 'true' if the updates were modified.
   * @return false if something failed
   */
  virtual bool filter(std::size_t start_timestep,
                      std::size_t num_timesteps,
                      int iteration_number,
                      const Eigen::MatrixXd& parameters,
                      Eigen::MatrixXd& updates,
                      bool& filtered) override;

  virtual std::string getGroupName() const
  {
    return group_name_;
  }

  virtual std::string getName() const
  {
    return name_ + "/" + group_name_;
  }

protected:

  std::string name_;
  std::string group_name_;

  // tool goal
  Eigen::Affine3d tool_goal_pose_;

  // ik
  Eigen::ArrayXd joint_update_rates_;
  Eigen::ArrayXi dof_nullity_;
  Eigen::ArrayXd cartesian_convergence_thresholds_;
  int max_iterations_;

  // robot
  moveit::core::RobotModelConstPtr robot_model_;
  moveit::core::RobotStatePtr state_;
  std::string tool_link_;
};

} /* namespace update_filters */
} /* namespace stomp_moveit */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_UPDATE_FILTERS_CONSTRAINED_CARTESIAN_GOAL_H_ */
