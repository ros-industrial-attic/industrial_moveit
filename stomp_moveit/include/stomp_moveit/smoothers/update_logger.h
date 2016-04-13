/*
 * update_logger.h
 *
 *  Created on: Apr 13, 2016
 *      Author: Jorge Nicho
 */

#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_SMOOTHERS_UPDATE_LOGGER_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_SMOOTHERS_UPDATE_LOGGER_H_

#include <stomp_moveit/smoothers/smoother_interface.h>
#include <fstream>

namespace stomp_moveit
{
namespace smoothers
{

class UpdateLogger : public SmootherInterface
{
public:
  UpdateLogger();
  virtual ~UpdateLogger();

  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,XmlRpc::XmlRpcValue& config);

  virtual bool configure(const XmlRpc::XmlRpcValue& config);

  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code);

  /**
   * Applies a smoothing scheme to the parameter updates
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
                      Eigen::MatrixXd& updates);

  virtual std::string getGroupName() const override
  {
    return group_name_;
  }

  virtual std::string getName() const override
  {
    return name_ + "/" + group_name_;
  }

  virtual void done(bool success, int total_iterations,double final_cost) override;

protected:

  std::string name_;
  std::string group_name_;

  // parameters
  std::string filename_;
  std::string package_;
  std::string directory_;

  // logging
  std::string full_file_name_;
  std::ofstream stream_;

};

} /* namespace smoothers */
} /* namespace stomp_moveit */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_SMOOTHERS_UPDATE_LOGGER_H_ */
