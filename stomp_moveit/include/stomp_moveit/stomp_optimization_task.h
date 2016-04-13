/*
 * stomp_optimization_task.h
 *
 *  Created on: Mar 23, 2016
 *      Author: ros-ubuntu
 */

#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_STOMP_OPTIMIZATION_TASK_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_STOMP_OPTIMIZATION_TASK_H_

#include <memory>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit/robot_model/robot_model.h>
#include <stomp_core/task.h>
#include <stomp_moveit/filters/stomp_filter.h>
#include <stomp_moveit/cost_functions/stomp_cost_function.h>
#include <XmlRpcValue.h>
#include <pluginlib/class_loader.h>
#include <stomp_moveit/smoothers/smoother_interface.h>


namespace stomp_moveit
{

typedef pluginlib::ClassLoader<stomp_moveit::cost_functions::StompCostFunction> CostFunctionLoader;
typedef std::shared_ptr<CostFunctionLoader> CostFuctionLoaderPtr;
typedef pluginlib::ClassLoader<stomp_moveit::filters::StompFilter> FilterLoader;
typedef std::shared_ptr<FilterLoader> FilterLoaderPtr;
typedef pluginlib::ClassLoader<smoothers::SmootherInterface> SmootherLoader;
typedef std::shared_ptr<SmootherLoader> SmootherLoaderPtr;

class StompOptimizationTask: public stomp_core::Task
{
public:
  StompOptimizationTask(moveit::core::RobotModelConstPtr robot_model_ptr, std::string group_name,
                        const XmlRpc::XmlRpcValue& config);
  virtual ~StompOptimizationTask();

  /**
   * @brief calls the updatePlanningScene method of each cost function and filter class it contains
   * @param planning_scene_ptr  A smart pointer to the planning scene
   * @param req                 The motion planning request
   * @param num_timesteps       The number of time steps in the trajectory
   * @param dt                  Time interval between consecutive points
   * @param error_code          Moveit error code
   * @return                    True if succeeded
   */
  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   int num_timesteps,
                   double dt,
                   moveit_msgs::MoveItErrorCodes& error_code);

  /**
   * @brief calls each loaded cost function class in order to compute the state costs
   * as a function of the parameters for each time step.
   *
   * @param parameters [num_dimensions] num_parameters - policy parameters to execute
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'   *
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    index of the noisy trajectory whose cost is being evaluated.   *
   * @param costs             vector containing the state costs per timestep.
   * @param validity          whether or not the trajectory is valid
   * @return true if cost were properly computed
   */
  virtual bool computeCosts(const Eigen::MatrixXd& parameters,
                            std::size_t start_timestep,
                            std::size_t num_timesteps,
                            int iteration_number,
                            int rollout_number,
                            Eigen::VectorXd& costs,
                            bool& validity) const override;

  /**
   * @brief Filters the given noisy parameters which is applied after noisy trajectory generation. It could be used for clipping
   * of joint limits or projecting into the null space of the Jacobian.
   *
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    The rollout index for this noisy parameter set
   * @param parameters        The noisy parameters
   * @return false if no filtering was done
   */
  virtual bool filterNoisyParameters(std::size_t start_timestep,
                                     std::size_t num_timesteps,
                                     int iteration_number,
                                     int rollout_number,
                                     Eigen::MatrixXd& parameters,
                                     bool& filtered) const override;

  /**
   * @brief Filters the given parameters which is applied after the update. It could be used for clipping of joint limits
   * or projecting into the null space of the Jacobian.
   *
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param parameters        The optimized parameters
   * @param filtered          False if no filtering was done
   * @return                  False if there was a failure
   */
  virtual bool filterParameters(std::size_t start_timestep,
                                std::size_t num_timesteps,
                                int iteration_number,
                                Eigen::MatrixXd& parameters,
                                bool& filtered) const override;

  /**
   * @brief Applies a smoothing scheme to the parameter updates
   *
   * @param start_timestep      start column index in the 'updates' matrix.
   * @param num_timestep        number of column-wise elements to use from the 'updates' matrix.
   * @param iteration_number    the current iteration count.
   * @param updates             the parameter updates.
   * @return                    False if there was a failure, true otherwise.
   */
  virtual bool smoothParameterUpdates(std::size_t start_timestep,
                                      std::size_t num_timesteps,
                                      int iteration_number,
                                      Eigen::MatrixXd& updates) const override;

protected:

  bool initializeCostFunctionPlugins(const XmlRpc::XmlRpcValue& config);
  bool initializeFilterPlugins(const XmlRpc::XmlRpcValue& config,std::string param_name,
                               std::vector<filters::StompFilterPtr>& filters);
  bool initializeSmootherPlugins(const XmlRpc::XmlRpcValue& config);

protected:

  // robot environment
  std::string group_name_;
  moveit::core::RobotModelConstPtr robot_model_ptr_;
  planning_scene::PlanningSceneConstPtr planning_scene_ptr_;

  // plugins
  CostFuctionLoaderPtr cost_function_loader_;
  FilterLoaderPtr filter_loader_;
  SmootherLoaderPtr smoother_loader_;

  std::vector<cost_functions::StompCostFunctionPtr> cost_functions_;
  std::vector<filters::StompFilterPtr> noisy_filters_;
  std::vector<filters::StompFilterPtr> filters_;
  std::vector<smoothers::SmootherInterfacePtr> smoothers_;
};


} /* namespace stomp_moveit */

#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_STOMP_OPTIMIZATION_TASK_H_ */
