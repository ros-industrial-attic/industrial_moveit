/*
 * stomp_cost_function.h
 *
 *  Created on: Mar 7, 2016
 *      Author: Jorge Nicho
 */

#ifndef INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_COST_FUNCTION_H_
#define INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_COST_FUNCTION_H_

#include <string>
#include <XmlRpc.h>
#include <stomp_core/utils.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>

namespace stomp_moveit
{

namespace cost_functions
{

class StompCostFunction;
typedef boost::shared_ptr<StompCostFunction> StompCostFunctionPtr;

class StompCostFunction
{
public:
  StompCostFunction():
    cost_weight_(1.0)
  {

  }

  virtual ~StompCostFunction(){}

  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,XmlRpc::XmlRpcValue& config) = 0;

  virtual bool configure(const XmlRpc::XmlRpcValue& config) = 0;

  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code) = 0;


  /**
   * @brief computes the state costs as a function of the parameters for each time step.
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
                            bool& validity) const = 0 ;

  /**
   * @brief Called by the Stomp Task at the end of the optimization process
   *
   * @param success           Whether the optimization succeeded
   * @param total_iterations  Number of iterations used
   * @param final_cost        The cost value after optimizing.
   */
  virtual void done(bool success,int total_iterations,double final_cost){}


  virtual std::string getGroupName() const
  {
    return "Not Implemented";
  }

  virtual double getWeight()
  {
    return cost_weight_;
  }

  virtual std::string getName() const
  {
    return "Not Implemented";
  }

  /**
   * @brief The index returned by this method will be passed by the Task to the corresponding plugin method
   *        as the 'rollout_number' argument when operating on the noiseless (optimized) parameters.
   *
   */
  virtual int getOptimizedIndex() const
  {
    return -1;
  }


protected:

  double cost_weight_;

};


} /* namespace cost_functions */
} /* namespace stomp */

#endif /* INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_COST_FUNCTION_H_ */
