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
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>

namespace stomp_moveit
{

namespace cost_functions
{

class StompCostFunction
{
public:
  StompCostFunction(){}
  virtual ~StompCostFunction(){}

  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,XmlRpc::XmlRpcValue& config) = 0;

  virtual bool updatePlanningScene(planning_scene::PlanningSceneConstPtr planning_scene_ptr) = 0;


  /**
   * @brief computes the state costs as a function of the parameters for each time step.
   * @param parameters [num_dimensions] num_parameters - policy parameters to execute
   * @param costs vector containing the state costs per timestep.
   * @param iteration_number
   * @param rollout_number index of the noisy trajectory whose cost is being evaluated.
   * @param validity whether or not the trajectory is valid
   * @return true if cost were properly computed
   */
  virtual bool computeCosts(std::vector<Eigen::VectorXd>& parameters,
                       Eigen::VectorXd& costs,
                       const int iteration_number,
                       const int rollout_number,
                       bool& validity) const = 0 ;

  std::string getGroupName() = 0;

  int getNumFeatures() = 0;
};

typedef boost::shared_ptr<StompCostFunction> StompCostFunctionPtr;

} /* namespace cost_functions */
} /* namespace stomp */

#endif /* INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_COST_FUNCTION_H_ */
