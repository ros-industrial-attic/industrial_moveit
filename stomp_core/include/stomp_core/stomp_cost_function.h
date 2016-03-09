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

namespace stomp_core
{

class StompCostFunction
{
public:
  StompCostFunction(){}
  virtual ~StompCostFunction(){}

  bool initialize(const std::string& group_name,XmlRpc::XmlRpcValue& config) = 0;
  bool updatePlanningScene(planning_scene::PlanningSceneConstPtr planning_scene_ptr) = 0;

  /*!
   * \ brief computes the cost values as a function of the variables at each time step.
   *
   * \param variables   An N x D array containing the variable values at each time step.  D is equal to the
   *                    number of dimensions.
   * \param costs       An N x F array containing the cost values that result from evaluating
   *                    'variables'.  The second dimensions size F is equal to the number of
   *                    features evaluated by this cost function.
   * \param validities  An array of size N indicating which variable values lead to an invalid state,
   */
  bool computeCost(const Eigen::VectorXd& variables,
                   Eigen::MatrixXd& costs,
                   std::vector<int>& validities) = 0;

  std::string getGroupName() = 0;
  int getNumFeatures() = 0;
};

typedef boost::shared_ptr<StompCostFunction> StompCostFunctionPtr;

} /* namespace stomp */

#endif /* INDUSTRIAL_MOVEIT_STOMP_CORE_INCLUDE_STOMP_CORE_STOMP_COST_FUNCTION_H_ */
