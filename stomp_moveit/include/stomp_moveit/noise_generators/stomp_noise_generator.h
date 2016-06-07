/*
 * noise_generator.h
 *
 *  Created on: May 31, 2016
 *      Author: Jorge Nicho
 */

#ifndef INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISE_GENERATORS_STOMP_NOISE_GENERATOR_H_
#define INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISE_GENERATORS_STOMP_NOISE_GENERATOR_H_

#include <Eigen/Core>
#include <XmlRpc.h>
#include <stomp_core/utils.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/MotionPlanRequest.h>

namespace stomp_moveit
{

namespace noise_generators
{

class StompNoiseGenerator;
typedef boost::shared_ptr<StompNoiseGenerator> StompNoiseGeneratorPtr;


class StompNoiseGenerator
{
public:
  StompNoiseGenerator(){}
  virtual ~StompNoiseGenerator(){}

  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,const XmlRpc::XmlRpcValue& config) = 0;

  virtual bool configure(const XmlRpc::XmlRpcValue& config) = 0;

  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code) = 0;

  /**
   * @brief Generates a noisy trajectory from the parameters.
   * @param parameters        [num_dimensions] x [num_parameters] the current value of the optimized parameters
   * @param start_timestep    start index into the 'parameters' array, usually 0.
   * @param num_timesteps     number of elements to use from 'parameters' starting from 'start_timestep'
   * @param iteration_number  The current iteration count in the optimization loop
   * @param rollout_number    index of the noisy trajectory.
   * @param parameters_noise  the parameters + noise
   * @param noise             the noise applied to the parameters
   * @return true if cost were properly computed
   */
  virtual bool generateNoise(const Eigen::MatrixXd& parameters,
                                       std::size_t start_timestep,
                                       std::size_t num_timesteps,
                                       int iteration_number,
                                       int rollout_number,
                                       Eigen::MatrixXd& parameters_noise,
                                       Eigen::MatrixXd& noise) = 0;

  /**
   * @brief Called by the Stomp at the end of the optimization process
   *
   * @param success           Whether the optimization succeeded
   * @param total_iterations  Number of iterations used
   * @param final_cost        The cost value after optimizing.
   */
  virtual void done(bool success,int total_iterations,double final_cost){}


  virtual std::string getName() const
  {
    return "Not implemented";
  }


  virtual std::string getGroupName() const
  {
    return "Not implemented";
  }
};

} /* namespace noise_generators */
} /* namespace stomp_moveit */



#endif /* INDUSTRIAL_MOVEIT_STOMP_MOVEIT_INCLUDE_STOMP_MOVEIT_NOISE_GENERATORS_STOMP_NOISE_GENERATOR_H_ */
