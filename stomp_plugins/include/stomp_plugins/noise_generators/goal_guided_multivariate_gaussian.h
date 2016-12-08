/*
 * goal_guided_multivariate_gaussian.h
 *
 *  Created on: Jun 14, 2016
 *      Author: Jorge Nicho
 */

#ifndef STOMP_PLUGINS_INCLUDE_STOMP_PLUGINS_NOISE_GENERATORS_GOAL_GUIDED_MULTIVARIATE_GAUSSIAN_H_
#define STOMP_PLUGINS_INCLUDE_STOMP_PLUGINS_NOISE_GENERATORS_GOAL_GUIDED_MULTIVARIATE_GAUSSIAN_H_

#include <stomp_moveit/noise_generators/stomp_noise_generator.h>
#include <stomp_moveit/utils/multivariate_gaussian.h>
#include "stomp_moveit/utils/kinematics.h"


namespace stomp_moveit
{
namespace noise_generators
{

typedef boost::mt19937 RGNType;
typedef boost::variate_generator< RGNType, boost::uniform_real<> > RandomGenerator;

class GoalGuidedMultivariateGaussian: public StompNoiseGenerator
{
public:
  GoalGuidedMultivariateGaussian();
  virtual ~GoalGuidedMultivariateGaussian();

  virtual bool initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                          const std::string& group_name,const XmlRpc::XmlRpcValue& config) override;

  virtual bool configure(const XmlRpc::XmlRpcValue& config) override;

  virtual bool setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code) override;

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
                                       Eigen::MatrixXd& noise) override;

  /**
   * @brief Called by the Stomp at the end of the optimization process
   *
   * @param success           Whether the optimization succeeded
   * @param total_iterations  Number of iterations used
   * @param final_cost        The cost value after optimizing.
   */
  virtual void done(bool success,int total_iterations,double final_cost);


  virtual std::string getName() const
  {
    return name_ + "/" + group_;
  }


  virtual std::string getGroupName() const
  {
    return group_;
  }

protected:

  virtual bool setNoiseGeneration(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code);

  virtual bool setGoalConstraints(const planning_scene::PlanningSceneConstPtr& planning_scene,
                   const moveit_msgs::MotionPlanRequest &req,
                   const stomp_core::StompConfiguration &config,
                   moveit_msgs::MoveItErrorCodes& error_code);

  virtual bool generateRandomGoal(const Eigen::VectorXd& seed,Eigen::VectorXd& goal_joint_pose);

protected:

  // names
  std::string name_;
  std::string group_;

  // goal tool constraints
  std::string tool_link_;

  // ros parameters
  Eigen::ArrayXi dof_nullity_;

  // noisy trajectory generation
  std::vector<utils::MultivariateGaussianPtr> traj_noise_generators_;
  Eigen::VectorXd raw_noise_;
  std::vector<double> stddev_;
  std::vector<double> goal_stddev_;

  // random goal generation
  boost::shared_ptr<RandomGenerator> goal_rand_generator_;
  Eigen::VectorXd joint_update_rates_;
  Eigen::VectorXd cartesian_convergence_thresholds_;

  // robot
  moveit::core::RobotModelConstPtr robot_model_;
  moveit::core::RobotStatePtr state_;

};

} /* namespace noise_generators */
} /* namespace stomp_moveit */

#endif /* STOMP_PLUGINS_INCLUDE_STOMP_PLUGINS_NOISE_GENERATORS_GOAL_GUIDED_MULTIVARIATE_GAUSSIAN_H_ */
