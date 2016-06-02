/*
 * chomp.h
 *
 *  Created on: Jun 20, 2012
 *      Author: kalakris
 */

#ifndef CHOMP_H_
#define CHOMP_H_

#include <ros/ros.h>
#include <stomp/task.h>
#include <stomp/covariant_movement_primitive.h>
#include <stomp/policy_improvement.h>

namespace stomp
{

/**
 *  CHOMP implementation, for testing and comparison purposes
 */
class CHOMP
{
public:
  CHOMP();
  virtual ~CHOMP();

  bool initialize(ros::NodeHandle& node_handle, boost::shared_ptr<Task> task);
  bool runSingleIteration(int iteration_number);
  void getNoiselessRollout(Rollout& rollout); // we call it a rollout anyway...

private:
  ros::NodeHandle node_handle_;
  int num_time_steps_;
  int num_dimensions_;

  boost::shared_ptr<Task> task_;
  boost::shared_ptr<CovariantMovementPrimitive> policy_;
  std::vector<Eigen::VectorXd> parameters_;
  double control_cost_weight_;

  double learning_rate_;
  double max_update_;

  Rollout noiseless_rollout_;
  std::vector<Eigen::MatrixXd> inv_control_costs_;
  std::vector<Eigen::MatrixXd> control_costs_;
  std::vector<Eigen::VectorXd> gradients_;
  std::vector<Eigen::VectorXd> control_cost_gradients_;
  //Eigen::VectorXd costs_;
  Eigen::MatrixXd weighted_feature_values;

  std::vector<Eigen::VectorXd> update_;

  bool readParameters();
};

} /* namespace stomp */
#endif /* CHOMP_H_ */
