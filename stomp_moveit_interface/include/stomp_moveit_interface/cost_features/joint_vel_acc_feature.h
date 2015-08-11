/*
 * joint_vel_acc_feature.h
 *
 *  Created on: May 30, 2012
 *      Author: kalakris
 */

#ifndef JOINT_VEL_ACC_FEATURE_H_
#define JOINT_VEL_ACC_FEATURE_H_

#include <stomp_ros_interface/cost_features/stomp_cost_feature.h>

namespace stomp_ros_interface
{

class JointVelAccFeature: public StompCostFeature
{
public:
  JointVelAccFeature();
  virtual ~JointVelAccFeature();

  virtual bool initialize(XmlRpc::XmlRpcValue& config, const StompRobotModel::StompPlanningGroup* planning_group);
  virtual int getNumValues() const;
  virtual void computeValuesAndGradients(boost::shared_ptr<learnable_cost_function::Input const> input, std::vector<double>& feature_values,
                                         bool compute_gradients, std::vector<Eigen::VectorXd>& gradients, bool& state_validity);
  virtual std::string getName() const;
  virtual boost::shared_ptr<learnable_cost_function::Feature> clone() const;

  void getNames(std::vector<std::string>& names) const;

private:
  const StompRobotModel::StompPlanningGroup* planning_group_;
  int num_joints_;
};

} /* namespace stomp_ros_interface */
#endif /* JOINT_VEL_ACC_FEATURE_H_ */
