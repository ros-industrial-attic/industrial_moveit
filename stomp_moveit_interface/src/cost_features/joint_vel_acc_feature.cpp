/*
 * joint_vel_acc_feature.cpp
 *
 *  Created on: May 30, 2012
 *      Author: kalakris
 */

#include <stomp_ros_interface/cost_features/joint_vel_acc_feature.h>
#include <stomp_ros_interface/stomp_cost_function_input.h>

PLUGINLIB_EXPORT_CLASS(stomp_ros_interface::JointVelAccFeature, stomp_ros_interface::StompCostFeature);

namespace stomp_ros_interface
{

JointVelAccFeature::JointVelAccFeature()
{
}

JointVelAccFeature::~JointVelAccFeature()
{
}

bool JointVelAccFeature::initialize(XmlRpc::XmlRpcValue& config, const StompRobotModel::StompPlanningGroup* planning_group)
{
  num_joints_ = planning_group->num_joints_;
  planning_group_ = planning_group;
  return true;
}

int JointVelAccFeature::getNumValues() const
{
  return num_joints_*2; // vel and acc for each joint
}

void JointVelAccFeature::getNames(std::vector<std::string>& names) const
{
  names.clear();
  std::vector<std::string> joint_names = planning_group_->getJointNames();
  for (int i=0; i<num_joints_; ++i)
  {
    names.push_back(getName()+"/"+joint_names[i]+"_Vel");
    names.push_back(getName()+"/"+joint_names[i]+"_Acc");
  }
}

void JointVelAccFeature::computeValuesAndGradients(boost::shared_ptr<learnable_cost_function::Input const> generic_input, std::vector<double>& feature_values,
                               bool compute_gradients, std::vector<Eigen::VectorXd>& gradients, bool& state_validity)
{
  boost::shared_ptr<stomp_ros_interface::StompCostFunctionInput const> input =
      boost::dynamic_pointer_cast<stomp_ros_interface::StompCostFunctionInput const>(generic_input);

  state_validity = true;

  // initialize arrays
  feature_values.clear();
  feature_values.resize(getNumValues(), 0.0);
  if (compute_gradients)
  {
    gradients.resize(getNumValues(), Eigen::VectorXd::Zero(input->getNumDimensions()));
  }

  for (int j=0; j<num_joints_; ++j)
  {
    feature_values[j*2 + 0] = input->joint_angles_vel_(j)*input->joint_angles_vel_(j);
    feature_values[j*2 + 1] = input->joint_angles_acc_(j)*input->joint_angles_acc_(j);
  }

}

std::string JointVelAccFeature::getName() const
{
  return "JointVelAccFeature";
}

boost::shared_ptr<learnable_cost_function::Feature> JointVelAccFeature::clone() const
{
  ROS_ASSERT(false); // FAIL!!
  boost::shared_ptr<JointVelAccFeature> ret(new JointVelAccFeature());
  return ret;
}

} /* namespace stomp_ros_interface */
