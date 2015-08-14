/*
 * cartesian_vel_acc_feature.cpp
 *
 *  Created on: May 30, 2012
 *      Author: kalakris
 */

#include <stomp_ros_interface/cost_features/cartesian_vel_acc_feature.h>
#include <stomp_ros_interface/stomp_cost_function_input.h>

PLUGINLIB_EXPORT_CLASS(stomp_ros_interface::CartesianVelAccFeature, stomp_ros_interface::StompCostFeature);

namespace stomp_ros_interface
{

CartesianVelAccFeature::CartesianVelAccFeature()
{
}

CartesianVelAccFeature::~CartesianVelAccFeature()
{
}

bool CartesianVelAccFeature::initialize(XmlRpc::XmlRpcValue& config, const StompRobotModel::StompPlanningGroup* planning_group)
{
  return true;
}

int CartesianVelAccFeature::getNumValues() const
{
  return 12; // vel and acc for each axis
}

void CartesianVelAccFeature::getNames(std::vector<std::string>& names) const
{
  names.clear();
  names.push_back(getName()+"/CartXLinearVel");
  names.push_back(getName()+"/CartXLinearAcc");
  names.push_back(getName()+"/CartYLinearVel");
  names.push_back(getName()+"/CartYLinearAcc");
  names.push_back(getName()+"/CartZLinearVel");
  names.push_back(getName()+"/CartZLinearAcc");
  names.push_back(getName()+"/CartXAngularVel");
  names.push_back(getName()+"/CartXAngularAcc");
  names.push_back(getName()+"/CartYAngularVel");
  names.push_back(getName()+"/CartYAngularAcc");
  names.push_back(getName()+"/CartZAngularVel");
  names.push_back(getName()+"/CartZAngularAcc");
}

void CartesianVelAccFeature::computeValuesAndGradients(boost::shared_ptr<learnable_cost_function::Input const> generic_input, std::vector<double>& feature_values,
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

  //int i = input->collision_point_pos_.size()-1;

  for (int i=0; i<3; ++i)
  {
    feature_values[i*2] = input->endeffector_vel_.vel.data[i];
    feature_values[i*2 + 1] = input->endeffector_acc_.vel.data[i];
    feature_values[6 + i*2] = input->endeffector_vel_.rot.data[i];
    feature_values[7 + i*2] = input->endeffector_acc_.rot.data[i];
  }
//  feature_values[0] = input->endeffector_vel_.vel.Norm();
//  feature_values[1] = input->endeffector_acc_.vel.Norm();
//  feature_values[2] = input->endeffector_vel_.rot.Norm();
//  feature_values[3] = input->endeffector_acc_.rot.Norm();

  // we need sq norm:
  for (int i=0; i<getNumValues(); ++i)
    feature_values[i] *= feature_values[i];

}

std::string CartesianVelAccFeature::getName() const
{
  return "CartesianVelAccFeature";
}

boost::shared_ptr<learnable_cost_function::Feature> CartesianVelAccFeature::clone() const
{
  boost::shared_ptr<CartesianVelAccFeature> ret(new CartesianVelAccFeature());
  return ret;
}

} /* namespace stomp_ros_interface */
