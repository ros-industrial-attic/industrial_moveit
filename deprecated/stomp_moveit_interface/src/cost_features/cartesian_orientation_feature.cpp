/*
 * cartesian_orientation_feature.cpp
 *
 *  Created on: May 30, 2012
 *      Author: kalakris
 */

#include <stomp_ros_interface/cost_features/cartesian_orientation_feature.h>
#include <stomp_ros_interface/stomp_cost_function_input.h>

PLUGINLIB_EXPORT_CLASS(stomp_ros_interface::CartesianOrientationFeature,stomp_ros_interface::StompCostFeature);

namespace stomp_ros_interface
{

CartesianOrientationFeature::CartesianOrientationFeature()
{
}

CartesianOrientationFeature::~CartesianOrientationFeature()
{
}

bool CartesianOrientationFeature::initialize(XmlRpc::XmlRpcValue& config, const StompRobotModel::StompPlanningGroup* planning_group)
{
  return true;
}

int CartesianOrientationFeature::getNumValues() const
{
  return 1;
}

void CartesianOrientationFeature::getNames(std::vector<std::string>& names) const
{
  names.clear();
  names.push_back(getName());
}

void CartesianOrientationFeature::computeValuesAndGradients(boost::shared_ptr<learnable_cost_function::Input const> generic_input, std::vector<double>& feature_values,
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

  // abs dot product of end effector orient and cart velocity
  KDL::Vector orient_vector = input->endeffector_frame_.M.UnitZ();
  KDL::Vector velocity_vector = input->endeffector_vel_.vel;
  feature_values[0] = fabs(KDL::dot(orient_vector, velocity_vector));

  // difference from trajectory end-point
  //int num_time_steps = input->per_rollout_data_->cost_function_input_.size();



}

std::string CartesianOrientationFeature::getName() const
{
  return "CartesianOrientationFeature";
}

boost::shared_ptr<learnable_cost_function::Feature> CartesianOrientationFeature::clone() const
{
  boost::shared_ptr<CartesianOrientationFeature> ret(new CartesianOrientationFeature());
  return ret;
}

} /* namespace stomp_ros_interface */
