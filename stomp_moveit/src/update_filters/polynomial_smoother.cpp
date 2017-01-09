/**
 * @file polynomial_smoother.cpp
 * @brief This defines a polynomial smoother update filter
 *
 * This smooths the noisy update.
 *
 * @author Jorge Nicho
 * @date June 3, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <ros/console.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <pluginlib/class_list_macros.h>
#include <stomp_moveit/update_filters/polynomial_smoother.h>
#include <stomp_core/utils.h>
#include <XmlRpcException.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::update_filters::PolynomialSmoother,stomp_moveit::update_filters::StompUpdateFilter)

namespace stomp_moveit
{
namespace update_filters
{

const double JOINT_LIMIT_MARGIN = 0.00001;
const int ALLOWED_SMOOTHING_ATTEMPTS = 5;

PolynomialSmoother::PolynomialSmoother():
    name_("ExponentialSmoother")
{
  // TODO Auto-generated constructor stub

}

PolynomialSmoother::~PolynomialSmoother()
{
  // TODO Auto-generated destructor stub
}

bool PolynomialSmoother::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,const XmlRpc::XmlRpcValue& config)
{
  group_name_ = group_name;
  robot_model_ = robot_model_ptr;

  return configure(config);
}

bool PolynomialSmoother::configure(const XmlRpc::XmlRpcValue& config)
{
  using namespace XmlRpc;

  try
  {
    XmlRpcValue params = config;
    poly_order_ = static_cast<int>(params["poly_order"]);
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to load parameters, %s",getName().c_str(),e.getMessage().c_str());
    return false;
  }

  return true;
}

bool PolynomialSmoother::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 const stomp_core::StompConfiguration &config,
                 moveit_msgs::MoveItErrorCodes& error_code)
{
  error_code.val = error_code.SUCCESS;
  return true;
}

bool PolynomialSmoother::filter(std::size_t start_timestep,
                    std::size_t num_timesteps,
                    int iteration_number,
                    const Eigen::MatrixXd& parameters,
                    Eigen::MatrixXd& updates,
                    bool& filtered)
{
  using namespace Eigen;
  using namespace moveit::core;
  filtered = true;

  const std::vector<const JointModel*> &joint_models = robot_model_->getJointModelGroup(group_name_)->getActiveJointModels();
  VectorXd poly_params;
  VectorXd y, domain_vals(num_timesteps);
  std::vector<int> fixed_indices;

  getDomainValues(joint_models, parameters, updates, domain_vals);

  fixed_indices.reserve(num_timesteps);
  for(auto r = 0; r < parameters.rows(); r++)
  {
    fixed_indices.resize(2);
    fixed_indices[0] = 0;
    fixed_indices[1] = num_timesteps - 1;

    VectorXd jv = updates.row(r) + parameters.row(r);
    y = jv;
    jv = stomp_core::polyFitWithFixedPoints(poly_order_, domain_vals, y, VectorXi::Map(fixed_indices.data(), fixed_indices.size()), poly_params);

    bool finished = false;
    int attempts = 0;
    while (!finished && attempts < ALLOWED_SMOOTHING_ATTEMPTS)
    {
      attempts += 1;

      bool s1 = std::signbit(jv(1) - jv(0));
      for (auto i = 2; i < parameters.cols(); ++i)
      {
        bool s2 = std::signbit(jv(i) - jv(i-1));
        if (s1 != s2)
        {
          double val = jv(i - 1);
          if (joint_models[r]->enforcePositionBounds(&val))
          {
            fixed_indices.insert(fixed_indices.end() - 1, i - 1);
            y(i - 1) = val;
          }

          s1 = s2;
        }
      }

      if (fixed_indices.size() > 2)
      {
        std::sort(fixed_indices.begin(), fixed_indices.end());
        jv = stomp_core::polyFitWithFixedPoints(poly_order_, domain_vals, y, VectorXi::Map(fixed_indices.data(), fixed_indices.size()), poly_params);
      }

      //  Now check if joint trajectory is within joint limits
      double min = jv.minCoeff();
      double max = jv.maxCoeff();
      finished = joint_models[r]->satisfiesPositionBounds(&min, JOINT_LIMIT_MARGIN) &&
                 joint_models[r]->satisfiesPositionBounds(&max, JOINT_LIMIT_MARGIN);
    }

    filtered &= finished;
    updates.row(r) = jv.transpose() - parameters.row(r);
  }

  if (!filtered)
    ROS_ERROR("Unable to polynomial smooth trajectory!");

  return filtered;
}

void PolynomialSmoother::getDomainValues(const std::vector<const  moveit::core::JointModel *> joint_models, const Eigen::MatrixXd &parameters, const Eigen::MatrixXd &updates, Eigen::VectorXd &domain_values) const
{
  Eigen::VectorXd distance(parameters.rows());
  Eigen::VectorXd velocity(parameters.rows());
  Eigen::VectorXd t(parameters.rows());
  Eigen::VectorXd domain_dist(parameters.cols());
  double max_t = 0;

  domain_values.resize(parameters.cols());
  for(auto r = 0; r < parameters.rows(); r++)
  {
    moveit::core::VariableBounds bound = joint_models[r]->getVariableBounds()[0];
    velocity(r) = bound.max_velocity_;
    distance(r) = 0.0;
    domain_dist(0) = 0.0;
    for(auto c = 1; c < parameters.cols(); c++)
    {
      distance(r) += std::abs((parameters(r, c) + updates(r, c)) - (parameters(r, c - 1) + updates(r, c - 1)));
      domain_dist(c) = distance(r);
    }

    t(r) = distance(r)/velocity(r);
    if (t(r) > max_t)
    {
      max_t = t(r);
      domain_values = domain_dist/velocity(r);
    }
  }
}

} /* namespace update_filters */
} /* namespace stomp_moveit */
