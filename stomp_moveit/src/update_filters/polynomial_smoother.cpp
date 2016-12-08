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
 * @license Software License Agreement (Apache License)\n
 * \n
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at\n
 * \n
 * http://www.apache.org/licenses/LICENSE-2.0\n
 * \n
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
#include <XmlRpcException.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::update_filters::PolynomialSmoother,stomp_moveit::update_filters::StompUpdateFilter)

namespace stomp_moveit
{
namespace update_filters
{

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
  using namespace Eigen;
  using namespace moveit::core;

  const JointModelGroup* joint_group = robot_model_->getJointModelGroup(group_name_);
  int num_joints = joint_group->getActiveJointModels().size();

  int num_r = poly_order_ + 1;
  int num_fixed = 2;

  /*
   * For the purpose of speed a portion of the equation shown in the class brief is cached.
   *
   * |p| - | 2*A*A', C |^-1 * | 2*A*b |
   * |z| - |     C', 0 |      |     d |
   *
   * Cached portion:
   *
   * | 2*A*A', C |^-1
   * |     C', 0 |
   *
   * Class Members:
   *   x_matrix_    (A) - Is the Vandermonde matrix of all (constrained and unconstrained) domain values
   *   xf_matrix_   (C) - Is the Vandermonde matrix of the constrained domain values
   *   full_matrix_     - Is the full matrix shown above
   *   full_inv_matrix_ - Is the inverse of the full matrix.
   */
  domain_vals_ = ArrayXd::LinSpaced(config.num_timesteps, 0, config.num_timesteps-1);

  domain_fvals_.resize(num_fixed);
  domain_fvals_[0] = domain_vals_.head(1)[0];
  domain_fvals_[1] = domain_vals_.tail(1)[0];

  fillVandermondeMatrix(domain_vals_, x_matrix_);
  fillVandermondeMatrix(domain_fvals_, xf_matrix_);
  x_matrix_t_ = x_matrix_.transpose();

  Eigen::MatrixXd top(num_r, num_r + num_fixed);
  Eigen::MatrixXd bot(num_fixed, num_r + num_fixed);
  top << 2.0*x_matrix_*x_matrix_t_, xf_matrix_;
  bot << xf_matrix_.transpose(), Eigen::MatrixXd::Zero(num_fixed, num_fixed);

  full_matrix_.resize(num_r + num_fixed, num_r + num_fixed);
  full_matrix_ << top,
                  bot;

  full_inv_matrix_ = full_matrix_.inverse();

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

  // updating local parameters
  int num_r = poly_order_ + 1;
  int num_fixed = 2;
  Eigen::VectorXd yf(num_fixed), poly_params(num_r);
  Eigen::VectorXd vec(num_r + num_fixed);
  Eigen::VectorXd y(domain_vals_.size());

  /*
   * For each of the noisy trajectories we need to solve the constrained
   * least-squares equation. See class brief for full description.
   *
   * |p| - | 2*A*A', C |^-1 * | 2*A*b |
   * |z| - |     C', 0 |      |     d |
   *
   * Where:
   *   x_matrix_   (A) - Is the Vandermonde matrix of all (constrained and unconstrained) domain values
   *   xf_matrix_  (C) - Is the Vandermonde matrix of the constrained domain values
   *   y           (b) - An array of the values to perform the fit on
   *   yf          (d) - An array of the values corresponding to the constrained domain values
   *   poly_params (p) - An array of the polynomial coefficients solved for.
   */
  for(auto r = 0; r < parameters.rows(); r++)
  {
    y = updates.row(r);
    yf[0] = y.head(1)[0];
    yf[1] = y.tail(1)[0];

    vec << 2*x_matrix_*y, yf;
    poly_params = (full_inv_matrix_ * vec).head(num_r);
    updates.row(r) = (x_matrix_t_ * poly_params).transpose();
  }

  filtered = true;
  return filtered;
}

void PolynomialSmoother::fillVandermondeMatrix(const Eigen::ArrayXd &domain_vals, Eigen::MatrixXd& v) const
{
  v = Eigen::MatrixXd::Ones(poly_order_+1, domain_vals.size());
  for(auto p = 1u; p <=  poly_order_; p++)
    v.row(p) = domain_vals.pow(p);
}

} /* namespace update_filters */
} /* namespace stomp_moveit */
