/**
 * @file polynomial.cpp
 * @brief TODO
 *
 * @author Jorge Nicho
 * @date Jan 6, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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

#include <stomp_moveit/utils/polynomial.h>

/**
 * @namespace stomp_moveit
 */
namespace stomp_moveit
{

/**
 * @namespace utils
 */
namespace utils
{

/**
 * @namespace smoothing
 */
namespace polynomial
{

Eigen::VectorXd polyFitWithFixedPoints(const int d,
                                       const Eigen::VectorXd &x,
                                       const Eigen::VectorXd &y,
                                       const Eigen::VectorXi &fixed_indices,
                                       Eigen::VectorXd &poly_params)
{

//  |p| - | 2*A*A', C |^-1 * | 2*A*b |
//  |z| - |     C', 0 |      |     d |
//
//  Variable Description:
//    x_t         (A) - Is the Vandermonde matrix of all (constrained and unconstrained) domain values
//    a_t         (C) - Is the Vandermonde matrix of the constrained domain values
//    y           (b) - An array of the values to perform the fit on
//    yf          (d) - An array of the values corresponding to the constrained domain values
//    poly_params (p) - An array of the polynomial coefficients solved for.

  int num_r = d + 1;
  int num_fixed = fixed_indices.size();
  Eigen::MatrixXd x_t(num_r, x.size());
  Eigen::MatrixXd a_t(num_r, num_fixed);
  Eigen::VectorXd xf(num_fixed), yf(num_fixed);

  // Get fixed position data
  for (auto i = 0; i < num_fixed; i++)
  {
    xf[i] = x[fixed_indices[i]];
    yf[i] = y[fixed_indices[i]];
  }


  for (auto r = 0; r < num_r; r++)
  {
    x_t.row(r) = x.array().pow(r);
    a_t.row(r) = xf.array().pow(r);
  }

  Eigen::MatrixXd top(num_r, num_r + num_fixed);
  Eigen::MatrixXd bot(num_fixed, num_r + num_fixed);
  Eigen::MatrixXd mat(num_r + num_fixed, num_r + num_fixed);
  Eigen::VectorXd vec(num_r + num_fixed);
  vec << 2*x_t*y, yf;
  top << 2*x_t*x_t.transpose(), a_t;
  bot << a_t.transpose(), Eigen::MatrixXd::Zero(num_fixed, num_fixed);
  mat << top,
         bot;

  poly_params = mat.lu().solve(vec).head(num_r);
  return x_t.transpose() * poly_params;
}

void fillVandermondeMatrix(const Eigen::ArrayXd &domain_vals, const int &order, Eigen::MatrixXd &v)
{
  v = Eigen::MatrixXd::Ones(order+1, domain_vals.size());
  for(auto p = 1u; p <=  order; p++)
    v.row(p) = domain_vals.pow(p);
}

void generateMinimumDomainValues(const std::vector<const  moveit::core::JointModel *> joint_models,
                                 const Eigen::MatrixXd &parameters,Eigen::VectorXd &domain_values)
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
      distance(r) += std::abs((parameters(r, c)) - (parameters(r, c - 1)));
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

bool applyPolynomialSmoothing(moveit::core::RobotModelConstPtr robot_model, const std::string& group_name, Eigen::MatrixXd& parameters,
                              int poly_order, int smoothing_attempts, double joint_limit_margin)
{
  using namespace Eigen;
  using namespace moveit::core;

  const std::vector<const JointModel*> &joint_models = robot_model->getJointModelGroup(group_name)->getActiveJointModels();
  VectorXd poly_params;
  VectorXd y;
  std::vector<int> fixed_indices;
  int num_timesteps = parameters.cols();

  fixed_indices.reserve(num_timesteps);
  VectorXd domain_vals;
  generateMinimumDomainValues(joint_models, parameters,domain_vals);

  for(auto r = 0; r < parameters.rows(); r++)
  {
    fixed_indices.resize(2);
    fixed_indices[0] = 0;
    fixed_indices[1] = num_timesteps - 1;

    VectorXd jv = parameters.row(r);
    y = jv;
    jv = polyFitWithFixedPoints(poly_order, domain_vals, y, VectorXi::Map(fixed_indices.data(), fixed_indices.size()), poly_params);

    bool finished = false;
    int attempts = 0;
    while (!finished && attempts < smoothing_attempts)
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
        jv = polyFitWithFixedPoints(poly_order, domain_vals, y, VectorXi::Map(fixed_indices.data(), fixed_indices.size()), poly_params);
      }

      //  Now check if joint trajectory is within joint limits
      double min = jv.minCoeff();
      double max = jv.maxCoeff();
      finished = joint_models[r]->satisfiesPositionBounds(&min, joint_limit_margin) &&
                 joint_models[r]->satisfiesPositionBounds(&max, joint_limit_margin);
    }

    if(!finished)
    {
      return false;
    }

    parameters.row(r) = jv.transpose();

  }

  return true;
}

} // end of namespace smoothing
} // end of namespace utils
} // end of namespace stomp_moveit




