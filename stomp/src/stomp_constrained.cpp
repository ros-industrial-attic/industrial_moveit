#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <map>
#include <cstdio>
#include <iostream>
#include <cmath>
#include <iostream>
#include <boost/progress.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <stomp/stomp_constrained.h>

StompConstrained::StompConstrained():
  normal_dist_(0.0, 1.0),
  gaussian_(rng_, normal_dist_)
{
}

void StompConstrained::initialize(const Eigen::MatrixXd& initial_trajectory,
                                  const double dt,
                                  const int free_vars_start,
                                  const int free_vars_end,
                                  const int num_samples,
                                  const std::vector<double>& derivative_costs)
{
  has_equality_constraints_ = false;
  num_joints_ = initial_trajectory.cols();
  num_time_steps_ = free_vars_end - free_vars_start + 1;
  num_time_steps_all_ = initial_trajectory.rows();
  free_vars_start_ = free_vars_start;
  free_vars_end_ = free_vars_end;
  num_samples_ = num_samples;
  num_variables_ = num_joints_ * num_time_steps_;
  dt_ = dt;
  derivative_costs_ = derivative_costs;

  printf("Creating diff matrices... \n");
  createDiffMatrices();
  printf("Assigning initial trajectory... \n");
  setInitialTrajectory(initial_trajectory);
  printf("Creating cost matrix... \n");
  createCostMatrix();
  printf("Computing cholesky decomposition... \n");
  computeCholeskyFactor();
  printf("Computing min control cost trajectory... \n");
  computeMinControlCostTrajectory();
  printf("STOMP initialized.\n");
}

void StompConstrained::createDiffMatrices()
{
  // cleanup
  diff_num_outputs_.clear();
  diff_output_offset_.clear();
  sparse_diff_matrix_.clear();
  sparse_diff_matrix_const_.clear();
  sparse_diff_vectors_const_.clear();

  // allocate memory
  diff_num_outputs_.resize(NUM_DIFF_RULES);
  diff_output_offset_.resize(NUM_DIFF_RULES);
  sparse_diff_matrix_.resize(NUM_DIFF_RULES);
  sparse_diff_matrix_const_.resize(NUM_DIFF_RULES);

  for (int a = 0; a < NUM_DIFF_RULES; ++a)
  {
    double mult = 1.0 / pow(dt_, a);
    int num_entries = num_time_steps_+DIFF_RULE_BANDWIDTHS[a]-1;
    diff_num_outputs_[a] = num_entries;
    diff_output_offset_[a] = (DIFF_RULE_BANDWIDTHS[a]/2);
    sparse_diff_matrix_[a] = Eigen::SparseMatrix<double>(num_entries, num_time_steps_);
    sparse_diff_matrix_const_[a] = Eigen::SparseMatrix<double>(num_entries, num_time_steps_all_);
    sparse_diff_matrix_[a].reserve(Eigen::VectorXi::Constant(num_time_steps_, DIFF_RULE_BANDWIDTHS[a]));
    sparse_diff_matrix_const_[a].reserve(Eigen::VectorXi::Constant(num_time_steps_all_, DIFF_RULE_BANDWIDTHS[a]));

    for (int i=0; i<num_entries; ++i)
    {
      for (int j=0; j<DIFF_RULE_BANDWIDTHS[a]; ++j)
      {
        int J = j + (DIFF_RULE_LENGTH - DIFF_RULE_BANDWIDTHS[a])/2;
        int k = i + j - (DIFF_RULE_BANDWIDTHS[a]-1);

        int full_traj_index = k + free_vars_start_;
        // if we're completely out of bounds, push it back in
        if (full_traj_index < 0)
          full_traj_index = 0;
        if (full_traj_index >= num_time_steps_all_)
          full_traj_index = num_time_steps_all_-1;

        if (k>=0 && k<num_time_steps_) // if we are inside the matrix
        {
          sparse_diff_matrix_[a].coeffRef(i, k) += mult * DIFF_RULES[a][J];
        }
        else // we are in the const part
        {
          sparse_diff_matrix_const_[a].coeffRef(i, full_traj_index) += mult * DIFF_RULES[a][J];
        }
      }
    }

    sparse_diff_matrix_[a].makeCompressed();
    sparse_diff_matrix_const_[a].makeCompressed();

  }
}

void StompConstrained::setInitialTrajectory(const Eigen::MatrixXd& initial_trajectory)
{
  initial_trajectory_ = initial_trajectory;
  sparse_diff_vectors_const_.resize(NUM_DIFF_RULES);

  // update the const part of differentiation vectors
  for (int a = 0; a < NUM_DIFF_RULES; ++a)
  {
    sparse_diff_vectors_const_[a] = Eigen::SparseMatrix<double>(diff_num_outputs_[a], num_joints_);
    sparse_diff_vectors_const_[a] = sparse_diff_matrix_const_[a] * initial_trajectory_;
  }

  // set the policy mean
  policy_mean_ = Eigen::VectorXd(num_variables_);
  for (int j=0; j<num_joints_; ++j)
  {
    int start_index = getIndex(j, 0);
    policy_mean_.segment(start_index, num_time_steps_) = initial_trajectory_.col(j).segment(free_vars_start_, num_time_steps_);
  }
}

void StompConstrained::createCostMatrix()
{
  // first the quadratic part
  sparse_quad_cost_ = Eigen::SparseMatrix<double>(num_variables_, num_variables_);
  sparse_quad_cost_.setZero();

  Eigen::SparseMatrix<double> R_block(num_time_steps_, num_time_steps_);

  for (int a=0; a<NUM_DIFF_RULES; ++a)
  {
    if (derivative_costs_[a] > 0.0)
    {
      R_block += derivative_costs_[a] * (sparse_diff_matrix_[a].transpose() * sparse_diff_matrix_[a]);
    }
  }

  int num_nonzeros = R_block.nonZeros();

  std::vector<Eigen::Triplet<double> > triplets;
  triplets.reserve(num_nonzeros * num_joints_);
  for (int k=0; k<R_block.outerSize(); ++k)
  {
    for (Eigen::SparseMatrix<double>::InnerIterator it(R_block,k); it; ++it)
    {
      for (int j=0; j<num_joints_; ++j)
      {
        int row_index = getIndex(j, it.row());
        int col_index = getIndex(j, it.col());
        triplets.push_back(Eigen::Triplet<double>(row_index, col_index, it.value()));
      }
    }
  }
  sparse_quad_cost_.setFromTriplets(triplets.begin(), triplets.end());

  // the linear part is done joint by joint anyway
  sparse_linear_cost_ = Eigen::SparseVector<double>(num_variables_);
  dense_linear_cost_ = Eigen::VectorXd::Zero(num_variables_);
  sparse_linear_cost_.setZero();
  for (int j=0; j<num_joints_; ++j)
  {
    Eigen::SparseVector<double> s_block(num_time_steps_);
    for (int a=0; a<NUM_DIFF_RULES; ++a)
    {
      s_block += derivative_costs_[a] * (sparse_diff_matrix_[a].transpose() * sparse_diff_vectors_const_[a].col(j));
    }
    for (Eigen::SparseVector<double>::InnerIterator it(s_block); it; ++it)
    {
      int index = getIndex(j, it.row());
      sparse_linear_cost_.coeffRef(index) = it.value();
    }
  }
  dense_linear_cost_ = sparse_linear_cost_;
}

void StompConstrained::computeCholeskyFactor()
{
  sparse_quad_cost_cholesky_.analyzePattern(sparse_quad_cost_);
  sparse_quad_cost_cholesky_.factorize(sparse_quad_cost_);
}

void StompConstrained::computeMinControlCostTrajectory()
{
  min_control_cost_trajectory_ = sparse_quad_cost_cholesky_.solve(-dense_linear_cost_);
}

void StompConstrained::setToMinControlCost()
{
  policy_mean_ = min_control_cost_trajectory_;
}

void StompConstrained::setEqualityConstraints(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
  has_equality_constraints_ = true;
  num_constraints_ = A.rows();
  dense_constraints_ = A;
  dense_constraints_const_ = b;

  computeConstraintProjector();
}

void StompConstrained::computeConstraintProjector()
{
//  Eigen::MatrixXd dense_V;
//  Eigen::MatrixXd dense_AV;
  STOMP_PROFILE(dense_V_ = sparse_quad_cost_cholesky_.solve(dense_constraints_.transpose()));
  STOMP_PROFILE(dense_AV_.noalias() = dense_constraints_ * dense_V_;
  dense_AV_ += 0.000001 * Eigen::VectorXd::Ones(num_constraints_).asDiagonal());
  //STOMP_PROFILE(dense_AV_ = dense_constraints_ * dense_V_);
  STOMP_PROFILE(dense_AV_solver_.compute(dense_AV_));
  //printf("AV rank = %d\n", dense_AV_solver_.rank());
  STOMP_PROFILE(dense_constraint_projector_ = (dense_AV_solver_.solve(dense_V_.transpose())).transpose());
}

void StompConstrained::getSamples(Eigen::MatrixXd& samples)
{
  generateSamples();
  samples = conditioned_samples_;
}

void StompConstrained::getSamples(std::vector<Eigen::MatrixXd*>& samples)
{
  generateSamples();
  for (int i=0; i<num_samples_; ++i)
  {
    trajectoryVectorToMatrix(conditioned_samples_.col(i), *(samples[i]));
  }
}

void StompConstrained::setSamples(const std::vector<Eigen::MatrixXd*>& samples)
{
  for (int i=0; i<num_samples_; ++i)
  {
    trajectoryMatrixToVector(*(samples[i]), conditioned_samples_.col(i));
  }
}


void StompConstrained::generateSamples()
{
  normal_samples_ = Eigen::MatrixXd(num_variables_, num_samples_);
  unconditioned_samples_ = Eigen::MatrixXd(num_variables_, num_samples_);
  conditioned_samples_ = Eigen::MatrixXd(num_variables_, num_samples_);

  for (int i=0; i<num_variables_; ++i)
  {
    for (int j=0; j<num_samples_; ++j)
    {
      normal_samples_(i,j) = noise_stddev_*gaussian_();
    }
  }

  unconditioned_samples_.noalias() = sparse_quad_cost_cholesky_.permutationPinv() *
      sparse_quad_cost_cholesky_.matrixU().triangularView<Eigen::Upper>().solve(normal_samples_);

  // add the mean
  for (int j=0; j<num_samples_; ++j)
  {
    unconditioned_samples_.col(j) += policy_mean_; // TODO: do some averaging with "min control cost" solution here
  }

  if (has_equality_constraints_)
  {
    Eigen::MatrixXd constraint_violations;
    constraint_violations.noalias() = (dense_constraints_ * unconditioned_samples_).colwise() + dense_constraints_const_;
    conditioned_samples_.noalias() = unconditioned_samples_ - dense_constraint_projector_ * constraint_violations;
  }
  else
  {
    conditioned_samples_ = unconditioned_samples_;
  }

}

void StompConstrained::setSampleCosts(Eigen::VectorXd& sample_costs)
{
  sample_costs_ = sample_costs;
}

void StompConstrained::updateMean()
{
  double costs_min = sample_costs_.minCoeff();
  double costs_max = sample_costs_.maxCoeff();
  double denom = costs_max - costs_min;
  if (denom < 1e-6)
    denom = 1e-6;

  sample_probabilities_ = ((-10.0*(sample_costs_.array() - costs_min))/denom).exp().matrix();
  double prob_sum = sample_probabilities_.sum();
  sample_probabilities_ = (sample_probabilities_.array() / prob_sum).matrix();

  policy_mean_ = Eigen::VectorXd::Zero(num_variables_);
  for (int s=0; s<num_samples_; ++s)
  {
    policy_mean_ += sample_probabilities_(s) * conditioned_samples_.col(s);
  }
  printf("Mean cost = %lf\n", double(policy_mean_.transpose() * sparse_quad_cost_ * policy_mean_ + sparse_linear_cost_.dot(policy_mean_)));
}

void StompConstrained::getMeanTrajectory(Eigen::MatrixXd& trajectory)
{
  trajectory = initial_trajectory_;

  for (int j=0; j<num_joints_; ++j)
  {
    int start_index = getIndex(j, 0);
    trajectory.col(j).segment(free_vars_start_, num_time_steps_) = policy_mean_.segment(start_index, num_time_steps_);
  }
}

template<typename Derived>
void StompConstrained::trajectoryVectorToMatrix(const Eigen::MatrixBase<Derived>& vector, Eigen::MatrixXd& matrix,
                                                bool only_free_vars)
{
  matrix.resize(num_time_steps_all_, num_joints_);

  if (!only_free_vars)
  {
    if (free_vars_start_ > 0)
      matrix.topRows(free_vars_start_) = initial_trajectory_.topRows(free_vars_start_);
    if (free_vars_end_ < num_time_steps_all_ - 1)
      matrix.bottomRows(num_time_steps_all_ - 1 - free_vars_end_) = initial_trajectory_.bottomRows(num_time_steps_all_ - 1 - free_vars_end_);
  }

  for (int j=0; j<num_joints_; ++j)
  {
    int start_index = getIndex(j, 0);
    matrix.col(j).segment(free_vars_start_, num_time_steps_) = vector.segment(start_index, num_time_steps_);
  }
}

template<typename Derived>
void StompConstrained::trajectoryMatrixToVector(const Eigen::MatrixXd& matrix, const Eigen::MatrixBase<Derived>& vector)
{
  for (int j=0; j<num_joints_; ++j)
  {
    int start_index = getIndex(j, 0);
    // Eigen requires this disgusting const_cast, see http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
    const_cast<Eigen::MatrixBase<Derived>& >(vector).segment(start_index, num_time_steps_) =
        matrix.col(j).segment(free_vars_start_, num_time_steps_);
  }
}
