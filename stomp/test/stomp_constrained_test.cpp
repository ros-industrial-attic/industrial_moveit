#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/QR>
#include <Eigen/OrderingMethods>
#include <vector>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <ros/time.h>
#include <boost/progress.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

static const double diff_2[2] = {-1.0, 1.0};
static const double diff_3[3] = {1.0, -2.0, 1.0};

class StompConstrainedTest
{
public:

  StompConstrainedTest():
    normal_dist_(0.0, 1.0),
    gaussian_(rng_, normal_dist_)
  {
    rviz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("visualization", 100, false);
  }

  int getIndex(int joint, int time_step)
  {
    return time_step + joint * num_time_steps;
  }

  void createDiffMatrix(int bandwidth=2)
  {
    // allocate memory
    sparse_A_ = Eigen::SparseMatrix<double>(num_time_steps+bandwidth-1, num_time_steps);

    const double* diff_rule;
    if (bandwidth==2)
      diff_rule = diff_2;
    else
      diff_rule = diff_3;

    // create triplets
    std::vector<Eigen::Triplet<double> > triplets;
    triplets.reserve(bandwidth*(num_time_steps+bandwidth-1));
    for (int i=0; i<num_time_steps+bandwidth-1; ++i)
    {
      for (int j=0; j<bandwidth; ++j)
      {
        int k = i + j - (bandwidth-1);
        if (k>=0 && k<num_time_steps)
        {
          triplets.push_back(Eigen::Triplet<double>(i, k, diff_rule[j]));
        }
      }

    }

     // fill the sparse matrix
    sparse_A_.setFromTriplets(triplets.begin(), triplets.end());
  }

  // creates a diff matrix for accelerations, allowing the end-point to float
  void createEndUnconstrainedDiffMatrix()
  {
    // allocate memory
    sparse_A_ = Eigen::SparseMatrix<double>(num_time_steps+1, num_time_steps);

    const int bandwidth = 3;
    const double* diff_rule = diff_3;

    // create triplets
    std::vector<Eigen::Triplet<double> > triplets;
    triplets.reserve(bandwidth*(num_time_steps+1));

    // first two rows for constrained start
    triplets.push_back(Eigen::Triplet<double>(0, 0, diff_rule[2]));
    triplets.push_back(Eigen::Triplet<double>(1, 0, diff_rule[1]));
    triplets.push_back(Eigen::Triplet<double>(1, 1, diff_rule[2]));
    int row=2;

    // middle elements
    for (int i=1; i<num_time_steps-1; ++i)
    {
      triplets.push_back(Eigen::Triplet<double>(row, i-1, diff_rule[0]));
      triplets.push_back(Eigen::Triplet<double>(row, i, diff_rule[1]));
      triplets.push_back(Eigen::Triplet<double>(row, i+1, diff_rule[2]));
      row++;
    }

    // last row is for the unconstrained end
    triplets.push_back(Eigen::Triplet<double>(row, num_time_steps - 2, diff_rule[0]));
    triplets.push_back(Eigen::Triplet<double>(row, num_time_steps - 1, diff_rule[1] + diff_rule[2]));

     // fill the sparse matrix
    sparse_A_.setFromTriplets(triplets.begin(), triplets.end());
  }

  // creates a diff matrix for accelerations, allowing the start and end-point to float
  void createUnconstrainedDiffMatrix()
  {
    // allocate memory
    sparse_A_ = Eigen::SparseMatrix<double>(num_time_steps, num_time_steps);

    const int bandwidth = 3;
    const double* diff_rule = diff_3;

    // create triplets
    std::vector<Eigen::Triplet<double> > triplets;
    triplets.reserve(bandwidth*(num_time_steps));

    // first row for constrained start
    triplets.push_back(Eigen::Triplet<double>(0, 0, diff_rule[0] + diff_rule[1]));
    triplets.push_back(Eigen::Triplet<double>(0, 1, diff_rule[2]));
    int row=1;

    // middle elements
    for (int i=1; i<num_time_steps-1; ++i)
    {
      triplets.push_back(Eigen::Triplet<double>(row, i-1, diff_rule[0]));
      triplets.push_back(Eigen::Triplet<double>(row, i, diff_rule[1]));
      triplets.push_back(Eigen::Triplet<double>(row, i+1, diff_rule[2]));
      row++;
    }

    // last row is for the unconstrained end
    triplets.push_back(Eigen::Triplet<double>(row, num_time_steps - 2, diff_rule[0]));
    triplets.push_back(Eigen::Triplet<double>(row, num_time_steps - 1, diff_rule[1] + diff_rule[2]));

     // fill the sparse matrix
    sparse_A_.setFromTriplets(triplets.begin(), triplets.end());
  }

  void createCostMatrix()
  {
    sparse_R_ = Eigen::SparseMatrix<double>(size,size);
    Eigen::SparseMatrix<double> R_block = sparse_A_.transpose() * sparse_A_;

    int num_nonzeros = R_block.nonZeros();

    std::vector<Eigen::Triplet<double> > triplets;
    triplets.reserve(num_nonzeros * num_joints);

    for (int k=0; k<R_block.outerSize(); ++k)
    {
      for (Eigen::SparseMatrix<double>::InnerIterator it(R_block,k); it; ++it)
      {
        for (int j=0; j<num_joints; ++j)
        {
          int row_index = getIndex(j, it.row());
          int col_index = getIndex(j, it.col());
          triplets.push_back(Eigen::Triplet<double>(row_index, col_index, it.value()));
        }
      }
    }

    sparse_R_.setFromTriplets(triplets.begin(), triplets.end());
    for (int i=0; i<size; ++i)
      sparse_R_.coeffRef(i,i) += 0.00001;
  }

  void computeCholeskyFactor()
  {
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double> >& sparse_solver = sparse_cholesky_solver_;

    sparse_solver.analyzePattern(sparse_R_);
    sparse_solver.factorize(sparse_R_);
  }

  void createConstraintMatrix()
  {
    //num_constraints_ = 2;
    //num_constraints_ = num_time_steps + 2;
    num_constraints_ = num_time_steps + 4;

    // allocate memory
    dense_C_ = Eigen::MatrixXd::Zero(num_constraints_, size);
    dense_b_ = Eigen::VectorXd::Zero(num_constraints_);

    for (int t=0; t<num_time_steps; ++t)
    {
      for (int j=0; j<num_joints; ++j)
      {
        int row = t;
        int col = getIndex(j, t);
        dense_C_(row, col) = 1.0;
      }
    }
  }

  void clip(double& value, const double limit)
  {
    if (value > limit)
      value = limit;
    if (value < -limit)
      value = -limit;
  }

  // update the constraints which are only locally linear
  void updateConstraintMatrix()
  {
    Eigen::MatrixXd error_vectors(num_joints, 2);

    // start and end-point constraints
    for (int i=0; i<2; ++i) // i==0 -> start;   i==1 -> end
    {
      double des_x=0.0, des_y=0.0;
      if (i == 0)
      {
        des_x = 0.8;
        des_y = -0.4;
      }
      else
      {
        des_x = 0.8;
        des_y = 0.4;
      }

      int t = i * (num_time_steps-1); // which time-step
      //int t = num_time_steps - 1;
      int c = num_time_steps + i*2; // which constraint index
      //c = 0;

      double cur_x = mean_link_positions_x_(getIndex(num_joints-1, t));
      double cur_y = mean_link_positions_y_(getIndex(num_joints-1, t));

      printf("i=%d, cur_x=%lf, cur_y=%lf\n", i, cur_x, cur_y);

      // first, error_vectors = joint positions
      error_vectors(0,0) = 0.0;
      error_vectors(0,1) = 0.0;
      for (int j=0; j<num_joints-1; ++j)
      {
        int ind = getIndex(j, t);
        error_vectors(j+1, 0) = mean_link_positions_x_(ind);
        error_vectors(j+1, 1) = mean_link_positions_y_(ind);
      }
      // now subtract: endeff - joint_pos
      error_vectors.col(0) = (cur_x - error_vectors.col(0).array()).matrix();
      error_vectors.col(1) = (cur_y - error_vectors.col(1).array()).matrix();

      // assign the constraint
      dense_b_(c) = 0.0;
      dense_b_(c+1) = 0.0;
      for (int j=0; j<num_joints; ++j)
      {
        int ind = getIndex(j, t);
        dense_C_(c, ind) = -error_vectors(j,1);
        dense_C_(c+1, ind) = error_vectors(j,0);

        dense_b_(c) += dense_C_(c, ind) * policy_mean_(ind);
        dense_b_(c+1) += dense_C_(c+1, ind) * policy_mean_(ind);
      }

      double des_x_vel = des_x - cur_x;
      double des_y_vel = des_y - cur_y;
      //clip(des_x_vel, 0.2);
      //clip(des_y_vel, 0.2);
      dense_b_(c) += des_x_vel;
      dense_b_(c+1) += des_y_vel;

      //clip(dense_b_(c), 0.05);
      //clip(dense_b_(c+1), 0.05);
      //std::cout << dense_C_.row(c) << std::endl;
      //std::cout << dense_b_.row(c) << std::endl;
      //std::cout << dense_C_.row(c+1) << std::endl;
      //std::cout << dense_b_.row(c+1) << std::endl;
    }

  }

  void computeConstraintProjector()
  {
      //Eigen::SparseMatrix<double> sparse_V = sparse_cholesky_solver.solve(sparse_C); // 2.71ms
      //Eigen::MatrixXd dense_V = sparse_V; // slow

    Eigen::MatrixXd dense_V = sparse_cholesky_solver_.solve(dense_C_.transpose()); // 1.5 ms
    Eigen::MatrixXd dense_AV = dense_C_ * dense_V;
    dense_projector_ = Eigen::MatrixXd(size, num_constraints_);
    dense_projector_.transpose() = dense_AV.ldlt().solve(dense_V.transpose());
  }

  void initializeMean()
  {
    policy_mean_ = Eigen::VectorXd::Zero(size);
//    double bend = 1.0;
//    double comp = -1.0 / (num_joints-1);
//    for (int t=0; t<num_time_steps; ++t)
//    {
//      policy_mean_(getIndex(0, t)) = bend;
//      for (int j=1; j<num_joints; ++j)
//      {
//        policy_mean_(getIndex(j, t)) = comp;
//      }
//    }
  }

  void generateSamples()
  {
    Eigen::MatrixXd normal_samples = Eigen::MatrixXd(size, num_samples);

    for (int i=0; i<size; ++i)
    {
      for (int j=0; j<num_samples; ++j)
      {
        //normal_samples(i,j) = 0.00*gaussian_();
        normal_samples(i,j) = noise_stddev_*gaussian_();
      }
    }
    unconditioned_samples_ = sparse_cholesky_solver_.permutationPinv() *
        sparse_cholesky_solver_.matrixU().triangularView<Eigen::Upper>().solve(normal_samples);

    // add the mean
    for (int j=0; j<num_samples; ++j)
    {
      unconditioned_samples_.col(j) += 0.975 * policy_mean_;
    }

    Eigen::MatrixXd constraint_violations = (dense_C_ * unconditioned_samples_).colwise() - dense_b_;
    conditioned_samples_ = unconditioned_samples_ - dense_projector_ * constraint_violations;

    printf("Constraint violations before = %f\n", constraint_violations.norm());
    constraint_violations = (dense_C_ * conditioned_samples_).colwise() - dense_b_; // num_constraints x num_samples
    printf("Constraint violations after = %f\n", constraint_violations.norm());
  }

  void computeFKForSamples()
  {
    samples_link_positions_x_.resize(conditioned_samples_.rows(), conditioned_samples_.cols());
    samples_link_positions_y_.resize(conditioned_samples_.rows(), conditioned_samples_.cols());
    computeFK(conditioned_samples_, samples_link_positions_x_, samples_link_positions_y_);
  }

  void computeFKForMean()
  {
    mean_link_positions_x_.resize(policy_mean_.rows());
    mean_link_positions_y_.resize(policy_mean_.rows());
    computeFK(policy_mean_, mean_link_positions_x_, mean_link_positions_y_);
  }

  void computeCostsForSamples()
  {
    sample_costs_ = Eigen::VectorXd::Zero(num_samples);
    sample_probabilities_ = Eigen::VectorXd::Zero(num_samples);

    double dist_threshold = 0.2;
    double sphere_center_x = 0.8;

    for (int s=0; s<num_samples; ++s)
    {
      for (int j=0; j<num_joints; ++j)
      {
        for (int t=1; t<num_time_steps-1; ++t)
        {
          int ind = getIndex(j, t);
          int prev_ind = getIndex(j, t-1);
          int next_ind = getIndex(j, t+1);
          double vel = sqrt(pow(samples_link_positions_x_(next_ind, s) - samples_link_positions_x_(prev_ind, s), 2.0) +
              pow(samples_link_positions_y_(next_ind, s) - samples_link_positions_y_(prev_ind, s), 2.0));

          double sqr_dist = pow(samples_link_positions_x_(ind, s) - sphere_center_x, 2.0) +
              pow(samples_link_positions_y_(ind, s), 2.0);
          if (sqr_dist < dist_threshold * dist_threshold)
          {
            //sample_costs_(s) += ((dist_threshold * dist_threshold) - sqr_dist) * vel;
            sample_costs_(s) += (num_joints - j) * vel;
          }

        }
      }
    }

    double costs_min = sample_costs_.minCoeff();
    double costs_max = sample_costs_.maxCoeff();
    double denom = costs_max - costs_min;
    if (denom < 1e-6)
      denom = 1e-6;

    sample_probabilities_ = ((-10.0*(sample_costs_.array() - costs_min))/denom).exp().matrix();
    double prob_sum = sample_probabilities_.sum();
    sample_probabilities_ = (sample_probabilities_.array() / prob_sum).matrix();
  }

  template<typename Derived1, typename Derived2>
  void computeFK(const Eigen::MatrixBase<Derived1>& samples, Eigen::MatrixBase<Derived2>& link_positions_x, Eigen::MatrixBase<Derived2>& link_positions_y)
  {
    double length = 1.0 / num_joints;

    int num_samples = samples.cols();
    Eigen::MatrixBase<Derived2>& angle_sums = link_positions_x; // just reusing the same memory
    angle_sums = samples;

    for (int j=1; j<num_joints; ++j)
    {
      int index = getIndex(j, 0);
      int prev_index = getIndex(j-1, 0);
      angle_sums.middleRows(index, num_time_steps) += angle_sums.middleRows(prev_index, num_time_steps);
    }
    link_positions_y = link_positions_x;

    link_positions_y = (link_positions_y.array().sin() * length).matrix();
    link_positions_x = (link_positions_x.array().cos() * length).matrix();

    for (int j=1; j<num_joints; ++j)
    {
      int index = getIndex(j, 0);
      int prev_index = getIndex(j-1, 0);
      link_positions_x.middleRows(index, num_time_steps) +=
          link_positions_x.middleRows(prev_index, num_time_steps);
      link_positions_y.middleRows(index, num_time_steps) +=
          link_positions_y.middleRows(prev_index, num_time_steps);
    }
  }

  void updateMean()
  {
    policy_mean_ = Eigen::VectorXd::Zero(size);
    for (int s=0; s<num_samples; ++s)
    {
      policy_mean_ += sample_probabilities_(s) * conditioned_samples_.col(s);
    }
    printf("Mean cost = %lf\n", double(policy_mean_.transpose() * sparse_R_ * policy_mean_));
  }

  void visualizeTrajectories()
  {
    if (!ros::ok())
      exit(0);
    for (int i=0; i<samples_link_positions_x_.cols(); ++i)
      visualizeTrajectory(samples_link_positions_x_, samples_link_positions_y_, false, i, double(sample_probabilities_(i)/2.0 + 0.5));
    //ros::Duration(1.0).sleep();
  }

  void visualizeTrajectory(Eigen::MatrixXd& link_positions_x, Eigen::MatrixXd& link_positions_y, bool noiseless, int id, double prob)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "BASE";
    marker.header.stamp = ros::Time::now();
    marker.ns="trajectory";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.resize(num_time_steps);
    //marker.colors.resize(num_time_steps_);
    for (int t=0; t<num_time_steps; ++t)
    {
      int j = getIndex(num_joints-1, t);
      marker.points[t].y = link_positions_y(j, id);
      marker.points[t].z = link_positions_x(j, id);
      marker.points[t].x = 0.0;
    }
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    if (noiseless)
    {
      marker.scale.x = 0.01;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }
    else
    {
      marker.scale.x = 0.002;
      marker.color.a = prob;
      marker.color.r = 0.2;
      marker.color.g = 0.5;
      marker.color.b = 0.5;
    }
    rviz_pub_.publish(marker);
  }

  void displayGoals()
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "BASE";
    marker.header.stamp = ros::Time::now();
    marker.ns="goals";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.resize(2);
    marker.points[0].z = 0.8;
    marker.points[0].y = -0.4;
    marker.points[0].x = 0.0;
    marker.points[1].z = 0.8;
    marker.points[1].y = 0.4;
    marker.points[1].x = 0.0;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    rviz_pub_.publish(marker);

    marker.points.resize(1);
    marker.points[0].z = 0.8;
    marker.points[0].y = 0.0;
    marker.points[0].x = 0.0;
    marker.ns = "obstacle";
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;
    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    rviz_pub_.publish(marker);

  }

  void animateTrajectories()
  {
    animateTrajectory(mean_link_positions_x_, mean_link_positions_y_, 0);
  }

  template<typename Derived>
  void animateTrajectory(Eigen::MatrixBase<Derived>& link_positions_x, Eigen::MatrixBase<Derived>& link_positions_y, int id)
  {
    for (int t=0; t<num_time_steps; ++t)
    {
      if (!ros::ok())
        exit(0);

      visualization_msgs::Marker marker;
      marker.header.frame_id = "BASE";
      marker.header.stamp = ros::Time::now();
      marker.ns="animation_lines";
      marker.id = id;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.points.resize(num_joints+1);
      marker.points[0].x = 0.0;
      marker.points[0].y = 0.0;
      marker.points[0].z = 0.0;
      //marker.colors.resize(num_time_steps_);
      for (int j=0; j<num_joints; ++j)
      {
        int ind = getIndex(j, t);
        marker.points[j+1].y = link_positions_y(ind, id);
        marker.points[j+1].z = link_positions_x(ind, id);
        marker.points[j+1].x = 0.0;
      }
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.002;
      marker.scale.y = 0.002;
      marker.scale.z = 0.002;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      rviz_pub_.publish(marker);

      marker.type = visualization_msgs::Marker::SPHERE_LIST;
      marker.ns="animation_spheres";
      marker.scale.x = 0.005;
      marker.scale.y = 0.005;
      marker.scale.z = 0.005;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.colors.resize(num_joints+1);
      for (int i=0; i<num_joints; ++i)
        marker.colors[i] = marker.color;
      marker.colors[num_joints].r = 0.0;
      marker.colors[num_joints].g = 1.0;
      marker.colors[num_joints].b = 0.0;
      marker.colors[num_joints].a = 1.0;
      rviz_pub_.publish(marker);

      ros::Duration(0.05).sleep();
    }
  }

  void writeToFile(int iter)
  {
    std::stringstream filename;
    filename << "/tmp/stomp_constrained_iter_" << iter << ".txt";
    FILE* file = fopen(filename.str().c_str(), "w");

    for (int j=0; j<num_joints; ++j)
    {
      for (int t=0; t<num_time_steps; t+=9)
      {
        int ind = getIndex(j, t);
        fprintf(file, "%f\t%f\t", mean_link_positions_x_(ind), mean_link_positions_y_(ind));
      }
      fprintf(file,"\n");
    }

    fclose(file);
  }

  void plotMeanTrajectory()
  {
    plotTrajectory(mean_link_positions_x_, mean_link_positions_y_, 0);
  }

  void plotInitialMeanTrajectory()
  {
    plotTrajectory(mean_link_positions_x_, mean_link_positions_y_, 0, true);
  }

  template<typename Derived>
  void plotTrajectory(Eigen::MatrixBase<Derived>& link_positions_x, Eigen::MatrixBase<Derived>& link_positions_y, int id, bool initial=false)
  {
    for (int t=0; t<num_time_steps; t+=9)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "BASE";
      marker.header.stamp = ros::Time::now();
      marker.ns="animation_lines";
      marker.id = t + (initial?0:num_time_steps);
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.points.resize(num_joints+1);
      marker.points[0].x = 0.0;
      marker.points[0].y = 0.0;
      marker.points[0].z = 0.0;
      //marker.colors.resize(num_time_steps_);
      for (int j=0; j<num_joints; ++j)
      {
        int ind = getIndex(j, t);
        marker.points[j+1].y = link_positions_y(ind, id);
        marker.points[j+1].z = link_positions_x(ind, id);
        marker.points[j+1].x = 0.0;
      }
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.002;
      marker.scale.y = 0.002;
      marker.scale.z = 0.002;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      if (initial)
      {
        marker.color.a = 0.2;
        marker.color.r = 0.0;
      }
      rviz_pub_.publish(marker);

      marker.type = visualization_msgs::Marker::SPHERE_LIST;
      marker.ns="animation_spheres";
      marker.scale.x = 0.005;
      marker.scale.y = 0.005;
      marker.scale.z = 0.005;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.colors.resize(num_joints+1);
      for (int i=0; i<num_joints; ++i)
        marker.colors[i] = marker.color;
      marker.colors[num_joints].r = 0.0;
      marker.colors[num_joints].g = 0.5;
      marker.colors[num_joints].b = 0.0;
      marker.colors[num_joints].a = 1.0;
      if (initial)
      {
        for (int i=0; i<num_joints; ++i)
        {
          marker.colors[i].a = 0.2;
          marker.colors[i].b = 0.0;
        }
      }
      rviz_pub_.publish(marker);

      //ros::Duration(0.05).sleep();
    }
  }

  double noise_stddev_;

private:
  // A = num diff matrix
  // R = A^T A  (quadratic cost matrix)
  // R = L L^T  (L -> chol_R)
  // C = constraint matrix
  Eigen::MatrixXd dense_C_, dense_projector_;
  Eigen::VectorXd dense_b_;
  Eigen::SparseMatrix<double> sparse_A_, sparse_R_;//, sparse_C_;

  Eigen::SimplicialLLT<Eigen::SparseMatrix<double> > sparse_cholesky_solver_;

  Eigen::VectorXd policy_mean_;

  Eigen::MatrixXd normal_samples_;
  Eigen::MatrixXd unconditioned_samples_;
  Eigen::MatrixXd conditioned_samples_;

  Eigen::MatrixXd samples_link_positions_x_;
  Eigen::MatrixXd samples_link_positions_y_;

  Eigen::VectorXd sample_costs_;
  Eigen::VectorXd sample_probabilities_;

  Eigen::VectorXd mean_link_positions_x_;
  Eigen::VectorXd mean_link_positions_y_;

  boost::mt19937 rng_;
  boost::normal_distribution<> normal_dist_;
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> > gaussian_;

  static const int num_time_steps = 100;
  static const int num_joints = 100;
  static const int num_samples = 50;
  static const int size = num_time_steps * num_joints;
  int num_constraints_;

  ros::NodeHandle node_handle_;
  ros::Publisher rviz_pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stomp_constrained_test");
  ros::AsyncSpinner async_spinner(0);
  async_spinner.start();

  // create the differentiation matrix

  StompConstrainedTest ept;
  //ept.createDiffMatrix(3);
  ept.createUnconstrainedDiffMatrix();
  ept.createCostMatrix();
  ept.computeCholeskyFactor();

  ept.initializeMean();
  ept.computeFKForMean();
  ros::Duration(1.0).sleep();
  ept.plotInitialMeanTrajectory();
  ept.createConstraintMatrix();
  ept.noise_stddev_ = 0.001;
  {
    boost::progress_timer t;
    for (int iter=0; iter<200; ++iter)
    {
      printf(".");
      fflush(stdout);
      if (!ros::ok())
        exit(0);

      ept.displayGoals();

      ept.computeFKForMean();
      ept.plotMeanTrajectory();
      ept.writeToFile(iter);
      ept.updateConstraintMatrix();
      ept.computeConstraintProjector();
      ept.generateSamples();
      ept.computeFKForSamples();
      ept.computeCostsForSamples();
      //ept.visualizeTrajectories();
      ept.updateMean();
      ept.noise_stddev_ *= 0.998;
    }
  }
  ept.computeFKForMean();
  //ept.animateTrajectories();
  ept.plotMeanTrajectory();

  return 0;
}
