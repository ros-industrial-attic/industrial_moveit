#include <string>
#include <cstdio>
#include <boost/shared_ptr.hpp>
#include <stomp/stomp_constrained.h>

void saveSamples(const std::string& output_dir,
                 const std::vector<Eigen::MatrixXd*>& samples)
{
  for (size_t i=0; i<samples.size(); ++i)
  {
    std::stringstream filename;
    filename << output_dir << "/" << i << ".txt";
    FILE* file = fopen(filename.str().c_str(), "w");

    for (int r=0; r<samples[i]->rows(); ++r)
    {
      for (int c=0; c<samples[i]->cols(); ++c)
      {
        fprintf(file, "%f\t", samples[i]->coeff(r,c));
      }
      fprintf(file, "\n");
    }

    fclose(file);
  }
}

boost::shared_ptr<StompConstrained> getStomp(
    const int num_time_steps_all,
    const int num_joints,
    const int num_fixed_vars_start,
    const int num_fixed_vars_end,
    const int num_samples,
    const double position_cost,
    const double velocity_cost,
    const double acceleration_cost,
    const double jerk_cost,
    const double dt,
    const double stddev,
    const std::string& output_dir,
    const Eigen::MatrixXd& initial_trajectory)
{
  // shouldn't need to mess with these
  const int free_vars_start = num_fixed_vars_start;
  const int free_vars_end = num_time_steps_all - num_fixed_vars_end - 1;
  std::vector<double> derivative_costs;
  derivative_costs.push_back(position_cost);
  derivative_costs.push_back(velocity_cost);
  derivative_costs.push_back(acceleration_cost);
  derivative_costs.push_back(jerk_cost);

  boost::shared_ptr<StompConstrained> stomp_constrained(new StompConstrained());
  stomp_constrained->initialize(initial_trajectory, dt, free_vars_start,
                               free_vars_end, num_samples, derivative_costs);
  stomp_constrained->setToMinControlCost();
  stomp_constrained->noise_stddev_ = stddev;

  return stomp_constrained;
}

void getSamples(boost::shared_ptr<StompConstrained> stomp, int num_samples, const std::string& output_dir)
{
  std::vector<Eigen::MatrixXd> samples(num_samples);
  std::vector<Eigen::MatrixXd*> sample_ptrs(num_samples);
  for (int i=0; i<num_samples; ++i) sample_ptrs[i] = &(samples[i]);
  stomp->getSamples(sample_ptrs);
  saveSamples(output_dir, sample_ptrs);
}

void testUnconstrained()
{
  // tunable parameters
  const int num_time_steps_all = 100;
  const int num_joints = 2;
  const int num_fixed_vars_start = 5;
  const int num_fixed_vars_end = 5;
  const int num_samples = 100;
  const double position_cost = 0.000001;
  const double velocity_cost = 0.0;
  const double acceleration_cost = 1.0;
  const double jerk_cost = 0.0;
  const double dt = 0.01;
  const double stddev = 10.0;
  const std::string output_dir="/tmp/unconstrained";
  Eigen::MatrixXd initial_trajectory(num_time_steps_all, num_joints);
  initial_trajectory.col(0).head(num_fixed_vars_start) = Eigen::VectorXd::Zero(num_fixed_vars_start);
  initial_trajectory.col(1).head(num_fixed_vars_start) = Eigen::VectorXd::Zero(num_fixed_vars_start);
  initial_trajectory.col(0).tail(num_fixed_vars_end) = Eigen::VectorXd::Ones(num_fixed_vars_end);
  initial_trajectory.col(1).tail(num_fixed_vars_end) = Eigen::VectorXd::Zero(num_fixed_vars_end);

  boost::shared_ptr<StompConstrained> stomp = getStomp(num_time_steps_all, num_joints, num_fixed_vars_start, num_fixed_vars_end, num_samples, position_cost, velocity_cost, acceleration_cost, jerk_cost, dt, stddev, output_dir, initial_trajectory);
  getSamples(stomp, num_samples, output_dir);
}

void testEndpointLinearConstraint()
{
  // tunable parameters
  const int num_time_steps_all = 200;
  const int num_joints = 2;
  const int num_fixed_vars_start = 5;
  const int num_fixed_vars_end = 5;
  const int num_samples = 100;
  const double position_cost = 0.000001;
  const double velocity_cost = 0.0;
  const double acceleration_cost = 1.0;
  const double jerk_cost = 0.0;
  const double dt = 0.01;
  const double stddev = 10.0;
  const std::string output_dir="/tmp/endpointLinearConstraint";
  Eigen::MatrixXd initial_trajectory(num_time_steps_all, num_joints);
  initial_trajectory.col(0).head(num_fixed_vars_start) = Eigen::VectorXd::Zero(num_fixed_vars_start);
  initial_trajectory.col(1).head(num_fixed_vars_start) = Eigen::VectorXd::Zero(num_fixed_vars_start);
  initial_trajectory.col(0).tail(num_fixed_vars_end) = Eigen::VectorXd::Ones(num_fixed_vars_end);
  initial_trajectory.col(1).tail(num_fixed_vars_end) = Eigen::VectorXd::Zero(num_fixed_vars_end);

  boost::shared_ptr<StompConstrained> stomp = getStomp(num_time_steps_all, num_joints, num_fixed_vars_start, num_fixed_vars_end, num_samples, position_cost, velocity_cost, acceleration_cost, jerk_cost, dt, stddev, output_dir, initial_trajectory);

  const int num_constraints = 7;
  Eigen::MatrixXd constraints_A(num_constraints, stomp->getNumVariables());
  Eigen::VectorXd constraints_b(num_constraints);
  constraints_A.setZero();
  constraints_b.setZero();

  // last time-step
  int t = 100;
  constraints_A(0, stomp->getIndex(0, t)) = 1.0;
  constraints_A(0, stomp->getIndex(1, t)) = 1.0;
  constraints_b(0) = -1.0;

  constraints_A(1, stomp->getIndex(0, t)) = -1.0;
  constraints_A(1, stomp->getIndex(0, t-1)) = 1.0;
  constraints_b(1) = 0.02;

  constraints_A(2, stomp->getIndex(1, t)) = -1.0;
  constraints_A(2, stomp->getIndex(1, t-1)) = 1.0;
  constraints_b(2) = 0.02;

  constraints_A(3, stomp->getIndex(0, t-1)) = -1.0;
  constraints_A(3, stomp->getIndex(0, t-2)) = 1.0;
  constraints_b(3) = 0.02;

  constraints_A(4, stomp->getIndex(1, t-1)) = -1.0;
  constraints_A(4, stomp->getIndex(1, t-2)) = 1.0;
  constraints_b(4) = 0.02;

  constraints_A(5, stomp->getIndex(0, t)) = -1.0;
  constraints_A(5, stomp->getIndex(0, t-1)) = 1.0;
  constraints_A(5, stomp->getIndex(1, t)) = 1.0;
  constraints_A(5, stomp->getIndex(1, t-1)) = -1.0;
  constraints_b(5) = 0.0;

  constraints_A(6, stomp->getIndex(0, t-1)) = -1.0;
  constraints_A(6, stomp->getIndex(0, t-2)) = 1.0;
  constraints_A(6, stomp->getIndex(1, t-1)) = 1.0;
  constraints_A(6, stomp->getIndex(1, t-2)) = -1.0;
  constraints_b(6) = 0.0;

  stomp->setEqualityConstraints(constraints_A, constraints_b);
  getSamples(stomp, num_samples, output_dir);
}

void testWaypointConstraint()
{
  // tunable parameters
  const int num_time_steps_all = 100;
  const int num_joints = 2;
  const int num_fixed_vars_start = 5;
  const int num_fixed_vars_end = 5;
  const int num_samples = 100;
  const double position_cost = 0.000001;
  const double velocity_cost = 0.0;
  const double acceleration_cost = 1.0;
  const double jerk_cost = 0.0;
  const double dt = 0.01;
  const double stddev = 10.0;
  const std::string output_dir="/tmp/waypointConstraint";
  Eigen::MatrixXd initial_trajectory(num_time_steps_all, num_joints);
  initial_trajectory.col(0).head(num_fixed_vars_start) = Eigen::VectorXd::Zero(num_fixed_vars_start);
  initial_trajectory.col(1).head(num_fixed_vars_start) = Eigen::VectorXd::Zero(num_fixed_vars_start);
  initial_trajectory.col(0).tail(num_fixed_vars_end) = Eigen::VectorXd::Ones(num_fixed_vars_end);
  initial_trajectory.col(1).tail(num_fixed_vars_end) = Eigen::VectorXd::Zero(num_fixed_vars_end);

  boost::shared_ptr<StompConstrained> stomp = getStomp(num_time_steps_all, num_joints, num_fixed_vars_start, num_fixed_vars_end, num_samples, position_cost, velocity_cost, acceleration_cost, jerk_cost, dt, stddev, output_dir, initial_trajectory);

  const int num_constraints = 2;
  Eigen::MatrixXd constraints_A(num_constraints, stomp->getNumVariables());
  Eigen::VectorXd constraints_b(num_constraints);
  constraints_A.setZero();
  constraints_b.setZero();

  // middle time-step
  int t = 45;

  constraints_A(0, stomp->getIndex(0, t)) = 1.0;
  constraints_b(0) = -0.5;

  constraints_A(1, stomp->getIndex(1, t)) = 1.0;
  constraints_b(1) = -0.1;

  stomp->setEqualityConstraints(constraints_A, constraints_b);
  getSamples(stomp, num_samples, output_dir);
}

void testWaypointWithVelocityConstraint()
{
  // tunable parameters
  const int num_time_steps_all = 100;
  const int num_joints = 2;
  const int num_fixed_vars_start = 5;
  const int num_fixed_vars_end = 5;
  const int num_samples = 100;
  const double position_cost = 0.000001;
  const double velocity_cost = 0.0;
  const double acceleration_cost = 1.0;
  const double jerk_cost = 0.0;
  const double dt = 0.01;
  const double stddev = 10.0;
  const std::string output_dir="/tmp/waypointWithVelocityConstraint";
  Eigen::MatrixXd initial_trajectory(num_time_steps_all, num_joints);
  initial_trajectory.col(0).head(num_fixed_vars_start) = Eigen::VectorXd::Zero(num_fixed_vars_start);
  initial_trajectory.col(1).head(num_fixed_vars_start) = Eigen::VectorXd::Zero(num_fixed_vars_start);
  initial_trajectory.col(0).tail(num_fixed_vars_end) = Eigen::VectorXd::Ones(num_fixed_vars_end);
  initial_trajectory.col(1).tail(num_fixed_vars_end) = Eigen::VectorXd::Zero(num_fixed_vars_end);

  boost::shared_ptr<StompConstrained> stomp = getStomp(num_time_steps_all, num_joints, num_fixed_vars_start, num_fixed_vars_end, num_samples, position_cost, velocity_cost, acceleration_cost, jerk_cost, dt, stddev, output_dir, initial_trajectory);

  const int num_constraints = 4;
  Eigen::MatrixXd constraints_A(num_constraints, stomp->getNumVariables());
  Eigen::VectorXd constraints_b(num_constraints);
  constraints_A.setZero();
  constraints_b.setZero();

  // middle time-step
  int t = 45;

  constraints_A(0, stomp->getIndex(0, t)) = 1.0;
  constraints_b(0) = -0.5;

  constraints_A(1, stomp->getIndex(1, t)) = 1.0;
  constraints_b(1) = -0.1;

  // dx / dt
  constraints_A(2, stomp->getIndex(0, t+1)) = 1.0;
  constraints_A(2, stomp->getIndex(0, t-1)) = -1.0;
  constraints_b(2) = -0.02;

  // dy / dt
  constraints_A(3, stomp->getIndex(1, t+1)) = 1.0;
  constraints_A(3, stomp->getIndex(1, t-1)) = -1.0;
  constraints_b(3) = -0.1;

  stomp->setEqualityConstraints(constraints_A, constraints_b);
  getSamples(stomp, num_samples, output_dir);
}

void testWaypointWallConstraint()
{
  // tunable parameters
  const int num_time_steps_all = 100;
  const int num_joints = 2;
  const int num_fixed_vars_start = 5;
  const int num_fixed_vars_end = 5;
  const int num_samples = 100;
  const double position_cost = 0.000001;
  const double velocity_cost = 0.0;
  const double acceleration_cost = 1.0;
  const double jerk_cost = 0.0;
  const double dt = 0.01;
  const double stddev = 10.0;
  const std::string output_dir="/tmp/waypointWallConstraint";
  Eigen::MatrixXd initial_trajectory(num_time_steps_all, num_joints);
  initial_trajectory.col(0).head(num_fixed_vars_start) = Eigen::VectorXd::Zero(num_fixed_vars_start);
  initial_trajectory.col(1).head(num_fixed_vars_start) = Eigen::VectorXd::Zero(num_fixed_vars_start);
  initial_trajectory.col(0).tail(num_fixed_vars_end) = Eigen::VectorXd::Ones(num_fixed_vars_end);
  initial_trajectory.col(1).tail(num_fixed_vars_end) = Eigen::VectorXd::Zero(num_fixed_vars_end);

  boost::shared_ptr<StompConstrained> stomp = getStomp(num_time_steps_all, num_joints, num_fixed_vars_start, num_fixed_vars_end, num_samples, position_cost, velocity_cost, acceleration_cost, jerk_cost, dt, stddev, output_dir, initial_trajectory);

  const int num_constraints = 11;
  Eigen::MatrixXd constraints_A(num_constraints, stomp->getNumVariables());
  Eigen::VectorXd constraints_b(num_constraints);
  constraints_A.setZero();
  constraints_b.setZero();

  // middle time-steps
  int c=0;
  for (int t=40; t<=50; ++t)
  {
    constraints_A(c, stomp->getIndex(0, t)) = 1.0;
    constraints_A(c, stomp->getIndex(1, t)) = 1.0;
    constraints_b(c) = -0.5;
    ++c;

//    constraints_A(c, stomp->getIndex(1, t+1)) = 1.0;
//    constraints_A(c, stomp->getIndex(1, t-1)) = -1.0;
//    constraints_b(c) = 0.0;
//    ++c;
  }

  stomp->setEqualityConstraints(constraints_A, constraints_b);
  getSamples(stomp, num_samples, output_dir);
}

int main(int argc, char** argv)
{
  testUnconstrained();
  testEndpointLinearConstraint();
  testWaypointConstraint();
  testWaypointWithVelocityConstraint();
  testWaypointWallConstraint();
}
