#include <stomp/covariant_movement_primitive.h>

using namespace stomp;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TestCMP");
  ros::NodeHandle node_handle;

  CovariantMovementPrimitive cmp;

  int num_time_steps = 20;
  int num_dimensions = 1;
  double movement_duration = 1.0;
  double start = -1.0;
  double goal = 1.0;
  int constraint_time_step = 10;
  double constraint_pos = -1.0;

  std::vector<Eigen::MatrixXd> derivative_costs;
  std::vector<Eigen::VectorXd> initial_trajectory;

  derivative_costs.resize(1, Eigen::MatrixXd::Zero(num_time_steps + 2*TRAJECTORY_PADDING, NUM_DIFF_RULES));
  initial_trajectory.resize(1, Eigen::VectorXd::Zero(num_time_steps + 2*TRAJECTORY_PADDING));

  initial_trajectory[0].head(TRAJECTORY_PADDING) = Eigen::VectorXd::Ones(TRAJECTORY_PADDING) * start;
  initial_trajectory[0].tail(TRAJECTORY_PADDING) = Eigen::VectorXd::Ones(TRAJECTORY_PADDING) * goal;
  initial_trajectory[0](constraint_time_step + TRAJECTORY_PADDING) = constraint_pos;

  derivative_costs[0].col(STOMP_ACCELERATION) = Eigen::VectorXd::Ones(num_time_steps + 2*TRAJECTORY_PADDING);
  for (int i=0; i<constraint_time_step + TRAJECTORY_PADDING; ++i)
  {
    derivative_costs[0](i, STOMP_POSITION) = 10000000000.0;
  }
  //derivative_costs[0](constraint_time_step + TRAJECTORY_PADDING, STOMP_POSITION) = 10000000.0;

  cmp.initialize(num_time_steps, num_dimensions, movement_duration,
                 derivative_costs, initial_trajectory);
  cmp.setToMinControlCost();
  cmp.writeToFile("/tmp/cmp.txt");

  return 0;
}
