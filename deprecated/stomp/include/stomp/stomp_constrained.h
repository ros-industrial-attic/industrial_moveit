#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/progress.hpp>

/////// a convenient macro for profiling a function
#define STOMP_PROFILE(func)             \
  {                                     \
    printf("Starting %s...\n", #func);  \
    boost::progress_timer t;            \
    func;                               \
    printf("%s took: ", #func);         \
  }

/////// definitions of finite differencing rules (all central differences)
static const int NUM_DIFF_RULES = 4;
static const int DIFF_RULE_LENGTH = 5;
static const int DIFF_RULE_BANDWIDTHS[NUM_DIFF_RULES] = {1, 3, 3, 5};
static const double DIFF_RULES[NUM_DIFF_RULES][DIFF_RULE_LENGTH] = {
    {   0,        0,        1,        0,       0 },       // position
    {   0,     -0.5,        0,      0.5,       0 },       // velocity
    {   0,        1,       -2,        1,       0 },       // acceleration
    {-0.5,        1,        0,       -1,     0.5 }        // jerk
};

class StompConstrained
{
public:
  StompConstrained();

  /**
   * Initialize STOMP
   *
   * @param initial_trajectory     num_time_steps_all x num_joints
   * @param dt                     time interval of a single time step
   * @param free_vars_start        at which time index in the trajectory do we start optimizing
   * @param free_vars_end          at which time index in the trajectory do we stop optimizing
   * @param num_samples            number of samples to generate per iteration
   */
  void initialize(const Eigen::MatrixXd& initial_trajectory,
                  const double dt,
                  const int free_vars_start,
                  const int free_vars_end,
                  const int num_samples,
                  const std::vector<double>& derivative_costs);

  /**
   * Overwrite policy mean with the minimum control cost trajectory (ignoring constraints)
   */
  void setToMinControlCost();

  /**
   * Gets the index in the optimization vector of the given joint and time-step
   * @param joint
   * @param time_step
   * @return
   */
  inline int getIndex(int joint, int time_step)
  {
    return time_step + joint * num_time_steps_;
  }

  /**
   * Set constraints on the variables: Ax + b = 0
   * @param A num_constraints x num_variables
   * @param b num_constraints
   */
  void setEqualityConstraints(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

  /**
   * Gets random samples around the mean (conditioned on the equality constraints)
   * @param samples - num_variables x num_samples
   */
  void getSamples(Eigen::MatrixXd& samples);

  /**
   * Gets random samples around the mean (conditioned on the equality constraints)
   * @param samples - [num_samples] num_time_steps_all x num_joints
   */
  void getSamples(std::vector<Eigen::MatrixXd*>& samples);

  /**
   * Sets the random samples (eg after clipping joint limits, or satisfying some other constraints)
   * @param samples - [num_samples] num_time_steps_all x num_joints
   */
  void setSamples(const std::vector<Eigen::MatrixXd*>& samples);

  /**
   * Set the costs of samples retrieved
   * @param sample_costs - num_samples
   */
  void setSampleCosts(Eigen::VectorXd& sample_costs);

  /**
   * Update the mean policy based on the sample costs
   */
  void updateMean();

  /**
   * Get the mean trajectory
   * @param trajectory (num_time_steps_all x num_joints)
   */
  void getMeanTrajectory(Eigen::MatrixXd& trajectory);

  double noise_stddev_;

  // getters
  inline int getNumVariables() const { return num_variables_; }

private:

  /**
   * Sets initial_trajectory, computes const parts of differentiation vectors, and policy mean
   * @param initial_trajectory            num_time_steps_all x num_joints
   */
  void setInitialTrajectory(const Eigen::MatrixXd& initial_trajectory);

  void createDiffMatrices();
  void createCostMatrix();
  void computeCholeskyFactor();
  void computeMinControlCostTrajectory();
  void computeConstraintProjector();
  void generateSamples();

  template<typename Derived>
  void trajectoryVectorToMatrix(const Eigen::MatrixBase<Derived>& vector, Eigen::MatrixXd& matrix,
                                bool only_free_vars=false);

  template<typename Derived>
  void trajectoryMatrixToVector(const Eigen::MatrixXd& matrix, const Eigen::MatrixBase<Derived>& vector);

  /**
   * matrices for numerical differentiation
   *
   * [num_diff_rules] 0 = position, 1 = vel, 2 = acc, 3 = jerk
   */
  std::vector<int> diff_num_outputs_; /**< how big is the vector of derivatives after differentiating */
  std::vector<int> diff_output_offset_; /**< where does timestep 0 lie in the output derivative vector */
  std::vector<Eigen::SparseMatrix<double> > sparse_diff_matrix_; /**< [num_diff_rules] diff_num_outputs x num_time_steps */
  std::vector<Eigen::SparseMatrix<double> > sparse_diff_matrix_const_; /**< [num_diff_rules] diff_num_outputs x num_time_steps_all */
  std::vector<Eigen::SparseMatrix<double> > sparse_diff_vectors_const_; /**< [num_diff_rules] diff_num_outputs x num_joints  - this happens after multiplying matrix_const * initial_trajectory */

  std::vector<double> derivative_costs_;
  Eigen::SparseMatrix<double> sparse_quad_cost_;
  Eigen::SparseVector<double> sparse_linear_cost_;
  Eigen::VectorXd dense_linear_cost_;
  Eigen::SimplicialLLT<Eigen::SparseMatrix<double> > sparse_quad_cost_cholesky_;

  // constraints are dense for now
  bool has_equality_constraints_;
  Eigen::MatrixXd dense_constraints_, dense_constraint_projector_;
  Eigen::VectorXd dense_constraints_const_;
  Eigen::MatrixXd dense_V_;   // intermediate matrix for constraint projector computation
  Eigen::MatrixXd dense_AV_;  // intermediate matrix for constraint projector computation
  Eigen::LDLT<Eigen::MatrixXd> dense_AV_solver_;

  Eigen::MatrixXd initial_trajectory_;

  Eigen::VectorXd policy_mean_;
  Eigen::VectorXd min_control_cost_trajectory_;

  Eigen::MatrixXd normal_samples_;
  Eigen::MatrixXd unconditioned_samples_;
  Eigen::MatrixXd conditioned_samples_;

  Eigen::VectorXd sample_costs_;
  Eigen::VectorXd sample_probabilities_;

  boost::mt19937 rng_;
  boost::normal_distribution<> normal_dist_;
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> > gaussian_;

  int num_time_steps_;
  int num_time_steps_all_; // including padding
  int num_joints_;
  int num_samples_;
  int num_variables_; // num joints * num time steps
  int num_constraints_;
  double dt_;
  int free_vars_start_;
  int free_vars_end_;

};
