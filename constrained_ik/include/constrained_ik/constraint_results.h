#ifndef CONSTRAINT_RESULTS
#define CONSTRAINT_RESULTS

#include <ros/ros.h>
#include <Eigen/Core>

namespace constrained_ik
{
  class ConstraintResults
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConstraintResults():status(true){}

    Eigen::VectorXd error;

    Eigen::MatrixXd jacobian;

    bool status;

    void append(const ConstraintResults &cdata)
    {
      appendError(cdata.error);
      appendJacobian(cdata.jacobian);
      status &= cdata.status;
    }

    bool isEmpty()
    {
      if (error.rows() == 0 || jacobian.rows() == 0)
        return true;
      else
        return false;
    }

  protected:
    // NOTE: This method does a resize in-place, and may be inefficient if called many times
    void appendError(const Eigen::VectorXd &addError)
    {
      if (addError.rows() == 0)
      {
        ROS_DEBUG("trying to add a Error with no data");
        return;
      }

      if (error.rows() == 0)
        error = addError;
      else
      {
        size_t nAddRows = addError.rows();
        error.conservativeResize(error.rows() + nAddRows);
        error.tail(nAddRows) = addError;
      }
    }

    // NOTE: This method does a resize in-place, and may be inefficient if called many times
    void appendJacobian(const Eigen::MatrixXd &addJacobian)
    {
      if(addJacobian.rows() == 0 || addJacobian.cols() == 0)
      {
        ROS_DEBUG("trying to add a Jacobian with no data");
        return;
      }
      if (jacobian.rows() == 0) // first call gets to set size
      {
        size_t nAddRows = addJacobian.rows();
        jacobian.conservativeResize(jacobian.rows() + nAddRows, addJacobian.cols());
        jacobian.bottomRows(nAddRows) = addJacobian;
      }
      else
      {
        ROS_ASSERT(jacobian.cols() == addJacobian.cols());
        size_t nAddRows = addJacobian.rows();
        jacobian.conservativeResize(jacobian.rows() + nAddRows, Eigen::NoChange);
        jacobian.bottomRows(nAddRows) = addJacobian;
      }
    }
  };
}
#endif // CONSTRAINT_RESULTS

