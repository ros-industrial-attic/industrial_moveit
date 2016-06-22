/**
 * @file constraint_results.h
 * @brief This is used to store a constraints results.
 *
 * @author Levi Armstrong
 * @date May 4, 2015
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2015, Southwest Research Institute
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

