/*
 * stomp_core_utils.cpp
 *
 *  Created on: Mar 9, 2016
 *      Author: Jorge Nicho
 */

#include <cmath>
#include "stomp_core/stomp_core_utils.h"

namespace stomp_core
{


bool generateFiniteDifferenceMatrix(int num_time_steps,
                                             DerivativeOrders::DerivativeOrder order,
                                             double dt, Eigen::MatrixXd& diff_matrix)
{

  diff_matrix = Eigen::MatrixXd::Zero(num_time_steps, num_time_steps);
  double multiplier = 1.0/pow(dt,(int)order);
  for (int i=0; i<num_time_steps; ++i)
  {
    for (int j=-FINITE_DIFF_RULE_LENGTH/2; j<=FINITE_DIFF_RULE_LENGTH/2; ++j)
    {
      int index = i+j;
      if (index < 0)
      {
        index = 0;
        continue;
      }
      if (index >= num_time_steps)
      {
        index = num_time_steps-1;
        continue;
      }

      diff_matrix(i,index) = multiplier * FINITE_DIFF_COEFFS[order][j+FINITE_DIFF_RULE_LENGTH/2];
    }
  }
}
}





