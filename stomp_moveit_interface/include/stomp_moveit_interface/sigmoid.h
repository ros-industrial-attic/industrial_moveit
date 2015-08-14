/*
 * sigmoid.h
 *
 *  Created on: Jun 1, 2012
 *      Author: kalakris
 */

#ifndef SIGMOID_H_
#define SIGMOID_H_

#include <cmath>

namespace stomp_ros_interface
{

static inline double sigmoid(double x, double center, double slope)
{
  double temp = exp(slope*(x-center));
  return temp/(1.0+temp);
}

}


#endif /* SIGMOID_H_ */
