/*
 * stomp.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: ros-ubuntu
 */

#include "stomp.h"

namespace stomp_core {

Stomp::Stomp():
    proceed_(true)
{

}

Stomp::~Stomp()
{

}

bool Stomp::solve(const StompParameters& params,TaskPtr task,Eigen::MatrixXf& optimized_parameters)
{
  optimization_params_ = params;
  task_ = task;

  setProceed(true);

  unsigned int iterations = 0;
  while(iterations < optimization_params_.num_iterations && runSingleIteration())
  {

    iterations++;
  }

  return true;
}

bool Stomp::cancel()
{
  setProceed(false);
}

bool Stomp::runSingleIteration()
{
  if(!getProceed())
  {
    return false;
  }

  bool succeed = getProceed() &&
      generateNoisyRollouts() &&
      computeRolloutsCosts() &&
      computeProbabilities() &&
      updateParameters() &&
      computeOptimizedCost();

  return succeed;
}

bool Stomp::generateNoisyRollouts()
{

}

bool Stomp::computeRolloutsCosts()
{

}

bool Stomp::computeProbabilities()
{

}

bool Stomp::updateParameters()
{

}

void Stomp::setProceed(bool proceed)
{
  proceed_mutex_.lock();
  proceed_ = proceed;
  proceed_mutex_.unlock();
}

bool Stomp::getProceed()
{
  bool proceed;
  proceed_mutex_.lock();
  proceed = proceed_;
  proceed_mutex_.unlock();
  return proceed;
}


} /* namespace stomp */
