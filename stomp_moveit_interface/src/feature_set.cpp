/*
 * feature_set.cpp
 *
 *  Created on: May 25, 2012
 *      Author: kalakris
 */

#include <stomp_moveit_interface/feature_set.h>

namespace stomp_moveit_interface
{

FeatureSet::FeatureSet(int num_threads):
    num_threads_(num_threads)
{
  clear();
}

FeatureSet::~FeatureSet()
{
}

int FeatureSet::getNumValues() const
{
  return num_values_;
}

void FeatureSet::computeValuesAndGradients(const boost::shared_ptr<StompTrajectory const>& trajectory,
                                       Eigen::MatrixXd& feature_values,         // num_features x num_time_steps
                                       bool compute_gradients,
                                       std::vector<Eigen::MatrixXd>& gradients, // [num_features] num_joints x num_time_steps
                                       std::vector<int>& validities,             // [num_time_steps] each state valid or not
                                       int thread_id,
                                       int start_timestep,                      // start and end timesteps (inclusive)
                                       int num_time_steps)
{
  std::vector<FeatureInfo>& thread_features = features_[thread_id];

  if (compute_gradients)
  {
    gradients.resize(num_values_, Eigen::VectorXd::Zero(trajectory->num_joints_, trajectory->num_time_steps_));
  }
  feature_values = Eigen::MatrixXd::Zero(trajectory->num_time_steps_, num_values_);
  validities.clear();
  validities.resize(trajectory->num_time_steps_, 1);

  int index = 0;
  for (unsigned int i=0; i<thread_features.size(); ++i)
  {
    bool validity = false;
    thread_features[i].feature->computeValuesAndGradients(trajectory, thread_features[i].values, compute_gradients,
                                                    thread_features[i].gradients, thread_features[i].validities,
                                                    thread_id, start_timestep, num_time_steps);
    for (int t=0; t<trajectory->num_time_steps_; ++t)
    {
      validities[t] = validities[t] && thread_features[i].validities[t];
    }
    for (int j=0; j<thread_features[i].num_values; ++j)
    {
      feature_values.col(index) = thread_features[i].values.col(j);
      if (compute_gradients)
      {
        gradients[index] = thread_features[i].gradients[j];
      }
      ++index;
    }
  }

}

void FeatureSet::addFeature(boost::shared_ptr<StompCostFeature> feature)
{
  for (int t=0; t<num_threads_; ++t)
  {
    FeatureInfo fi;
    fi.feature = feature;
    fi.num_values = feature->getNumValues();
    features_[t].push_back(fi);
  }
  num_values_ += feature->getNumValues();
}

void FeatureSet::clear()
{
  features_.clear();
  features_.resize(num_threads_);
  num_values_ = 0;
}

void FeatureSet::getNames(std::vector<std::string>& names) const
{
  names.clear();
  for (unsigned int i=0; i<features_[0].size(); ++i)
  {
    std::vector<std::string> my_names;
    features_[0][i].feature->getNames(my_names);
    if (my_names.size() > 0)
      names.insert(names.end(), my_names.begin(), my_names.end());
  }
}

void FeatureSet::setPlanningScene(planning_scene::PlanningSceneConstPtr planning_scene)
{
  for (unsigned int i=0; i<features_[0].size(); ++i)
  {
    features_[0][i].feature->setPlanningScene(planning_scene);
  }
}

} /* namespace stomp_ros_interface */
