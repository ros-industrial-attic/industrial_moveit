/*
 * feature_set.h
 *
 *  Created on: May 25, 2012
 *      Author: kalakris
 */

#ifndef STOMP_FEATURE_SET_H_
#define STOMP_FEATURE_SET_H_

#include <stomp_moveit_interface/cost_features/stomp_cost_feature.h>

namespace stomp_moveit_interface
{

struct FeatureInfo
{
  boost::shared_ptr<StompCostFeature> feature;
  int num_values;
  //std::vector<double> weights;
  Eigen::MatrixXd values;
  std::vector<Eigen::MatrixXd> gradients;
  std::vector<int> validities;
};

class FeatureSet
{
public:
  FeatureSet(int num_threads);
  virtual ~FeatureSet();

  int getNumValues() const;
  void computeValuesAndGradients(const boost::shared_ptr<StompTrajectory const>& trajectory,
                                 Eigen::MatrixXd& feature_values,           // num_time_steps x num_features
                                 bool compute_gradients,
                                 std::vector<Eigen::MatrixXd>& gradients,   // [num_features] num_joints x num_time_steps
                                 std::vector<int>& validities,             // [num_time_steps] each state valid or not
                                 int thread_id,
                                 int start_timestep,                      // start timestep
                                 int num_time_steps);

  void addFeature(boost::shared_ptr<StompCostFeature> feature);
  void clear();

  void getNames(std::vector<std::string>& names) const;
  void setPlanningScene(planning_scene::PlanningSceneConstPtr planning_scene);

private:
  std::vector<std::vector<FeatureInfo> > features_; // [num_threads][num_features]
  int num_values_;
  int num_threads_;

};

} /* namespace stomp_ros_interface */
#endif /* LEARNABLE_COST_FUNCTION_FEATURE_SET_H_ */
