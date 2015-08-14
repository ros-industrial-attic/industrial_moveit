/*
 * collision_feature.h
 *
 *  Created on: May 25, 2012
 *      Author: kalakris
 */

#ifndef COLLISION_FEATURE_H_
#define COLLISION_FEATURE_H_

#include <stomp_moveit_interface/cost_features/stomp_cost_feature.h>
#include <moveit/collision_distance_field/collision_common_distance_field.h>

namespace stomp_moveit_interface
{

class CollisionFeature: public StompCostFeature
{
public:
  CollisionFeature();
  virtual ~CollisionFeature();

  virtual int getNumValues() const;
  virtual void computeValuesAndGradients(const boost::shared_ptr<StompTrajectory const>& trajectory,
                                         Eigen::MatrixXd& feature_values,         // num_time_steps x num_features
                                         bool compute_gradients,
                                         std::vector<Eigen::MatrixXd>& gradients, // [num_features] num_joints x num_time_steps
                                         std::vector<int>& validities,             // [num_time_steps] each state valid or not
                                         int thread_id,
                                         int start_timestep,                      // start timestep
                                         int num_time_steps) const;
  virtual std::string getName() const;
  void getNames(std::vector<std::string>& names) const;

protected:
  virtual bool initialize(XmlRpc::XmlRpcValue& config);

private:
  collision_detection::CollisionRequest collision_request_;

  mutable std::vector<boost::shared_ptr<collision_detection::GroupStateRepresentation> > group_state_representations_;

  double clearance_;
  bool report_validity_;
  bool debug_collisions_;

  double num_sigmoids_;
  std::vector<double> sigmoid_centers_;
  std::vector<double> sigmoid_slopes_;
};

} /* namespace stomp_ros_interface */
#endif /* COLLISION_FEATURE_H_ */
