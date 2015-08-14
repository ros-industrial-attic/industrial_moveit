/*
 * exact_collision_feature.h
 *
 *  Created on: Jul 22, 2012
 *      Author: kalakris
 */

#ifndef EXACT_COLLISION_FEATURE_H_
#define EXACT_COLLISION_FEATURE_H_

#include <stomp_moveit_interface/cost_features/stomp_cost_feature.h>
#include <std_msgs/ColorRGBA.h>

namespace stomp_moveit_interface
{

class ExactCollisionFeature: public StompCostFeature
{
public:
  ExactCollisionFeature();
  virtual ~ExactCollisionFeature();

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

  bool debug_collisions_;
  ros::Publisher collision_viz_pub_;
  ros::Publisher collision_array_viz_pub_;
  std_msgs::ColorRGBA collision_color;
  ros::NodeHandle node_handle_;


};

} /* namespace stomp_ros_interface */
#endif /* EXACT_COLLISION_FEATURE_H_ */
