/*
 * DisplayRobotModel.h
 *
 *  Created on: May 23, 2012
 *      Author: kalakris
 */

#ifndef DISPLAYROBOTMODEL_H_
#define DISPLAYROBOTMODEL_H_

#include <stomp_ros_interface/stomp_robot_model.h>

namespace stomp_ros_interface
{

class DisplayRobotModel
{
public:
  DisplayRobotModel();
  virtual ~DisplayRobotModel();

private:
  ros::NodeHandle node_handle_;
  boost::shared_ptr<StompRobotModel> robot_model_;
  //boost::shared_ptr<StompCollisionSpace> collision_space_;
  const StompRobotModel::StompPlanningGroup* group_;
  ros::Subscriber joint_states_sub_;
  ros::Publisher marker_pub_;

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
};

} /* namespace stomp_ros_interface */
#endif /* DISPLAYROBOTMODEL_H_ */
