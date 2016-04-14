/*
 * trajectory_visualization.cpp
 *
 *  Created on: Apr 14, 2016
 *      Author: Jorge Nicho
 */


#include <moveit/robot_state/conversions.h>
#include <tf/transform_datatypes.h>
#include <stomp_moveit/filters/trajectory_visualization.h>


typedef std::vector<geometry_msgs::Point> ToolLine;
using namespace moveit::core;

void createToolPathMarker(const Eigen::MatrixXd& tool_line, int id, std::string frame_id,
                          const std::vector<double>& rgb,double line_width,
                          std::string ns,visualization_msgs::Marker& m,)
{
  m.ns = ns;
  m.id = id;
  m.header.frame_id = frame_id;
  m.type = m.LINE_STRIP;
  m.action = m.ADD;
  tf::poseTFToMsg(tf::Transform::getIdentity(),m.pose);
  m.scale.x = line_width;

  if(tool_line.cols() == 0)
  {
    return;
  }

  m.points.resize(tool_line.cols());
  for(auto t = 0u; t < tool_line.cols(); t++)
  {
    m.points[t].x = tool_line(0,t);
    m.points[t].y = tool_line(1,t);
    m.points[t].z = tool_line(2,t);
  }
}

namespace stomp_moveit
{
namespace filters
{

TrajectoryVisualization::TrajectoryVisualization():
    nh_("~"),
    line_width_(0.0),
    publish_intermediate_(false)
{
  // TODO Auto-generated constructor stub

}

TrajectoryVisualization::~TrajectoryVisualization()
{
  // TODO Auto-generated destructor stub
}

bool TrajectoryVisualization::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,const XmlRpc::XmlRpcValue& config)
{
  robot_model_ = robot_model_ptr;
  group_name_ = group_name;

  if(!configure())
  {
    return false;
  }

  // initializing publisher
  viz_pub_ = nh_.advertise(marker_topic_,1);

  return true;
}

bool TrajectoryVisualization::configure(const XmlRpc::XmlRpcValue& config)
{
  auto toStdVector = [] (const XmlRpc::XmlRpcValue& v)
  {
    return static_cast<double>(v);
  };

  XmlRpc::XmlRpcValue c = config;
  XmlRpc::XmlRpcValue::ValueArray rgb_array_;
  rgb_.resize(3);
  try
  {
    line_width_ = static_cast<double>(c["line_width"]);
    rgb_array_ = static_cast<XmlRpc::XmlRpcValue::ValueArray>(c["rgb"]);
    rgb_ = std::transform(rgb_array_.begin(),rgb_array_.end(),rgb_.begin(),toStdVector);
    publish_intermediate_ = static_cast<bool>(c["publish_intermediate"]);
    marker_topic_ = static_cast<std::string>(c["topic"]);
    marker_namespace_ = static_cast<std::string>(c["maker_namespace"]);
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to find required parameters, %s",getName().c_str(),e.getMessage().c_str());
    return false;
  }

  return true;
}

bool TrajectoryVisualization::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 const stomp_core::StompConfiguration &config,
                 moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace moveit::core;

  error_code.val = error_code.SUCCESS;


  // initializing points array
  tool_line_ = Eigen::MatrixXd::Zero(3,config.num_timesteps);

  // updating state
  state_.reset(new RobotState(robot_model_ptr));
  if(!robotStateMsgToRobotState(req.start_state,*state_,true))
  {
    ROS_ERROR("%s Failed to get current robot state from request",getName().c_str());
    return false;
  }

  //delete current markers
  visualization_msgs::Marker m;
  createToolPathMarker(Eigen::MatrixXd(),0,robot_model_->getRootLinkName(),rgb_,line_width_,marker_namespace_,m);
  m.action = m.DELETE;
  viz_pub_.publish(m);

  return true;
}

bool TrajectoryVisualization::filter(std::size_t start_timestep,
                    std::size_t num_timesteps,
                    int iteration_number,
                    int rollout_number,
                    Eigen::MatrixXd& parameters,
                    bool& filtered) const
{

  if(rollout_number != getOptimizedIndex())
  {
    // noisy rollout, do not process
    return true;
  }

  if(!state_)
  {
    ROS_ERROR("%s Robot State has not been updated",getName().c_str());
    return false;
  }

  // FK on each point
  const moveit::core::JointModelGroup* joint_group = robot_model_->getJointModelGroup(group_name_);
  std::string tool_link = joint_group->getUpdatedLinkModelNames().back();
  for(auto t = 0u; t < parameters.cols();t++)
  {
    state_->setJointGroupPositions(joint_group,parameters.col(t));
    Eigen::Affine3d tool_pos = state_->getFrameTransform(tool_link);
    tool_line_(0,t) = tool_pos.translation()(0);
    tool_line_(1,t) = tool_pos.translation()(1);
    tool_line_(2,t) = tool_pos.translation()(2);
  }

  if(publish_intermediate_)
  {

  }

  return true;
}


void TrajectoryVisualization::done(bool success,int total_iterations,double final_cost)
{

}

} /* namespace filters */
} /* namespace stomp_moveit */
