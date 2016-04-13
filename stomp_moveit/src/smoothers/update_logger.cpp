/*
 * update_logger.cpp
 *
 *  Created on: Apr 13, 2016
 *      Author: Jorge Nicho
 */

#include <stomp_moveit/smoothers/update_logger.h>
#include <boost/filesystem.hpp>
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>

PLUGINLIB_EXPORT_CLASS(stomp_moveit::smoothers::UpdateLogger,stomp_moveit::smoothers::SmootherInterface);

namespace stomp_moveit
{
namespace smoothers
{

UpdateLogger::UpdateLogger():
    name_("UpdateLogger")
{

}

UpdateLogger::~UpdateLogger()
{
  // TODO Auto-generated destructor stub
}

bool UpdateLogger::initialize(moveit::core::RobotModelConstPtr robot_model_ptr,
                        const std::string& group_name,XmlRpc::XmlRpcValue& config)
{
  group_name_ = group_name;
  return configure(config);
}

bool UpdateLogger::configure(const XmlRpc::XmlRpcValue& config)
{
  XmlRpc::XmlRpcValue c = config;
  try
  {
    filename_ = static_cast<std::string>(c["filename"]);
    directory_ = static_cast<std::string>(c["directory"]);
    package_ = static_cast<std::string>(c["package"]);
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s failed to find the required parameters",getName().c_str());
    return false;
  }

  return true;
}

bool UpdateLogger::setMotionPlanRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const moveit_msgs::MotionPlanRequest &req,
                 const stomp_core::StompConfiguration &config,
                 moveit_msgs::MoveItErrorCodes& error_code)
{
  using namespace boost::filesystem;

  std::string full_dir_name = ros::package::getPath(package_) + "/" + directory_;
  full_file_name_ = full_dir_name + "/" + filename_;
  path dir_path(full_dir_name);

  if(!boost::filesystem::is_directory(dir_path))
  {
    // create directory
    if(!boost::filesystem::create_directory(dir_path))
    {
      ROS_ERROR("Unable to create the update logging directory in the path %s",full_dir_name.c_str());
      return false;
    }
  }

  stream_.open(full_file_name_);
  if(stream_ < 0)
  {
    ROS_ERROR("Unable to create/open update log file %s",full_file_name_.c_str());
    return false;
  }

  return true;
}

bool UpdateLogger::smooth(std::size_t start_timestep,
                    std::size_t num_timesteps,
                    int iteration_number,
                    Eigen::MatrixXd& updates)
{
  stream_<<updates<<"\n";
  return true;
}

void UpdateLogger::done(bool success, int total_iterations,double final_cost)
{
  stream_.close();
  ROS_INFO("Saved update log file %s",full_file_name_.c_str());
}

} /* namespace smoothers */
} /* namespace stomp_moveit */
