/**
 * @file cartesian_planner.cpp
 * @brief Cartesian path planner for moveit.
 *
 * This class is used to represent a cartesian path planner for moveit.
 * It finds a straight line path between the start and goal position. This
 * planner does not have the inherent ability to avoid collision. It does
 * check if the path created is collision free before it returns a trajectory.
 * If a collision is found it returns an empty trajectory and moveit error.
 *
 * @author Levi Armstrong
 * @date May 4, 2015
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2015, Southwest Research Institute
 *
 * @license Software License Agreement (Apache License)\n
 * \n
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at\n
 * \n
 * http://www.apache.org/licenses/LICENSE-2.0\n
 * \n
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <constrained_ik/moveit_interface/cartesian_planner.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>

namespace constrained_ik
{
  void CartesianPlanner::initialize()
  {
    robot_model_ = planning_scene_->getRobotModel();
    CartesianPlanner::getConstrainedIKSolverData();
    solver_allocator_ = boost::bind(&CartesianPlanner::allocKinematicsSolver, this, _1);

    try
    {
      kinematics_loader_.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>("moveit_core", "kinematics::KinematicsBase"));
    }
    catch(pluginlib::PluginlibException& e)
    {
      ROS_ERROR("Unable to construct kinematics loader. Error: %s", e.what());
    }

    for (std::size_t i = 0 ; i < groups_.size() ; ++i)
    {
      robot_model::JointModelGroup *jmg = new robot_model::JointModelGroup(*robot_model_->getJointModelGroup(groups_[i]));
      jmg->setSolverAllocators(solver_allocator_);
      joint_model_groups_.insert(std::make_pair<std::string, robot_model::JointModelGroup*>(groups_[i], jmg));
    }

    initialized_ = true;
  }

  bool CartesianPlanner::getConstrainedIKSolverData()
  {
    groups_.clear();
    const std::vector<srdf::Model::Group> &known_groups = robot_model_->getSRDF()->getGroups();

    // read data using ROS params
    ros::NodeHandle nh("~");

    // read the list of plugin names for possible kinematics solvers
    for (std::size_t i = 0 ; i < known_groups.size() ; ++i)
    {
      std::string base_param_name = "constrained_ik_solver/" + known_groups[i].name_;
      ROS_DEBUG_NAMED("kinematics_plugin_loader","Looking for param %s ", (base_param_name + "/kinematics_solver").c_str());
      std::string ksolver_param_name;
      bool found = nh.searchParam(base_param_name + "/kinematics_solver", ksolver_param_name);
      if (!found || !nh.hasParam(ksolver_param_name))
      {
        base_param_name = robot_description_ + "_kinematics/" + known_groups[i].name_;
        ROS_DEBUG_NAMED("kinematics_plugin_loader","Looking for param %s ", (base_param_name + "/kinematics_solver").c_str());
        found = nh.searchParam(base_param_name + "/kinematics_solver", ksolver_param_name);
      }
      if (found)
      {
        ROS_DEBUG_NAMED("kinematics_plugin_loader","Found param %s", ksolver_param_name.c_str());
        std::string ksolver;
        if (nh.getParam(ksolver_param_name, ksolver))
        {
          std::stringstream ss(ksolver);
          bool first = true;
          while (ss.good() && !ss.eof())
          {
            if (first)
            {
              first = false;
              groups_.push_back(known_groups[i].name_);
            }
            std::string solver; ss >> solver >> std::ws;
            possible_kinematics_solvers_[known_groups[i].name_].push_back(solver);
            ROS_DEBUG_NAMED("kinematics_plugin_loader","Using kinematics solver '%s' for group '%s'.", solver.c_str(), known_groups[i].name_.c_str());
          }
        }
      }

      std::string ksolver_timeout_param_name;
      if (nh.searchParam(base_param_name + "/kinematics_solver_timeout", ksolver_timeout_param_name))
      {
        double ksolver_timeout;
        if (nh.getParam(ksolver_timeout_param_name, ksolver_timeout))
          ik_timeout_[known_groups[i].name_] = ksolver_timeout;
        else
        {// just in case this is an int
          int ksolver_timeout_i;
          if (nh.getParam(ksolver_timeout_param_name, ksolver_timeout_i))
            ik_timeout_[known_groups[i].name_] = ksolver_timeout_i;
        }
      }

      std::string ksolver_attempts_param_name;
      if (nh.searchParam(base_param_name + "/kinematics_solver_attempts", ksolver_attempts_param_name))
      {
        int ksolver_attempts;
        if (nh.getParam(ksolver_attempts_param_name, ksolver_attempts))
          ik_attempts_[known_groups[i].name_] = ksolver_attempts;
      }

      std::string ksolver_res_param_name;
      if (nh.searchParam(base_param_name + "/kinematics_solver_search_resolution", ksolver_res_param_name))
      {
        std::string ksolver_res;
        if (nh.getParam(ksolver_res_param_name, ksolver_res))
        {
          std::stringstream ss(ksolver_res);
          while (ss.good() && !ss.eof())
          {
            double res; ss >> res >> std::ws;
            search_res_[known_groups[i].name_].push_back(res);
          }
        }
        else
        { // handle the case this param is just one value and parsed as a double
          double res;
          if (nh.getParam(ksolver_res_param_name, res))
            search_res_[known_groups[i].name_].push_back(res);
          else
          {
            int res_i;
            if (nh.getParam(ksolver_res_param_name, res_i))
              search_res_[known_groups[i].name_].push_back(res_i);
          }
        }
      }

      // Allow a kinematic solver's tip link to be specified on the rosparam server
      // Depreciated in favor of array version now
      std::string ksolver_ik_link_param_name;
      if (nh.searchParam(base_param_name + "/kinematics_solver_ik_link", ksolver_ik_link_param_name))
      {
        std::string ksolver_ik_link;
        if (nh.getParam(ksolver_ik_link_param_name, ksolver_ik_link)) // has a custom rosparam-based tip link
        {
          ROS_WARN_STREAM_NAMED("kinematics_plugin_loader","Using kinematics_solver_ik_link rosparam is deprecated in favor of kinematics_solver_ik_links rosparam array.");
          iksolver_to_tip_links_[known_groups[i].name_].push_back(ksolver_ik_link);
        }
      }

      // Allow a kinematic solver's tip links to be specified on the rosparam server as an array
      std::string ksolver_ik_links_param_name;
      if (nh.searchParam(base_param_name + "/kinematics_solver_ik_links", ksolver_ik_links_param_name))
      {
        XmlRpc::XmlRpcValue ksolver_ik_links;
        if (nh.getParam(ksolver_ik_links_param_name, ksolver_ik_links)) // has custom rosparam-based tip link(s)
        {
          if (ksolver_ik_links.getType() != XmlRpc::XmlRpcValue::TypeArray)
          {
            ROS_WARN_STREAM_NAMED("kinematics_plugin_loader","rosparam '" << ksolver_ik_links_param_name << "' should be an XmlRpc value type 'Array'");
          }
          else
          {
            for (int32_t j = 0; j < ksolver_ik_links.size(); ++j)
            {
              ROS_ASSERT(ksolver_ik_links[j].getType() == XmlRpc::XmlRpcValue::TypeString);
              ROS_DEBUG_STREAM_NAMED("kinematics_plugin_loader","found tip " << static_cast<std::string>(ksolver_ik_links[j]) << " for group " << known_groups[i].name_ );
              iksolver_to_tip_links_[known_groups[i].name_].push_back( static_cast<std::string>(ksolver_ik_links[j]) );
            }
          }
        }
      }

      // make sure there is a default resolution at least specified for every solver (in case it was not specified on the param server)
      while (search_res_[known_groups[i].name_].size() < possible_kinematics_solvers_[known_groups[i].name_].size())
        search_res_[known_groups[i].name_].push_back(0.0);
    }
  }

  /**
   * \brief Helper function to decide which, and how many, tip frames a planning group has
   * \param jmg - joint model group pointer
   * \return tips - list of valid links in a planning group to plan for
   */
  std::vector<std::string> CartesianPlanner::chooseTipFrames(const robot_model::JointModelGroup *jmg)
  {
    std::vector<std::string> tips;
    std::map<std::string, std::vector<std::string> >::const_iterator ik_it = iksolver_to_tip_links_.find(jmg->getName());

    // Check if tips were loaded onto the rosparam server previously
    if (ik_it != iksolver_to_tip_links_.end())
    {
      // the tip is being chosen based on a corresponding rosparam ik link
      ROS_DEBUG_STREAM_NAMED("kinematics_plugin_loader","Chooing tip frame of kinematic solver for group "
        << jmg->getName() << " based on values in rosparam server.");
      tips = ik_it->second;
    }
    else
    {
      // get the last link in the chain
      ROS_DEBUG_STREAM_NAMED("kinematics_plugin_loader","Chooing tip frame of kinematic solver for group "
        << jmg->getName() << " based on last link in chain");

      tips.push_back(jmg->getLinkModels().back()->getName());
    }

    // Error check
    if (tips.empty())
    {
      ROS_ERROR_STREAM_NAMED("kinematics_plugin_loader","Error choosing kinematic solver tip frame(s).");
    }

    // Debug tip choices
    std::stringstream tip_debug;
    tip_debug << "Planning group '" << jmg->getName() << "' has tip(s): ";
    for (std::size_t i = 0; i < tips.size(); ++i)
      tip_debug << tips[i] << ", ";
    ROS_DEBUG_STREAM_NAMED("kinematics_plugin_loader", tip_debug.str());

    return tips;
  }

  boost::shared_ptr<kinematics::KinematicsBase> CartesianPlanner::allocKinematicsSolver(const robot_model::JointModelGroup *jmg)
  {
    boost::shared_ptr<kinematics::KinematicsBase> result;
    if (!jmg)
    {
      ROS_ERROR("Specified group is NULL. Cannot allocate kinematics solver.");
      return result;
    }

    ROS_DEBUG("Received request to allocate kinematics solver for group '%s'", jmg->getName().c_str());

    if (kinematics_loader_ && jmg)
    {
      std::map<std::string, std::vector<std::string> >::const_iterator it = possible_kinematics_solvers_.find(jmg->getName());
      if (it != possible_kinematics_solvers_.end())
      {
        // just to be sure, do not call the same pluginlib instance allocation function in parallel
        boost::mutex::scoped_lock slock(mutex_);

        for (std::size_t i = 0 ; !result && i < it->second.size() ; ++i)
        {
          try
          {
            result.reset(kinematics_loader_->createUnmanagedInstance(it->second[i]));
            if (result)
            {
              const std::vector<const robot_model::LinkModel*> &links = jmg->getLinkModels();
              if (!links.empty())
              {
                const std::string &base = links.front()->getParentJointModel()->getParentLinkModel() ?
                  links.front()->getParentJointModel()->getParentLinkModel()->getName() : jmg->getParentModel().getModelFrame();

                // choose the tip of the IK solver
                const std::vector<std::string> tips = chooseTipFrames(jmg);

                // choose search resolution
                double search_res = search_res_.find(jmg->getName())->second[i]; // we know this exists, by construction

                if (!result->initialize(robot_description_, jmg->getName(),
                                        (base.empty() || base[0] != '/') ? base : base.substr(1) , tips, search_res))
                {
                  ROS_ERROR("Kinematics solver of type '%s' could not be initialized for group '%s'", it->second[i].c_str(), jmg->getName().c_str());
                  result.reset();
                }
                else
                {
                  result->setDefaultTimeout(jmg->getDefaultIKTimeout());
                  ROS_DEBUG("Successfully allocated and initialized a kinematics solver of type '%s' with search resolution %lf for group '%s' at address %p",
                            it->second[i].c_str(), search_res, jmg->getName().c_str(), result.get());
                }
              }
              else
                ROS_ERROR("No links specified for group '%s'", jmg->getName().c_str());
            }
          }
          catch (pluginlib::PluginlibException& e)
          {
            ROS_ERROR("The kinematics plugin (%s) failed to load. Error: %s", it->first.c_str(), e.what());
          }
        }
      }
      else
        ROS_DEBUG("No kinematics solver available for this group");
    }

    if (!result)
    {
      ROS_DEBUG("No usable kinematics solver was found for this group.");
      ROS_DEBUG("Did you load kinematics.yaml into your node's namespace?");
    }
    return result;
  }

  bool CartesianPlanner::solve(planning_interface::MotionPlanResponse &res)
  {
    ros::WallTime start_time = ros::WallTime::now();
    robot_state::RobotStatePtr mid_state;
    std::map<std::string, robot_model::JointModelGroup*>::const_iterator it;
    robot_model::JointModelGroup *jmg;
    std::vector<std::string> joint_names, link_names;
    Eigen::Affine3d start_pose, goal_pose;
    std::vector<double> pos(1);

    // Initialize solver, if it has not been already done.
    if (!initialized_)
    {
      initialize();
    }
    robot_state::RobotState start_state(robot_model_);
    robot_state::robotStateMsgToRobotState(request_.start_state, start_state);
    robot_state::RobotState goal_state = start_state;
    robot_trajectory::RobotTrajectoryPtr traj(new robot_trajectory::RobotTrajectory(robot_model_, request_.group_name));

    it = joint_model_groups_.find(request_.group_name);
    if (it == joint_model_groups_.end())
    {
      ROS_INFO_STREAM("Could not find Joint Model Group for namespace " << request_.group_name);
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
      return false;
    }
    jmg = it->second;
    joint_names = jmg->getActiveJointModelNames();
    link_names = jmg->getLinkModelNames();
    start_pose = start_state.getFrameTransform(link_names.back());

    ROS_INFO_STREAM("Cartesian Planning for Group: " << request_.group_name);

    // if we have path constraints, we prefer interpolating in pose space
    if (!request_.goal_constraints[0].joint_constraints.empty())
    {
      for(unsigned int i = 0; i < request_.goal_constraints[0].joint_constraints.size(); i++)
      {
        pos[0]=request_.goal_constraints[0].joint_constraints[i].position;
        goal_state.setJointPositions(joint_names[i], pos);
      }
      goal_pose = goal_state.getFrameTransform(link_names.back());
    }
    else
    {
      geometry_msgs::Pose pose;
      if (!request_.goal_constraints[0].position_constraints.empty() && !request_.goal_constraints[0].orientation_constraints.empty())
      {
        pose.position = request_.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position;
        pose.orientation = request_.goal_constraints[0].orientation_constraints[0].orientation;
      }
      else if (!request_.goal_constraints[0].position_constraints.empty() && request_.goal_constraints[0].orientation_constraints.empty())
      {
        tf::poseEigenToMsg(start_state.getFrameTransform(link_names.back()), pose);
        pose.position = request_.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position;
      }
      else if (request_.goal_constraints[0].position_constraints.empty() && !request_.goal_constraints[0].orientation_constraints.empty())
      {
        tf::poseEigenToMsg(start_state.getFrameTransform(link_names.back()), pose);
        pose.orientation = request_.goal_constraints[0].orientation_constraints[0].orientation;
      }
      else
      {
        ROS_ERROR("No constraint was passed with request!");
        return false;
      }
      tf::poseMsgToEigen(pose, goal_pose);
    }

    ROS_DEBUG_NAMED("clik", "Setting Position x from %f to %f", start_pose.translation()(0),goal_pose.translation()(0));
    ROS_DEBUG_NAMED("clik", "Setting Position y from %f to %f", start_pose.translation()(1),goal_pose.translation()(1));
    ROS_DEBUG_NAMED("clik", "Setting Position z from %f to %f", start_pose.translation()(2),goal_pose.translation()(2));
    ROS_DEBUG_NAMED("clik", "Setting Position yaw   from %f to %f", start_pose.rotation().eulerAngles(3,2,1)(0),goal_pose.rotation().eulerAngles(3,2,1)(0));
    ROS_DEBUG_NAMED("clik", "Setting Position pitch from %f to %f", start_pose.rotation().eulerAngles(3,2,1)(1),goal_pose.rotation().eulerAngles(3,2,1)(1));
    ROS_DEBUG_NAMED("clik", "Setting Position roll  from %f to %f", start_pose.rotation().eulerAngles(3,2,1)(2),goal_pose.rotation().eulerAngles(3,2,1)(2));

    // Generate Interpolated Cartesian Poses
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > poses = interpolateCartesian(start_pose, goal_pose, config_.cartesian_discretization_step);

    // Generate Cartesian Trajectory
    int steps = poses.size();
    mid_state = robot_model::RobotStatePtr(new robot_model::RobotState(start_state));
    for (int j=0; j<steps; j++)
    {
      if (j!=0)
      {
        mid_state->setFromIK(jmg, poses[j], link_names.back());
      }
      traj->addSuffixWayPoint(*mid_state, 0.0);

      if (terminate_)
        break;
    }

    res.planning_time_ = (ros::WallTime::now() - start_time).toSec();

    // Check if planner was terminated
    if (terminate_)
    {
      ROS_INFO("Cartesian Trajectory was terminated!");
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
      return false;
    }

    // Check if traj is a collision free path
    if (planning_scene_->isPathValid(*traj, request_.group_name))
    {
      ROS_INFO("Cartesian Trajectory is collision free! :)");
      res.trajectory_=traj;
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      return true;
    }
    else
    {
      ROS_INFO("Cartesian Trajectory is not collision free. :(");
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
      return false;
    }
  }

  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> >
  CartesianPlanner::interpolateCartesian(const Eigen::Affine3d& start,
                                            const Eigen::Affine3d& stop,
                                            double ds) const
  {
    // Required position change
    Eigen::Vector3d delta = (stop.translation() - start.translation());
    Eigen::Vector3d start_pos = start.translation();

    // Calculate number of steps
    unsigned steps = static_cast<unsigned>(delta.norm() / ds) + 1;

    // Step size
    Eigen::Vector3d step = delta / steps;

    // Orientation interpolation
    Eigen::Quaterniond start_q (start.rotation());
    Eigen::Quaterniond stop_q (stop.rotation());
    double slerp_ratio = 1.0 / steps;

    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > result;
    Eigen::Vector3d trans;
    Eigen::Quaterniond q;
    Eigen::Affine3d pose;
    result.reserve(steps+1);
    for (unsigned i = 0; i <= steps; ++i)
    {
      trans = start_pos + step * i;
      q = start_q.slerp(slerp_ratio * i, stop_q);
      pose = (Eigen::Translation3d(trans) * q);
      result.push_back(pose);
    }
    return result;
  }
} //namespace constrained_ik

