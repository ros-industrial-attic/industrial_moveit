/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

#include <stomp_ros_interface/stomp_robot_model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <cstdio>
#include <iostream>
#include <visualization_msgs/MarkerArray.h>
#include <planning_models/kinematic_state.h>
#include <geometric_shapes/bodies.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>

using namespace std;
using namespace arm_navigation_msgs;

namespace stomp_ros_interface
{

StompRobotModel::StompRobotModel(ros::NodeHandle node_handle):
  node_handle_(node_handle)
{
}

StompRobotModel::~StompRobotModel()
{
}

bool StompRobotModel::init(const std::string& reference_frame)
{
  reference_frame_ = reference_frame;

  max_radius_clearance_ = 0.0;

  robot_models_.reset(new planning_environment::RobotModels("/robot_description"));

  // get the urdf as a string:
  string urdf_string;
  if (!node_handle_.getParam("/robot_description", urdf_string))
  {
    return false;
  }

  // get some other params:
  double joint_update_limit;
  node_handle_.param("collision_clearance", collision_clearance_default_, 0.10);
  node_handle_.param("joint_update_limit", joint_update_limit, 0.05);

  // Construct the KDL tree
  if (!kdl_parser::treeFromString(urdf_string, kdl_tree_))
  {
    ROS_ERROR("Failed to construct KDL tree from URDF.");
    return false;
  }
  num_kdl_joints_ = kdl_tree_.getNrOfJoints();

  // create the joint_segment_mapping, which used to be created by the URDF -> KDL parser
  // but not any more, but the rest of the code depends on it, so we simply generate the mapping here:
  KDL::SegmentMap segment_map = kdl_tree_.getSegments();

  for (KDL::SegmentMap::const_iterator it = segment_map.begin(); it != segment_map.end(); ++it)
  {
    if (it->second.segment.getJoint().getType() != KDL::Joint::None)
    {
      std::string joint_name = it->second.segment.getJoint().getName();
      std::string segment_name = it->first;
      joint_segment_mapping_.insert(make_pair(joint_name, segment_name));
    }
  }

  // create the fk solver:

  std::vector<bool> active_joints;
  active_joints.resize(num_kdl_joints_, true);
  fk_solver_ = new KDL::TreeFkSolverJointPosAxisPartial(kdl_tree_, reference_frame_, active_joints);

  kdl_number_to_urdf_name_.resize(num_kdl_joints_);
  // Create the inverse mapping - KDL segment to joint name
  // (at the same time) Create a mapping from KDL numbers to URDF joint names and vice versa
  for (map<string, string>::iterator it = joint_segment_mapping_.begin(); it!= joint_segment_mapping_.end(); ++it)
  {
    std::string joint_name = it->first;
    std::string segment_name = it->second;
  //  std::cout << joint_name << " -> " << segment_name << std::endl;
    segment_joint_mapping_.insert(make_pair(segment_name, joint_name));
    int kdl_number = kdl_tree_.getSegment(segment_name)->second.q_nr;
    if (kdl_tree_.getSegment(segment_name)->second.segment.getJoint().getType() != KDL::Joint::None)
    {
  //    std::cout << "Kdl number is " << kdl_number << std::endl;
      kdl_number_to_urdf_name_[kdl_number] = joint_name;
      urdf_name_to_kdl_number_.insert(make_pair(joint_name, kdl_number));
    }
  }

  XmlRpc::XmlRpcValue planning_groups_xml;
  if (!node_handle_.getParam("planning_groups", planning_groups_xml) || planning_groups_xml.getType()!=XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("planning_groups parameter needs to be an array");
    return false;
  }
  for (int i=0; i<planning_groups_xml.size(); ++i)
  {
    std::string name;
    ROS_VERIFY(usc_utilities::getParam(planning_groups_xml[i], "name", name));
    std::string end_effector;
    ROS_VERIFY(usc_utilities::getParam(planning_groups_xml[i], "end_effector", end_effector));
    std::vector<string> joint_names;
    ROS_VERIFY(usc_utilities::getParam(planning_groups_xml[i], "joints", joint_names));

    StompPlanningGroup group;
    group.name_ = name;
    group.end_effector_name_ = end_effector;
    ROS_DEBUG_STREAM("Planning group " << group.name_);
    int num_joints = joint_names.size();
    group.num_joints_ = 0;

    std::vector<bool> active_joints;
    active_joints.resize(num_kdl_joints_, false);
    for (int i=0; i<num_joints; i++)
    {
      std::string joint_name = joint_names[i];
      map<string, string>::iterator link_name_it = joint_segment_mapping_.find(joint_name);
      if (link_name_it == joint_segment_mapping_.end())
      {
        ROS_ERROR("Joint name %s did not have containing KDL segment.", joint_name.c_str());
        return false;
      }
      std::string link_name = link_name_it->second;
      const KDL::Segment* segment = &(kdl_tree_.getSegment(link_name)->second.segment);
      KDL::Joint::JointType joint_type =  segment->getJoint().getType();
      if (joint_type != KDL::Joint::None)
      {
        StompJoint joint;
        joint.stomp_joint_index_ = group.num_joints_;
        joint.kdl_joint_index_ = kdl_tree_.getSegment(link_name)->second.q_nr;
        joint.kdl_joint_ = &(segment->getJoint());
        joint.link_name_ = link_name;
        joint.joint_name_ = segment_joint_mapping_[link_name];
        joint.joint_update_limit_ = joint_update_limit;
        const planning_models::KinematicModel::JointModel* kin_model_joint = robot_models_->getKinematicModel()->getJointModel(joint.joint_name_);
        if (const planning_models::KinematicModel::RevoluteJointModel* revolute_joint = dynamic_cast<const planning_models::KinematicModel::RevoluteJointModel*>(kin_model_joint))
        {
          joint.wrap_around_ = revolute_joint->continuous_;
          joint.has_joint_limits_ = !(joint.wrap_around_);
          std::pair<double,double> bounds;
          if (!joint.wrap_around_)
          {
            revolute_joint->getVariableBounds(revolute_joint->getName(), bounds);
            joint.joint_limit_min_ = bounds.first;
            joint.joint_limit_max_ = bounds.second;
          }
          ROS_DEBUG_STREAM("Setting bounds for joint " << revolute_joint->getName() << " to " << joint.joint_limit_min_ << " " << joint.joint_limit_max_);
        }
        else if (const planning_models::KinematicModel::PrismaticJointModel* prismatic_joint = dynamic_cast<const planning_models::KinematicModel::PrismaticJointModel*>(kin_model_joint))
        {
          joint.wrap_around_ = false;
          joint.has_joint_limits_ = true;
          std::pair<double,double> bounds;
          prismatic_joint->getVariableBounds(prismatic_joint->getName(), bounds);
          joint.joint_limit_min_ = bounds.first;
          joint.joint_limit_max_ = bounds.second;
          ROS_DEBUG_STREAM("Setting bounds for joint " << prismatic_joint->getName() << " to " << joint.joint_limit_min_ << " " << joint.joint_limit_max_);
        }
        else
        {
          ROS_WARN("STOMP cannot handle floating or planar joints yet.");
        }

        group.num_joints_++;
        group.stomp_joints_.push_back(joint);
        active_joints[joint.kdl_joint_index_] = true;
      }

    }
    group.fk_solver_.reset(new KDL::TreeFkSolverJointPosAxisPartial(kdl_tree_, reference_frame_, active_joints));
    group.end_effector_segment_index_ = group.fk_solver_->segmentNameToIndex(group.end_effector_name_);

    // create a KDL::Chain from the tree, hardcoded for PR2 right arm for ICRA experiments!!
    //KDL::Chain chain;
//    kdl_tree_.getChain("torso_lift_link", "r_gripper_tool_frame", group.kdl_chain_);
//    KDL::Vector gravity(0,0,-9.8);
//    group.id_solver_.reset(new KDL::ChainIdSolver_RNE(group.kdl_chain_, gravity));

    planning_groups_.insert(make_pair(name, group));

  }

  generateLinkCollisionPoints();
  populatePlanningGroupCollisionPoints();

  ROS_DEBUG("Initialized stomp robot model in %s reference frame.", reference_frame_.c_str());

  return true;
}

void StompRobotModel::getLinkInformation(const std::string link_name, std::vector<int>& active_joints, int& segment_number)
{
  // check if the link already exists in the map, if not, add it:
  if (link_collision_points_.find(link_name) == link_collision_points_.end())
  {
    link_collision_points_.insert(make_pair(link_name, std::vector<StompCollisionPoint>()));
  }

  // check if the link already exists in the attached object map, if not, add it:
  if (link_attached_object_collision_points_.find(link_name) == link_attached_object_collision_points_.end())
  {
    link_attached_object_collision_points_.insert(make_pair(link_name, std::vector<StompCollisionPoint>()));
  }

  ROS_DEBUG_STREAM("Link info for " << link_name);

  // identify the joints that contribute to this link
  active_joints.clear();
  KDL::SegmentMap::const_iterator segment_iter = kdl_tree_.getSegment(link_name);

  // go up the tree until we find the root:
  while (segment_iter != kdl_tree_.getRootSegment())
  {
    KDL::Joint::JointType joint_type =  segment_iter->second.segment.getJoint().getType();
    if (joint_type != KDL::Joint::None)
    {
      active_joints.push_back(segment_iter->second.q_nr);
      ROS_DEBUG_STREAM("Adding parent " << segment_iter->second.segment.getJoint().getName());
    }
    segment_iter = segment_iter->second.parent;
  }
  ROS_DEBUG(" ");

  segment_number = fk_solver_->segmentNameToIndex(link_name);

}

void StompRobotModel::addCollisionPointsFromLink(std::string link_name, double clearance)
{
  bodies::Body* body = bodies::createBodyFromShape(robot_models_->getKinematicModel()->getLinkModel(link_name)->getLinkShape());
  //body->setPadding(monitor_->getEnvironmentModel()->getCurrentLinkPadding(link_state->getName()));
  tf::Transform link_transform = robot_models_->getKinematicModel()->getLinkModel(link_name)->getCollisionOriginTransform();
  //body->setPose(link_state->getGlobalLinkTransform());
  body->setScale(1.0);
  bodies::BoundingCylinder cyl;
  body->computeBoundingCylinder(cyl);
  delete body;

  std::vector<int> active_joints;
  //KDL::SegmentMap::const_iterator segment_iter = kdl_tree_.getSegment(link_name);
  int segment_number;

  ROS_DEBUG_STREAM("Link " << link_name << " length " << cyl.length << " radius " << cyl.radius);

  getLinkInformation(link_name, active_joints, segment_number);
  std::vector<StompCollisionPoint>& collision_points_vector = link_collision_points_.find(link_name)->second;

  // new method, directly using the bounding cylinder
  double spacing = cyl.radius;
  double distance = cyl.length;
  int num_points = ceil(distance/spacing)+1;
  spacing = distance/(num_points-1.0);
  tf::Vector3 point_pos;
  tf::Vector3 point_pos_transformed;
  KDL::Vector point_pos_kdl;
  for (int i=0; i<num_points; ++i)
  {
    point_pos.setX(0.0);
    point_pos.setY(0.0);
    point_pos.setZ(-cyl.length/2.0 + cyl.length*(i/(num_points-1.0)));
    point_pos_transformed = link_transform * cyl.pose * point_pos;
    point_pos_kdl.x(point_pos_transformed.x());
    point_pos_kdl.y(point_pos_transformed.y());
    point_pos_kdl.z(point_pos_transformed.z());
    collision_points_vector.push_back(StompCollisionPoint(active_joints, cyl.radius, clearance, segment_number, point_pos_kdl));
    if(max_radius_clearance_ < cyl.radius + clearance)
    {
      max_radius_clearance_ = cyl.radius + clearance;
    }

  }

  ROS_DEBUG_STREAM("Link " << link_name << " has " << collision_points_vector.size() << " points");

}

bool StompRobotModel::StompPlanningGroup::addCollisionPoint(StompCollisionPoint& collision_point, StompRobotModel& robot_model)
{
  // create the new parent joints indexing vector:
  std::vector<int> parent_joints(num_joints_, 0);

  //ROS_INFO_STREAM("Num joints is " << num_joints_ << " parent size is " << collision_point.getParentJoints().size());

  // check if this collision point is controlled by any joints which belong to the group
  bool add_this_point=false;
  for (int i=0; i<num_joints_; i++)
  {
    if (collision_point.isParentJoint(stomp_joints_[i].kdl_joint_index_))
    {
      add_this_point = true;
      break;
    }
  }

  if (!add_this_point)
    return false;

  collision_points_.push_back(StompCollisionPoint(collision_point, collision_point.getParentJoints()));

  return true;

}

void StompRobotModel::getLinkCollisionPoints(std::string link_name, std::vector<StompCollisionPoint>& points)
{
  std::map<std::string, std::vector<StompCollisionPoint> >::iterator it = link_collision_points_.find(link_name);
  if (it==link_collision_points_.end())
    return;

  points = it->second;
}

// void StompRobotModel::attachedObjectCallback(const mapping_msgs::AttachedCollisionObjectConstPtr& attached_object)
// {
//   attached_objects_[attached_object->link_name] =  *attached_object;
//   generateCollisionPoints();
// }

void StompRobotModel::generateLinkCollisionPoints()
{
  // clear out link collision points:
  link_collision_points_.clear();

  std::vector<std::string> all_links_list;
  ROS_VERIFY(usc_utilities::read(node_handle_, "collision_links", all_links_list));

  for(std::vector<std::string>::iterator it = all_links_list.begin();
      it != all_links_list.end();
      it++)
  {
    addCollisionPointsFromLink(*it, collision_clearance_default_);
  }
  collision_links_ = all_links_list;
}

void StompRobotModel::populatePlanningGroupCollisionPoints() {
  // put all collision points into all groups:
  for (std::map<std::string, StompPlanningGroup>::iterator group_it=planning_groups_.begin(); group_it!=planning_groups_.end(); ++group_it)
  {
    // clear out collision points for this group:
    group_it->second.collision_points_.clear();
    
    ROS_DEBUG_STREAM("Group is " << group_it->first);

    //for(std::vector<std::string>::iterator link_name_it = group_it->second.link_names_.begin();
    //    link_name_it != group_it->second.link_names_.end();
    //     link_name_it++) {

    for (vector<string>::const_iterator link_name_it=collision_links_.begin();
         link_name_it!=collision_links_.end(); ++link_name_it)
    {
      std::map<std::string, std::vector<StompCollisionPoint> >::iterator link_it = link_collision_points_.find(*link_name_it);
      if (link_it != link_collision_points_.end())
      {
        ROS_DEBUG_STREAM("Adding points for link " << *link_name_it << " point num " << link_it->second.size());
        for (std::vector<StompCollisionPoint>::iterator point_it=link_it->second.begin(); point_it!=link_it->second.end(); ++point_it)
        {
          if(group_it->second.addCollisionPoint(*point_it, *this)) {
            ROS_DEBUG_STREAM("Adding point " << group_it->second.collision_points_.size()-1 << " from link " << *link_name_it);
          }
        }
      } else {
        //ROS_INFO_STREAM("Link " << *link_name_it << " not found in link_collision_points_");
      }
      link_it = link_attached_object_collision_points_.find(*link_name_it);
      if (link_it != link_attached_object_collision_points_.end())
      {
        ROS_DEBUG("\t%s", link_it->first.c_str());
        for (std::vector<StompCollisionPoint>::iterator point_it=link_it->second.begin(); point_it!=link_it->second.end(); ++point_it)
        {
          group_it->second.addCollisionPoint(*point_it, *this);
        }
      }
    }
    ROS_DEBUG_STREAM("Group " << group_it->first << " point num " << group_it->second.collision_points_.size());
  }
}

std::vector<std::string> StompRobotModel::StompPlanningGroup::getJointNames() const
{
  std::vector<std::string> ret;
  for (unsigned int i=0; i<stomp_joints_.size(); ++i)
    ret.push_back(stomp_joints_[i].joint_name_);
  return ret;
}

std::vector<double> StompRobotModel::StompPlanningGroup::getJointArrayFromJointState(const sensor_msgs::JointState& msg) const
{
  std::vector<double> ret(num_joints_, 0.0);
  for (unsigned int i=0; i<msg.name.size(); ++i)
  {
    for (int j=0; j<num_joints_; ++j)
    {
      if (msg.name[i] == stomp_joints_[j].joint_name_)
      {
        ret[j] = msg.position[i];
      }
    }
  }
  return ret;
}

std::vector<double> StompRobotModel::StompPlanningGroup::getJointArrayFromGoalConstraints(const arm_navigation_msgs::Constraints& msg) const
{
  std::vector<double> ret(num_joints_, 0.0);
  for (unsigned int i=0; i<msg.joint_constraints.size(); ++i)
  {
    for (int j=0; j<num_joints_; ++j)
    {
      if (msg.joint_constraints[i].joint_name == stomp_joints_[j].joint_name_)
      {
        ret[j] = msg.joint_constraints[i].position;
      }
    }
  }
  return ret;
}

boost::shared_ptr<KDL::TreeFkSolverJointPosAxisPartial> StompRobotModel::StompPlanningGroup::getNewFKSolver() const
{
  boost::shared_ptr<KDL::TreeFkSolverJointPosAxisPartial> ret(
      new KDL::TreeFkSolverJointPosAxisPartial(*fk_solver_));
  return ret;
}

} // namespace stomp
