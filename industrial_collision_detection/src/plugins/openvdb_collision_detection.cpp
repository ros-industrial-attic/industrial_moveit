/**
 * @file openvdb_collision_detection.cpp
 * @brief TODO
 *
 * @author Jorge Nicho
 * @date Aug 31, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
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

#include <XmlRpcException.h>
#include <ros/node_handle.h>
#include <ros/package.h>
#include <pluginlib/class_list_macros.h>
#include <moveit/collision_detection/collision_plugin.h>
#include <industrial_collision_detection/collision_detection/collision_robot_openvdb.h>
#include <industrial_collision_detection/collision_detection/collision_world_industrial.h>

static const std::string COLLISION_ROBOT_PARAM = "collision_robot_openvdb";

namespace collision_detection
{
  /** \brief An allocator for Industrial Moveit FCL collision detectors */
  class OpenVDBCollisionDetectorAllocator :
      public collision_detection::CollisionDetectorAllocatorTemplate< CollisionWorldIndustrial,
      CollisionRobotOpenVDB, OpenVDBCollisionDetectorAllocator >
  {

  public:
    virtual CollisionRobotPtr allocateRobot(const robot_model::RobotModelConstPtr& robot_model) const override
    {
      CollisionRobotPtr col_robot;

      // loading parameters
      ros::NodeHandle nh("~");
      XmlRpc::XmlRpcValue param;
      if(!nh.getParam(COLLISION_ROBOT_PARAM,param))
      {
        ROS_WARN("the '%s' parameter was not found, CollisionRobotOpenVDB will use default values",COLLISION_ROBOT_PARAM.c_str());
        col_robot.reset(new CollisionRobotOpenVDB(robot_model));
      }
      else
      {

        // loading distance field from file
        if (param.hasMember("package") && param.hasMember("relative_path"))
        {
          std::string package = static_cast<std::string>(param["package"]);
          std::string relative_path = static_cast<std::string>(param["relative_path"]);
          const auto full_path = ros::package::getPath(package) + relative_path;

          ROS_INFO("Loading OpenVDB Distance Field from existing file");
          col_robot.reset(new CollisionRobotOpenVDB(robot_model,full_path));
        }
        else // creating distance field
        {
          ROS_INFO("Generating OpenVDB Distance Field from urdf");
          auto members = {"voxel_size","background","exterior_bandwidth","interior_bandwidth"};
          bool found_all = true;
          for(auto& m : members)
          {
            if(!param.hasMember(m))
            {
              ROS_WARN("%s is missing one or more parameters, CollisionRobotOpenVDB will use default values",COLLISION_ROBOT_PARAM.c_str());
              found_all = false;
              break;
            }
          }

          if(found_all)
          {
            try
            {
              double voxel = static_cast<double>(param["voxel_size"]);
              double background = static_cast<double>(param["background"]);
              double ext_bandwidth = static_cast<double>(param["exterior_bandwidth"]);
              double int_bandwidth = static_cast<double>(param["interior_bandwidth"]);
              col_robot.reset(new CollisionRobotOpenVDB(robot_model,voxel,background,ext_bandwidth,int_bandwidth));
            }
            catch(XmlRpc::XmlRpcException& e)
            {
              ROS_WARN("Failed while loading of CollisionRobotOpenVDB parameters, using default values");
              col_robot.reset(new CollisionRobotOpenVDB(robot_model));
            }
          }
          else
          {
            col_robot.reset(new CollisionRobotOpenVDB(robot_model));
          }
        }
      }
      return col_robot;
    }

    virtual CollisionRobotPtr allocateRobot(const CollisionRobotConstPtr& orig) const
    {
      return CollisionRobotPtr(new CollisionRobotOpenVDB(dynamic_cast<const CollisionRobotOpenVDB&>(*orig)));
    }


  public:
    static const std::string NAME_; // defined in collision_world_industrial.cpp


  };



  const std::string OpenVDBCollisionDetectorAllocator::NAME_ = "CollisionDetectionOpenVDB";

  class OpenVDBPluginLoader : public CollisionPlugin
  {
  public:
    virtual bool initialize(const planning_scene::PlanningScenePtr& scene, bool exclusive) const
    {
      scene->setActiveCollisionDetector(OpenVDBCollisionDetectorAllocator::create(), exclusive);
      return true;
    }

  };

}

PLUGINLIB_EXPORT_CLASS(collision_detection::OpenVDBPluginLoader, collision_detection::CollisionPlugin)



