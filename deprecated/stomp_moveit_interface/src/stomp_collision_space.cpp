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

#include <stomp_ros_interface/stomp_collision_space.h>
#include <planning_environment/util/construct_object.h>
#include <planning_environment/models/model_utils.h>
#include <sstream>

namespace stomp_ros_interface
{

StompCollisionSpace::StompCollisionSpace(ros::NodeHandle node_handle):
  node_handle_(node_handle)
{
  viz_pub_ = node_handle_.advertise<visualization_msgs::Marker>("collision_space", 10, true);
}

StompCollisionSpace::~StompCollisionSpace()
{
}

bool StompCollisionSpace::init(double max_radius_clearance, std::string& reference_frame)
{
  double size_x, size_y, size_z;
  double origin_x, origin_y, origin_z;
  double resolution;

  reference_frame_ = reference_frame;

  node_handle_.param("collision_space/size_x", size_x, 2.0);
  node_handle_.param("collision_space/size_y", size_y, 3.0);
  node_handle_.param("collision_space/size_z", size_z, 4.0);
  node_handle_.param("collision_space/origin_x", origin_x, 0.1);
  node_handle_.param("collision_space/origin_y", origin_y, -1.5);
  node_handle_.param("collision_space/origin_z", origin_z, -2.0);
  node_handle_.param("collision_space/resolution", resolution, 0.02);
  resolution_ = resolution;
  max_expansion_ = max_radius_clearance;

  distance_field_.reset(new distance_field::SignedPropagationDistanceField(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z, max_radius_clearance));

  ROS_DEBUG("Initialized stomp collision space in %s reference frame with %f expansion radius.", reference_frame_.c_str(), max_expansion_);
  return true;
}

void StompCollisionSpace::setPlanningScene(const arm_navigation_msgs::PlanningScene& planning_scene)
{
  planning_scene_ = planning_scene;
  ros::WallTime start = ros::WallTime::now();

  distance_field_->reset();

  std::vector<tf::Vector3> all_points;

  //tf::Transform id;
  //id.setIdentity();

  for (unsigned int i=0; i<planning_scene.collision_objects.size(); ++i)
  {
    const arm_navigation_msgs::CollisionObject& object = planning_scene.collision_objects[i];
    addCollisionObjectToPoints(all_points, object);
  }

  ROS_INFO_STREAM("All points size " << all_points.size());
  
  distance_field_->addPointsToField(all_points);

  ros::WallDuration t_diff = ros::WallTime::now() - start;
  ROS_INFO_STREAM("Took " << t_diff.toSec() << " to set distance field");

  tf::Transform identity;
  identity.setIdentity();
  visualization_msgs::Marker marker;
  distance_field_->getIsoSurfaceMarkers(0.0, 0.03, reference_frame_, ros::Time::now(), identity, marker);
  viz_pub_.publish(marker);
}

const arm_navigation_msgs::PlanningScene& StompCollisionSpace::getPlanningScene()
{
  return planning_scene_;
}

void StompCollisionSpace::addCollisionObjectToPoints(std::vector<tf::Vector3>& points, const arm_navigation_msgs::CollisionObject& object)
{
  ROS_ASSERT(object.shapes.size() == object.poses.size());
  for (unsigned int j=0; j<object.shapes.size(); ++j)
  {
    // TODO everything handled using MESH method, could be much faster for primitive types!!

    tf::Transform pose;
    tf::poseMsgToTF(object.poses[j], pose);

    if (object.shapes[j].type == arm_navigation_msgs::Shape::BOX)
    {
      double x_extent = object.shapes[j].dimensions[0]/2.0;
      double y_extent = object.shapes[j].dimensions[1]/2.0;
      double z_extent = object.shapes[j].dimensions[2]/2.0;

      for(double x = -x_extent; x <= x_extent+resolution_; x += resolution_)
      {
        for(double y = -y_extent; y <= y_extent+resolution_; y += resolution_)
        {
          for(double z = -z_extent; z <= z_extent+resolution_; z += resolution_)
          {
            tf::Vector3 p(x, y, z);
            tf::Vector3 pp = pose*p;
            //ROS_INFO("Point: %f, %f, %f", pp.x(), pp.y(), pp.z());
            points.push_back(pp);
          }
        }
      }

    }
    else if (object.shapes[j].type == arm_navigation_msgs::Shape::MESH)
    {
      shapes::Shape* shape = planning_environment::constructObject(object.shapes[j]);
      bodies::Body *body = bodies::createBodyFromShape(shape);
      tf::Transform body_pose;
      tf::poseMsgToTF(object.poses[j], body_pose);
      body->setPose(body_pose);
      getVoxelsInBody(*body, points);
      delete body;
      delete shape;
    }
  }
/*

  for(unsigned int j = 0; j < n; j++) {
      if (no.shape[j]->type == shapes::MESH) {
        bodies::Body *body = bodies::createBodyFromShape(no.shape[j]);
        body->setPose(inv*no.shapePose[j]);
        std::vector<tf::Vector3> body_points;
        getVoxelsInBody(*body, body_points);
        points.insert(points.end(), body_points.begin(), body_points.end());
        delete body;
      } else {
        geometric_shapes_msgs::Shape object;
        if(!planning_environment::constructObjectMsg(no.shape[j], object)) {
          ROS_WARN("Shap cannot be converted");
          continue;
        }
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(no.shapePose[j], pose);
        KDL::Rotation rotation = KDL::Rotation::Quaternion(pose.orientation.x,
                                                           pose.orientation.y,
                                                           pose.orientation.z,
                                                           pose.orientation.w);
        KDL::Vector position(pose.position.x, pose.position.y, pose.position.z);
        KDL::Frame f(rotation, position);
        if (object.type == geometric_shapes_msgs::Shape::CYLINDER)
        {
          if (object.dimensions.size() != 2) {
            ROS_INFO_STREAM("Cylinder must have exactly 2 dimensions, not " 
                            << object.dimensions.size()); 
            continue;
          }
          // generate points:
          double radius = object.dimensions[0];
          //ROS_INFO_STREAM("Divs " << xdiv << " " << ydiv << " " << zdiv);
          
          double xlow = pose.position.x - object.dimensions[0];
          double ylow = pose.position.y - object.dimensions[0];
          double zlow = pose.position.z - object.dimensions[1]/2.0;

          //ROS_INFO("pose " << pose.position.x << " " << pose.position.y);
          
          for(double x = xlow; x <= xlow+object.dimensions[0]*2.0+resolution_; x += resolution_) {
            for(double y = ylow; y <= ylow+object.dimensions[0]*2.0+resolution_; y += resolution_) {
              for(double z = zlow; z <= zlow+object.dimensions[1]+resolution_; z += resolution_) {
                double xdist = fabs(pose.position.x-x);
                double ydist = fabs(pose.position.y-y);
                //if(pose.position.z-z == 0.0) {
                //  ROS_INFO_STREAM("X " << x << " Y " << y << " Dists " << xdist << " " << ydist << " Rad " << sqrt(xdist*xdist+ydist*ydist));
                // }
                if(sqrt(xdist*xdist+ydist*ydist) <= radius) {
                  KDL::Vector p(pose.position.x-x,pose.position.y-y,pose.position.z-z);                  
                  KDL::Vector p2;
                  p2 = f*p;
                  points.push_back(inv*tf::Vector3(p2(0),p2(1),p2(2)));
                }
              }
            }
          }
        } else if (object.type == geometric_shapes_msgs::Shape::BOX) {
          if(object.dimensions.size() != 3) {
            ROS_INFO_STREAM("Box must have exactly 3 dimensions, not " 
                            << object.dimensions.size());
            continue;
          }
              
          double xlow = pose.position.x - object.dimensions[0]/2.0;
          double ylow = pose.position.y - object.dimensions[1]/2.0;
          double zlow = pose.position.z - object.dimensions[2]/2.0;
          
          for(double x = xlow; x <= xlow+object.dimensions[0]+resolution_; x += resolution_) {
            for(double y = ylow; y <= ylow+object.dimensions[1]+resolution_; y += resolution_) {
              for(double z = zlow; z <= zlow+object.dimensions[2]+resolution_; z += resolution_) {
                KDL::Vector p(pose.position.x-x,pose.position.y-y,pose.position.z-z);                  
                KDL::Vector p2;
                p2 = f*p;
                points.push_back(inv*tf::Vector3(p2(0),p2(1),p2(2)));
              }
            }
          }
        } 
      }
    }
  }*/
}

void StompCollisionSpace::getVoxelsInBody(const bodies::Body &body, std::vector<tf::Vector3> &voxels)
{
  bodies::BoundingSphere bounding_sphere;

  body.computeBoundingSphere(bounding_sphere);
  int x,y,z,x_min,x_max,y_min,y_max,z_min,z_max;
  double xw,yw,zw;
  tf::Vector3 v;
	
  worldToGrid(bounding_sphere.center,bounding_sphere.center.x()-bounding_sphere.radius,bounding_sphere.center.y()-bounding_sphere.radius,	
              bounding_sphere.center.z()-bounding_sphere.radius, x_min,y_min,z_min);
  worldToGrid(bounding_sphere.center,bounding_sphere.center.x()+bounding_sphere.radius,bounding_sphere.center.y()+bounding_sphere.radius,	
              bounding_sphere.center.z()+bounding_sphere.radius, x_max,y_max,z_max);
	
  for(x = x_min; x <= x_max; ++x)
  {
    for(y = y_min; y <= y_max; ++y)
    {
      for(z = z_min; z <= z_max; ++z)
      {
        gridToWorld(bounding_sphere.center,x,y,z,xw,yw,zw);

        v.setX(xw);
        v.setY(yw);
        v.setZ(zw);
        // compute all intersections
        int count=0;
        std::vector<tf::Vector3> pts;
        body.intersectsRay(v, tf::Vector3(0, 0, 1), &pts, count);

        // if we have an odd number of intersections, we are inside
        if (pts.size() % 2 == 1)
          voxels.push_back(v);
      }
    }
  }


  // 	ROS_INFO("number of occupied voxels in bounding sphere: %i", voxels.size());
}

}
