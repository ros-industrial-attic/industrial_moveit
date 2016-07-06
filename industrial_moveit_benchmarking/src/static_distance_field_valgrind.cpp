/**
 * @file static_distance_field_valgrind.cpp
 * @brief This is used to benchmark static distance field.
 *
 * @author Levi Armstrong
 * @date June 22, 2016
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
#include <ros/ros.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include "static_distance_field.h"
#include <eigen_conversions/eigen_msg.h>
#include <boost/filesystem.hpp>
#include <cmath>
#include <ostream>

EigenSTL::vector_Vector3d createSphere(Eigen::Vector3d origin, double radius)
{
  // generate points that repsent a sphere
  EigenSTL::vector_Vector3d points;
  double r =radius;
  double u, x, y, z;
  double phi, theta;
  for(double l=r/50; l<=r; l+=r/10)
  {
    for(phi=0;phi<M_PI;phi+=(M_PI/10))
    {
      u=l*cos(phi);
      for(theta=0;theta<2*M_PI;theta+=(2*M_PI/20))
      {
        x=origin(0)+sqrt(l*l-u*u)*cos(theta);
        y=origin(1)+sqrt(l*l-u*u)*sin(theta);
        z=origin(2)+u;
        points.push_back(Eigen::Vector3d(x,y,z));
      }
    }
  }
  return points;
}

int main (int argc, char *argv[])
{
  ros::init(argc, argv, "static_distance_field_valgrind");
  ros::NodeHandle pnh("~");
  ros::Rate loop_rate(100);
  visualization_msgs::Marker obs_markers, dist_markers, pick_marker;
  visualization_msgs::MarkerArray grad_markers;
  ros::Publisher dist_pub = pnh.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
  ros::Publisher grad_pub = pnh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000);
  double res = 0.02;
  ros::Time start;
  bool flag;

  double dist, grad_x, grad_y, grad_z, norm, t;
  Eigen::Vector3d pick_point(4.5, 4.0, 3.274);
  bool in_bounds;

  //Get Temp directory
  boost::filesystem::path temp_file = boost::filesystem::temp_directory_path();
  temp_file += "/distance_field.txt";
  ROS_INFO("File Path: %s", temp_file.string().c_str());

  t=0;
  flag = true;
  if (flag)
  {
    ROS_INFO("Initializing Distance Field");
    start = ros::Time::now();
    distance_field::PropagationDistanceField df(8, 8, 8, res, 0, 0, 0, 1.0, true);
    ROS_INFO("Time Elapsed: %f (sec)",(ros::Time::now() - start).toSec());
    ROS_INFO("Cell Count: %f", pow(8.0/res, 3));

    // generate points that represent the main sphere
    EigenSTL::vector_Vector3d main_sphere = createSphere(Eigen::Vector3d(4, 4, 4), 0.5);
    for(int i = 0; i<main_sphere.size();i++)
    {
      geometry_msgs::Point p;
      tf::pointEigenToMsg(main_sphere[i], p);
      obs_markers.points.push_back(p);
    }
    obs_markers.header.frame_id = "map";
    obs_markers.header.stamp = ros::Time::now();
    obs_markers.ns = "obstical_points";
    obs_markers.id = 1;
    obs_markers.type = visualization_msgs::Marker::CUBE_LIST;
    obs_markers.action = visualization_msgs::Marker::MODIFY;
    obs_markers.color.a = 1.0;
    obs_markers.color.r = 0.0;
    obs_markers.color.g = 1.0;
    obs_markers.color.b = 0.0;
    obs_markers.scale.x = 0.01;
    obs_markers.scale.y = 0.01;
    obs_markers.scale.z = 0.01;

    ROS_INFO("Adding sphere to distance field.");
    start = ros::Time::now();
    df.addPointsToField(main_sphere);
    ROS_INFO("Time Elapsed: %f (sec)",(ros::Time::now() - start).toSec());

    // Visualize the distance info in rviz
    df.getIsoSurfaceMarkers(0.001, 0.25, "map", ros::Time::now(), dist_markers);
    dist_markers.color.a = 1.0;
    dist_markers.scale.x = 0.01;
    dist_markers.scale.y = 0.01;
    dist_markers.scale.z = 0.01;

    df.getGradientMarkers(0.0, 0.1, "map", ros::Time::now(), grad_markers);
    for(int i=0;i<grad_markers.markers.size();i++)
    {
      grad_markers.markers[i].scale.x = 0.1;
      grad_markers.markers[i].scale.y = 0.01;
      grad_markers.markers[i].scale.z = 0.01;
    }

    for(int i=0; i<10; i++)
    {
      start = ros::Time::now();
      dist = df.getDistanceGradient(pick_point(0), pick_point(1), pick_point(2), grad_x, grad_y, grad_z, in_bounds);
      t+=(ros::Time::now() - start).toSec();
    }

    norm = sqrt(grad_x*grad_x + grad_y*grad_y + grad_z*grad_z);
    ROS_INFO("Distance: %f", dist);
    ROS_INFO("Gradient: %f %f %f", grad_x/norm, grad_y/norm, grad_z/norm);
    ROS_INFO("Average Time Elapsed: %0.8f (sec)",t/10.0);

    geometry_msgs::Point p1, p2;
    Eigen::Translation3d m(Eigen::Vector3d(grad_x, grad_y, grad_z));
    tf::pointEigenToMsg(pick_point,p1);
    tf::pointEigenToMsg(m*pick_point,p2);

    pick_marker.points.push_back(p1);
    pick_marker.points.push_back(p2);
    pick_marker.header.frame_id = "map";
    pick_marker.header.stamp = ros::Time::now();
    pick_marker.ns = "pick_point";
    pick_marker.id = 1;
    pick_marker.type = visualization_msgs::Marker::ARROW;
    pick_marker.action = visualization_msgs::Marker::MODIFY;
    pick_marker.color.a = 1.0;
    pick_marker.color.r = 0.0;
    pick_marker.color.g = 0.0;
    pick_marker.color.b = 1.0;
    pick_marker.scale.x = 0.03;
    pick_marker.scale.y = 0.03;
    pick_marker.scale.z = 0.03;

    //write distance field to file
    start = ros::Time::now();
    std::ofstream out_stream(temp_file.string());
    distance_field::writeToStreamStatic<float>(out_stream, df);
    out_stream.close();
    ROS_INFO("Time to write file: %f (sec)",(ros::Time::now() - start).toSec());

    while(ros::ok())
    {
      dist_pub.publish(obs_markers);
      dist_pub.publish(dist_markers);
      dist_pub.publish(pick_marker);
      grad_pub.publish(grad_markers);
      loop_rate.sleep();
    }
  }
  else
  {
    //read distance field from file
    start = ros::Time::now();
    std::ifstream in_stream(temp_file.string());
    distance_field::StaticDistanceField<float> df2(in_stream);
    in_stream.close();
    ROS_INFO("Time to read file: %f (sec)",(ros::Time::now() - start).toSec());

    for(int i=0; i<10; i++)
    {
      start = ros::Time::now();
      dist = df2.getDistanceGradient(pick_point(0), pick_point(1), pick_point(2), grad_x, grad_y, grad_z, in_bounds);
      t+=(ros::Time::now() - start).toSec();
    }

    norm = sqrt(grad_x*grad_x + grad_y*grad_y + grad_z*grad_z);
    ROS_INFO("Distance: %f", dist);
    ROS_INFO("Gradient: %f %f %f", grad_x/norm, grad_y/norm, grad_z/norm);
    ROS_INFO("Average Time Elapsed: %0.8f (sec)",t/10.0);

    while(ros::ok())
    {
      loop_rate.sleep();
    }
  }
}
