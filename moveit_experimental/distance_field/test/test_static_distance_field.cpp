/**
 * @file test_static_distance_field.h
 * @brief This contains gtest code to test static distance field.
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
#include <gtest/gtest.h>
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

TEST(TestStaticDistanceField, TestWriteImportData)
{
  double res = 0.2;

  //Get Temp directory
  boost::filesystem::path temp_file = boost::filesystem::temp_directory_path();
  temp_file += "/distance_field.txt";

  // Create Propagation Distance Field
  distance_field::PropagationDistanceField df(8, 8, 8, res, 0, 0, 0, 1.0, true);

  // generate points that represent the main sphere
  EigenSTL::vector_Vector3d main_sphere = createSphere(Eigen::Vector3d(4, 4, 4), 0.5);
  for(int i = 0; i<main_sphere.size();i++)
  {
    geometry_msgs::Point p;
    tf::pointEigenToMsg(main_sphere[i], p);
  }
  df.addPointsToField(main_sphere);

  // write distance field to file
  std::ofstream out_stream(temp_file.string());
  distance_field::writeToStreamStatic<float>(out_stream, df);
  out_stream.close();

  // Create static distance field from file
  std::ifstream in_stream(temp_file.string());
  distance_field::StaticDistanceField<float> df2(in_stream);
  in_stream.close();

  double dist1, dist2, grad_x1, grad_y1, grad_z1, grad_x2, grad_y2, grad_z2, norm;
  Eigen::Vector3d pick_point(4.5, 4.0, 3.274);
  bool in_bounds, valid;

  dist1 = df.getDistanceGradient(pick_point(0), pick_point(1), pick_point(2), grad_x1, grad_y1, grad_z1, in_bounds);
  dist2 = df2.getDistanceGradient(pick_point(0), pick_point(1), pick_point(2), grad_x2, grad_y2, grad_z2, in_bounds);
  valid = false;
  if (abs(dist1-dist2) <= 1e-6)
  {
    valid = true;
  }
  EXPECT_TRUE(valid);

  valid = false;
  if (abs(grad_x1-grad_x2) <= 1e-6 && abs(grad_y1-grad_y2) <= 1e-6  && abs(grad_z1-grad_z2) <= 1e-6 )
  {
    valid = true;
  }
  EXPECT_TRUE(valid);
}

int main (int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
