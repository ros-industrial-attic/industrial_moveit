#include "openvdb_visualization.h"

distance_field::PointCloud::Ptr distance_field::toPointCloud(const openvdb::FloatGrid &grid)
{
  PointCloud::Ptr cloud (new distance_field::PointCloud);

  for (auto iter = grid.beginValueOn(); iter; ++iter)
  {
    pcl::PointXYZRGB pt;

    auto value = iter.getValue();
    auto point = grid.transform().indexToWorld(iter.getCoord());

    // fill out point
    pt.x = point.x();
    pt.y = point.y();
    pt.z = point.z();

    pt.r = value > 0 ? 255 : 0;
    pt.g = 0;
    pt.b = value > 0 ? 0 : 255;

    cloud->push_back(pt);
  }

  return cloud;
}

visualization_msgs::Marker distance_field::toSphere(const openvdb::Vec4s &sphere_data, int id)
{
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.ns = "spheres";
  m.id = id;
  m.type = m.SPHERE;
  m.action = m.ADD;
  m.pose.orientation.w = 1.0;
  m.pose.position.x = sphere_data.x();
  m.pose.position.y = sphere_data.y();
  m.pose.position.z = sphere_data.z();
  m.scale.x = sphere_data.w();
  m.scale.y = sphere_data.w();
  m.scale.z = sphere_data.w();
  m.color.a = 1.0;
  m.color.g = 1.0;
  m.lifetime = ros::Duration();
  return m;
}

std::pair<distance_field::PointCloud::Ptr, distance_field::PointCloud::Ptr>
distance_field::toInsideOutsidePointCloud(const openvdb::FloatGrid &grid)
{
  std::pair<distance_field::PointCloud::Ptr, distance_field::PointCloud::Ptr> ret;
  ret.first.reset(new distance_field::PointCloud); // inside
  ret.second.reset(new distance_field::PointCloud); // outside

  PointCloud& inside = *ret.first;
  PointCloud& outside = *ret.second;

  for (auto iter = grid.beginValueOn(); iter; ++iter)
  {
    pcl::PointXYZRGB pt;

    auto value = iter.getValue();
    auto point = grid.transform().indexToWorld(iter.getCoord());

    // fill out point
    pt.x = point.x();
    pt.y = point.y();
    pt.z = point.z();

    pt.r = value > 0 ? 255 : 0;
    pt.g = 0;
    pt.b = value > 0 ? 0 : 255;

    if (value > 0) outside.push_back(pt);
    else inside.push_back(pt);
  }

  return ret;
}
