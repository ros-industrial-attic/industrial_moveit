#ifndef OPENVDB_VISUALIZATION_H
#define OPENVDB_VISUALIZATION_H

#include <openvdb/openvdb.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>

namespace distance_field
{
using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;

// Deep copies data from openvdb grid into a PCL point cloud
// that we can use for visualization sake
PointCloud::Ptr toPointCloud(const openvdb::FloatGrid& grid);

// Same as above but divides the grid into two point clouds. The 'first' contains
// all points that are inside a surface (distance < 0), and the 'second' contains
// all points that are outside a surface.
std::pair<PointCloud::Ptr, PointCloud::Ptr> toInsideOutsidePointCloud(const openvdb::FloatGrid& grid);

// Creates a visualization messages marker SPHERE out of the data in
// the 4-dimension vector from OpenVDB (center x y z & radius)
visualization_msgs::Marker toSphere(const openvdb::Vec4s& sphere_data, int id);

}

#endif // OPENVDB_VISUALIZATION_H
