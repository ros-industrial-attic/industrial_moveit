#include <industrial_collision_detection/distance_field/openvdb_distance_field.h>

#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/Composite.h>
#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/VolumeToSpheres.h>
#include <openvdb/tools/GridTransformer.h>
#include <openvdb/tools/Interpolation.h>
#include <ros/assert.h>
#include <algorithm>
#include <math.h>

distance_field::OpenVDBDistanceField::OpenVDBDistanceField(float voxel_size, float background) :
  voxel_size_(voxel_size),
  background_(background)
{
  openvdb::initialize();
  transform_ = openvdb::math::Transform::createLinearTransform(voxel_size_);
}

distance_field::OpenVDBDistanceField::OpenVDBDistanceField(openvdb::FloatGrid::Ptr grid)
{
  openvdb::initialize();
  voxel_size_ = grid->metaValue<float>("voxel_size");
  background_ = grid->metaValue<float>("background");

  transform_ = openvdb::math::Transform::createLinearTransform(voxel_size_);
  grid_ = grid;
  accessor_ = std::make_shared<openvdb::FloatGrid::ConstAccessor>(grid_->getConstAccessor());
}

openvdb::FloatGrid::Ptr distance_field::OpenVDBDistanceField::getGrid() const
{
  return grid_;
}

void distance_field::OpenVDBDistanceField::saveMetaData(const std::string &name) const
{
 grid_->setName(name);
 grid_->insertMeta("voxel_size", openvdb::FloatMetadata(voxel_size_));
 grid_->insertMeta("background", openvdb::FloatMetadata(background_));
}

void distance_field::OpenVDBDistanceField::display() const
{
  ROS_INFO_STREAM("Grid: " << grid_->getName());
  ROS_INFO_STREAM("\tMem-Usage: " << grid_->memUsage());
  ROS_INFO_STREAM("\tTF: " << grid_->transform());
  ROS_INFO_STREAM("\tLeafs: " << grid_->tree().leafCount());
  ROS_INFO_STREAM("\tActive Voxels: " << grid_->activeVoxelCount());
  ROS_INFO_STREAM("\tB-Box: " << grid_->evalActiveVoxelBoundingBox());
  ROS_INFO_STREAM("\tBackground: " << grid_->background());
}

double distance_field::OpenVDBDistanceField::getVoxelSize() const
{
  return voxel_size_;
}

openvdb::math::Transform::Ptr distance_field::OpenVDBDistanceField::getTransform() const
{
  return transform_;
}

double distance_field::OpenVDBDistanceField::getDistance(const Eigen::Vector3f &point, bool thread_safe) const
{
  return getDistance(point(0), point(1), point(2), thread_safe);
}

double distance_field::OpenVDBDistanceField::getDistance(const openvdb::math::Coord &coord, bool thread_safe) const
{
  if (thread_safe)
  {
    return grid_->tree().getValue(coord);
  }

  if (accessor_)
  {
    return accessor_->getValue(coord);
  }
  else
  {
    ROS_ERROR("Tried to get distance data from and empty grid.");
    return 0;
  }
}

double distance_field::OpenVDBDistanceField::getDistance(const float &x, const float &y, const float &z, bool thread_safe) const
{
  openvdb::math::Vec3s xyz(x, y, z);
  return getDistance(transform_->worldToIndexNodeCentered(xyz), thread_safe);
}

bool distance_field::OpenVDBDistanceField::getGradient(const Eigen::Vector3f &point, Eigen::Vector3d &gradient, bool thread_safe) const
{
  return getGradient(point(0), point(1), point(2), gradient, thread_safe);
}

bool distance_field::OpenVDBDistanceField::getGradient(const openvdb::math::Coord &coord, Eigen::Vector3d &gradient, bool thread_safe) const
{
  openvdb::Vec3f result;
  if (thread_safe)
  {
    gradient(0) = (grid_->tree().getValue(coord.offsetBy(1, 0, 0)) - grid_->tree().getValue(coord.offsetBy(-1,  0,  0)))/(2*grid_->voxelSize()[0]);
    gradient(1) = (grid_->tree().getValue(coord.offsetBy(0, 1, 0)) - grid_->tree().getValue(coord.offsetBy( 0, -1,  0)))/(2*grid_->voxelSize()[1]);
    gradient(2) = (grid_->tree().getValue(coord.offsetBy(0, 0, 1)) - grid_->tree().getValue(coord.offsetBy( 0,  0, -1)))/(2*grid_->voxelSize()[2]);
  }
  else
  {
    if (accessor_)
    {
      result =  openvdb::math::ISGradient<openvdb::math::CD_2ND>::result(*accessor_, coord);
    }
    else
    {
      ROS_ERROR("Tried to get distance and gradient data from and empty grid.");
      return false;
    }

    gradient(0) = result(0);
    gradient(1) = result(1);
    gradient(2) = result(2);
  }

  if (gradient.norm() != 0)
  {
    gradient.normalize();
    return true;
  }
  else
  {
    return false;
  }
}

bool distance_field::OpenVDBDistanceField::getGradient(const float &x, const float &y, const float &z, Eigen::Vector3d &gradient, bool thread_safe) const
{
  openvdb::math::Vec3s xyz(x, y, z);
  return getGradient(transform_->worldToIndexNodeCentered(xyz), gradient, thread_safe);
}

void distance_field::OpenVDBDistanceField::fillWithSpheres(SphereModel &spheres, int maxSphereCount, bool overlapping, float minRadius, float maxRadius, float isovalue, int instanceCount)
{
  std::vector<openvdb::math::Vec4s> s;
  openvdb::tools::fillWithSpheres<openvdb::FloatGrid>(*grid_, s, maxSphereCount, overlapping, minRadius, maxRadius, isovalue, instanceCount);

  // convert data to eigen data types
  for (auto it = s.begin(); it != s.end(); ++it)
  {
    spheres.push_back(std::make_pair(it->getVec3(), (*it)[3]));
  }

  if (spheres.size()== 0)
    ROS_WARN("Unable to fill grid with spheres.");
}

void distance_field::OpenVDBDistanceField::addLinkToField(const moveit::core::LinkModel *link, const Eigen::Affine3d &pose, const float exBandWidth, const float inBandWidth)
{
  std::vector<shapes::ShapeConstPtr> shapes = link->getShapes();
  EigenSTL::vector_Affine3d shape_poses = link->getCollisionOriginTransforms();

  for (int j = 0; j < shapes.size(); ++j)
  {
    addShapeToField(shapes[j].get(), pose * shape_poses[j], exBandWidth, inBandWidth);
  }
}

void distance_field::OpenVDBDistanceField::addShapeToField(const shapes::Shape *shape, const Eigen::Affine3d &pose, const float exBandWidth, const float inBandWidth)
{
  openvdb::FloatGrid::Ptr grid;

  switch(shape->type)
  {
    case shapes::BOX:
    {
      const shapes::Box *box = static_cast<const shapes::Box *>(shape);
      const openvdb::math::Vec3s pmax = openvdb::math::Vec3s(std::abs(box->size[0])/2.0, std::abs(box->size[1])/2.0, std::abs(box->size[2])/2.0);
      const openvdb::math::Vec3s pmin = -1.0*pmax;

      std::vector<openvdb::math::Vec3s> points(8);
      points[0] = openvdb::math::Vec3s(pmin[0], pmin[1], pmin[2]);
      points[1] = openvdb::math::Vec3s(pmin[0], pmin[1], pmax[2]);
      points[2] = openvdb::math::Vec3s(pmax[0], pmin[1], pmax[2]);
      points[3] = openvdb::math::Vec3s(pmax[0], pmin[1], pmin[2]);
      points[4] = openvdb::math::Vec3s(pmin[0], pmax[1], pmin[2]);
      points[5] = openvdb::math::Vec3s(pmin[0], pmax[1], pmax[2]);
      points[6] = openvdb::math::Vec3s(pmax[0], pmax[1], pmax[2]);
      points[7] = openvdb::math::Vec3s(pmax[0], pmax[1], pmin[2]);

      std::vector<openvdb::Vec4I> quads(6);
      quads[0] = openvdb::Vec4I(0, 1, 2, 3); // bottom
      quads[1] = openvdb::Vec4I(7, 6, 5, 4); // top
      quads[2] = openvdb::Vec4I(4, 5, 1, 0); // front
      quads[3] = openvdb::Vec4I(6, 7, 3, 2); // back
      quads[4] = openvdb::Vec4I(0, 3, 7, 4); // left
      quads[5] = openvdb::Vec4I(1, 5, 6, 2); // right

      // Tranform point location
      TransformVec3s(pose, points.data(), points.size());

      // Convert data from world to index
      WorldToIndex(transform_, points.data(), points.size());

      openvdb::tools::QuadAndTriangleDataAdapter<openvdb::Vec3s, openvdb::Vec4I> mesh (points.data(), points.size(),
                                                                                       quads.data(), quads.size());
      grid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(mesh, *transform_, exBandWidth, inBandWidth);
      break;
    }

    case shapes::CONE:
    {
      const shapes::Cone *cone = static_cast<const shapes::Cone *>(shape);

      // Number of sides
      int sides = std::ceil(2 * M_PI/(voxel_size_/cone->radius));

      // Cone Data
      double dtheta = 2 * M_PI/sides;
      double dh = cone->length/2.0;
      std::vector<openvdb::math::Vec3s> points(sides+2);
      std::vector<openvdb::Vec4I> quads(2*sides);

      // Create Vertices
      double theta = 0;
      for(int i = 0; i < sides; ++i)
      {
        double x = cone->radius * std::cos(theta);
        double y = cone->radius * std::sin(theta);
        points[i] = openvdb::math::Vec3s(x, y, -dh);
        theta += dtheta;
      }
      int top_pdx = sides;
      int bot_pdx = sides + 1;
      points[top_pdx] = openvdb::math::Vec3s(0.0, 0.0, dh);
      points[bot_pdx] = openvdb::math::Vec3s(0.0, 0.0, -dh);

      // Create polygons for wall and bottom
      for(int i = 0; i < sides; ++i)
      {
        int d = i;
        int d1 = i + 1;
        bool last = (i == (sides-1));

        if (last)
          d1 = 0;

        // Create wall
        quads[i] = openvdb::Vec4I(top_pdx, d, d1, openvdb::util::INVALID_IDX);

        // Create bottom cap
        quads[sides+i] = openvdb::Vec4I(bot_pdx, d1, d, openvdb::util::INVALID_IDX);
      }

      // Transform point location
      TransformVec3s(pose, points.data(), points.size());

      // Convert data from world to index
      WorldToIndex(transform_, points.data(), points.size());

      openvdb::tools::QuadAndTriangleDataAdapter<openvdb::Vec3s, openvdb::Vec4I> mesh (points.data(), points.size(),
                                                                                       quads.data(), quads.size());
      grid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(mesh, *transform_, exBandWidth, inBandWidth);

      break;
    }

    case shapes::CYLINDER:
    {
      const shapes::Cylinder *cylinder = static_cast<const shapes::Cylinder *>(shape);

      // Number of sides
      int sides = std::ceil(2 * M_PI/(voxel_size_/cylinder->radius));

      // Cylinder Precision
      double dtheta = 2 * M_PI/sides;

      double dh = cylinder->length/2.0;
      std::vector<openvdb::math::Vec3s> points(2*(sides+1));
      std::vector<openvdb::Vec4I> quads(3*sides);

      // Create Vertices
      double theta = 0;
      int sign = 1;
      for(int i = 0; i < sides; ++i)
      {
        double x = cylinder->radius * std::cos(theta);
        double y = cylinder->radius * std::sin(theta);
        points[2 * i] = openvdb::math::Vec3s(x, y, sign * dh);
        points[(2 * i)+1] = openvdb::math::Vec3s(x, y, -sign * dh);
        theta += dtheta;
        sign = -1 * sign;
      }
      int top_pdx = (2 * sides);
      int bot_pdx = (2 * sides) + 1;
      points[top_pdx] = openvdb::math::Vec3s(0.0, 0.0, dh);
      points[bot_pdx] = openvdb::math::Vec3s(0.0, 0.0, -dh);

      // Create polygons for wall and caps
      for(int i = 0; i < sides; ++i)
      {
        int d = i * 2;
        int d1 = d + 1;
        int d2 = d + 2;
        int d3 = d + 3;

        bool last = (i == (sides-1));
        if( i % 2 == 0)
        {
          if (last)
          {
            d2 = 1;
            d3 = 0;
          }

          // Create wall
          quads[i] = openvdb::Vec4I(d, d1, d2, d3);

          // Create top cap
          quads[sides+i] = openvdb::Vec4I(top_pdx, d, d3, openvdb::util::INVALID_IDX);

          // Create bottom cap
          quads[(2*sides)+i] = openvdb::Vec4I(bot_pdx, d1, d2, openvdb::util::INVALID_IDX);
        }
        else
        {
          if (last)
          {
            d2 = 0;
            d3 = 1;
          }

          // Create wall
          quads[i] = openvdb::Vec4I(d3, d2, d1, d);

          // Create top cap
          quads[sides+i] = openvdb::Vec4I(top_pdx, d1, d2, openvdb::util::INVALID_IDX);

          // Create bottom cap
          quads[(2*sides)+i] = openvdb::Vec4I(bot_pdx, d, d3, openvdb::util::INVALID_IDX);
        }
      }

      // Tranform point location
      TransformVec3s(pose, points.data(), points.size());

      // Convert data from world to index
      WorldToIndex(transform_, points.data(), points.size());

      openvdb::tools::QuadAndTriangleDataAdapter<openvdb::Vec3s, openvdb::Vec4I> mesh (points.data(), points.size(),
                                                                                       quads.data(), quads.size());
      grid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(mesh, *transform_, exBandWidth, inBandWidth);

      break;
    }

    case shapes::OCTREE:
    {
      ROS_ERROR("OpenVDB Distance Field: Shape Type OCTREE is not implemented.");
      return;
    }

    case shapes::MESH:
    {
      shapes::Mesh *mesh = static_cast<shapes::Mesh *>(shape->clone());

      // Now need to clean verticies
      mesh->mergeVertices(0.0001);

      // Get points and triangles from Shape
      MeshData mesh_data = ShapeMeshToOpenVDB(mesh, pose);

      // Convert data from world to index
      WorldToIndex(transform_, mesh_data.points.data(), mesh_data.points.size());

      openvdb::tools::QuadAndTriangleDataAdapter<openvdb::Vec3s, openvdb::Vec3I> mesh2 (mesh_data.points.data(), mesh_data.points.size(),
                                                                                       mesh_data.triangles.data(), mesh_data.triangles.size());
      grid = openvdb::tools::meshToVolume<openvdb::FloatGrid>(mesh2, *transform_, exBandWidth, inBandWidth);

      delete mesh;
      break;
    }

    case shapes::PLANE:
    {
      ROS_ERROR("OpenVDB Distance Field: Shape Type PLANE is not implemented.");
      return;
    }

    case shapes::SPHERE:
    {
      const shapes::Sphere *sphere = static_cast<const shapes::Sphere *>(shape);

      grid = openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(sphere->radius,
                                                                      openvdb::Vec3f(pose.translation()(0),
                                                                                     pose.translation()(1),
                                                                                     pose.translation()(2)),
                                                                      voxel_size_, exBandWidth);
      break;
    }

    case shapes::UNKNOWN_SHAPE:
    {
      ROS_ERROR("OpenVDB Distance Field: Unknown Shape Type");
      return;
    }

  }

  if (!grid_)
  {
    grid_ = grid;
  }
  else
  {
    ros::Time start = ros::Time::now();
    openvdb::tools::csgUnion(*grid_, *grid, true);
    ROS_INFO("CSG Union Time Elapsed: %f (sec)",(ros::Time::now() - start).toSec());
  }

  accessor_ = std::make_shared<openvdb::FloatGrid::ConstAccessor>(grid_->getConstAccessor());
}

void distance_field::OpenVDBDistanceField::writeToFile(const std::string file_path)
{
  // Create a VDB file object.
  openvdb::io::File vdbFile(file_path);

  // Add the grid pointer to a container.
  openvdb::GridPtrVec grids;
  grids.push_back(grid_);

  // Write out the contents of the container.
  vdbFile.write(grids);
  vdbFile.close();
}

uint64_t distance_field::OpenVDBDistanceField::memUsage() const
{
  return grid_->memUsage();
}

distance_field::MeshData distance_field::ShapeMeshToOpenVDB(const shapes::Mesh *mesh, const Eigen::Affine3d &pose)
{
  MeshData output;
  openvdb::math::Mat4d tf;

  output.points.resize(mesh->vertex_count);
  output.triangles.resize(mesh->triangle_count);
  output.quads.resize(mesh->triangle_count);

  // Convert Eigen to OpenVDB
  Affine3dToMat4d(pose, tf);

  // Populate Verticies/Triangles/Quads
  for (unsigned int i = 0 ; i < mesh->triangle_count ; ++i)
  {
    unsigned int i3 = i * 3;
    unsigned int v1 = mesh->triangles[i3];
    unsigned int v2 = mesh->triangles[i3 + 1];
    unsigned int v3 = mesh->triangles[i3 + 2];
    openvdb::math::Vec3s p1(mesh->vertices[v1 * 3], mesh->vertices[v1 * 3 + 1], mesh->vertices[v1 * 3 + 2]);
    openvdb::math::Vec3s p2(mesh->vertices[v2 * 3], mesh->vertices[v2 * 3 + 1], mesh->vertices[v2 * 3 + 2]);
    openvdb::math::Vec3s p3(mesh->vertices[v3 * 3], mesh->vertices[v3 * 3 + 1], mesh->vertices[v3 * 3 + 2]);

    output.points[v1] = tf*p1;
    output.points[v2] = tf*p2;
    output.points[v3] = tf*p3;

    openvdb::Vec3I triangle(v1, v2, v3);
    output.triangles[i] = triangle;

    openvdb::Vec4I quad(v1, v2, v3, openvdb::util::INVALID_IDX);
    output.quads[i] = quad;
  }

  ROS_ASSERT(mesh->vertex_count == output.points.size());
  ROS_ASSERT(mesh->triangle_count == output.triangles.size());
  ROS_ASSERT(mesh->triangle_count == output.quads.size());

  return output;
}

void distance_field::Affine3dToMat4d(const Eigen::Affine3d &input, openvdb::math::Mat4d &output)
{
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      output(i, j) = input(i, j);

}

void distance_field::Affine3dToMat4dAffine(const Eigen::Affine3d &input, openvdb::math::Mat4d &output)
{
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      output(j, i) = input(i, j);
}

void distance_field::WorldToIndex(const openvdb::math::Transform::Ptr transform, std::vector<openvdb::math::Vec3s> &points)
{
  WorldToIndex(transform, points.data(), points.size());
}

void distance_field::TransformVec3s(const Eigen::Affine3d &pose, std::vector<openvdb::math::Vec3s> &points)
{
  TransformVec3s(pose, points.data(), points.size());
}

void distance_field::WorldToIndex(const openvdb::math::Transform::Ptr transform, openvdb::math::Vec3s *points, std::size_t size)
{
  std::transform(points, points + size, points,
                 [&transform](openvdb::math::Vec3s point){ return transform->worldToIndex(point); });
}

void distance_field::TransformVec3s(const Eigen::Affine3d &pose, openvdb::math::Vec3s *points, std::size_t size)
{
  openvdb::math::Mat4d tf;
  Affine3dToMat4d(pose, tf);
  std::transform(points, points + size, points,
                 [&tf](openvdb::math::Vec3s point){ return tf * point; });
}
