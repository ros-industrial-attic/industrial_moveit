#include "openvdb_distance_field.h"
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/Composite.h>
#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/VolumeToSpheres.h>
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

openvdb::FloatGrid::Ptr distance_field::OpenVDBDistanceField::getGrid() const
{
  return grid_;
}

double distance_field::OpenVDBDistanceField::getDistance(Eigen::Vector3f &point)
{
  openvdb::math::Vec3s xyz(point[0], point[1], point[2]);
  xyz = transform_->worldToIndex(xyz);
  openvdb::Coord c = openvdb::util::nearestCoord(xyz);
  return grid_->getAccessor().getValue(c);
}

void distance_field::OpenVDBDistanceField::addRobotToField(const moveit::core::RobotStatePtr robot_state, const float exBandWidth, const float inBandWidth)
{
  std::vector<const moveit::core::LinkModel*> links = robot_state->getRobotModel()->getLinkModelsWithCollisionGeometry();
  for (int i = 0; i < links.size(); ++i)
  {
    std::vector<shapes::ShapeConstPtr> shapes = links[i]->getShapes();
    EigenSTL::vector_Affine3d shape_poses = links[i]->getCollisionOriginTransforms();

    Eigen::Affine3d link_pose = robot_state->getFrameTransform(links[i]->getName());

    for (int j = 0; j < shapes.size(); ++j)
    {
      addShapeToField(shapes[j].get(), link_pose * shape_poses[j], exBandWidth, inBandWidth);
    }
  }
}

void distance_field::OpenVDBDistanceField::addShapeToField(const shapes::Shape *shape, const Eigen::Affine3d &pose, const float exBandWidth, const float inBandWidth)
{
  openvdb::FloatGrid::Ptr grid;

  switch(shape->type)
  {
    case shapes::BOX:
    {
      return;
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
      TransformVec3s(pose, points);

      // Convert data from world to index
      WorldToIndex(transform_, points);

      openvdb::tools::MeshToVolume<openvdb::FloatGrid> voxelizer(transform_);

      voxelizer.convertToLevelSet(points, quads, exBandWidth, inBandWidth);

      grid = voxelizer.distGridPtr()->deepCopy();

      break;
    }

    case shapes::CONE:
    {
      const shapes::Cone *cone = static_cast<const shapes::Cone *>(shape);

      // Number of sides
      int sides = 30;

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
      TransformVec3s(pose, points);

      // Convert data from world to index
      WorldToIndex(transform_, points);

      openvdb::tools::MeshToVolume<openvdb::FloatGrid> voxelizer(transform_);

      voxelizer.convertToLevelSet(points, quads, exBandWidth, inBandWidth);

      grid = voxelizer.distGridPtr()->deepCopy();

      break;
    }

    case shapes::CYLINDER:
    {
      const shapes::Cylinder *cylinder = static_cast<const shapes::Cylinder *>(shape);

      // Number of sides
      int sides = 30;

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
      TransformVec3s(pose, points);

      // Convert data from world to index
      WorldToIndex(transform_, points);

      openvdb::tools::MeshToVolume<openvdb::FloatGrid> voxelizer(transform_);

      voxelizer.convertToLevelSet(points, quads, exBandWidth, inBandWidth);

      grid = voxelizer.distGridPtr()->deepCopy();

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
      WorldToIndex(transform_, mesh_data.points);

      openvdb::tools::MeshToVolume<openvdb::FloatGrid> voxelizer(transform_);

      voxelizer.convertToLevelSet(mesh_data.points, mesh_data.quads, exBandWidth, inBandWidth);

      grid = voxelizer.distGridPtr()->deepCopy();

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
      grid = openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(sphere->radius,  openvdb::Vec3f(pose.translation()(0), pose.translation()(1), pose.translation()(2)), voxel_size_, exBandWidth);
      break;
    }

    case shapes::UNKNOWN_SHAPE:
    {
      ROS_ERROR("OpenVDB Distance Field: Unknown Shape Type");
      return;
    }

  }

  // Set the default background distance
  grid->setBackground(background_);
  if (!grid_)
  {
    grid_ = grid;
  }
  else
  {
    openvdb::tools::csgUnion(*grid_, *grid, true);
  }
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

void distance_field::WorldToIndex(const openvdb::math::Transform::Ptr transform, std::vector<openvdb::math::Vec3s> &points)
{
  std::transform(points.begin(), points.end(), points.begin(),
                 [&transform](openvdb::math::Vec3s point){ return transform->worldToIndex(point); });
}

void distance_field::TransformVec3s(const Eigen::Affine3d &pose, std::vector<openvdb::v2_1::math::Vec3s> &points)
{
  openvdb::math::Mat4d tf;
  Affine3dToMat4d(pose, tf);
  std::transform(points.begin(), points.end(), points.begin(),
                 [&tf](openvdb::math::Vec3s point){ return tf * point; });
}
