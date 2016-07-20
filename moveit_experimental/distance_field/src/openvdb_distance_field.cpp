#include "openvdb_distance_field.h"
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/Composite.h>
#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/VolumeToSpheres.h>
#include <ros/assert.h>
#include <algorithm>
#include <math.h>

distance_field::CollisionRobotOpenVDB::CollisionRobotOpenVDB(const moveit::core::RobotModelConstPtr &model,
                                                             const float voxel_size, const float background,
                                                             const float exBandWidth, const float inBandWidth)
  : robot_model_(model), voxel_size_(voxel_size), background_(background), exBandWidth_(exBandWidth), inBandWidth_(inBandWidth), links_(model->getLinkModelsWithCollisionGeometry())
{

  createStaticSDFs();

  createActiveSDFs();

  createDynamicSDFs();
}

void distance_field::CollisionRobotOpenVDB::createStaticSDFs()
{
  const robot_model::LinkModel *root_link = robot_model_->getRootLink();

  // Check to make sure link has collision geometry to add. I don't think this is required,
  // because it will be world link and I don't think it will ever have geometry.
  if (std::find(links_.begin(), links_.end(), robot_model_->getRootLink()) != links_.end())
  {
    OpenVDBDistanceFieldPtr sdf(new OpenVDBDistanceField(voxel_size_, background_));
    sdf->addLinkToField(root_link, Eigen::Affine3d::Identity(), exBandWidth_, inBandWidth_);
    static_links_.push_back(root_link);
    static_sdf_.push_back(OpenVDBDistanceFieldConstPtr(sdf));
  }


  addAssociatedFixedTransforms(root_link);
}

void distance_field::CollisionRobotOpenVDB::createActiveSDFs()
{
  const std::vector<const robot_model::JointModelGroup*> groups = robot_model_->getJointModelGroups();
  for (std::size_t i = 0 ; i < groups.size() ; ++i)
  {
    const std::vector<const robot_model::LinkModel*> links = groups[i]->getLinkModels();
    for (std::size_t j = 0 ; j < links.size() ; ++j)
    {
      auto it = std::find(active_links_.begin(), active_links_.end(), links[j]);
      auto it2 = std::find(links_.begin(), links_.end(), links[j]); // Check to make sure it has collision geometry
      if (it == active_links_.end() && it2 != links_.end())
      {
        active_links_.push_back(links[j]);
      }
    }
  }

  active_sdf_.resize(active_links_.size());
  active_spheres_.resize(active_links_.size());
  for (std::size_t i = 0 ; i < active_links_.size() ; ++i)
  {
    OpenVDBDistanceFieldPtr sdf;
    float v = voxel_size_;

    // This dynamicly changes the voxel size to try and ensure that a sphere model is found.
    for (std::size_t j = 0 ; j < 10 ; ++j)
    {
      sdf.reset(new OpenVDBDistanceField(v, background_));

      sdf->addLinkToField(active_links_[i], Eigen::Affine3d::Identity(), (voxel_size_/v) * exBandWidth_, (voxel_size_/v) * inBandWidth_);

      sdf->fillWithSpheres(active_spheres_[i], 10);

      if (active_spheres_[i].size() != 0)
        break;

      v = v * 0.5;
    }

    if (active_spheres_[i].size() == 0)
    {
      ROS_ERROR("Unable to generate spheres for link: %s", active_links_[i]->getName().c_str());
    }

    active_sdf_[i] = OpenVDBDistanceFieldConstPtr(sdf);
  }
}

void distance_field::CollisionRobotOpenVDB::createDynamicSDFs()
{
  dynamic_links_ = links_;

  // remove static links from list
  for (std::size_t i = 0 ; i < static_links_.size() ; ++i)
  {
    dynamic_links_.erase(std::remove(dynamic_links_.begin(), dynamic_links_.end(), static_links_[i]));
  }

  // remove active links from list
  for (std::size_t i = 0 ; i < active_links_.size() ; ++i)
  {
    dynamic_links_.erase(std::remove(dynamic_links_.begin(), dynamic_links_.end(), active_links_[i]));
  }

  dynamic_sdf_.resize(dynamic_links_.size());
  for (std::size_t i = 0 ; i < dynamic_links_.size() ; ++i)
  {
    OpenVDBDistanceFieldPtr sdf(new OpenVDBDistanceField(voxel_size_, background_));

    sdf->addLinkToField(dynamic_links_[i], Eigen::Affine3d::Identity(), exBandWidth_, inBandWidth_);

    dynamic_sdf_[i] = OpenVDBDistanceFieldConstPtr(sdf);
  }
}

void distance_field::CollisionRobotOpenVDB::addAssociatedFixedTransforms(const robot_model::LinkModel *link)
{
  const moveit::core::LinkTransformMap fixed_attached = link->getAssociatedFixedTransforms();

  for (auto it = fixed_attached.begin(); it!=fixed_attached.end(); ++it)
  {
    // only add child links
    if (link->getParentLinkModel() != it->first)
    {
      // Check to make sure link has collision geometry to add
      if (std::find(links_.begin(), links_.end(), it->first) != links_.end())
      {
        OpenVDBDistanceFieldPtr sdf(new OpenVDBDistanceField(voxel_size_, background_));
        sdf->addLinkToField(it->first, it->second, exBandWidth_, inBandWidth_);
        static_links_.push_back(it->first);
        static_sdf_.push_back(OpenVDBDistanceFieldConstPtr(sdf));
      }

      addAssociatedFixedTransforms(it->first);
    }
  }
}

void distance_field::CollisionRobotOpenVDB::writeToFile(const std::string file_path)
{
  // Create a VDB file object.
  openvdb::io::File vdbFile(file_path);

  // Add the static grids to grid array
  openvdb::GridPtrVec grids;
  for (std::size_t i = 0 ; i < static_sdf_.size() ; ++i)
  {
    grids.push_back(static_sdf_[i]->getGrid());
  }

  // Add the dynamic grids to grid array
  for (std::size_t i = 0 ; i < dynamic_sdf_.size() ; ++i)
  {
    grids.push_back(dynamic_sdf_[i]->getGrid());
  }

  // Write out the contents of the container.
  vdbFile.write(grids);
  vdbFile.close();
}

uint64_t distance_field::CollisionRobotOpenVDB::memUsage() const
{
  uint64_t mem_usage = 0;

  for (std::size_t i = 0 ; i < static_sdf_.size() ; ++i)
  {
    mem_usage += static_sdf_[i]->memUsage();
  }

  for (std::size_t i = 0 ; i < dynamic_sdf_.size() ; ++i)
  {
    mem_usage += dynamic_sdf_[i]->memUsage();
  }

  for (std::size_t i = 0 ; i < active_sdf_.size() ; ++i)
  {
    mem_usage += active_sdf_[i]->memUsage();
  }

  return mem_usage;
}

void distance_field::CollisionRobotOpenVDB::distanceSelf(const collision_detection::DistanceRequest &req, collision_detection::DistanceResult &res, const moveit::core::RobotState &state) const
{
  std::vector<Eigen::Affine3d> dynamic_poses;
  std::vector<Eigen::Affine3d> active_poses;
  std::vector<std::pair<std::vector<std::pair<Eigen::Vector3d, double> >, OpenVDBDistanceFieldConstPtr> > dist_query;
  std::vector<Eigen::Affine3d> dist_query_poses;
  std::vector<std::pair<std::string, std::string> > dist_query_names;

  int dist_size = active_links_.size() * (dynamic_links_.size() + static_links_.size() + ((active_links_.size() - 1) / 2));
  dynamic_poses.reserve(dynamic_links_.size());
  active_poses.reserve(active_links_.size());
  dist_query.reserve(dist_size);
  dist_query_poses.reserve(dist_size);
  dist_query_names.reserve(dist_size);

  // get all dynamic link pose information
  for (std::size_t i = 0 ; i < dynamic_links_.size() ; ++i)
  {
    dynamic_poses[i] = state.getGlobalLinkTransform(dynamic_links_[i]);
  }

  // get all active link pose information
  for (std::size_t i = 0 ; i < active_links_.size() ; ++i)
  {
    active_poses[i] = state.getGlobalLinkTransform(active_links_[i]);
  }

  // build distance query vector
  int cnt = 0;
  for (std::size_t i = 0 ; i < active_links_.size() ; ++i)
  {
    for (std::size_t j = 0 ; j < static_links_.size() ; ++j)
    {
      if (isCollisionAllowed(active_links_[i]->getName(), static_links_[j]->getName(), req.acm))
      {
        if (active_spheres_[i].size() != 0)
        {
          dist_query.push_back(std::make_pair(active_spheres_[i], static_sdf_[j]));
          dist_query_poses.push_back(active_poses[i]);
          dist_query_names.push_back(std::make_pair(active_links_[i]->getName(), static_links_[j]->getName()));
          cnt += 1;
        }
        else
        {
          ROS_ERROR("Link %s has zeros spheres, unable to perform check between links %s and %s.", active_links_[i]->getName().c_str(), active_links_[i]->getName().c_str(), static_links_[j]->getName().c_str());
        }
      }
    }

    for (std::size_t j = 0 ; j < dynamic_links_.size() ; ++j)
    {
      if (isCollisionAllowed(active_links_[i]->getName(), dynamic_links_[j]->getName(), req.acm))
      {
        if (active_spheres_[i].size() != 0)
        {
          dist_query.push_back(std::make_pair(active_spheres_[i], dynamic_sdf_[j]));
          dist_query_poses.push_back(dynamic_poses[j].inverse() * active_poses[i]);
          dist_query_names.push_back(std::make_pair(active_links_[i]->getName(), dynamic_links_[j]->getName()));
          cnt += 1;
        }
        else
        {
          ROS_ERROR("Link %s has zeros spheres, unable to perform check between links %s and %s.", active_links_[i]->getName().c_str(), active_links_[i]->getName().c_str(), dynamic_links_[j]->getName().c_str());
        }
      }
    }

    if (i != active_links_.size())
    {
      for (std::size_t j = i + 1 ; j < active_links_.size(); ++j)
      {
        if (isCollisionAllowed(active_links_[i]->getName(), active_links_[j]->getName(), req.acm))
        {
          if (active_spheres_[i].size() != 0)
          {
            dist_query.push_back(std::make_pair(active_spheres_[i], active_sdf_[j]));
            dist_query_poses.push_back(active_poses[j].inverse() * active_poses[i]);
            dist_query_names.push_back(std::make_pair(active_links_[i]->getName(), active_links_[j]->getName()));
            cnt += 1;
          }
          else
          {
            ROS_ERROR("Link %s has zeros spheres, unable to perform check between links %s and %s.", active_links_[i]->getName().c_str(), active_links_[i]->getName().c_str(), active_links_[j]->getName().c_str());
          }
        }
      }
    }
  }

  Eigen::VectorXd dist(cnt);
  std::vector<int> sphere_index;
  sphere_index.reserve(cnt);

  // Compute minimum distance
//  #pragma omp parallel for
  for (std::size_t i = 0 ; i < cnt ; ++i)
  {
    // transform sphere origins into correct coordinate system
    std::transform(dist_query[i].first.begin(), dist_query[i].first.end(), dist_query[i].first.begin(),
                   [&dist_query_poses, &i](std::pair<Eigen::Vector3d, double> p){ p.first = (dist_query_poses[i] * p.first); return p; });

    dist[i] = background_;
    distanceSelfHelper(dist_query[i].first, dist_query[i].second, dist[i], sphere_index[i]);
  }

  int index;
  res.minimum_distance.min_distance = dist.minCoeff(&index);
  res.minimum_distance.hasNearestPoints = false;
  res.minimum_distance.hasGradient = true;
  res.minimum_distance.link_name[0] = dist_query_names[index].first;
  res.minimum_distance.link_name[1] = dist_query_names[index].second;

  // Compute gradient
  // get all active link pose information
  int active_index;
  for (std::size_t i = 0 ; i < active_links_.size() ; ++i)
  {
    if( active_links_[i]->getName() == dist_query_names[index].first)
    {
      active_index = i;
      break;
    }
  }
  Eigen::Vector3d grad;
  grad.setZero();
  cnt = 0;
  for (std::size_t i = 0 ; i < active_links_.size() ; ++i)
  {
    if (i != active_index)
    {
      for (std::size_t j = 0 ; j < static_links_.size() ; ++j)
      {
        if (isCollisionAllowed(active_links_[i]->getName(), static_links_[j]->getName(), req.acm))
        {
          if (active_spheres_[i].size() != 0)
          {
            Eigen::Vector3d center = active_poses[i] * active_spheres_[i][sphere_index[index]].first;
            grad += static_sdf_[j]->getGradient(center(0), center(1), center(2), false);
            cnt += 1;
          }
          else
          {
            ROS_ERROR("Link %s has zeros spheres, unable to perform check between links %s and %s.", active_links_[i]->getName().c_str(), active_links_[i]->getName().c_str(), static_links_[j]->getName().c_str());
          }
        }
      }

      for (std::size_t j = 0 ; j < dynamic_links_.size() ; ++j)
      {
        if (isCollisionAllowed(active_links_[i]->getName(), dynamic_links_[j]->getName(), req.acm))
        {
          if (active_spheres_[i].size() != 0)
          {
            Eigen::Affine3d inv = dynamic_poses[j].inverse();
            Eigen::Vector3d center = (inv * active_poses[i]) * active_spheres_[i][sphere_index[index]].first;
            grad += (inv * dynamic_sdf_[j]->getGradient(center(0), center(1), center(2), false));
            cnt += 1;
          }
          else
          {
            ROS_ERROR("Link %s has zeros spheres, unable to perform check between links %s and %s.", active_links_[i]->getName().c_str(), active_links_[i]->getName().c_str(), dynamic_links_[j]->getName().c_str());
          }
        }
      }

      if (i != active_links_.size())
      {
        for (std::size_t j = i + 1 ; j < active_links_.size(); ++j)
        {
          if (isCollisionAllowed(active_links_[i]->getName(), active_links_[j]->getName(), req.acm))
          {
            if (active_spheres_[i].size() != 0)
            {
              Eigen::Affine3d inv = active_poses[j].inverse();
              Eigen::Vector3d center = (inv * active_poses[i]) * active_spheres_[i][sphere_index[index]].first;
              grad += (inv * active_sdf_[j]->getGradient(center(0), center(1), center(2), false));
              cnt += 1;
            }
            else
            {
              ROS_ERROR("Link %s has zeros spheres, unable to perform check between links %s and %s.", active_links_[i]->getName().c_str(), active_links_[i]->getName().c_str(), active_links_[j]->getName().c_str());
            }
          }
        }
      }
    }
  }

  grad = grad/cnt;
  ROS_ERROR("Gradient: %f, %f, %f", grad(0), grad(1), grad(2));
}


bool distance_field::CollisionRobotOpenVDB::isCollisionAllowed(const std::string &l1, const std::string &l2, const collision_detection::AllowedCollisionMatrix *acm) const
{
    // use the collision matrix (if any) to avoid certain distance checks
    if (acm)
    {
      collision_detection::AllowedCollision::Type type;

      bool found = acm->getAllowedCollision(l1, l2, type);
      if (found)
      {
        // if we have an entry in the collision matrix, we read it
        if (type == collision_detection::AllowedCollision::ALWAYS)
        {
          return false;
        }
      }
    }
    return true;
}

void distance_field::CollisionRobotOpenVDB::distanceSelfHelper(const std::vector<std::pair<Eigen::Vector3d, double> > &spheres, OpenVDBDistanceFieldConstPtr sdf, double &min_dist, int &index) const
{
  Eigen::VectorXf dist(spheres.size());
  for (int i = 0; i < spheres.size(); ++i)
  {
    dist[i] = sdf->getDistance(spheres[i].first(0), spheres[i].first(1), spheres[i].first(2), true) - spheres[i].second;
  }

  min_dist = dist.minCoeff(&index);
}

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

Eigen::Vector3d distance_field::OpenVDBDistanceField::getGradient(const Eigen::Vector3f &point, bool thread_safe) const
{
  return getGradient(point(0), point(1), point(2), thread_safe);
}

Eigen::Vector3d distance_field::OpenVDBDistanceField::getGradient(const openvdb::math::Coord &coord, bool thread_safe) const
{
  openvdb::Vec3f result;
  Eigen::Vector3d gradient;
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
      return gradient;
    }

    gradient(0) = result(0);
    gradient(1) = result(1);
    gradient(2) = result(2);
  }

  gradient.normalize();
  return gradient;
}

Eigen::Vector3d distance_field::OpenVDBDistanceField::getGradient(const float &x, const float &y, const float &z, bool thread_safe) const
{
  openvdb::math::Vec3s xyz(x, y, z);
  return getGradient(transform_->worldToIndexNodeCentered(xyz), thread_safe);
}

void distance_field::OpenVDBDistanceField::fillWithSpheres(std::vector<std::pair<Eigen::Vector3d, double> > &spheres, int maxSphereCount, bool overlapping, float minRadius, float maxRadius, float isovalue, int instanceCount)
{
  std::vector<openvdb::math::Vec4s> s;
  openvdb::tools::fillWithSpheres<openvdb::FloatGrid>(*grid_, s, maxSphereCount, overlapping, minRadius, maxRadius, isovalue, instanceCount);

  // convert data to eigen data types
  for (auto it = s.begin(); it != s.end(); ++it)
  {
    spheres.push_back(std::make_pair(Eigen::Vector3d((*it)[0], (*it)[1], (*it)[2]), (*it)[3]));
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
    ros::Time start = ros::Time::now();
    openvdb::tools::csgUnion(*grid_, *grid, true);
    ROS_ERROR("CSG Union Time Elapsed: %f (sec)",(ros::Time::now() - start).toSec());
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
