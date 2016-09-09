#include <industrial_collision_detection/collision_detection/collision_robot_openvdb.h>
#include <openvdb/tools/GridOperators.h>
#include <ros/assert.h>
#include <algorithm>
#include <mutex>
#include <math.h>

const static std::string voxel_size_meta_name = "voxel_size";
const static std::string background_meta_name = "background";
const static std::string ex_bandwidth_meta_name = "exBandWidth";
const static std::string in_bandwidth_meta_name = "inBandWidth";

typedef std::map<std::string,std::shared_ptr<collision_detection::DistanceFieldCache> > DistanceFieldCacheMap;

namespace collision_detection
{

using namespace distance_field;

class DFManager
{
public:

  DistanceFieldCacheMap& getDistanceFieldCacheMap()
  {
    static DistanceFieldCacheMap cache;
    return cache;
  }

  void registerDistanceFieldCache(const moveit::core::RobotModelConstPtr &model,
                                                               const float voxel_size, const float background,
                                                               const float exBandWidth, const float inBandWidth)
  {
    std::lock_guard<std::mutex> guard(mtx_);
    DistanceFieldCacheMap& cache = getDistanceFieldCacheMap();
    if(cache.count(model->getName()) == 0)
    {
      std::shared_ptr<collision_detection::DistanceFieldCache> dfc(new DistanceFieldCache(model,voxel_size,background,exBandWidth,inBandWidth));
      cache[model->getName()] = dfc;
    }
    else
    {
      ROS_DEBUG("Distance Fields for robot '%s' already exists, skipping registration",model->getName().c_str());
    }
  }

  void registerDistanceFieldCache(const moveit::core::RobotModelConstPtr &model,const std::string &file_path)
  {
    std::lock_guard<std::mutex> guard(mtx_);
    DistanceFieldCacheMap& cache = getDistanceFieldCacheMap();
    if(cache.count(model->getName()) == 0)
    {
      std::shared_ptr<collision_detection::DistanceFieldCache> dfc(new DistanceFieldCache(model,file_path));
      cache[model->getName()] = dfc;
    }
    else
    {
      ROS_DEBUG("Distance Field Cache for robot '%s' already exists, skipping registration",model->getName().c_str());
    }
  }

  std::shared_ptr<collision_detection::DistanceFieldCache> getDistanceFieldCacheEntry(const moveit::core::RobotModelConstPtr &model)
  {
    std::lock_guard<std::mutex> guard(mtx_);
    DistanceFieldCacheMap& cache = getDistanceFieldCacheMap();
    std::shared_ptr<collision_detection::DistanceFieldCache> dfc;
    if(cache.count(model->getName()) > 0)
    {
      dfc  = cache[model->getName()];
    }
    else
    {
      auto msg = "Distance Field Cache for robot '" + model->getName() + "' was not found";
      ROS_ERROR_STREAM(msg);
      throw std::runtime_error(msg);
    }

    return dfc;
  }

public:
  std::mutex mtx_;

};

static DFManager DistanceFieldManager;


DistanceFieldCache::DistanceFieldCache(const moveit::core::RobotModelConstPtr &model,
                                                             const float voxel_size, const float background,
                                                             const float exBandWidth, const float inBandWidth):
  robot_model_(model), voxel_size_(voxel_size), background_(background),
  exBandWidth_(exBandWidth), inBandWidth_(inBandWidth),
  links_(model->getLinkModelsWithCollisionGeometry())
{
  ROS_DEBUG("Creating Distance Fields for robot %s",model->getName().c_str());
  createDefaultAllowedCollisionMatrix();
  createStaticSDFs();
  createActiveSDFs();
  createDynamicSDFs();
  createDefaultDistanceQuery();
}

DistanceFieldCache::DistanceFieldCache(const moveit::core::RobotModelConstPtr &model,
                                                             const std::string &file_path):
  robot_model_(model), links_(model->getLinkModelsWithCollisionGeometry())
{
  ROS_DEBUG("Creating Distance Fields for robot %s",model->getName().c_str());

  openvdb::initialize();
  // Step 1: Load the OpenVDB archive
  auto grid_data = readFromFile(file_path);
  const auto& grids = *grid_data.first;
  const auto& metadata = *grid_data.second;

  // Step 2: Process meta-data at the file level - load nominal voxel size, background, etc...
  voxel_size_ = metadata.metaValue<float>(voxel_size_meta_name);
  background_ = metadata.metaValue<float>(background_meta_name);
  exBandWidth_ = metadata.metaValue<float>(ex_bandwidth_meta_name);
  inBandWidth_ = metadata.metaValue<float>(in_bandwidth_meta_name);

  // Step 3: For each static, active, & dynamic link, let's construct it's distance field
  //         using archived data.
  static_links_ = identifyStaticLinks();
  active_links_ = identifyActiveLinks();
  dynamic_links_ = identifyDynamicLinks(static_links_, active_links_);

  loadStaticLinks(static_links_, grids, static_sdf_);
  loadActiveLinks(active_links_, grids, active_sdf_);
  loadDynamicLinks(dynamic_links_, grids, dynamic_sdf_);

  // Step 4: Create default queries & parse the allowed collision matrix
  createDefaultAllowedCollisionMatrix();
  createDefaultDistanceQuery();
}

void DistanceFieldCache::createStaticSDFs()
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

  const auto& fixed_links = root_link->getAssociatedFixedTransforms();
  for (const auto& pair : fixed_links)
  {
    if (hasCollisionGeometry(pair.first))
    {
      OpenVDBDistanceFieldPtr sdf(new OpenVDBDistanceField(voxel_size_, background_));
      sdf->addLinkToField(pair.first, pair.second, exBandWidth_, inBandWidth_);
      static_links_.push_back(pair.first);
      static_sdf_.push_back(OpenVDBDistanceFieldConstPtr(sdf));
    }
  }
}

void DistanceFieldCache::createActiveSDFs()
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

      const auto n_spheres = 20;
      const auto can_overlap = true;
      const auto min_voxel_size = 1.0f;
      const auto max_voxel_size = std::numeric_limits<float>::max();
      const auto iso_surface = 0.0f; // the value at which the surface exists; 0.0 for solid models (clouds are different)
      const auto n_instances = 100000; // num of voxels to consider when fitting spheres

      sdf->fillWithSpheres(active_spheres_[i], n_spheres, can_overlap, min_voxel_size,
                           max_voxel_size, iso_surface, n_instances);

      if (active_spheres_[i].size() > 1) // openvdb appears to ALWAYS insert one sphere, so we want more
        break;

      v = v * 0.5; // try again with voxels of half the size
    }

    if (active_spheres_[i].size() == 0)
    {
      ROS_ERROR("Unable to generate spheres for link: %s", active_links_[i]->getName().c_str());
    }

    active_sdf_[i] = OpenVDBDistanceFieldConstPtr(sdf);
  }
}

void DistanceFieldCache::createDynamicSDFs()
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

void DistanceFieldCache::writeToFile(const std::string& file_path)
{
  // Create a VDB file object.
  openvdb::io::File vdbFile(file_path);

  // Add the static grids to grid array
  openvdb::GridPtrVec grids;

  for (std::size_t i = 0 ; i < static_sdf_.size() ; ++i)
  {
    // TODO: This saveMetaData should really be done when the SDF is created
    static_sdf_[i]->saveMetaData(static_links_[i]->getName());
    grids.push_back(static_sdf_[i]->getGrid());
  }

  // Add the dynamic grids to grid array
  for (std::size_t i = 0 ; i < dynamic_sdf_.size() ; ++i)
  {
    dynamic_sdf_[i]->saveMetaData(dynamic_links_[i]->getName());
    grids.push_back(dynamic_sdf_[i]->getGrid());
  }

  for (std::size_t i = 0 ; i < active_sdf_.size() ; ++i)
  {
    active_sdf_[i]->saveMetaData(active_links_[i]->getName());
    grids.push_back(active_sdf_[i]->getGrid());
  }

  // This structure is used to save "File Level" meta-data.
  openvdb::MetaMap metadata;
  metadata.insertMeta(voxel_size_meta_name, openvdb::FloatMetadata(voxel_size_));
  metadata.insertMeta(background_meta_name, openvdb::FloatMetadata(background_));
  metadata.insertMeta(ex_bandwidth_meta_name, openvdb::FloatMetadata(exBandWidth_));
  metadata.insertMeta(in_bandwidth_meta_name, openvdb::FloatMetadata(inBandWidth_));

  vdbFile.write(grids, metadata);
  vdbFile.close();
}

std::pair<openvdb::GridPtrVecPtr, openvdb::MetaMap::Ptr>
DistanceFieldCache::readFromFile(const std::string& file_path)
{
  openvdb::io::File file(file_path);
  // Open the file.  This reads the file header, but not any grids.
  file.open();

  if (!file.isOpen())
  {
    const std::string error_text = "Unable to load openvdb models from file: " + file_path;
    throw std::runtime_error(error_text);
  }

  std::pair<openvdb::GridPtrVecPtr, openvdb::MetaMap::Ptr> grid_data;
  grid_data.first = file.getGrids();
  grid_data.second = file.getMetadata();

  // Error checking
  if (!grid_data.first) throw std::runtime_error("Unable to load any grids from file " + file_path);
  if (!grid_data.second) throw std::runtime_error("Unable to load meta-data from file: " + file_path);

  file.close(); // RAII?

  return grid_data;
}


static openvdb::math::Transform::Ptr makeTransform(const openvdb::FloatGrid& grid, const openvdb::Mat4d& tf)
{
  auto ptr = openvdb::math::Transform::createLinearTransform(tf);
  ptr->preScale(grid.transformPtr()->voxelSize());
  return ptr;
}


std::pair<distance_field::PointCloud::Ptr, distance_field::PointCloud::Ptr>
DistanceFieldCache::voxelGridToPointClouds(const moveit::core::RobotState &state) const
{
  return voxelGridToPointClouds(state, std::vector<std::string>());
}

std::pair<distance_field::PointCloud::Ptr, distance_field::PointCloud::Ptr>
DistanceFieldCache::voxelGridToPointClouds(const moveit::core::RobotState &state,
                                                              const std::vector<std::string>& exclude_list) const
{
  std::pair<distance_field::PointCloud::Ptr, distance_field::PointCloud::Ptr> pair;
  pair.first.reset(new distance_field::PointCloud());
  pair.second.reset(new distance_field::PointCloud());

  auto& inside_cloud = *pair.first;
  auto& outside_cloud = *pair.second;

  // Active links
  for (std::size_t i = 0; i < active_links_.size(); ++i)
  {
    if (std::find(exclude_list.begin(), exclude_list.end(),
                  active_links_[i]->getName()) != exclude_list.end()) continue;

    openvdb::Mat4d tf;
    Affine3dToMat4dAffine(state.getGlobalLinkTransform(active_links_[i]), tf);
    auto transform = makeTransform(*active_sdf_[i]->getGrid(), tf);

    auto copy = active_sdf_[i]->getGrid()->deepCopy();
    copy->setTransform(transform);
    auto f = distance_field::toInsideOutsidePointCloud(*copy);

    inside_cloud.insert(inside_cloud.end(), f.first->begin(), f.first->end());
    outside_cloud.insert(outside_cloud.end(), f.second->begin(), f.second->end());
  }

  // Dynamic links
  for (std::size_t i = 0; i < dynamic_links_.size(); ++i)
  {
    if (std::find(exclude_list.begin(), exclude_list.end(), dynamic_links_[i]->getName()) != exclude_list.end()) continue;

    openvdb::Mat4d tf;
    Affine3dToMat4dAffine(state.getGlobalLinkTransform(dynamic_links_[i]), tf);
    auto transform = makeTransform(*dynamic_sdf_[i]->getGrid(), tf);

    auto copy = dynamic_sdf_[i]->getGrid()->deepCopy();
    copy->setTransform(transform);
    auto f = distance_field::toInsideOutsidePointCloud(*copy);

    inside_cloud.insert(inside_cloud.end(), f.first->begin(), f.first->end());
    outside_cloud.insert(outside_cloud.end(), f.second->begin(), f.second->end());
  }

  // static links
  for (std::size_t i = 0; i < static_links_.size(); ++i)
  {
    if (std::find(exclude_list.begin(), exclude_list.end(),
                  static_links_[i]->getName()) != exclude_list.end()) continue;

    auto grid = static_sdf_[i]->getGrid();
    auto f = distance_field::toInsideOutsidePointCloud(*grid);

    inside_cloud.insert(inside_cloud.end(), f.first->begin(), f.first->end());
    outside_cloud.insert(outside_cloud.end(), f.second->begin(), f.second->end());
  }

  return pair;
}

visualization_msgs::MarkerArray
DistanceFieldCache::spheresToVisualizationMarkers(const moveit::core::RobotState &state) const
{
  std::vector<DistanceQueryData> dist_query(dist_query_);

  visualization_msgs::MarkerArray ma;
  int marker_id = 0;

  for (std::size_t i = 0; i < active_links_.size(); ++i)
  {
    openvdb::Mat4d tf;
    Affine3dToMat4d(state.getGlobalLinkTransform(active_links_[i]), tf);
    dist_query[i].spheres = active_spheres_[i];

    // transform sphere origins into world coordinate system
    std::transform(dist_query[i].spheres.begin(), dist_query[i].spheres.end(), dist_query[i].spheres.begin(),
                   [&tf](std::pair<openvdb::math::Vec3d, double> p){ p.first = (tf * p.first); return p; });

    for (const auto& sphere : dist_query[i].spheres)
    {
      auto origin = sphere.first;
      auto radius = sphere.second;
      auto combined = openvdb::math::Vec4s(origin.x(), origin.y(), origin.z(), radius);

      auto m = distance_field::toSphere(combined, marker_id++);
      ma.markers.push_back(m);
    }
  }

  return ma;
}

uint64_t DistanceFieldCache::memUsage() const
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

void DistanceFieldCache::createDefaultDistanceQuery()
{
  // Calculate distance to objects that were added dynamically into the planning scene
  for (std::size_t j = 0; j < active_links_.size(); ++j)
  {
    DistanceQueryData data;
    data.parent_name = active_links_[j]->getName();

    if (active_spheres_[j].size() == 0)
    {
      dist_query_.push_back(std::move(data));
      continue;
    }

    data.empty = false;

    // add active links
    for (std::size_t i = 0; i < active_links_.size() ; ++i)
    {
      if (i != j && !isCollisionAllowed(active_links_[i]->getName(), active_links_[j]->getName(), acm_.get()))
      {
        data.child_name.push_back(active_links_[i]->getName());
        data.child_index.push_back(i);
        data.child_type.push_back(Active);
      }
    }

    // add dynamic links
    for (std::size_t i = 0; i < dynamic_links_.size() ; ++i)
    {
      if (!isCollisionAllowed(dynamic_links_[i]->getName(), active_links_[j]->getName(), acm_.get()))
      {
        data.child_name.push_back(dynamic_links_[i]->getName());
        data.child_index.push_back(i);
        data.child_type.push_back(Dynamic);
      }
    }

    // add static links
    for (std::size_t i = 0; i < static_links_.size() ; ++i)
    {
      if (!isCollisionAllowed(static_links_[i]->getName(), active_links_[j]->getName(), acm_.get()))
      {
        data.child_name.push_back(static_links_[i]->getName());
        data.child_index.push_back(i);
        data.child_type.push_back(Static);
      }
    }

    dist_query_.push_back(std::move(data));
  }
}

void DistanceFieldCache::distanceSelf(const collision_detection::DistanceRequest &req, collision_detection::DistanceResult &res, const moveit::core::RobotState &state) const
{
  std::vector<DistanceQueryData> dist_query(dist_query_);
  std::vector<std::vector<SDFData> > data(3);

  data[Static].reserve(static_links_.size());
  data[Dynamic].reserve(dynamic_links_.size());
  data[Active].reserve(active_links_.size());

  // collecting group links
  const moveit::core::JointModelGroup* group = robot_model_->getJointModelGroup(req.group_name);
  const auto& group_links = group->getUpdatedLinkModelsWithGeometryNames();

  for (std::size_t i = 0; i < active_links_.size(); ++i)
  {
    openvdb::Mat4d tf;
    Affine3dToMat4d(state.getGlobalLinkTransform(active_links_[i]), tf);
    dist_query[i].spheres = active_spheres_[i];
    dist_query[i].gradient = req.gradient;

    // transform sphere origins into world coordinate system
    std::transform(dist_query[i].spheres.begin(), dist_query[i].spheres.end(), dist_query[i].spheres.begin(),
                   [&tf](std::pair<openvdb::math::Vec3d, double> p){ p.first = (tf * p.first); return p; });

    // NOTE that we transform spheres before transposing 'tf'. OpenVDB is using a different convention than
    // we are (I guess?)
    tf = tf.transpose();
    SDFData d(active_sdf_[i]->getGrid(), tf);
    data[Active].push_back(d);
  }

  for (std::size_t i = 0; i < dynamic_links_.size(); ++i)
  {
    openvdb::Mat4d tf;
    Affine3dToMat4dAffine(state.getGlobalLinkTransform(dynamic_links_[i]), tf);

    SDFData d(dynamic_sdf_[i]->getGrid(), tf);
    data[Dynamic].push_back(d);
  }

  for (std::size_t i = 0; i < static_links_.size(); ++i)
  {
    SDFData d(static_sdf_[i]->getGrid());
    data[Static].push_back(d);
  }

  // Compute minimum distance
  for (std::size_t i = 0 ; i < dist_query.size() ; ++i)
  {
    if (!dist_query[i].empty)
    {
      collision_detection::DistanceResultsData d;
      distanceSelfHelper(dist_query[i], data, d);
      res.distance.insert(std::make_pair(d.link_name[0], d));
    }
  }

  double d = background_;
  std::string index = res.distance.begin()->second.link_name[0];
  for (auto it = res.distance.begin(); it != res.distance.end(); ++it)
  {
    if (it->second.min_distance < d)
    {
      index = it->second.link_name[0];
      d = it->second.min_distance;
    }
  }
  res.minimum_distance = res.distance[index];
}

bool DistanceFieldCache::isCollisionAllowed(const std::string &l1, const std::string &l2, const collision_detection::AllowedCollisionMatrix *acm) const
{
    // use the collision matrix (if any) to avoid certain distance checks
    if (acm)
    {
      collision_detection::AllowedCollision::Type type;

      bool found = acm->getAllowedCollision(l1, l2, type);
      if (found)
      {
        // if we have an entry in the collision matrix, we read it
        return type == collision_detection::AllowedCollision::ALWAYS;
      }
    }
    return false;
}

void DistanceFieldCache::createDefaultAllowedCollisionMatrix()
{
  acm_.reset(new collision_detection::AllowedCollisionMatrix());
  // Use default collision operations in the SRDF to setup the acm
  const std::vector<std::string>& collision_links = robot_model_->getLinkModelNamesWithCollisionGeometry();
  acm_->setEntry(collision_links, collision_links, false);

  // allow collisions for pairs that have been disabled
  const std::vector<srdf::Model::DisabledCollision> &dc = robot_model_->getSRDF()->getDisabledCollisionPairs();
  for (std::vector<srdf::Model::DisabledCollision>::const_iterator it = dc.begin(); it != dc.end(); ++it)
    acm_->setEntry(it->link1_, it->link2_, true);
}

template <typename T>
static bool approxEqual(T a, T b, const T eps = 0.00001)
{
  return std::abs(a - b) < eps;
}

void DistanceFieldCache::distanceSelfHelper(const DistanceQueryData &data, std::vector<std::vector<SDFData> > &sdfs_data, collision_detection::DistanceResultsData &res) const
{
  res.min_distance = background_;
  res.link_name[0] = data.parent_name;
  res.hasNearestPoints = false;

  // Variables to keep track of gradient information, if requested
  openvdb::Vec3f gradient = openvdb::Vec3f::zero();
  float total_weights = 0.0f;

  for (std::size_t i = 0; i < data.child_index.size(); ++i)
  {
    float child_min = background_;
    openvdb::math::Coord child_min_ijk;
    bool dist_found = false;
    SDFData &child_data = sdfs_data[data.child_type[i]][data.child_index[i]];

    for (std::size_t j = 0; j < data.spheres.size(); ++j)
    {
      openvdb::math::Coord ijk = child_data.transform->worldToIndexNodeCentered(data.spheres[j].first);
      float child_dist = child_data.accessor.getValue(ijk);

      if (!approxEqual(child_dist, background_))
      {
        child_dist -= data.spheres[j].second;
        if (child_dist < child_min)
        {
          child_min = child_dist;
          child_min_ijk = ijk;
          dist_found = true;
        }
      }
    }

    if (dist_found)
    {
      // update link minimum distance
      if (child_min < res.min_distance)
      {
        res.min_distance = child_min;
        res.link_name[1] = data.child_name[i];
      }

      // compute gradient
      if (data.gradient)
      {
        // Compute gradient
        openvdb::Vec3f result = openvdb::math::ISGradient<openvdb::math::CD_2ND>::result(child_data.accessor, child_min_ijk);
        if (result.sum() != 0.0) // gradients can come back as [0, 0, 0]
        {
          float weight = background_ - child_min;
          total_weights += weight;
          result = child_data.transform->baseMap()->applyIJT(result);
          result.normalize();
          result = result * weight;
          gradient += result;
          res.hasGradient = true;
        }
      }
    }
  }

  if (res.hasGradient)
  {
    if (total_weights == 0.0)
    {
      res.gradient = Eigen::Vector3d(0, 0, 0);
    }
    else
    {
      res.gradient = Eigen::Vector3d(gradient(0) / total_weights, gradient(1) / total_weights, gradient(2) / total_weights);
      res.gradient.normalize();
    }
  }
}

bool DistanceFieldCache::hasCollisionGeometry(const moveit::core::LinkModel *link) const
{
  return std::find(links_.begin(), links_.end(), link) != links_.end();
}

std::vector<const moveit::core::LinkModel *> DistanceFieldCache::identifyStaticLinks() const
{
  // The purpose of this function is to walk the tree of robot links & find all of the links that are connected
  // to one another via a static transformation chain that includes the root link.
  const robot_model::LinkModel *root_link = robot_model_->getRootLink();

  std::vector<const moveit::core::LinkModel*> static_links;
  std::vector<const robot_model::LinkModel*> considered_set;
  considered_set.push_back(root_link);

  // Check to make sure link has collision geometry to add. I don't think this is required,
  // because it will be world link and I don't think it will ever have geometry.
  if (hasCollisionGeometry(root_link))
  {
    static_links.push_back(root_link);
  }

  identifyStaticLinksHelper(root_link, static_links, considered_set);
  return static_links;
}

void DistanceFieldCache::identifyStaticLinksHelper(const moveit::core::LinkModel *link,
                                                                      std::vector<const moveit::core::LinkModel *> &in_set,
                                                                      std::vector<const moveit::core::LinkModel *> &considered) const
{
  // Consider all of the links that are attached to this one
  const moveit::core::LinkTransformMap fixed_attached = link->getAssociatedFixedTransforms();

  // Helper function to test if link has already been looked at
  auto is_in_set = [&considered] (const robot_model::LinkModel* l) {
    return std::find(considered.begin(), considered.end(), l) != considered.end();
  };

  for (auto it = fixed_attached.begin(); it!=fixed_attached.end(); ++it)
  {
    if (!is_in_set(it->first))
    {
      considered.push_back(it->first);

      if (hasCollisionGeometry(it->first))
      {
        in_set.push_back(it->first);
      }

      identifyStaticLinksHelper(it->first, in_set, considered);
    }
  } // end of associated links
}

std::vector<const moveit::core::LinkModel *> DistanceFieldCache::identifyActiveLinks() const
{
  std::vector<const moveit::core::LinkModel *> active_links;

  const auto& groups = robot_model_->getJointModelGroups();

  for (const auto& group : groups) // for each planning group
  {
    const auto& links = group->getLinkModels();
    for (const auto& link : links) // for each link inside a given group
    {
      bool already_added = std::find(active_links.begin(), active_links.end(), link) != active_links.end();
      if (!already_added && hasCollisionGeometry(link))
      {
        active_links.push_back(link);
      }
    }
  }

  return active_links;
}

std::vector<const moveit::core::LinkModel *>
DistanceFieldCache::identifyDynamicLinks(std::vector<const moveit::core::LinkModel *>& static_links,
                                                            std::vector<const moveit::core::LinkModel *>& active_links) const
{
  auto dynamic_links = links_;

  // remove static links from list
  for (std::size_t i = 0 ; i < static_links.size() ; ++i)
  {
    dynamic_links.erase(std::remove(dynamic_links.begin(), dynamic_links.end(), static_links[i]));
  }

  // remove active links from list
  for (std::size_t i = 0 ; i < active_links.size() ; ++i)
  {
    dynamic_links.erase(std::remove(dynamic_links.begin(), dynamic_links.end(), active_links[i]));
  }

  return dynamic_links;
}

// LOCAL helper function
static openvdb::FloatGrid::Ptr findGridInVec(const std::string& name, const openvdb::GridPtrVec& grids)
{
  for (const auto& grid : grids)
  {
    if (grid->getName() == name)
    {
      return openvdb::gridPtrCast<openvdb::FloatGrid>(grid);
    }
  }
  return openvdb::FloatGrid::Ptr();
}

void DistanceFieldCache::loadStaticLinks(std::vector<const moveit::core::LinkModel *> &static_links,
                                                            const openvdb::GridPtrVec &grids,
                                                            std::vector<OpenVDBDistanceFieldConstPtr>& fields)
{
  for (const auto link : static_links)
  {
    // Find the associated grid pointer
    auto ptr = findGridInVec(link->getName(), grids);
    if (!ptr)
    {
      throw std::runtime_error("No grid with name " + link->getName() + " in VDB file.");
    }

    OpenVDBDistanceFieldPtr sdf(new OpenVDBDistanceField(ptr));
    fields.push_back(OpenVDBDistanceFieldConstPtr(sdf));
  }
}

void DistanceFieldCache::loadActiveLinks(std::vector<const moveit::core::LinkModel *> &active_links,
                                                            const openvdb::GridPtrVec &grids,
                                                            std::vector<OpenVDBDistanceFieldConstPtr>& fields)
{
  // for every active link we also want to generate spheres
  active_spheres_.resize(active_links.size());

  const auto n_spheres = 20;
  const auto can_overlap = true;
  const auto min_voxel_size = 1.0f;
  const auto max_voxel_size = std::numeric_limits<float>::max();
  const auto iso_surface = 0.0f; // the value at which the surface exists; 0.0 for solid models (clouds are different)
  const auto n_instances = 100000; // num of voxels to consider when fitting spheres

  for (size_t i = 0; i < active_links.size(); ++i)
  {
    const auto link = active_links[i];
    // Find the associated grid pointer
    auto ptr = findGridInVec(link->getName(), grids);
    if (!ptr)
    {
      throw std::runtime_error("No grid with name " + link->getName() + " in VDB file.");
    }
    OpenVDBDistanceFieldPtr sdf(new OpenVDBDistanceField(ptr));
    sdf->fillWithSpheres(active_spheres_[i], n_spheres, can_overlap, min_voxel_size,
                         max_voxel_size, iso_surface, n_instances);

    fields.push_back(OpenVDBDistanceFieldConstPtr(sdf));
  }
}

void DistanceFieldCache::loadDynamicLinks(std::vector<const moveit::core::LinkModel *> &dynamic_links,
                                                             const openvdb::GridPtrVec &grids,
                                                             std::vector<OpenVDBDistanceFieldConstPtr>& fields)
{
  for (const auto link : dynamic_links)
  {
    // Find the associated grid pointer
    auto ptr = findGridInVec(link->getName(), grids);
    if (!ptr)
    {
      throw std::runtime_error("No grid with name " + link->getName() + " in VDB file.");
    }
    OpenVDBDistanceFieldPtr sdf(new OpenVDBDistanceField(ptr));
    fields.push_back(OpenVDBDistanceFieldConstPtr(sdf));
  }
}

CollisionRobotOpenVDB::CollisionRobotOpenVDB(const moveit::core::RobotModelConstPtr &model,
                                                             const float voxel_size, const float background,
                                                             const float exBandWidth, const float inBandWidth):
  CollisionRobotIndustrial(model,0.0,1.0)
{
  DistanceFieldManager.registerDistanceFieldCache(model,voxel_size,background,exBandWidth,inBandWidth);
  distance_field_cache_ = DistanceFieldManager.getDistanceFieldCacheEntry(robot_model_);
}

CollisionRobotOpenVDB::CollisionRobotOpenVDB(const moveit::core::RobotModelConstPtr &model,
                                                             const std::string &file_path):
  CollisionRobotIndustrial(model,0.0,1.0)
{
  DistanceFieldManager.registerDistanceFieldCache(model,file_path);
  distance_field_cache_ = DistanceFieldManager.getDistanceFieldCacheEntry(robot_model_);
}

void CollisionRobotOpenVDB::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state) const
{
  if(req.distance)
  {
    // calculate distance only
    DistanceRequest dreq;
    DistanceResult dres;
    dreq.group_name = req.group_name;
    dreq.acm = NULL;
    distance_field_cache_->distanceSelf(dreq,dres,state);
    res.collision = dres.collision;
    res.distance = dres.minimum_distance.min_distance;
    res.contact_count = 1;
  }
  else
  {
    CollisionRobotIndustrial::checkSelfCollision(req,res,state);
  }
}

void CollisionRobotOpenVDB::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state
                                , const AllowedCollisionMatrix &acm) const
{
  if(req.distance)
  {
    // calculate distance only
    DistanceRequest dreq;
    DistanceResult dres;
    dreq.group_name = req.group_name;
    dreq.acm = &acm;
    distance_field_cache_->distanceSelf(dreq,dres,state);
    res.collision = dres.collision;
    res.distance = dres.minimum_distance.min_distance;
    res.contact_count = 1;
  }
  else
  {
    CollisionRobotIndustrial::checkSelfCollision(req,res,state,acm);
  }
}

}
