#include <collision_robot_openvdb.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

struct DistanceFieldOptions
{
  float voxel_size;
  float background;
  std::string save_file;
};

void createDistanceFieldAndSave(moveit::core::RobotModelConstPtr model, const DistanceFieldOptions& options)
{
  auto exBandWidth = options.background / options.voxel_size;
  auto inBandWidth = options.background/ options.voxel_size;

  ROS_INFO_STREAM("Generating OpenVDB model for robot: " << model->getName());
  distance_field::CollisionRobotOpenVDB vdb_model(model, options.voxel_size, options.background, exBandWidth, inBandWidth);
  ROS_INFO_STREAM("Done generating OpenVDB model for robot: " << model->getName() << "; Saving to file");
  vdb_model.writeToFile(options.save_file);
  ROS_INFO_STREAM("Done saving to file: " << options.save_file);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "signed_distance_field_builder");

  ros::NodeHandle nh ("~");

  const std::string robot_description = "robot_description";
  robot_model_loader::RobotModelLoader loader (robot_description, false);

  auto model = loader.getModel();

  if (!model)
  {
    ROS_ERROR("Unable to load robot description from '%s' parameter.", robot_description.c_str());
    return -1;
  }

  // Load parameters for options
  const auto default_voxel_size = 0.02; // m
  const auto default_background_distance = 0.2; // m
  const auto default_save_file = std::string("robot.vdb");

  DistanceFieldOptions options;

  options.voxel_size = nh.param("voxel_size", default_voxel_size);
  options.background = nh.param("background", default_background_distance);
  options.save_file = nh.param("save_file", default_save_file);

  createDistanceFieldAndSave(model, options);
}
