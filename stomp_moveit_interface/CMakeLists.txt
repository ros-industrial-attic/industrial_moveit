cmake_minimum_required(VERSION 2.8.3)
project(stomp_moveit_interface)

find_package(catkin REQUIRED COMPONENTS stomp moveit_core pluginlib moveit_experimental cmake_modules)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stomp_moveit_interface
  CATKIN_DEPENDS stomp moveit_core pluginlib moveit_experimental
  DEPENDS 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# cost features
add_library(stomp_cost_features
  src/cost_features/collision_feature.cpp
  src/cost_features/obstacle_avoidance_feature.cpp
  src/cost_features/stomp_cost_feature.cpp)

target_link_libraries(stomp_cost_features
  ${catkin_LIBRARIES})

# stomp planner interface
add_library(stomp_moveit_interface
  src/feature_set.cpp
  src/stomp_planner.cpp
  src/stomp_trajectory.cpp
  src/stomp_optimization_task.cpp
  src/cost_features/stomp_cost_feature.cpp
#  src/cost_features/collision_feature.cpp
#  src/cost_features/cartesian_orientation_feature.cpp
#  src/cost_features/cartesian_vel_acc_feature.cpp
#  src/cost_features/joint_vel_acc_feature.cpp
#  src/stomp_collision_point.cpp
#  src/stomp_collision_space.cpp
#  src/stomp_cost_function_input.cpp
#  src/stomp_robot_model.cpp
#  src/treefksolverjointposaxis_partial.cpp
)

target_link_libraries(stomp_moveit_interface
  stomp_cost_features ${catkin_LIBRARIES})

add_library(stomp_planner_plugin
  src/stomp_planner_manager.cpp)
target_link_libraries(stomp_planner_plugin stomp_moveit_interface ${catkin_LIBRARIES})






