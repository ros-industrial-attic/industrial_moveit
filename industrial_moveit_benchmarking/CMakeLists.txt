cmake_minimum_required(VERSION 2.8.3)
project(industrial_moveit_benchmarking)

add_definitions("-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_planning
  stomp_moveit
  constrained_ik
  cmake_modules
  pluginlib
  moveit_core
)

# This is required because there is a bug in moveit_ros_planning they are not exporting
# this library.

## System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(stomp_benchmarking_node src/stomp_valgrind.cpp)
add_executable(joint_interpolated_benchmarking_node src/joint_interpolated_valgrind.cpp)
add_executable(cartesian_benchmarking_node src/clik_valgrind.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(stomp_benchmarking_node ${catkin_LIBRARIES})
target_link_libraries(joint_interpolated_benchmarking_node ${catkin_LIBRARIES})
target_link_libraries(cartesian_benchmarking_node ${catkin_LIBRARIES})
