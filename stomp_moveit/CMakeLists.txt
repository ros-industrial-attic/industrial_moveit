cmake_minimum_required(VERSION 2.8.3)
project(stomp_moveit)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  stomp_core
  cmake_modules
  pluginlib
)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_definitions("-std=c++11")


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS moveit_ros_planning moveit_core stomp_core cmake_modules pluginlib
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/stomp_optimization_task.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############

