cmake_minimum_required(VERSION 2.8.3)
project(stomp_core)

add_definitions("-std=c++11")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stomp_core
  CATKIN_DEPENDS roscpp cmake_modules
  DEPENDS EIGEN3
)

###########
## Build ##
###########
include_directories(
  include
  examples
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/stomp.cpp
   src/utils.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_example examples/stomp_example.cpp)
target_link_libraries(${PROJECT_NAME}_example ${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  set(UTEST_SRC_FILES test/utest.cpp
      test/stomp_3dof.cpp)
  catkin_add_gtest(${PROJECT_NAME}_utest ${UTEST_SRC_FILES})
  target_link_libraries(${PROJECT_NAME}_utest ${PROJECT_NAME})

endif()
