  cmake_minimum_required(VERSION 2.8.3)
project(stomp)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS roscpp cmake_modules)
find_package(OpenMP REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stomp
  CATKIN_DEPENDS roscpp
  DEPENDS Eigen
)


add_library(stomp
  src/chomp.cpp
  src/covariant_movement_primitive.cpp
  src/policy_improvement.cpp
  src/stomp.cpp
  src/stomp_utils.cpp
)

add_executable(test_stomp2d
  test/stomp_2d_test.cpp
)
add_executable(test_cmp
  test/test_cmp.cpp
)
#add_executable(test_eigen_perf
#  test/eigen_perf_test.cpp
#)
#add_executable(stomp_constrained_test
#  test/stomp_constrained_test.cpp
#)

add_executable(stomp_test_distributions
  test/test_distributions.cpp
  src/stomp_constrained.cpp
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenMP_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS} ${OpenMP_CXX_FLAGS})
#add_definitions(-DEIGEN_NO_DEBUG -DNDEBUG)

target_link_libraries(test_stomp2d stomp)
target_link_libraries(test_cmp stomp)
target_link_libraries(stomp ${catkin_LIBRARIES} ${OpenMP_CXX_FLAGS})
#target_link_libraries(test_eigen_perf ${Eigen_LIBRARIES} ${OpenMP_CXX_FLAGS})
#target_link_libraries(stomp_constrained_test ${catkin_LIBRARIES} ${OpenMP_CXX_FLAGS})
target_link_libraries(stomp_test_distributions ${catkin_LIBRARIES} ${OpenMP_CXX_FLAGS})
