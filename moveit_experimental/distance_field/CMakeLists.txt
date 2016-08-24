set(MOVEIT_LIB_NAME moveit_openvdb)

FIND_PACKAGE(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message("OPENMP FOUND")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_library(OPENVDB_LIBRARY NAMES openvdb)

find_library(TBB_LIBRARY NAMES tbb)

add_library(${MOVEIT_LIB_NAME} src/openvdb_distance_field.cpp src/openvdb_visualization.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${OPENVDB_LIBRARY} ${TBB_LIBRARY})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)


if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_static_distance_field test/test_static_distance_field.cpp)
  target_link_libraries(test_static_distance_field ${catkin_LIBRARIES})
endif()
