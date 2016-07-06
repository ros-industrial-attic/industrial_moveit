set(MOVEIT_LIB_NAME moveit_openvdb)

find_library(OPENVDB_LIBRARY NAMES openvdb)

add_library(${MOVEIT_LIB_NAME} src/openvdb_distance_field.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${OPENVDB_LIBRARY})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_static_distance_field test/test_static_distance_field.cpp)
  target_link_libraries(test_static_distance_field ${catkin_LIBRARIES})
endif()
