install(DIRECTORY include/
  DESTINATION include)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_static_distance_field test/test_static_distance_field.cpp)
  target_link_libraries(test_static_distance_field ${catkin_LIBRARIES})
endif()
