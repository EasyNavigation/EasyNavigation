ament_add_gtest(navstate_tests navstate_tests.cpp)
target_link_libraries(navstate_tests
  ${PROJECT_NAME}
)

ament_add_gtest(parameters_tests parameters_tests.cpp)
target_link_libraries(parameters_tests
  ${PROJECT_NAME}
)

ament_add_gtest(robot_geometry_tests robot_geometry_tests.cpp)
target_link_libraries(robot_geometry_tests
  ${PROJECT_NAME}
)

ament_add_gtest(transform_listener_tests transform_listener_tests.cpp)
target_link_libraries(transform_listener_tests
  ${PROJECT_NAME}
  ${tf2_msgs_TARGETS}
)
