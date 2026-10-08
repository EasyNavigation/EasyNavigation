ament_add_gtest(perceptions_tests perceptions_tests.cpp)
target_link_libraries(perceptions_tests
  ${PROJECT_NAME}
)

ament_add_gtest(pointperceptionsops_tests pointperceptionsops_tests.cpp)
target_link_libraries(pointperceptionsops_tests
  ${PROJECT_NAME}
)

ament_add_gtest(sensors_node_tests sensors_node_tests.cpp)
target_link_libraries(sensors_node_tests
  ${PROJECT_NAME}
)

ament_add_gtest(plugin_tests plugin_tests.cpp)
target_link_libraries(plugin_tests
  ${PROJECT_NAME}
)

ament_add_gtest(sensors_lifecycle_tests sensors_lifecycle_tests.cpp)
target_link_libraries(sensors_lifecycle_tests
  ${PROJECT_NAME}
)

ament_add_gtest(perception_locking_tests perception_locking_tests.cpp)
target_link_libraries(perception_locking_tests
  ${PROJECT_NAME}
)

ament_add_gtest(sensors_data_age_tests sensors_data_age_tests.cpp)
target_link_libraries(sensors_data_age_tests
  ${PROJECT_NAME}
  ${diagnostic_msgs_TARGETS}
)

ament_add_gtest(perception_handlers_tests perception_handlers_tests.cpp)
target_link_libraries(perception_handlers_tests
  ${PROJECT_NAME}
)
