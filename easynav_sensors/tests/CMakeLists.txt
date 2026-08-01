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
