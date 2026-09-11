ament_add_gtest(core_method_test core_method_test.cpp)
target_link_libraries(core_method_test
  ${PROJECT_NAME}
)

ament_add_gtest(obstacle_proximity_tests obstacle_proximity_tests.cpp)
target_link_libraries(obstacle_proximity_tests
  ${PROJECT_NAME}
)
