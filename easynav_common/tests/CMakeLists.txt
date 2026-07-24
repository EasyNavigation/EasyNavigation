ament_add_gtest(navstate_tests navstate_tests.cpp)
target_link_libraries(navstate_tests
  ${PROJECT_NAME}
)
