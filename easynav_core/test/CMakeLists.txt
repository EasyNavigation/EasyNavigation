find_package(easynav_sensors REQUIRED)

ament_add_gtest(core_method_test core_method_test.cpp)
target_link_libraries(core_method_test
  ${PROJECT_NAME}
  easynav_sensors::easynav_sensors
)

ament_add_gtest(recovery_manager_base_tests recovery_manager_base_tests.cpp)
target_link_libraries(recovery_manager_base_tests
  ${PROJECT_NAME}
)

ament_add_gtest(rate_monitor_tests rate_monitor_tests.cpp)
target_link_libraries(rate_monitor_tests
  ${PROJECT_NAME}
)

ament_add_gtest(method_base_schedule_tests method_base_schedule_tests.cpp)
target_link_libraries(method_base_schedule_tests
  ${PROJECT_NAME}
)
