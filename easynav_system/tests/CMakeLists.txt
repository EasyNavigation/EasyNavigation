# Logs (and saved configurations) of the tests stay in the build directory.
ament_add_gtest(goalmanager_tests goalmanager_tests.cpp
  ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log)
target_link_libraries(goalmanager_tests ${PROJECT_NAME})

ament_add_gtest(system_tfinfo_tests system_tfinfo_tests.cpp
  ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log)
target_link_libraries(system_tfinfo_tests ${PROJECT_NAME})

ament_add_gtest(system_pause_tests system_pause_tests.cpp
  ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log)
target_link_libraries(system_pause_tests ${PROJECT_NAME})

ament_add_gtest(system_lifecycle_cycle_tests system_lifecycle_cycle_tests.cpp
  ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log)
target_link_libraries(system_lifecycle_cycle_tests ${PROJECT_NAME})

ament_add_gtest(system_lifecycle_mission_tests system_lifecycle_mission_tests.cpp
  ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log)
target_link_libraries(system_lifecycle_mission_tests ${PROJECT_NAME})

ament_add_gtest(system_velocity_smoothing_tests system_velocity_smoothing_tests.cpp
  ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log)
target_link_libraries(system_velocity_smoothing_tests ${PROJECT_NAME})

ament_add_gtest(system_shutdown_request_tests system_shutdown_request_tests.cpp
  ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log)
target_link_libraries(system_shutdown_request_tests ${PROJECT_NAME})

ament_add_gtest(system_actions_tests system_actions_tests.cpp
  ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log)
target_link_libraries(system_actions_tests ${PROJECT_NAME})

ament_add_gtest(system_robot_geometry_tests system_robot_geometry_tests.cpp
  ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log)
target_link_libraries(system_robot_geometry_tests ${PROJECT_NAME})

ament_add_gtest(system_fault_injection_tests system_fault_injection_tests.cpp TIMEOUT 120
  ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log)
target_link_libraries(system_fault_injection_tests ${PROJECT_NAME})

ament_add_gtest(system_safety_mode_tests system_safety_mode_tests.cpp TIMEOUT 300
  ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log)
target_link_libraries(system_safety_mode_tests ${PROJECT_NAME})

ament_add_gtest(real_time_tests real_time_tests.cpp
  ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log)
target_link_libraries(real_time_tests ${PROJECT_NAME})

ament_add_gtest(safety_module_tests safety_module_tests.cpp
  ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log)
target_link_libraries(safety_module_tests ${PROJECT_NAME})

ament_add_gtest(rt_monitor_tests rt_monitor_tests.cpp
  ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log)
target_link_libraries(rt_monitor_tests ${PROJECT_NAME})
