cmake_minimum_required(VERSION 3.20)
project(easynav_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(easynav_interfaces)
find_package(easynav_common)
find_package(easynav_controller)
find_package(easynav_localizer)
find_package(easynav_maps_manager)
find_package(easynav_planner)
find_package(easynav_sensors)
find_package(geometry_msgs)
find_package(tf2_geometry_msgs)
find_package(tf2)

set(dependencies
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  easynav_common::easynav_common
  easynav_controller::easynav_controller
  easynav_localizer::easynav_localizer
  easynav_maps_manager::easynav_maps_manager
  easynav_planner::easynav_planner
  easynav_sensors::easynav_sensors
  tf2::tf2
  ${geometry_msgs_TARGETS}
  ${easynav_interfaces_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
)

set(package_dependencies
  rclcpp
  rclcpp_lifecycle
  easynav_interfaces
  easynav_common
  easynav_controller
  easynav_localizer
  easynav_maps_manager
  easynav_planner
  easynav_sensors
  geometry_msgs
  tf2_geometry_msgs
  lifecycle_msgs
  tf2
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/easynav_system/SystemNode.cpp
  src/easynav_system/GoalManager.cpp
  src/easynav_system/GoalManagerClient.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  easynav_common::easynav_common
  easynav_core::easynav_core
  easynav_controller::easynav_controller
  easynav_localizer::easynav_localizer
  easynav_maps_manager::easynav_maps_manager
  easynav_planner::easynav_planner
  easynav_sensors::easynav_sensors
  tf2_geometry_msgs::tf2_geometry_msgs
  ${easynav_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
)

add_executable(system_main src/system_main.cpp)
target_link_libraries(system_main PUBLIC
  ${PROJECT_NAME}
)

add_executable(goalmanager_test_main src/goalmanager_test_main.cpp)
target_link_libraries(goalmanager_test_main PUBLIC
  ${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  system_main
  goalmanager_test_main
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  easynav_interfaces
  easynav_common
  easynav_controller
  easynav_localizer
  easynav_maps_manager
  easynav_planner
  easynav_sensors
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_geometry_msgs
  lifecycle_msgs
)
ament_package()
