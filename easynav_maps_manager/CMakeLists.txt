cmake_minimum_required(VERSION 3.20)
project(easynav_maps_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(dependencies
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  easynav_common::easynav_common
  easynav_core::easynav_core
  pluginlib::pluginlib
)

set(package_dependencies
  rclcpp
  rclcpp_lifecycle
  easynav_common
  easynav_core
  pluginlib
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/easynav_maps_manager/MapsManagerNode.cpp
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
  pluginlib::pluginlib
  ${lifecycle_msgs_TARGETS}
)

add_library(dummy_maps_manager SHARED
  src/easynav_maps_manager/DummyMapsManager.cpp
)
target_include_directories(dummy_maps_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(dummy_maps_manager PUBLIC ${dependencies})

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  dummy_maps_manager
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
ament_export_libraries(${PROJECT_NAME} dummy_maps_manager)
ament_export_targets(export_${PROJECT_NAME})

# Register the planning pluging
pluginlib_export_plugin_description_file(easynav_core easynav_maps_manager_plugins.xml)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  easynav_common
  easynav_core
  pluginlib
  lifecycle_msgs
)
ament_package()
