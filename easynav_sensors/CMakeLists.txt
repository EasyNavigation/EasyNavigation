cmake_minimum_required(VERSION 3.20)
project(easynav_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(easynav_common REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pluginlib REQUIRED)

set(dependencies
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  easynav_common::easynav_common
)

set(package_dependencies
  rclcpp
  rclcpp_lifecycle
  easynav_common
)

add_library(${PROJECT_NAME} SHARED
  src/easynav_sensors/SensorsNode.cpp
  src/easynav_sensors/types/ImagePerception.cpp
  src/easynav_sensors/types/PointPerception.cpp
  src/easynav_sensors/types/IMUPerception.cpp
  src/easynav_sensors/types/GNSSPerception.cpp
  src/easynav_sensors/types/OdometryPerception.cpp
  src/easynav_sensors/types/DetectionsPerception.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  easynav_common::easynav_common
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  pcl_conversions::pcl_conversions
  cv_bridge::cv_bridge
  pluginlib::pluginlib
  ${sensor_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${PCL_LIBRARIES}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
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
  find_package(pcl_conversions REQUIRED)
  find_package(PCL REQUIRED COMPONENTS common io)

  add_subdirectory(tests)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

# Register PerceptionHandler plugins so external packages can discover them
pluginlib_export_plugin_description_file(
  easynav_sensors easynav_sensors_perception_handler_plugins.xml)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  easynav_common
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  vision_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  cv_bridge
  pluginlib
  PCL
)
ament_package()
