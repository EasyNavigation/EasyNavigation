cmake_minimum_required(VERSION 3.20)
project(easynav_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_sensors REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/easynav_core/ControllerMethodBase.cpp
  src/easynav_core/LocalizerMethodBase.cpp
  src/easynav_core/MapsManagerBase.cpp
  src/easynav_core/MethodBase.cpp
  src/easynav_core/ObstacleProximity.cpp
  src/easynav_core/PlannerMethodBase.cpp
  src/easynav_core/RecoveryEvaluatorBase.cpp
  src/easynav_core/RecoveryMitigationBase.cpp
  src/easynav_core/SafetyReflexBase.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  easynav_common::easynav_common
  easynav_sensors::easynav_sensors
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${diagnostic_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
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
  add_subdirectory(test)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_dependencies(${package_dependencies})
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(
  easynav_common
  easynav_sensors
  rclcpp_lifecycle
  tf2_ros
  diagnostic_msgs
  rcl_interfaces
)
ament_package()
