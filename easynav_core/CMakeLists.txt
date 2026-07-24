cmake_minimum_required(VERSION 3.20)
project(easynav_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(easynav_common REQUIRED)
find_package(easynav_sensors REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

add_library(${PROJECT_NAME} SHARED
  src/easynav_core/ControllerMethodBase.cpp
  src/easynav_core/LocalizerMethodBase.cpp
  src/easynav_core/MapsManagerBase.cpp
  src/easynav_core/MethodBase.cpp
  src/easynav_core/PlannerMethodBase.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  easynav_common::easynav_common
  easynav_sensors::easynav_sensors
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${visualization_msgs_TARGETS}
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
  visualization_msgs
  tf2_ros
  PCL
)
ament_package()
