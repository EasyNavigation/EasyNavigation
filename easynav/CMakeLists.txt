cmake_minimum_required(VERSION 3.20)
project(easynav)

find_package(ament_cmake REQUIRED)

ament_export_dependencies(
  easynav_common
  easynav_controller
  easynav_core
  easynav_interfaces
  easynav_maps_manager
  easynav_planner
  easynav_localizer
  easynav_recovery
  easynav_sensors
  easynav_system
)
ament_package()
