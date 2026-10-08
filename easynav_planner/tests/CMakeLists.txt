ament_add_gtest(planner_node_tests planner_node_tests.cpp)
target_link_libraries(planner_node_tests
  easynav_planner
)

ament_add_gtest(faulty_planner_tests faulty_planner_tests.cpp)
target_link_libraries(faulty_planner_tests
  faulty_planner
)

ament_add_gtest(planner_path_check_tests planner_path_check_tests.cpp)
target_link_libraries(planner_path_check_tests
  easynav_planner
  ${diagnostic_msgs_TARGETS}
)
