ament_add_gtest(goalmanager_tests goalmanager_tests.cpp)
target_link_libraries(goalmanager_tests ${PROJECT_NAME})

ament_add_gtest(system_tfinfo_tests system_tfinfo_tests.cpp)
target_link_libraries(system_tfinfo_tests ${PROJECT_NAME})

ament_add_gtest(system_safety_reflex_tests system_safety_reflex_tests.cpp)
target_link_libraries(system_safety_reflex_tests ${PROJECT_NAME})
