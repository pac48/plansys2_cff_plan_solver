ament_add_gtest(cff_test cff_test.cpp)
target_link_libraries(cff_test ${PROJECT_NAME} dl)