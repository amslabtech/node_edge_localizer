add_rostest_gtest(node_edge_localizer_test
    test/node_edge_localizer.test
    src/node_edge_localizer_test.cpp
    ../src/calculation.cpp
)
target_link_libraries(node_edge_localizer_test ${catkin_LIBRARIES})