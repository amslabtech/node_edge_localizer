add_rostest_gtest(node_edge_localizer_test
    test/node_edge_localizer.test
    src/node_edge_localizer_test.cpp
    ../src/calculation.cpp
)
target_link_libraries(node_edge_localizer_test ${catkin_LIBRARIES})

add_rostest_gtest(localization_test
    test/localization.test
    src/localization_test.cpp
)
target_link_libraries(localization_test ${catkin_LIBRARIES})

add_rostest_gtest(observation_test
    test/observation.test
    src/observation_test.cpp
)
target_link_libraries(observation_test ${catkin_LIBRARIES})