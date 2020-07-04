cmake_minimum_required(VERSION 2.8.3)
project(node_edge_localizer)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  amsl_navigation_msgs
  amsl_navigation_managers
  roscpp
  rospy
  std_msgs
  tf
  cmake_modules
)

find_package(Eigen3 REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES node_edge_localizer
#  CATKIN_DEPENDS amsl_navigation_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(node_edge_localizer src/node_edge_localizer.cpp
                                   src/node_edge_particle.cpp
                                   src/calculation.cpp
                                   src/node_edge_map_management.cpp)

add_dependencies(node_edge_localizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(node_edge_localizer
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(node_edge_localizer_test
        test/node_edge_localizer.test
        test/node_edge_localizer_test.cpp
        src/calculation.cpp
    )
    target_link_libraries(node_edge_localizer_test ${catkin_LIBRARIES})
endif()
