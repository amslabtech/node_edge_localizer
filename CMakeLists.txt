cmake_minimum_required(VERSION 3.5)
project(node_edge_localizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED COMPONENTS
  amsl_navigation_msgs
  amsl_navigation_managers
  roscpp
  rospy
  std_msgs
  tf
  cmake_modules
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS)

find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

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

add_executable(node_edge_localizer
  src/node_edge_localizer.cpp
  src/node_edge_particle.cpp
  src/calculation.cpp
  src/node_edge_map_management.cpp
)
add_executable(localizer
  src/localizer_node.cpp
  src/localizer.cpp
  src/distance_map.cpp
  src/calculation.cpp
)

add_dependencies(node_edge_localizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(localizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(node_edge_localizer
  ${catkin_LIBRARIES}
)
target_link_libraries(localizer
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
