cmake_minimum_required(VERSION 3.0.2)
project(mcl3d_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(PCL REQUIRED)

find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp QUIET)
if(NOT YAMLCPP_FOUND)
  find_package(yaml-cpp 0.6 REQUIRED)
  set(YAMLCPP_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})
  set(YAMLCPP_LIBRARIES ${YAML_CPP_LIBRARIES})
  add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
else()
  if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
  endif()
  link_directories(${YAMLCPP_LIBRARY_DIRS})
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mcl3d_ros
#  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
)

add_executable(mcl src/mcl_node.cpp src/mcl.cpp src/distance_field.cpp src/imu.cpp)
target_link_libraries(mcl ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(pc_to_df src/pc_to_df.cpp src/distance_field.cpp)
target_link_libraries(pc_to_df ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(sensor_points_merger src/sensor_points_merger.cpp)
target_link_libraries(sensor_points_merger ${catkin_LIBRARIES} ${YAMLCPP_LIBRARIES})

