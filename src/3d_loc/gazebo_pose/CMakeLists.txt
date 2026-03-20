cmake_minimum_required(VERSION 3.0.2)
project(gazebo_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  gazebo_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    gazebo_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ugv_pose_from_gz_exe src/ugv_pose_from_gz.cpp)

add_dependencies(ugv_pose_from_gz_exe
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ugv_pose_from_gz_exe
  ${catkin_LIBRARIES}
)