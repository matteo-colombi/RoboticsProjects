cmake_minimum_required(VERSION 3.0.2)
project(second_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  move_base
  move_base_msgs
  tf2_ros
  actionlib
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_tf_publisher src/odom_tf_publisher.cpp)
target_link_libraries(odom_tf_publisher ${catkin_LIBRARIES})
add_executable(pointcloud_remap src/pointcloud_remap.cpp)
target_link_libraries(pointcloud_remap ${catkin_LIBRARIES})
add_executable(goal_publisher src/goal_publisher.cpp)
target_link_libraries(goal_publisher ${catkin_LIBRARIES})