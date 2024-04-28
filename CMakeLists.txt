cmake_minimum_required(VERSION 3.0.2)
project(lidar_manipulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_manipulator_node src/lidar_manipulator_node.cpp)
target_link_libraries(lidar_manipulator_node ${catkin_LIBRARIES})
