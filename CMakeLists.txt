cmake_minimum_required(VERSION 3.0.2)
project(lidar_accumulator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## catkin specific configuration ##
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy pcl_ros sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(lidar_accumulator_node src/lidar_accumulator_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(lidar_accumulator_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS lidar_accumulator_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/lidar_accumulator.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Add the msgs directory to generate_messages dependencies
add_dependencies(lidar_accumulator_node ${catkin_EXPORTED_TARGETS})
