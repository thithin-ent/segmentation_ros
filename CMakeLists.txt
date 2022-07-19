cmake_minimum_required(VERSION 3.0.2)
project(segmentation_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES segmentation_ros
  CATKIN_DEPENDS geometry_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)


# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/segmentation_ros.cpp
# )

add_executable(${PROJECT_NAME} src/segmentation_ros_node.cpp src/segmentation_ros.cpp)
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES} ${CERES_LIBRARIES}
 )


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
