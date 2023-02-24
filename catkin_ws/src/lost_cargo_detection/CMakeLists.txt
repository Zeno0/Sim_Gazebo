cmake_minimum_required(VERSION 2.8.12)
project(lost_cargo_detection)

find_package(catkin REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  genmsg
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  roslint
  tf2_ros
  catkin
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sphero_move
CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)


find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

#add_executable(ground_plane_detection_node src/ground_plane_detection.cpp)
#target_link_libraries(ground_plane_detection_node ${catkin_LIBRARIES})

roslint_cpp()
roslint_add_test()
