cmake_minimum_required(VERSION 2.8.3)
project(vehicle_sim_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  gazebo_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    gazebo_msgs
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(vehicle_input_subscriber src/vehicle_input_subscriber.cpp)
add_dependencies(vehicle_input_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_input_subscriber ${catkin_LIBRARIES})

add_executable(vehicle_info_publisher src/vehicle_info_publisher.cpp)
add_dependencies(vehicle_info_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_info_publisher ${catkin_LIBRARIES})

install(TARGETS vehicle_input_subscriber vehicle_info_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
