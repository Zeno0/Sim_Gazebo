cmake_minimum_required(VERSION 2.8.3)
project(vehicle_sim_launcher)

find_package(catkin REQUIRED
  urdf_tutorial
  controller_manager
  controller_manager
  robot_state_publisher
  joint_state_controller
)

catkin_package(
  CATKIN_DEPENDS
    urdf_tutorial
    controller_manager
    robot_state_publisher
    joint_state_controller
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
