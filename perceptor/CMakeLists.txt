cmake_minimum_required(VERSION 2.8.3)
project(perceptor)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  message_generation
  sensor_msgs
  cv_bridge
)

add_service_files(
    DIRECTORY srv
    FILES
    #MoveJoints.srv
)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS ./scripts/ros_interface.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)