cmake_minimum_required(VERSION 2.8.3)
project(perceptor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  message_generation
  sensor_msgs
  geometry_msgs
  cv_bridge
)

add_service_files(
    DIRECTORY srv
    FILES
)

add_message_files(FILES Keypoint.msg Pose.msg Poses.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS ./backend/ros_interface.js 
                               ./backend/image_converter.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)