cmake_minimum_required(VERSION 3.0.2)
project(antenna)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)



catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS 
  roscpp
  rospy 
  std_msgs
  sensor_msgs
  message_runtime
  LIBRARIES human_control_interface
)

catkin_install_python(PROGRAMS 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
