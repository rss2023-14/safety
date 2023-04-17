cmake_minimum_required(VERSION 3.0.2)
project(safety)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)