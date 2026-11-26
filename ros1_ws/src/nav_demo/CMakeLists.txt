cmake_minimum_required(VERSION 3.0.2)
project(nav_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/teleop_keyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
