cmake_minimum_required(VERSION 3.0.2)
project(odin_interfaces)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package()

# TODO: Add ODIN1 hardware driver nodes when available
