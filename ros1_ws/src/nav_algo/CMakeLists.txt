cmake_minimum_required(VERSION 3.0.2)
project(nav_algo)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs tf pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# terrain_analysis_node
add_executable(terrain_analysis_node src/terrain_analysis_node.cpp)
target_link_libraries(terrain_analysis_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# local_planner_node
add_executable(local_planner_node src/local_planner_node.cpp)
target_link_libraries(local_planner_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# path_follower_node
add_executable(path_follower_node src/path_follower_node.cpp)
target_link_libraries(path_follower_node ${catkin_LIBRARIES})

install(TARGETS terrain_analysis_node local_planner_node path_follower_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
