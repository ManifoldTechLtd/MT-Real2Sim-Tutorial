cmake_minimum_required(VERSION 3.16.3)
project(al_ilqr_ros)

# ─── C++ standard (match upstream) ───────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ─── Catkin dependencies ─────────────────────────────────────────────────────
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  pcl_ros
  eigen_conversions
)

# ─── Eigen3 (upstream uses NO_MODULE) ────────────────────────────────────────
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ─── Catkin package declaration ─────────────────────────────────────────────
catkin_package(
  INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
  LIBRARIES
    al_ilqr_core
  CATKIN_DEPENDS
    roscpp std_msgs nav_msgs geometry_msgs sensor_msgs
    tf2 tf2_ros pcl_ros eigen_conversions
  DEPENDS
    EIGEN3
)

# ─── AL-iLQR core library ────────────────────────────────────────────────────
# Reference the upstream al-ilqr-pro source tree without modifying it.
# Only the core solver/dynamics/constraint sources are compiled; all GUI code
# (ImGui, GLFW, OpenGL, rerun) and apps/ are excluded.
set(AL_ILQR_PRO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

add_library(al_ilqr_core STATIC
  ${AL_ILQR_PRO_DIR}/src/core/trajectory.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/multi_circle_vehicle_obstacle_constraint.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/straight_reference_line.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/speed_limit_constraint.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/vehicle_bicycle_config.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/vehicle_circle_approximation.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/global_guidance_planner.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/guidance_tracking_cost.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/lane_tracking_cost.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/road_boundary_constraint.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/multi_circle_road_boundary_constraint.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/circular_obstacle_constraint.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/convex_polygon.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/lse_polygon_obstacle_constraint.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/polygon_collision_constraint.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/lattice_initial_guess.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/sl_dp_planner.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/guidance_tracker.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/control_rate_cost.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/pure_pursuit_tracker.cpp
  ${AL_ILQR_PRO_DIR}/src/autodrive/demo_scenario.cpp
  ${AL_ILQR_PRO_DIR}/src/dynamics/linear_point_mass_model.cpp
  ${AL_ILQR_PRO_DIR}/src/dynamics/unicycle_model.cpp
  ${AL_ILQR_PRO_DIR}/src/dynamics/dynamics_model.cpp
  ${AL_ILQR_PRO_DIR}/src/dynamics/kinematic_bicycle_model.cpp
  ${AL_ILQR_PRO_DIR}/src/cost/cost_function.cpp
  ${AL_ILQR_PRO_DIR}/src/cost/composite_cost_function.cpp
  ${AL_ILQR_PRO_DIR}/src/cost/quadratic_cost.cpp
  ${AL_ILQR_PRO_DIR}/src/constraints/constraint_function.cpp
  ${AL_ILQR_PRO_DIR}/src/constraints/control_box_constraint.cpp
  ${AL_ILQR_PRO_DIR}/src/constraints/terminal_goal_constraint.cpp
  ${AL_ILQR_PRO_DIR}/src/al/al_ilqr_solver.cpp
  ${AL_ILQR_PRO_DIR}/src/al/augmented_lagrangian_cost.cpp
  ${AL_ILQR_PRO_DIR}/src/problems/optimal_control_problem.cpp
  ${AL_ILQR_PRO_DIR}/src/problems/constrained_optimal_control_problem.cpp
  ${AL_ILQR_PRO_DIR}/src/lqr/finite_horizon_lqr_solver.cpp
  ${AL_ILQR_PRO_DIR}/src/ilqr/ilqr_solver.cpp
  ${AL_ILQR_PRO_DIR}/src/visualization/al_ilqr_log_writer.cpp
  ${AL_ILQR_PRO_DIR}/src/visualization/obstacle_prediction_writer.cpp
  ${AL_ILQR_PRO_DIR}/src/visualization/trajectory_writer.cpp
)

target_include_directories(al_ilqr_core
  PUBLIC
    ${AL_ILQR_PRO_DIR}/include
)

# MY_AL_ILQR_SOURCE_DIR is used at runtime to locate config/vehicle_bicycle_config.ini
target_compile_definitions(al_ilqr_core
  PUBLIC
    MY_AL_ILQR_SOURCE_DIR="${AL_ILQR_PRO_DIR}"
)

target_link_libraries(al_ilqr_core
  PUBLIC
    Eigen3::Eigen
)

# ─── ROS node ────────────────────────────────────────────────────────────────
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(al_ilqr_ros_node
  src/al_ilqr_ros_node.cpp
)

target_link_libraries(al_ilqr_ros_node
  ${catkin_LIBRARIES}
  al_ilqr_core
)

# ─── Install ─────────────────────────────────────────────────────────────────
install(TARGETS al_ilqr_core al_ilqr_ros_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
