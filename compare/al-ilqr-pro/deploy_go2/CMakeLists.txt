cmake_minimum_required(VERSION 3.16.3)
project(al_ilqr_go2)

# ─── Default build type: Release ──────────────────────────────────
# iLQR/AL 求解器重度依赖 Eigen 模板运算，未优化(-O0)时单次求解可达
# 秒级（NX 上实测 ~2200ms）；Release(-O3 -DNDEBUG) 后降至 ~120ms。
# 若未显式指定 CMAKE_BUILD_TYPE（如 catkin build 默认），强制 Release，
# 避免意外的未优化构建导致实时性不足。
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build (Debug/Release/RelWithDebInfo/MinSizeRel)" FORCE)
endif()
message(STATUS "al_ilqr_go2 build type: ${CMAKE_BUILD_TYPE}")

# ─── C++ standard ───────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ─── Catkin dependencies ───────────────────────────────────────────────────
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  eigen_conversions
)

# ─── Eigen3 ─────────────────────────────────────────────────────────────────
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ─── Catkin package declaration ────────────────────────────────────────────
catkin_package(
  INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  LIBRARIES
    al_ilqr_core
  CATKIN_DEPENDS
    roscpp std_msgs nav_msgs geometry_msgs sensor_msgs
    tf2 tf2_ros tf2_geometry_msgs pcl_ros eigen_conversions
  DEPENDS
    EIGEN3
)

# ─── AL-iLQR core library (fully self-contained, no ../ references) ────────
# Only the core solver/dynamics/constraint sources are compiled; all GUI code
# (ImGui, GLFW, OpenGL, rerun) and apps/ are excluded — no OpenGL/GLFW deps.
set(AL_ILQR_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_library(al_ilqr_core STATIC
  ${AL_ILQR_SRC_DIR}/core/trajectory.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/multi_circle_vehicle_obstacle_constraint.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/straight_reference_line.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/speed_limit_constraint.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/vehicle_bicycle_config.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/vehicle_circle_approximation.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/global_guidance_planner.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/guidance_tracking_cost.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/lane_tracking_cost.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/road_boundary_constraint.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/multi_circle_road_boundary_constraint.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/circular_obstacle_constraint.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/convex_polygon.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/lse_polygon_obstacle_constraint.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/polygon_collision_constraint.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/lattice_initial_guess.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/sl_dp_planner.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/guidance_tracker.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/control_rate_cost.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/pure_pursuit_tracker.cpp
  ${AL_ILQR_SRC_DIR}/autodrive/demo_scenario.cpp
  ${AL_ILQR_SRC_DIR}/dynamics/linear_point_mass_model.cpp
  ${AL_ILQR_SRC_DIR}/dynamics/unicycle_model.cpp
  ${AL_ILQR_SRC_DIR}/dynamics/dynamics_model.cpp
  ${AL_ILQR_SRC_DIR}/dynamics/kinematic_bicycle_model.cpp
  ${AL_ILQR_SRC_DIR}/cost/cost_function.cpp
  ${AL_ILQR_SRC_DIR}/cost/composite_cost_function.cpp
  ${AL_ILQR_SRC_DIR}/cost/quadratic_cost.cpp
  ${AL_ILQR_SRC_DIR}/constraints/constraint_function.cpp
  ${AL_ILQR_SRC_DIR}/constraints/control_box_constraint.cpp
  ${AL_ILQR_SRC_DIR}/constraints/terminal_goal_constraint.cpp
  ${AL_ILQR_SRC_DIR}/al/al_ilqr_solver.cpp
  ${AL_ILQR_SRC_DIR}/al/augmented_lagrangian_cost.cpp
  ${AL_ILQR_SRC_DIR}/problems/optimal_control_problem.cpp
  ${AL_ILQR_SRC_DIR}/problems/constrained_optimal_control_problem.cpp
  ${AL_ILQR_SRC_DIR}/lqr/finite_horizon_lqr_solver.cpp
  ${AL_ILQR_SRC_DIR}/ilqr/ilqr_solver.cpp
  ${AL_ILQR_SRC_DIR}/visualization/al_ilqr_log_writer.cpp
  ${AL_ILQR_SRC_DIR}/visualization/obstacle_prediction_writer.cpp
  ${AL_ILQR_SRC_DIR}/visualization/trajectory_writer.cpp
)

target_include_directories(al_ilqr_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# MY_AL_ILQR_SOURCE_DIR is used at runtime to locate config/vehicle_bicycle_config.ini.
# Point it at this self-contained package directory so it resolves deploy_go2/config/.
target_compile_definitions(al_ilqr_core
  PUBLIC
    MY_AL_ILQR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(al_ilqr_core
  PUBLIC
    Eigen3::Eigen
)

# ─── ROS node ──────────────────────────────────────────────────────────────
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(al_ilqr_go2_node
  src/al_ilqr_ros_node.cpp
)

target_link_libraries(al_ilqr_go2_node
  ${catkin_LIBRARIES}
  al_ilqr_core
)

# ─── Install ───────────────────────────────────────────────────────────────
install(TARGETS al_ilqr_core al_ilqr_go2_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
