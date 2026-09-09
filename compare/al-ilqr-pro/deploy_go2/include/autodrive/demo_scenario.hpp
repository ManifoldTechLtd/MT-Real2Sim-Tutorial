#pragma once

#include <memory>
#include <vector>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/control_rate_cost.hpp"
#include "ilqr/ilqr_solver.hpp"  // for DerivativeMode
#include "autodrive/global_guidance_planner.hpp"
#include "autodrive/lattice_initial_guess.hpp"
#include "autodrive/vehicle_bicycle_config.hpp"
#include "autodrive/vehicle_circle_approximation.hpp"
#include "autodrive/straight_reference_line.hpp"
#include "core/types.hpp"
#include "problems/constrained_optimal_control_problem.hpp"
#include "problems/optimal_control_problem.hpp"

namespace my_al_ilqr {

enum class InitGuessMode {
  kNone = 0,
  kGlobalGuidance = 1,
  kSLDP = 2,
};

enum class InitControlsMode {
  kPlanner = 0,
  kPP = 1,
  kPPILQR = 2,
};

enum class ScenarioPreset {
  kStraightRoad = 0,
  kStaggeredObstacles = 1,
  kOvertaking = 2,
  kIntersection = 3,
  kCarFollowing = 4,
};

enum class ObstacleType {
  kCircle = 0,
  kPolygonLSE = 1,
  kPolygonExact = 2,
};

struct DynamicObstacleScenarioConfig {
  double obstacle_center_x = 6.0;
  double obstacle_initial_y = -2.0;
  double obstacle_speed_y = 1.0;
  double obstacle_radius = 0.9;
  double road_half_width = 4.5;
  double target_speed = 5.0;
  double target_longitudinal_position = 12.0;
  double stage_lateral_weight = 3.0;
  double stage_heading_weight = 0.5;
  double stage_speed_weight = 0.5;
  double stage_accel_weight = 0.2;
  double stage_steering_weight = 0.4;
  double terminal_longitudinal_weight = 20.0;
  double terminal_lateral_weight = 30.0;
  double terminal_heading_weight = 8.0;
  double terminal_speed_weight = 8.0;
  double initial_accel_guess = 0.15;
  int accel_guess_steps = 10;
  double initial_steering_guess = 0.0;
};

struct CrossRoadConfig {
  bool enabled = false;
  double center_x = 0.0;
  double half_width = 4.0;
};

struct StaticObstacleScenarioConfig {
  ObstacleType obstacle_type = ObstacleType::kPolygonLSE;
  double obstacle_center_x = 6.0;
  double obstacle_center_y = 0.0;
  double obstacle_radius = 0.8;
  double obstacle_half_length = 2.3;
  double obstacle_half_width = 0.9;
  double obstacle_yaw = 0.0;
  double lse_alpha = 20.0;
  double obstacle_speed_x = 0.0;
  double road_half_width = 4.5;
  double target_speed = 5.0;
  double target_longitudinal_position = 12.0;
  int planning_horizon = 80;
  double planning_dt = 0.1;
  double initial_x = 0.0;
  double initial_y = 0.0;
  double initial_yaw = 0.0;
  double initial_speed = 2.5;
  InitGuessMode init_guess_mode = InitGuessMode::kGlobalGuidance;
  InitControlsMode init_controls_mode = InitControlsMode::kPPILQR;
  bool use_guidance_tracking = true;

  double ref_origin_y = 0.0;
  int num_lanes = 1;
  double lane_width = 3.5;
  CrossRoadConfig cross_road;
  bool enable_obstacle_recycling = true;

  // SL-DP planner parameters
  int dp_layers = 4;
  int dp_lateral_samples = 7;
  double dp_lateral_span = 2.5;
  int dp_speed_samples = 3;
  double dp_max_speed_ratio = 1.2;
  int dp_max_lateral_jump = 2;
  double dp_lateral_weight = 2.0;
  double dp_lateral_change_weight = 3.0;
  double dp_heading_weight = 2.0;
  double dp_speed_weight = 1.0;
  double dp_accel_weight = 0.5;
  double dp_steering_weight = 0.5;
  double dp_path_length_weight = 0.2;
  double dp_collision_penalty = 1.0e4;
  double dp_proximity_buffer = 2.0;
  double dp_proximity_weight = 15.0;
  double dp_anticipation_distance = 15.0;
  double dp_anticipation_lateral_clearance = 1.8;
  double dp_anticipation_weight = 12.0;

  // Global guidance planner parameters
  int guidance_search_layers = 2;
  int guidance_search_lateral_samples = 5;
  double guidance_search_lateral_span = 2.5;
  double guidance_search_max_speed_ratio = 1.2;
  double guidance_search_lateral_weight = 2.0;
  double guidance_search_speed_weight = 1.0;
  double guidance_search_accel_weight = 0.3;
  double guidance_search_steering_weight = 0.5;
  double guidance_search_path_length_weight = 0.2;
  double guidance_search_collision_penalty = 1.0e4;
  double guidance_search_proximity_buffer = 1.2;
  double guidance_search_proximity_weight = 12.0;
  double guidance_search_follow_lateral_band = 0.9;
  double guidance_search_follow_base_gap = 6.0;
  double guidance_search_follow_headway = 1.2;
  double guidance_search_follow_penalty = 16.0;
  double guidance_search_bypass_lateral_clearance = 1.6;
  double guidance_search_bypass_anticipation_distance = 16.0;
  double guidance_search_bypass_anticipation_weight = 18.0;
  double guidance_search_post_bypass_hold_distance = 12.0;
  double guidance_search_post_bypass_hold_weight = 12.0;
  double guidance_search_consistency_penalty = 8.0;
  double guidance_search_consistency_geometry_weight = 0.15;
  double stage_lateral_weight = 0.3;
  double stage_heading_weight = 0.5;
  double stage_speed_weight = 0.5;
  double stage_accel_weight = 0.2;
  double stage_steering_weight = 1.5;
  double jerk_weight = 2.0;
  double steering_rate_weight = 3.0;
  double terminal_longitudinal_weight = 5.0;
  double terminal_lateral_weight = 1.0;
  double terminal_heading_weight = 1.0;
  double terminal_speed_weight = 2.0;
  double guidance_position_weight = 5.0;
  double guidance_heading_weight = 2.0;
  double guidance_speed_weight = 1.0;
  double guidance_terminal_position_weight = 15.0;
  double guidance_terminal_heading_weight = 5.0;
  double guidance_terminal_speed_weight = 3.0;

  // Derivative computation mode
  DerivativeMode derivative_mode = DerivativeMode::kFiniteDifference;
};

struct AutodriveDemoScenario {
  std::shared_ptr<StraightReferenceLine> reference_line;
  std::shared_ptr<OptimalControlProblem> base_problem;
  std::shared_ptr<ConstrainedOptimalControlProblem> constrained_problem;
  VehicleBicycleConfig vehicle_config;
  VehicleCircleApproximation vehicle_geometry{VehicleBodyConfig{}};
  Vector initial_state;
  std::vector<Vector> initial_controls;
  ALILQROptions solver_options;
  double road_lower_bound = 0.0;
  double road_upper_bound = 0.0;
  double obstacle_center_x = 0.0;
  double obstacle_center_y = 0.0;
  double obstacle_radius = 0.0;
  std::vector<Vector> obstacle_centers;
  double vehicle_length = 0.0;
  double vehicle_width = 0.0;
  double rear_axle_to_rear = 0.0;
  ObstacleType obstacle_type = ObstacleType::kPolygonLSE;
  double obstacle_half_length = 2.3;
  double obstacle_half_width = 0.9;
  double obstacle_yaw = 0.0;
  double lse_alpha = 20.0;
  std::vector<GuidanceCandidate> guidance_candidates;
  std::vector<GuidancePoint> selected_guidance;
  int selected_guidance_topology_id = -1;
  std::vector<LatticeCandidate> lattice_candidates;
  std::vector<DPLatticeNode> dp_lattice_nodes;
  std::vector<std::shared_ptr<ControlRateCost>> control_rate_costs;
};

AutodriveDemoScenario CreateAutodriveDemoScenario();
AutodriveDemoScenario CreateSingleStaticObstacleTestScenario();
AutodriveDemoScenario CreateSingleStaticObstacleTestScenario(
    const VehicleBicycleConfig& vehicle_config,
    const StaticObstacleScenarioConfig& config);
AutodriveDemoScenario CreateSingleStaticObstacleTestScenario(
    const VehicleBicycleConfig& vehicle_config,
    const StaticObstacleScenarioConfig& config,
    const std::vector<GuidancePoint>* selected_guidance);
AutodriveDemoScenario CreateDynamicObstacleDemoScenario();
AutodriveDemoScenario CreateDynamicObstacleDemoScenario(const DynamicObstacleScenarioConfig& config);
AutodriveDemoScenario CreateDynamicObstacleSpeedStudyScenario(double obstacle_speed_y);

}  // namespace my_al_ilqr
