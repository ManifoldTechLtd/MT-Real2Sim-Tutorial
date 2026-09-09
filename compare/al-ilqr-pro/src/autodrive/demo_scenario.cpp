#include "autodrive/demo_scenario.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "autodrive/convex_polygon.hpp"
#include "autodrive/guidance_tracking_cost.hpp"
#include "autodrive/lattice_initial_guess.hpp"
#include "autodrive/lane_tracking_cost.hpp"
#include "autodrive/lse_polygon_obstacle_constraint.hpp"
#include "autodrive/multi_circle_vehicle_obstacle_constraint.hpp"
#include "autodrive/multi_circle_road_boundary_constraint.hpp"
#include "autodrive/polygon_collision_constraint.hpp"
#include "autodrive/road_boundary_constraint.hpp"
#include "autodrive/speed_limit_constraint.hpp"
#include "autodrive/vehicle_bicycle_config.hpp"
#include "constraints/control_box_constraint.hpp"
#include "cost/composite_cost_function.hpp"
#include "dynamics/kinematic_bicycle_model.hpp"

namespace my_al_ilqr {

namespace {

Vector ObstacleCenter(double x, double y) {
  Vector center(2);
  center << x, y;
  return center;
}

AutodriveDemoScenario CreateScenarioSkeleton(const DynamicObstacleScenarioConfig& config,
                                            const VehicleBicycleConfig& vehicle_config,
                                            const StaticObstacleScenarioConfig* static_guidance_config = nullptr,
                                            const std::vector<GuidancePoint>* selected_guidance = nullptr) {
  AutodriveDemoScenario scenario;
  scenario.vehicle_config = vehicle_config;
  scenario.vehicle_geometry = VehicleCircleApproximation(vehicle_config.body);
  const double ref_y = static_guidance_config ? static_guidance_config->ref_origin_y : 0.0;
  scenario.reference_line = std::make_shared<StraightReferenceLine>(0.0, ref_y, 0.0);
  scenario.road_lower_bound = -config.road_half_width - ref_y;
  scenario.road_upper_bound = config.road_half_width - ref_y;
  scenario.obstacle_radius = config.obstacle_radius;
  scenario.vehicle_length = vehicle_config.body.length;
  scenario.vehicle_width = vehicle_config.body.width;
  scenario.rear_axle_to_rear = vehicle_config.body.rear_axle_to_rear;

  auto dynamics = std::make_shared<KinematicBicycleModel>(vehicle_config.model.wheelbase);
  auto lane_cost = std::make_shared<LaneTrackingCost>(scenario.reference_line,
                                                      config.target_speed,
                                                      config.target_longitudinal_position,
                                                      config.stage_lateral_weight,
                                                      config.stage_heading_weight,
                                                      config.stage_speed_weight,
                                                      config.stage_accel_weight,
                                                      config.stage_steering_weight,
                                                      config.terminal_longitudinal_weight,
                                                      config.terminal_lateral_weight,
                                                      config.terminal_heading_weight,
                                                      config.terminal_speed_weight);

  const int horizon = static_guidance_config != nullptr
      ? static_guidance_config->planning_horizon : 80;
  const double dt = static_guidance_config != nullptr
      ? static_guidance_config->planning_dt : 0.1;
  const bool use_guidance_tracking =
      static_guidance_config != nullptr &&
      static_guidance_config->use_guidance_tracking &&
      selected_guidance != nullptr &&
      !selected_guidance->empty();

  const double jerk_w = static_guidance_config ? static_guidance_config->jerk_weight : 0.0;
  const double steer_rate_w = static_guidance_config ? static_guidance_config->steering_rate_weight : 0.0;
  const bool use_rate_cost = jerk_w > 1e-9 || steer_rate_w > 1e-9;

  if (use_rate_cost) {
    scenario.control_rate_costs.reserve(horizon);
    for (int k = 0; k < horizon; ++k) {
      scenario.control_rate_costs.push_back(
          std::make_shared<ControlRateCost>(jerk_w, steer_rate_w));
    }
  }

  if (use_guidance_tracking || use_rate_cost) {
    std::vector<std::shared_ptr<CostFunction>> stage_costs;
    stage_costs.reserve(horizon);
    for (int k = 0; k < horizon; ++k) {
      std::vector<std::shared_ptr<CostFunction>> terms;
      terms.push_back(lane_cost);
      if (use_guidance_tracking) {
        const GuidancePoint& point =
            (*selected_guidance)[std::min(k, static_cast<int>(selected_guidance->size()) - 1)];
        terms.push_back(std::make_shared<GuidanceTrackingCost>(
            point,
            static_guidance_config->guidance_position_weight,
            static_guidance_config->guidance_heading_weight,
            static_guidance_config->guidance_speed_weight,
            false));
      }
      if (use_rate_cost) {
        terms.push_back(scenario.control_rate_costs[k]);
      }
      stage_costs.push_back(std::make_shared<CompositeCostFunction>(std::move(terms)));
    }

    std::vector<std::shared_ptr<CostFunction>> terminal_terms;
    terminal_terms.push_back(lane_cost);
    if (use_guidance_tracking) {
      const GuidancePoint& terminal_point =
          (*selected_guidance)[std::min(horizon, static_cast<int>(selected_guidance->size()) - 1)];
      terminal_terms.push_back(std::make_shared<GuidanceTrackingCost>(
          terminal_point,
          static_guidance_config->guidance_terminal_position_weight,
          0.0, 0.0, true, scenario.reference_line, true));
    }
    auto terminal_cost =
        std::make_shared<CompositeCostFunction>(std::move(terminal_terms));

    scenario.base_problem = std::make_shared<OptimalControlProblem>(
        dynamics, std::move(stage_costs), terminal_cost, dt);
  } else {
    scenario.base_problem = std::make_shared<OptimalControlProblem>(dynamics, lane_cost, horizon, dt);
  }
  scenario.constrained_problem =
      std::make_shared<ConstrainedOptimalControlProblem>(scenario.base_problem);

  Vector u_lb(2);
  u_lb << vehicle_config.model.min_acceleration, vehicle_config.model.min_steering;
  Vector u_ub(2);
  u_ub << vehicle_config.model.max_acceleration, vehicle_config.model.max_steering;
  for (int k = 0; k < scenario.base_problem->Horizon(); ++k) {
    scenario.constrained_problem->AddStageConstraint(
        k, std::make_shared<ControlBoxConstraint>(scenario.base_problem->StateDim(), u_lb, u_ub));
    scenario.constrained_problem->AddStageConstraint(
        k, std::make_shared<MultiCircleRoadBoundaryConstraint>(
               scenario.reference_line, scenario.vehicle_geometry,
               scenario.road_lower_bound, scenario.road_upper_bound));
    scenario.constrained_problem->AddStageConstraint(
        k, std::make_shared<SpeedLimitConstraint>(vehicle_config.model.min_speed,
                                                  vehicle_config.model.max_speed));
  }

  scenario.initial_state = Vector(4);
  scenario.initial_state << 0.0, 0.0, 0.0, 2.5;

  ILQROptions inner_options;
  inner_options.max_iterations = 35;
  inner_options.cost_tolerance = 1e-5;
  inner_options.derivative_epsilon = 1e-4;
  inner_options.regularization_init = 1e-5;
  inner_options.regularization_min = 1e-8;
  inner_options.regularization_max = 1e5;
  inner_options.regularization_increase_factor = 10.0;
  inner_options.regularization_decrease_factor = 5.0;
  inner_options.line_search_max_iterations = 10;

  scenario.solver_options.inner_options = inner_options;
  scenario.solver_options.max_outer_iterations = 18;
  scenario.solver_options.constraint_tolerance = 1e-1;
  scenario.solver_options.initial_penalty = 20.0;
  scenario.solver_options.penalty_scaling = 5.0;
  scenario.solver_options.penalty_update_ratio = 0.4;
  scenario.solver_options.max_penalty = 1e8;

  return scenario;
}

void AddObstacleConstraints(AutodriveDemoScenario* scenario) {
  if (!scenario) {
    return;
  }
  if (scenario->obstacle_centers.empty()) {
    return;
  }
  if (static_cast<int>(scenario->obstacle_centers.size()) < scenario->base_problem->Horizon()) {
    throw std::invalid_argument("Obstacle prediction must provide at least one center per stage.");
  }
  for (int k = 0; k < scenario->base_problem->Horizon(); ++k) {
    const double cx = scenario->obstacle_centers[k](0);
    const double cy = scenario->obstacle_centers[k](1);

    switch (scenario->obstacle_type) {
      case ObstacleType::kCircle:
        scenario->constrained_problem->AddStageConstraint(
            k, std::make_shared<MultiCircleVehicleObstacleConstraint>(
                   scenario->vehicle_geometry, cx, cy, scenario->obstacle_radius));
        break;

      case ObstacleType::kPolygonLSE: {
        auto polygon = MakeRectangle(cx, cy, scenario->obstacle_half_length,
                                     scenario->obstacle_half_width,
                                     scenario->obstacle_yaw);
        scenario->constrained_problem->AddStageConstraint(
            k, std::make_shared<LSEPolygonObstacleConstraint>(
                   scenario->vehicle_geometry, std::move(polygon),
                   scenario->lse_alpha));
        break;
      }

      case ObstacleType::kPolygonExact: {
        auto polygon = MakeRectangle(cx, cy, scenario->obstacle_half_length,
                                     scenario->obstacle_half_width,
                                     scenario->obstacle_yaw);
        scenario->constrained_problem->AddStageConstraint(
            k, std::make_shared<PolygonCollisionConstraint>(
                   scenario->vehicle_config.body.length,
                   scenario->vehicle_config.body.width,
                   scenario->vehicle_config.body.rear_axle_to_rear,
                   std::move(polygon), 0.1));
        break;
      }
    }
  }
}

}  // namespace

AutodriveDemoScenario CreateAutodriveDemoScenario() {
  return CreateSingleStaticObstacleTestScenario();
}

AutodriveDemoScenario CreateSingleStaticObstacleTestScenario() {
  const VehicleBicycleConfig vehicle_config =
      LoadVehicleBicycleConfig(DefaultVehicleBicycleConfigPath());
  StaticObstacleScenarioConfig config;
  return CreateSingleStaticObstacleTestScenario(vehicle_config, config, nullptr);
}

AutodriveDemoScenario CreateSingleStaticObstacleTestScenario(
    const VehicleBicycleConfig& vehicle_config,
    const StaticObstacleScenarioConfig& static_config) {
  return CreateSingleStaticObstacleTestScenario(vehicle_config, static_config, nullptr);
}

AutodriveDemoScenario CreateSingleStaticObstacleTestScenario(
    const VehicleBicycleConfig& vehicle_config,
    const StaticObstacleScenarioConfig& static_config,
    const std::vector<GuidancePoint>* selected_guidance) {
  DynamicObstacleScenarioConfig config;
  config.obstacle_center_x = static_config.obstacle_center_x;
  config.obstacle_initial_y = static_config.obstacle_center_y;
  config.obstacle_speed_y = 0.0;
  config.road_half_width = static_config.road_half_width;
  config.obstacle_radius = static_config.obstacle_radius;
  config.target_speed = static_config.target_speed;
  config.target_longitudinal_position = static_config.target_longitudinal_position;
  config.stage_lateral_weight = static_config.stage_lateral_weight;
  config.stage_heading_weight = static_config.stage_heading_weight;
  config.stage_speed_weight = static_config.stage_speed_weight;
  config.stage_accel_weight = static_config.stage_accel_weight;
  config.stage_steering_weight = static_config.stage_steering_weight;
  config.terminal_longitudinal_weight = static_config.terminal_longitudinal_weight;
  config.terminal_lateral_weight = static_config.terminal_lateral_weight;
  config.terminal_heading_weight = static_config.terminal_heading_weight;
  config.terminal_speed_weight = static_config.terminal_speed_weight;
  AutodriveDemoScenario scenario =
      CreateScenarioSkeleton(config, vehicle_config, &static_config, selected_guidance);
  scenario.solver_options.inner_options.derivative_mode = static_config.derivative_mode;
  scenario.obstacle_center_x = static_config.obstacle_center_x;
  scenario.obstacle_center_y = static_config.obstacle_center_y;
  scenario.obstacle_type = static_config.obstacle_type;
  scenario.obstacle_half_length = static_config.obstacle_half_length;
  scenario.obstacle_half_width = static_config.obstacle_half_width;
  scenario.obstacle_yaw = static_config.obstacle_yaw;
  scenario.lse_alpha = static_config.lse_alpha;
  {
    const int H = scenario.base_problem->Horizon();
    const double dt = scenario.base_problem->TimeStep();
    scenario.obstacle_centers.resize(H + 1);
    for (int k = 0; k <= H; ++k) {
      scenario.obstacle_centers[k] = ObstacleCenter(
          scenario.obstacle_center_x + static_config.obstacle_speed_x * k * dt,
          scenario.obstacle_center_y);
    }
  }
  AddObstacleConstraints(&scenario);

  scenario.initial_state << static_config.initial_x,
      static_config.initial_y,
      static_config.initial_yaw,
      static_config.initial_speed;

  if (selected_guidance != nullptr) {
    scenario.selected_guidance = *selected_guidance;
  }

  const int H = scenario.base_problem->Horizon();
  scenario.initial_controls.assign(H, Vector::Zero(2));
  for (int k = 0; k < H; ++k) {
    scenario.initial_controls[k](0) = (k < 8) ? 0.2 : 0.0;
    if (k < 15) {
      scenario.initial_controls[k](1) = 0.28;
    } else if (k < 30) {
      scenario.initial_controls[k](1) = -0.24;
    } else {
      scenario.initial_controls[k](1) = 0.08;
    }
  }

  return scenario;
}

AutodriveDemoScenario CreateDynamicObstacleDemoScenario() {
  DynamicObstacleScenarioConfig config;
  config.obstacle_radius = 0.75;
  config.road_half_width = 4.5;
  config.stage_lateral_weight = 1.5;
  config.terminal_lateral_weight = 16.0;
  config.initial_steering_guess = 0.04;
  return CreateDynamicObstacleDemoScenario(config);
}

AutodriveDemoScenario CreateDynamicObstacleDemoScenario(const DynamicObstacleScenarioConfig& config) {
  const VehicleBicycleConfig vehicle_config =
      LoadVehicleBicycleConfig(DefaultVehicleBicycleConfigPath());
  AutodriveDemoScenario scenario = CreateScenarioSkeleton(config, vehicle_config, nullptr, nullptr);
  scenario.obstacle_center_x = config.obstacle_center_x;
  scenario.obstacle_center_y = config.obstacle_initial_y;
  scenario.obstacle_centers.reserve(scenario.base_problem->Horizon() + 1);
  for (int k = 0; k <= scenario.base_problem->Horizon(); ++k) {
    const double time = k * scenario.base_problem->TimeStep();
    const double center_x = config.obstacle_center_x;
    const double center_y = config.obstacle_initial_y + config.obstacle_speed_y * time;
    scenario.obstacle_centers.push_back(ObstacleCenter(center_x, center_y));
  }
  AddObstacleConstraints(&scenario);

  scenario.initial_controls.assign(scenario.base_problem->Horizon(), Vector::Zero(2));
  for (int k = 0; k < scenario.base_problem->Horizon(); ++k) {
    scenario.initial_controls[k](0) = (k < config.accel_guess_steps) ? config.initial_accel_guess : 0.0;
    scenario.initial_controls[k](1) = config.initial_steering_guess;
  }

  scenario.solver_options.max_outer_iterations = 14;
  scenario.solver_options.constraint_tolerance = 1e-1;
  scenario.solver_options.initial_penalty = 15.0;
  scenario.solver_options.penalty_scaling = 5.0;
  scenario.solver_options.penalty_update_ratio = 0.45;
  scenario.solver_options.max_outer_iterations = 18;

  return scenario;
}

AutodriveDemoScenario CreateDynamicObstacleSpeedStudyScenario(double obstacle_speed_y) {
  DynamicObstacleScenarioConfig config;
  config.obstacle_initial_y = -0.35;
  config.obstacle_speed_y = obstacle_speed_y;
  config.obstacle_radius = 0.7;
  config.road_half_width = 4.5;
  config.stage_lateral_weight = 9.0;
  config.stage_heading_weight = 1.0;
  config.stage_speed_weight = 1.2;
  config.terminal_lateral_weight = 28.0;
  config.terminal_speed_weight = 10.0;
  config.initial_accel_guess = 0.0;
  config.accel_guess_steps = 0;
  config.initial_steering_guess = 0.0;
  return CreateDynamicObstacleDemoScenario(config);
}

}  // namespace my_al_ilqr
