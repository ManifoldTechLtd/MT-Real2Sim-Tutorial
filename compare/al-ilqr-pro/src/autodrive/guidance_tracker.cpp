#include "autodrive/guidance_tracker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/multi_circle_road_boundary_constraint.hpp"
#include "autodrive/pure_pursuit_tracker.hpp"
#include "autodrive/speed_limit_constraint.hpp"
#include "constraints/control_box_constraint.hpp"
#include "cost/cost_function.hpp"
#include "dynamics/kinematic_bicycle_model.hpp"
#include "problems/constrained_optimal_control_problem.hpp"
#include "problems/optimal_control_problem.hpp"

namespace my_al_ilqr {

namespace {

double NormalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

class TrackingStageCost final : public CostFunction {
 public:
  TrackingStageCost(const GuidancePoint& ref,
                    double pos_w, double hdg_w, double spd_w,
                    double acc_w, double steer_w)
      : ref_(ref), pos_w_(pos_w), hdg_w_(hdg_w), spd_w_(spd_w),
        acc_w_(acc_w), steer_w_(steer_w) {}

  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }

  double StageCost(const Vector& state, const Vector& control) const override {
    const double dx = state(0) - ref_.x;
    const double dy = state(1) - ref_.y;
    const double he = NormalizeAngle(state(2) - ref_.yaw);
    const double se = state(3) - ref_.speed;
    return 0.5 * pos_w_ * (dx * dx + dy * dy) +
           0.5 * hdg_w_ * he * he +
           0.5 * spd_w_ * se * se +
           0.5 * acc_w_ * control(0) * control(0) +
           0.5 * steer_w_ * control(1) * control(1);
  }

  double TerminalCost(const Vector&) const override { return 0.0; }

 private:
  GuidancePoint ref_;
  double pos_w_, hdg_w_, spd_w_, acc_w_, steer_w_;
};

class TrackingTerminalCost final : public CostFunction {
 public:
  TrackingTerminalCost(const GuidancePoint& ref,
                       double pos_w, double hdg_w, double spd_w)
      : ref_(ref), pos_w_(pos_w), hdg_w_(hdg_w), spd_w_(spd_w) {}

  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }

  double StageCost(const Vector&, const Vector&) const override { return 0.0; }

  double TerminalCost(const Vector& state) const override {
    const double dx = state(0) - ref_.x;
    const double dy = state(1) - ref_.y;
    const double he = NormalizeAngle(state(2) - ref_.yaw);
    const double se = state(3) - ref_.speed;
    return 0.5 * pos_w_ * (dx * dx + dy * dy) +
           0.5 * hdg_w_ * he * he +
           0.5 * spd_w_ * se * se;
  }

 private:
  GuidancePoint ref_;
  double pos_w_, hdg_w_, spd_w_;
};

}  // namespace

GuidanceTracker::GuidanceTracker(const VehicleBicycleConfig& vehicle_config,
                                 double road_lower_bound,
                                 double road_upper_bound,
                                 const VehicleCircleApproximation& vehicle_geometry)
    : vehicle_config_(vehicle_config),
      road_lower_bound_(road_lower_bound),
      road_upper_bound_(road_upper_bound),
      vehicle_geometry_(vehicle_geometry) {}

GuidanceTrackerResult GuidanceTracker::Track(
    const Vector& initial_state,
    const std::vector<GuidancePoint>& guidance,
    const std::vector<Vector>& initial_controls,
    int horizon, double dt,
    const GuidanceTrackerConfig& config) const {
  GuidanceTrackerResult result;
  if (guidance.empty() || horizon <= 0) {
    return result;
  }

  auto dynamics = std::make_shared<KinematicBicycleModel>(
      vehicle_config_.model.wheelbase);

  const int gs = static_cast<int>(guidance.size());

  std::vector<std::shared_ptr<CostFunction>> stage_costs;
  stage_costs.reserve(horizon);
  for (int k = 0; k < horizon; ++k) {
    const auto& pt = guidance[std::min(k, gs - 1)];
    stage_costs.push_back(std::make_shared<TrackingStageCost>(
        pt, config.position_weight, config.heading_weight,
        config.speed_weight, config.accel_reg_weight,
        config.steering_reg_weight));
  }

  const auto& term_pt = guidance[std::min(horizon, gs - 1)];
  auto terminal_cost = std::make_shared<TrackingTerminalCost>(
      term_pt, config.terminal_position_weight,
      config.terminal_heading_weight, config.terminal_speed_weight);

  auto ocp = std::make_shared<OptimalControlProblem>(
      dynamics, std::move(stage_costs), terminal_cost, dt);
  auto constrained =
      std::make_shared<ConstrainedOptimalControlProblem>(ocp);

  Vector u_lb(2), u_ub(2);
  u_lb << vehicle_config_.model.min_acceleration,
      vehicle_config_.model.min_steering;
  u_ub << vehicle_config_.model.max_acceleration,
      vehicle_config_.model.max_steering;

  auto ref_line = std::make_shared<StraightReferenceLine>(0.0, 0.0, 0.0);

  for (int k = 0; k < horizon; ++k) {
    constrained->AddStageConstraint(
        k, std::make_shared<ControlBoxConstraint>(
               ocp->StateDim(), u_lb, u_ub));
    constrained->AddStageConstraint(
        k, std::make_shared<MultiCircleRoadBoundaryConstraint>(
               ref_line, vehicle_geometry_,
               road_lower_bound_, road_upper_bound_));
    constrained->AddStageConstraint(
        k, std::make_shared<SpeedLimitConstraint>(
               vehicle_config_.model.min_speed,
               vehicle_config_.model.max_speed));
  }

  ALILQROptions opts;
  opts.max_outer_iterations = config.max_outer_iterations;
  opts.initial_penalty = config.initial_penalty;
  opts.penalty_scaling = config.penalty_scaling;
  opts.constraint_tolerance = config.constraint_tolerance;
  opts.max_penalty = 1e8;
  opts.penalty_update_ratio = 0.4;
  opts.return_best_trajectory = true;

  opts.inner_options.max_iterations = config.max_inner_iterations;
  opts.inner_options.cost_tolerance = 1e-4;
  opts.inner_options.derivative_epsilon = 1e-4;
  opts.inner_options.regularization_init = 1e-5;
  opts.inner_options.regularization_min = 1e-8;
  opts.inner_options.regularization_max = 1e5;
  opts.inner_options.regularization_increase_factor = 10.0;
  opts.inner_options.regularization_decrease_factor = 5.0;
  opts.inner_options.line_search_max_iterations = 10;

  const double wb = vehicle_config_.model.wheelbase;
  const double min_a = vehicle_config_.model.min_acceleration;
  const double max_a = vehicle_config_.model.max_acceleration;
  const double min_s = vehicle_config_.model.min_steering;
  const double max_s = vehicle_config_.model.max_steering;

  std::vector<Vector> init_u(horizon, Vector::Zero(2));
  bool externals_valid = static_cast<int>(initial_controls.size()) >= horizon;
  if (externals_valid) {
    double norm2 = 0.0;
    for (int k = 0; k < horizon; ++k) norm2 += initial_controls[k].squaredNorm();
    externals_valid = norm2 > 1e-8;
  }
  if (externals_valid) {
    for (int k = 0; k < horizon; ++k) init_u[k] = initial_controls[k];
  } else {
    auto pp_result = PurePursuitTracker::Track(
        guidance, initial_state, dynamics, u_lb, u_ub, horizon, dt, wb);
    for (int k = 0; k < horizon; ++k) {
      init_u[k] = pp_result.controls[k];
    }
  }

  ALILQRSolver solver(*constrained, opts);

  const auto t0 = std::chrono::steady_clock::now();
  auto al_result = solver.Solve(initial_state, init_u);
  result.solve_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

  result.converged = al_result.converged;
  result.max_violation = al_result.best_violation;
  result.outer_iterations = static_cast<int>(al_result.outer_logs.size());
  for (const auto& log : al_result.outer_logs) {
    result.total_inner_iterations += log.inner_iterations;
  }

  result.states.resize(horizon + 1);
  result.controls.resize(horizon);
  for (int k = 0; k <= horizon; ++k) {
    result.states[k] = al_result.trajectory.State(k);
  }
  for (int k = 0; k < horizon; ++k) {
    result.controls[k] = al_result.trajectory.Control(k);
  }

  return result;
}

}  // namespace my_al_ilqr
