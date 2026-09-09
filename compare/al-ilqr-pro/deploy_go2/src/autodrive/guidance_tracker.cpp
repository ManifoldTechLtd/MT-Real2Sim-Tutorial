#include "autodrive/guidance_tracker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <utility>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/multi_circle_road_boundary_constraint.hpp"
#include "autodrive/multi_circle_vehicle_obstacle_constraint.hpp"
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

  // --- 解析导数 -------------------------------------------------------
  // 代价为对角二次型，导数为闭式，避免 ILQRSolver 对每个 knot 做
  // 4n²+4m²+4nm 次代价函数差分（而每次差分还会遍历所有 AL 约束）。
  bool HasAnalyticalDerivatives() const override { return true; }

  CostExpansion AnalyticalStageCostExpansion(const Vector& state,
                                             const Vector& control) const override {
    const double dx = state(0) - ref_.x;
    const double dy = state(1) - ref_.y;
    const double he = NormalizeAngle(state(2) - ref_.yaw);
    const double se = state(3) - ref_.speed;

    Vector lx(4);
    lx << pos_w_ * dx, pos_w_ * dy, hdg_w_ * he, spd_w_ * se;

    Vector lu(2);
    lu << acc_w_ * control(0), steer_w_ * control(1);

    Matrix lxx = Matrix::Zero(4, 4);
    lxx(0, 0) = pos_w_;
    lxx(1, 1) = pos_w_;
    lxx(2, 2) = hdg_w_;
    lxx(3, 3) = spd_w_;

    Matrix luu = Matrix::Zero(2, 2);
    luu(0, 0) = acc_w_;
    luu(1, 1) = steer_w_;

    return CostExpansion{lx, lu, lxx, Matrix::Zero(2, 4), luu};
  }

  std::pair<Vector, Matrix> AnalyticalTerminalCostExpansion(
      const Vector& /*state*/) const override {
    return {Vector::Zero(4), Matrix::Zero(4, 4)};
  }

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

  // --- 解析导数 -------------------------------------------------------
  bool HasAnalyticalDerivatives() const override { return true; }

  CostExpansion AnalyticalStageCostExpansion(const Vector& /*state*/,
                                             const Vector& /*control*/) const override {
    return CostExpansion{Vector::Zero(4), Vector::Zero(2), Matrix::Zero(4, 4),
                         Matrix::Zero(2, 4), Matrix::Zero(2, 2)};
  }

  std::pair<Vector, Matrix> AnalyticalTerminalCostExpansion(
      const Vector& state) const override {
    const double dx = state(0) - ref_.x;
    const double dy = state(1) - ref_.y;
    const double he = NormalizeAngle(state(2) - ref_.yaw);
    const double se = state(3) - ref_.speed;

    Vector lx(4);
    lx << pos_w_ * dx, pos_w_ * dy, hdg_w_ * he, spd_w_ * se;

    Matrix lxx = Matrix::Zero(4, 4);
    lxx(0, 0) = pos_w_;
    lxx(1, 1) = pos_w_;
    lxx(2, 2) = hdg_w_;
    lxx(3, 3) = spd_w_;

    return {lx, lxx};
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
    const GuidanceTrackerConfig& config,
    const std::vector<DynamicObstacle>& obstacles,
    bool enable_road_boundary) const {
  GuidanceTrackerResult result;
  if (guidance.empty() || horizon <= 0 || dt <= 0.0) {
    return result;
  }
  // 数值安全：初始状态必须有限（上游 odom/TF 异常时直接拒绝求解，
  // 避免 NaN 传入 Eigen 分解导致不可预测行为）。
  if (initial_state.size() != 4 || !initial_state.allFinite()) {
    return result;
  }
  for (const auto& pt : guidance) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.yaw) ||
        !std::isfinite(pt.speed)) {
      return result;
    }
  }
  if (vehicle_config_.model.wheelbase <= 1e-6) {
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
    // 道路边界约束（可选）：基于 StraightReferenceLine(0,0,0)（沿 x 轴的直线），
    // 约束机器人在绝对坐标系 y∈[road_lower_bound, road_upper_bound] 走廊内，
    // 并非真实车道线。真实部署默认禁用（enable_road_boundary=false）。
    if (enable_road_boundary) {
      constrained->AddStageConstraint(
          k, std::make_shared<MultiCircleRoadBoundaryConstraint>(
                 ref_line, vehicle_geometry_,
                 road_lower_bound_, road_upper_bound_));
    }
    constrained->AddStageConstraint(
        k, std::make_shared<SpeedLimitConstraint>(
               vehicle_config_.model.min_speed,
               vehicle_config_.model.max_speed));

    // 动态障碍物约束：每个障碍物在每个 stage 都添加一个
    // MultiCircleVehicleObstacleConstraint（静态假设：障碍物位置不变）
    for (const auto& obs : obstacles) {
      constrained->AddStageConstraint(
          k, std::make_shared<MultiCircleVehicleObstacleConstraint>(
                 vehicle_geometry_, obs.x, obs.y, obs.radius));
    }
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
  // 求导模式：由外部配置（solver/derivative_mode）控制。
  // 处理链：ros_node yaml -> GuidanceTrackerConfig::use_analytical_derivatives
  //         -> ALILQROptions::inner_options.derivative_mode -> ILQRSolver 分派。
  opts.inner_options.derivative_mode = config.use_analytical_derivatives
                                           ? DerivativeMode::kAnalytical
                                           : DerivativeMode::kFiniteDifference;
  opts.inner_options.cost_tolerance =
      (config.convergence_threshold > 0.0) ? config.convergence_threshold : 1e-4;
  opts.inner_options.derivative_epsilon = 1e-4;
  opts.inner_options.regularization_init = 1e-5;
  opts.inner_options.regularization_min = 1e-8;
  opts.inner_options.regularization_max = 1e5;
  opts.inner_options.regularization_increase_factor = 10.0;
  opts.inner_options.regularization_decrease_factor = 5.0;
  opts.inner_options.line_search_max_iterations = 10;

  const double wb = vehicle_config_.model.wheelbase;

  std::vector<Vector> init_u(horizon, Vector::Zero(2));
  bool externals_valid = static_cast<int>(initial_controls.size()) >= horizon;
  if (externals_valid) {
    // warm start 必须维度正确且数值有限，否则回退到 Pure-Pursuit 初值
    for (int k = 0; k < horizon; ++k) {
      if (initial_controls[k].size() != 2 || !initial_controls[k].allFinite()) {
        externals_valid = false;
        break;
      }
    }
  }
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

  // 数值安全：求解结果包含 NaN/inf 时不得下发，清空并标记未收敛
  for (const auto& s : result.states) {
    if (!s.allFinite()) {
      result.states.clear();
      result.controls.clear();
      result.converged = false;
      return result;
    }
  }
  for (const auto& u : result.controls) {
    if (!u.allFinite()) {
      result.states.clear();
      result.controls.clear();
      result.converged = false;
      return result;
    }
  }

  return result;
}

}  // namespace my_al_ilqr
