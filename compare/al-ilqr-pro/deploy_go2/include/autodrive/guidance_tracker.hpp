#pragma once

#include <memory>
#include <vector>

#include "autodrive/global_guidance_planner.hpp"
#include "autodrive/vehicle_bicycle_config.hpp"
#include "autodrive/vehicle_circle_approximation.hpp"
#include "core/types.hpp"

namespace my_al_ilqr {

/// @brief 动态圆形障碍物（odom 坐标系下）。
struct DynamicObstacle {
  double x = 0.0;
  double y = 0.0;
  double radius = 0.0;
};

struct GuidanceTrackerConfig {
  double position_weight = 5.0;
  double heading_weight = 2.0;
  double speed_weight = 1.0;
  double terminal_position_weight = 8.0;
  double terminal_heading_weight = 4.0;
  double terminal_speed_weight = 2.0;
  double accel_reg_weight = 0.2;
  double steering_reg_weight = 0.4;

  int max_outer_iterations = 4;
  int max_inner_iterations = 15;
  double initial_penalty = 20.0;
  double penalty_scaling = 5.0;
  double constraint_tolerance = 1e-1;

  /// 内层 iLQR 代价收敛阈值（对应 ILQROptions::cost_tolerance）。
  double convergence_threshold = 1e-4;

  /// 求导模式：true = 解析求导（闭式 Jacobian/Hessian，快 10x 量级），
  /// false = 中心差分（对照/兜底）。对应 ILQROptions::derivative_mode。
  /// 注意：解析模式要求动力学/代价/约束均实现 HasAnalytical*()，否则
  /// ILQRSolver 会对未实现的组件自动回退到差分。
  bool use_analytical_derivatives = true;
};

struct GuidanceTrackerResult {
  std::vector<Vector> states;
  std::vector<Vector> controls;
  bool converged = false;
  double max_violation = 0.0;
  double solve_time_ms = 0.0;
  int outer_iterations = 0;
  int total_inner_iterations = 0;
};

class GuidanceTracker {
 public:
  GuidanceTracker(const VehicleBicycleConfig& vehicle_config,
                  double road_lower_bound,
                  double road_upper_bound,
                  const VehicleCircleApproximation& vehicle_geometry);

  GuidanceTrackerResult Track(
      const Vector& initial_state,
      const std::vector<GuidancePoint>& guidance,
      const std::vector<Vector>& initial_controls,
      int horizon, double dt,
      const GuidanceTrackerConfig& config = {},
      const std::vector<DynamicObstacle>& obstacles = {},
      bool enable_road_boundary = true) const;

 private:
  VehicleBicycleConfig vehicle_config_;
  double road_lower_bound_;
  double road_upper_bound_;
  VehicleCircleApproximation vehicle_geometry_;
};

}  // namespace my_al_ilqr
