#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#include "al/al_ilqr_solver.hpp"
#include "constraints/control_box_constraint.hpp"
#include "autodrive/guidance_tracker.hpp"
#include "cost/cost_function.hpp"
#include "dynamics/dynamics_model.hpp"
#include "dynamics/kinematic_bicycle_model.hpp"
#include "dynamics/unicycle_model.hpp"
#include "autodrive/global_guidance_planner.hpp"
#include "autodrive/pure_pursuit_tracker.hpp"
#include "autodrive/vehicle_bicycle_config.hpp"
#include "autodrive/vehicle_circle_approximation.hpp"
#include "problems/constrained_optimal_control_problem.hpp"
#include "problems/optimal_control_problem.hpp"

using namespace my_al_ilqr;

static constexpr double kDt = 0.1;

static double NormalizeAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static double NiceTickStep(double range, int target = 6) {
  if (range <= 0) return 1.0;
  double raw = range / target;
  double mag = std::pow(10.0, std::floor(std::log10(raw)));
  double n = raw / mag;
  double s = n < 1.5 ? 1.0 : n < 3.5 ? 2.0 : n < 7.5 ? 5.0 : 10.0;
  return s * mag;
}

// ─── 2D Double-Integrator (point mass) ─────────────────────────────────────
class DoubleIntegrator2D final : public DynamicsModel {
 public:
  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }
  Vector NextState(const Vector& x, const Vector& u, double dt) const override {
    Vector nx(4);
    nx(0) = x(0) + x(2) * dt;
    nx(1) = x(1) + x(3) * dt;
    nx(2) = x(2) + u(0) * dt;
    nx(3) = x(3) + u(1) * dt;
    return nx;
  }
};

// ─── Tracking cost classes (stage + terminal) for each model ───────────────
struct TrackWeights {
  double pos = 5.0, hdg = 2.0, spd = 1.0;
  double ctrl0 = 0.2, ctrl1 = 0.4;
  double term_pos = 8.0, term_hdg = 4.0, term_spd = 2.0;
};

struct TrackWeightsUI {
  float pos = 5.0f, hdg = 2.0f, spd = 1.0f;
  float ctrl0 = 0.2f, ctrl1 = 0.4f;
  float term_pos = 8.0f, term_hdg = 4.0f, term_spd = 2.0f;
  void SyncTo(TrackWeights& w) const {
    w.pos = pos; w.hdg = hdg; w.spd = spd;
    w.ctrl0 = ctrl0; w.ctrl1 = ctrl1;
    w.term_pos = term_pos; w.term_hdg = term_hdg; w.term_spd = term_spd;
  }
};

class PM2DStageCost final : public CostFunction {
 public:
  PM2DStageCost(const GuidancePoint& r, const TrackWeights& w)
      : r_(r), w_(w) {}
  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }
  double StageCost(const Vector& s, const Vector& u) const override {
    double dx = s(0) - r_.x, dy = s(1) - r_.y;
    double spd = std::sqrt(s(2) * s(2) + s(3) * s(3));
    return 0.5 * w_.pos * (dx * dx + dy * dy) +
           0.5 * w_.spd * (spd - r_.speed) * (spd - r_.speed) +
           0.5 * w_.ctrl0 * u(0) * u(0) +
           0.5 * w_.ctrl1 * u(1) * u(1);
  }
  double TerminalCost(const Vector&) const override { return 0.0; }
 private:
  GuidancePoint r_; TrackWeights w_;
};

class PM2DTermCost final : public CostFunction {
 public:
  PM2DTermCost(const GuidancePoint& r, const TrackWeights& w) : r_(r), w_(w) {}
  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }
  double StageCost(const Vector&, const Vector&) const override { return 0.0; }
  double TerminalCost(const Vector& s) const override {
    double dx = s(0) - r_.x, dy = s(1) - r_.y;
    double spd = std::sqrt(s(2) * s(2) + s(3) * s(3));
    return 0.5 * w_.term_pos * (dx * dx + dy * dy) +
           0.5 * w_.term_spd * (spd - r_.speed) * (spd - r_.speed);
  }
 private:
  GuidancePoint r_; TrackWeights w_;
};

class UniStageCost final : public CostFunction {
 public:
  UniStageCost(const GuidancePoint& r, const TrackWeights& w) : r_(r), w_(w) {}
  int StateDim() const override { return 3; }
  int ControlDim() const override { return 2; }
  double StageCost(const Vector& s, const Vector& u) const override {
    double dx = s(0) - r_.x, dy = s(1) - r_.y;
    double he = NormalizeAngle(s(2) - r_.yaw);
    double se = u(0) - r_.speed;
    return 0.5 * w_.pos * (dx * dx + dy * dy) +
           0.5 * w_.hdg * he * he +
           0.5 * w_.spd * se * se +
           0.5 * w_.ctrl1 * u(1) * u(1);
  }
  double TerminalCost(const Vector&) const override { return 0.0; }
 private:
  GuidancePoint r_; TrackWeights w_;
};

class UniTermCost final : public CostFunction {
 public:
  UniTermCost(const GuidancePoint& r, const TrackWeights& w) : r_(r), w_(w) {}
  int StateDim() const override { return 3; }
  int ControlDim() const override { return 2; }
  double StageCost(const Vector&, const Vector&) const override { return 0.0; }
  double TerminalCost(const Vector& s) const override {
    double dx = s(0) - r_.x, dy = s(1) - r_.y;
    double he = NormalizeAngle(s(2) - r_.yaw);
    return 0.5 * w_.term_pos * (dx * dx + dy * dy) +
           0.5 * w_.term_hdg * he * he;
  }
 private:
  GuidancePoint r_; TrackWeights w_;
};

class BicStageCost final : public CostFunction {
 public:
  BicStageCost(const GuidancePoint& r, const TrackWeights& w) : r_(r), w_(w) {}
  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }
  double StageCost(const Vector& s, const Vector& u) const override {
    double dx = s(0) - r_.x, dy = s(1) - r_.y;
    double he = NormalizeAngle(s(2) - r_.yaw);
    double se = s(3) - r_.speed;
    return 0.5 * w_.pos * (dx * dx + dy * dy) +
           0.5 * w_.hdg * he * he +
           0.5 * w_.spd * se * se +
           0.5 * w_.ctrl0 * u(0) * u(0) +
           0.5 * w_.ctrl1 * u(1) * u(1);
  }
  double TerminalCost(const Vector&) const override { return 0.0; }
 private:
  GuidancePoint r_; TrackWeights w_;
};

class BicTermCost final : public CostFunction {
 public:
  BicTermCost(const GuidancePoint& r, const TrackWeights& w) : r_(r), w_(w) {}
  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }
  double StageCost(const Vector&, const Vector&) const override { return 0.0; }
  double TerminalCost(const Vector& s) const override {
    double dx = s(0) - r_.x, dy = s(1) - r_.y;
    double he = NormalizeAngle(s(2) - r_.yaw);
    double se = s(3) - r_.speed;
    return 0.5 * w_.term_pos * (dx * dx + dy * dy) +
           0.5 * w_.term_hdg * he * he +
           0.5 * w_.term_spd * se * se;
  }
 private:
  GuidancePoint r_; TrackWeights w_;
};

// ─── Application state ────────────────────────────────────────────────────
enum class DynModel { kPointMass2D, kUnicycle, kBicycle };
static const char* kModelNames[] = {"PointMass 2D", "Unicycle", "Bicycle"};
enum class TrackerMode { kPP, kPPILQR };
static const char* kTrackerModeNames[] = {"PP", "PP+iLQR"};

struct TrackerTestState {
  int model_idx = 2;
  int tracker_mode_idx = 1;
  float target_speed = 3.0f;

  float pm_max_accel = 5.0f;

  float uni_max_speed = 6.0f;
  float uni_max_omega = 1.5f;

  float bic_wheelbase = 2.8f;
  float bic_max_accel = 1.0f;
  float bic_max_steering = 0.45f;
  float bic_max_speed = 6.0f;

  TrackWeights weights;
  TrackWeightsUI weights_ui;
  int max_outer = 10;
  int max_inner = 80;

  std::vector<ImVec2> waypoints;
  std::vector<GuidancePoint> guidance;

  struct Result {
    std::vector<Vector> states;
    std::vector<Vector> controls;
    std::vector<Vector> pp_states;
    std::vector<Vector> pp_controls;
    bool converged = false;
    bool used_optimizer = false;
    double max_violation = 0.0;
    double solve_time_ms = 0.0;
    int outer_iters = 0, total_inner = 0;
    double pp_max_err = 0.0;
    double final_max_err = 0.0;
  } result;
  bool has_result = false;
  bool show_pp_overlay = true;
  float road_half_width = 4.5f;

  double pan_x = 0.0, pan_y = 0.0, zoom = 1.0;
};

// ─── Catmull-Rom spline interpolation for smooth guidance ─────────────────
static ImVec2 CatmullRom(ImVec2 p0, ImVec2 p1, ImVec2 p2, ImVec2 p3, float t) {
  float t2 = t * t, t3 = t2 * t;
  return ImVec2(
      0.5f * ((2*p1.x) + (-p0.x+p2.x)*t + (2*p0.x-5*p1.x+4*p2.x-p3.x)*t2 +
              (-p0.x+3*p1.x-3*p2.x+p3.x)*t3),
      0.5f * ((2*p1.y) + (-p0.y+p2.y)*t + (2*p0.y-5*p1.y+4*p2.y-p3.y)*t2 +
              (-p0.y+3*p1.y-3*p2.y+p3.y)*t3));
}

static std::vector<ImVec2> ResampleSpline(const std::vector<ImVec2>& wps, int samples_per_seg) {
  if (wps.size() < 2) return wps;
  std::vector<ImVec2> out;
  int n = static_cast<int>(wps.size());
  for (int i = 0; i < n - 1; ++i) {
    ImVec2 p0 = wps[std::max(0, i - 1)];
    ImVec2 p1 = wps[i];
    ImVec2 p2 = wps[std::min(n - 1, i + 1)];
    ImVec2 p3 = wps[std::min(n - 1, i + 2)];
    int steps = (i == n - 2) ? samples_per_seg + 1 : samples_per_seg;
    for (int j = 0; j < steps; ++j) {
      float t = static_cast<float>(j) / samples_per_seg;
      out.push_back(CatmullRom(p0, p1, p2, p3, t));
    }
  }
  return out;
}

// ─── Guidance builder with curvature-aware speed ──────────────────────────
static std::vector<GuidancePoint> BuildGuidance(
    const std::vector<ImVec2>& wps, double speed, double max_lat_accel) {
  if (wps.size() < 2) return {};

  auto smooth = ResampleSpline(wps, 20);
  if (smooth.size() < 2) return {};

  std::vector<double> seg;
  double total = 0.0;
  for (size_t i = 1; i < smooth.size(); ++i) {
    double dx = smooth[i].x - smooth[i - 1].x, dy = smooth[i].y - smooth[i - 1].y;
    seg.push_back(std::sqrt(dx * dx + dy * dy));
    total += seg.back();
  }
  if (total < 0.01) return {};

  std::vector<double> cum(smooth.size(), 0.0);
  for (size_t i = 1; i < smooth.size(); ++i) cum[i] = cum[i - 1] + seg[i - 1];

  std::vector<double> yaws(smooth.size());
  for (size_t i = 0; i < smooth.size(); ++i) {
    double dx = 1.0, dy = 0.0;
    if (i + 1 < smooth.size()) { dx = smooth[i+1].x - smooth[i].x; dy = smooth[i+1].y - smooth[i].y; }
    else if (i > 0) { dx = smooth[i].x - smooth[i-1].x; dy = smooth[i].y - smooth[i-1].y; }
    yaws[i] = std::atan2(dy, dx);
  }

  std::vector<double> curvature(smooth.size(), 0.0);
  for (size_t i = 1; i + 1 < smooth.size(); ++i) {
    double ds = 0.5 * (seg[i - 1] + seg[i]);
    if (ds > 1e-6)
      curvature[i] = std::abs(NormalizeAngle(yaws[i + 1] - yaws[i - 1])) / ds;
  }

  double sp = std::max(speed, 0.5);
  std::vector<double> speed_limit(smooth.size());
  for (size_t i = 0; i < smooth.size(); ++i) {
    if (curvature[i] > 1e-4)
      speed_limit[i] = std::min(sp, std::sqrt(max_lat_accel / curvature[i]));
    else
      speed_limit[i] = sp;
    speed_limit[i] = std::max(speed_limit[i], 0.3);
  }

  double decel = max_lat_accel;
  for (int i = static_cast<int>(smooth.size()) - 2; i >= 0; --i) {
    double ds = seg[i];
    double v_max_from_next = std::sqrt(speed_limit[i + 1] * speed_limit[i + 1] + 2.0 * decel * ds);
    speed_limit[i] = std::min(speed_limit[i], v_max_from_next);
  }
  for (size_t i = 1; i < smooth.size(); ++i) {
    double ds = seg[i - 1];
    double v_max_from_prev = std::sqrt(speed_limit[i - 1] * speed_limit[i - 1] + 2.0 * decel * ds);
    speed_limit[i] = std::min(speed_limit[i], v_max_from_prev);
  }

  double time_est = 0.0;
  for (size_t i = 0; i + 1 < smooth.size(); ++i) {
    double avg_v = 0.5 * (speed_limit[i] + speed_limit[i + 1]);
    if (avg_v > 0.1) time_est += seg[i] / avg_v;
    else time_est += seg[i] / 0.3;
  }
  int H = std::max(10, static_cast<int>(std::ceil(time_est / kDt)));

  std::vector<GuidancePoint> pts;
  pts.reserve(H + 1);
  for (int k = 0; k <= H; ++k) {
    double s = (double(k) / H) * total;
    size_t si = 0;
    for (size_t i = 1; i < smooth.size(); ++i) { if (cum[i] >= s) { si = i - 1; break; } si = i - 1; }
    double r = seg[si] > 1e-6 ? std::clamp((s - cum[si]) / seg[si], 0.0, 1.0) : 0.0;
    GuidancePoint p;
    p.x = smooth[si].x + r * (smooth[si + 1].x - smooth[si].x);
    p.y = smooth[si].y + r * (smooth[si + 1].y - smooth[si].y);
    p.speed = speed_limit[si] + r * (speed_limit[std::min(si + 1, smooth.size() - 1)] - speed_limit[si]);
    p.t = k * kDt;
    p.s = s;
    pts.push_back(p);
  }
  for (size_t i = 0; i < pts.size(); ++i) {
    double dx = 1.0, dy = 0.0;
    if (i + 1 < pts.size()) { dx = pts[i+1].x - pts[i].x; dy = pts[i+1].y - pts[i].y; }
    else if (i > 0) { dx = pts[i].x - pts[i-1].x; dy = pts[i].y - pts[i-1].y; }
    pts[i].yaw = std::atan2(dy, dx);
  }
  return pts;
}

static double ComputeMaxPosError(const std::vector<Vector>& states,
                                 const std::vector<GuidancePoint>& guidance) {
  if (states.empty() || guidance.empty()) {
    return 0.0;
  }
  const int gs = static_cast<int>(guidance.size());
  double max_err = 0.0;
  for (size_t k = 0; k < states.size(); ++k) {
    const auto& ref = guidance[std::min(static_cast<int>(k), gs - 1)];
    const double dx = states[k](0) - ref.x;
    const double dy = states[k](1) - ref.y;
    max_err = std::max(max_err, std::sqrt(dx * dx + dy * dy));
  }
  return max_err;
}

// ─── Solve tracking OCP ──────────────────────────────────────────────────
static void RunTracker(TrackerTestState* s) {
  double sp = static_cast<double>(s->target_speed);
  DynModel model = static_cast<DynModel>(s->model_idx);
  TrackerMode tracker_mode = static_cast<TrackerMode>(s->tracker_mode_idx);

  double max_lat_a;
  switch (model) {
    case DynModel::kPointMass2D: max_lat_a = s->pm_max_accel * 0.8; break;
    case DynModel::kUnicycle:    max_lat_a = s->uni_max_speed * s->uni_max_omega * 0.6; break;
    case DynModel::kBicycle: {
      double v_ref = sp;
      double max_yaw_rate = v_ref * std::tan(s->bic_max_steering) / s->bic_wheelbase;
      max_lat_a = v_ref * max_yaw_rate * 0.6;
      break;
    }
  }

  s->guidance = BuildGuidance(s->waypoints, sp, max_lat_a);
  s->has_result = false;
  if (s->guidance.size() < 2) return;

  int H = static_cast<int>(s->guidance.size()) - 1;
  int gs = static_cast<int>(s->guidance.size());
  const TrackWeights& w = s->weights;

  std::shared_ptr<DynamicsModel> dynamics;
  Vector x0;
  std::vector<std::shared_ptr<CostFunction>> stage_costs;
  std::shared_ptr<CostFunction> term_cost;
  Vector u_lb, u_ub;
  std::vector<Vector> init_u(H);

  double yaw0 = s->guidance[0].yaw;

  double wb_pp = 0.0;
  switch (model) {
    case DynModel::kPointMass2D: {
      dynamics = std::make_shared<DoubleIntegrator2D>();
      x0.resize(4);
      x0 << s->waypoints[0].x, s->waypoints[0].y,
            sp * std::cos(yaw0), sp * std::sin(yaw0);
      double ma = s->pm_max_accel;
      u_lb.resize(2); u_ub.resize(2);
      u_lb << -ma, -ma;
      u_ub <<  ma,  ma;
      for (int k = 0; k < H; ++k)
        stage_costs.push_back(std::make_shared<PM2DStageCost>(
            s->guidance[std::min(k, gs - 1)], w));
      term_cost = std::make_shared<PM2DTermCost>(s->guidance[gs - 1], w);
      break;
    }
    case DynModel::kUnicycle: {
      dynamics = std::make_shared<UnicycleModel>();
      x0.resize(3);
      x0 << s->waypoints[0].x, s->waypoints[0].y, yaw0;
      double mv = s->uni_max_speed, mw = s->uni_max_omega;
      u_lb.resize(2); u_ub.resize(2);
      u_lb << 0.0, -mw;
      u_ub << mv,   mw;
      for (int k = 0; k < H; ++k)
        stage_costs.push_back(std::make_shared<UniStageCost>(
            s->guidance[std::min(k, gs - 1)], w));
      term_cost = std::make_shared<UniTermCost>(s->guidance[gs - 1], w);
      break;
    }
    case DynModel::kBicycle: {
      wb_pp = s->bic_wheelbase;
      dynamics = std::make_shared<KinematicBicycleModel>(wb_pp);
      x0.resize(4);
      x0 << s->waypoints[0].x, s->waypoints[0].y, yaw0, sp;
      double ma = s->bic_max_accel, ms = s->bic_max_steering;
      u_lb.resize(2); u_ub.resize(2);
      u_lb << -ma, -ms;
      u_ub <<  ma,  ms;
      for (int k = 0; k < H; ++k)
        stage_costs.push_back(std::make_shared<BicStageCost>(
            s->guidance[std::min(k, gs - 1)], w));
      term_cost = std::make_shared<BicTermCost>(s->guidance[gs - 1], w);
      break;
    }
  }

  auto pp_result = PurePursuitTracker::Track(
      s->guidance, x0, dynamics, u_lb, u_ub, H, kDt, wb_pp);
  for (int k = 0; k < H; ++k) init_u[k] = pp_result.controls[k];

  auto& R = s->result;
  R.pp_states = pp_result.states;
  R.pp_controls = pp_result.controls;
  R.pp_max_err = pp_result.max_pos_error;

  if (tracker_mode == TrackerMode::kPP) {
    R.states = pp_result.states;
    R.controls = pp_result.controls;
    R.converged = false;
    R.used_optimizer = false;
    R.max_violation = 0.0;
    R.solve_time_ms = 0.0;
    R.outer_iters = 0;
    R.total_inner = 0;
    R.final_max_err = ComputeMaxPosError(R.states, s->guidance);
    s->has_result = true;
    return;
  }

  if (model == DynModel::kBicycle) {
    VehicleBicycleConfig vehicle_config;
    vehicle_config.model.wheelbase = s->bic_wheelbase;
    vehicle_config.model.min_acceleration = -s->bic_max_accel;
    vehicle_config.model.max_acceleration = s->bic_max_accel;
    vehicle_config.model.min_steering = -s->bic_max_steering;
    vehicle_config.model.max_steering = s->bic_max_steering;
    vehicle_config.model.min_speed = 0.0;
    vehicle_config.model.max_speed = s->bic_max_speed;

    VehicleCircleApproximation vehicle_geometry(vehicle_config.body);
    GuidanceTracker tracker(
        vehicle_config,
        -static_cast<double>(s->road_half_width),
        static_cast<double>(s->road_half_width),
        vehicle_geometry);

    GuidanceTrackerConfig track_cfg;
    track_cfg.position_weight = w.pos;
    track_cfg.heading_weight = w.hdg;
    track_cfg.speed_weight = w.spd;
    track_cfg.terminal_position_weight = w.term_pos;
    track_cfg.terminal_heading_weight = w.term_hdg;
    track_cfg.terminal_speed_weight = w.term_spd;
    track_cfg.accel_reg_weight = w.ctrl0;
    track_cfg.steering_reg_weight = w.ctrl1;
    track_cfg.max_outer_iterations = s->max_outer;
    track_cfg.max_inner_iterations = s->max_inner;

    auto track_result = tracker.Track(
        x0, s->guidance, pp_result.controls, H, kDt, track_cfg);

    R.solve_time_ms = track_result.solve_time_ms;
    R.converged = track_result.converged;
    R.used_optimizer = true;
    R.max_violation = track_result.max_violation;
    R.outer_iters = track_result.outer_iterations;
    R.total_inner = track_result.total_inner_iterations;
    R.states = track_result.states;
    R.controls = track_result.controls;
    R.final_max_err = ComputeMaxPosError(R.states, s->guidance);
    s->has_result = true;
    return;
  }

  auto ocp = std::make_shared<OptimalControlProblem>(
      dynamics, stage_costs, term_cost, kDt);
  auto constrained = std::make_shared<ConstrainedOptimalControlProblem>(ocp);
  for (int k = 0; k < H; ++k)
    constrained->AddStageConstraint(
        k, std::make_shared<ControlBoxConstraint>(ocp->StateDim(), u_lb, u_ub));

  ALILQROptions opts;
  opts.max_outer_iterations = s->max_outer;
  opts.initial_penalty = 50.0;
  opts.penalty_scaling = 5.0;
  opts.constraint_tolerance = 1e-2;
  opts.max_penalty = 1e8;
  opts.penalty_update_ratio = 0.25;
  opts.return_best_trajectory = true;
  opts.inner_options.max_iterations = s->max_inner;
  opts.inner_options.cost_tolerance = 1e-6;
  opts.inner_options.derivative_epsilon = 1e-4;
  opts.inner_options.regularization_init = 1e-6;
  opts.inner_options.regularization_min = 1e-9;
  opts.inner_options.regularization_max = 1e6;
  opts.inner_options.regularization_increase_factor = 10.0;
  opts.inner_options.regularization_decrease_factor = 5.0;
  opts.inner_options.line_search_max_iterations = 10;

  ALILQRSolver solver(*constrained, opts);
  auto t0 = std::chrono::steady_clock::now();
  auto res = solver.Solve(x0, init_u);
  double ms_elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

  R.solve_time_ms = ms_elapsed;
  R.converged = res.converged;
  R.used_optimizer = true;
  R.max_violation = res.best_violation;
  R.outer_iters = static_cast<int>(res.outer_logs.size());
  R.total_inner = 0;
  for (auto& ol : res.outer_logs) R.total_inner += ol.inner_iterations;
  R.states.resize(H + 1);
  R.controls.resize(H);
  for (int k = 0; k <= H; ++k) R.states[k] = res.trajectory.State(k);
  for (int k = 0; k < H; ++k) R.controls[k] = res.trajectory.Control(k);
  R.final_max_err = ComputeMaxPosError(R.states, s->guidance);
  s->has_result = true;
}

// ─── Drawing utilities ───────────────────────────────────────────────────
static ImVec2 W2S(double wx, double wy, const ImVec2& o, const ImVec2& sz,
                  double x0, double x1, double y0, double y1) {
  return {o.x + float((wx - x0) / (x1 - x0)) * sz.x,
          o.y + sz.y - float((wy - y0) / (y1 - y0)) * sz.y};
}

static void DrawPoly(ImDrawList* dl, const std::vector<ImVec2>& p, ImU32 c, float t) {
  for (size_t i = 0; i + 1 < p.size(); ++i) dl->AddLine(p[i], p[i + 1], c, t);
}

static void DrawGridTicks(ImDrawList* dl, const ImVec2& po, const ImVec2& ps,
                          double bx0, double bx1, double by0, double by1) {
  char buf[16];
  double tx = NiceTickStep(bx1 - bx0), ty = NiceTickStep(by1 - by0);
  auto W = [&](double x, double y) { return W2S(x, y, po, ps, bx0, bx1, by0, by1); };

  dl->PushClipRect(po, ImVec2(po.x + ps.x, po.y + ps.y), true);
  for (double gx = std::ceil(bx0 / tx) * tx; gx <= bx1; gx += tx)
    dl->AddLine(W(gx, by0), W(gx, by1), IM_COL32(45, 50, 60, 180), 1.0f);
  for (double gy = std::ceil(by0 / ty) * ty; gy <= by1; gy += ty)
    dl->AddLine(W(bx0, gy), W(bx1, gy), IM_COL32(45, 50, 60, 180), 1.0f);
  if (by0 <= 0 && 0 <= by1)
    dl->AddLine(W(bx0, 0), W(bx1, 0), IM_COL32(80, 85, 100, 200), 1.0f);
  if (bx0 <= 0 && 0 <= bx1)
    dl->AddLine(W(0, by0), W(0, by1), IM_COL32(80, 85, 100, 200), 1.0f);
  dl->PopClipRect();

  for (double gx = std::ceil(bx0 / tx) * tx; gx <= bx1; gx += tx) {
    auto p = W(gx, by0);
    std::snprintf(buf, sizeof(buf), "%.4g", gx);
    ImVec2 tsz = ImGui::CalcTextSize(buf);
    dl->AddText(ImVec2(p.x - tsz.x * 0.5f, po.y + ps.y + 3),
                IM_COL32(140, 150, 165, 220), buf);
  }
  for (double gy = std::ceil(by0 / ty) * ty; gy <= by1; gy += ty) {
    auto p = W(bx0, gy);
    std::snprintf(buf, sizeof(buf), "%.4g", gy);
    ImVec2 tsz = ImGui::CalcTextSize(buf);
    dl->AddText(ImVec2(po.x - tsz.x - 4, p.y - tsz.y * 0.5f),
                IM_COL32(140, 150, 165, 220), buf);
  }
}

static void DrawCanvas(TrackerTestState* s) {
  ImVec2 cp = ImGui::GetCursorScreenPos();
  ImVec2 av = ImGui::GetContentRegionAvail();
  ImVec2 cs(std::max(200.0f, av.x), std::max(200.0f, av.y));
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(cp, ImVec2(cp.x + cs.x, cp.y + cs.y), IM_COL32(18, 22, 30, 255), 6.0f);
  dl->AddRect(cp, ImVec2(cp.x + cs.x, cp.y + cs.y), IM_COL32(60, 68, 85, 255), 6.0f);

  const float ml = 42, mt = 20, mr = 12, mb = 22;
  ImVec2 po(cp.x + ml, cp.y + mt), ps(cs.x - ml - mr, cs.y - mt - mb);
  if (ps.x < 10 || ps.y < 10) { ImGui::Dummy(cs); return; }

  double vw = 35.0 / s->zoom, vh = vw * ps.y / ps.x;
  double cx = 15.0 + s->pan_x, cy = s->pan_y;
  double bx0 = cx - vw / 2, bx1 = cx + vw / 2;
  double by0 = cy - vh / 2, by1 = cy + vh / 2;

  ImGuiIO& io = ImGui::GetIO();
  if (ImGui::IsMouseHoveringRect(cp, ImVec2(cp.x + cs.x, cp.y + cs.y), true)) {
    if (std::abs(io.MouseWheel) > 1e-6f)
      s->zoom = std::clamp(s->zoom * (io.MouseWheel > 0 ? 1.12 : 1.0 / 1.12), 0.2, 20.0);
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
      double sc = vw / ps.x;
      s->pan_x -= io.MouseDelta.x * sc;
      s->pan_y += io.MouseDelta.y * sc;
    }
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.KeyCtrl) {
      ImVec2 mp = io.MousePos;
      if (mp.x >= po.x && mp.x <= po.x + ps.x && mp.y >= po.y && mp.y <= po.y + ps.y) {
        double wx = bx0 + (mp.x - po.x) / ps.x * (bx1 - bx0);
        double wy = by1 - (mp.y - po.y) / ps.y * (by1 - by0);
        s->waypoints.push_back(ImVec2(float(wx), float(wy)));
      }
    }
  }

  auto W = [&](double x, double y) { return W2S(x, y, po, ps, bx0, bx1, by0, by1); };

  DrawGridTicks(dl, po, ps, bx0, bx1, by0, by1);
  dl->AddRect(po, ImVec2(po.x + ps.x, po.y + ps.y), IM_COL32(80, 88, 105, 200));

  dl->PushClipRect(po, ImVec2(po.x + ps.x, po.y + ps.y), true);
  if (!s->guidance.empty()) {
    std::vector<ImVec2> gp;
    for (auto& pt : s->guidance) gp.push_back(W(pt.x, pt.y));
    DrawPoly(dl, gp, IM_COL32(80, 140, 255, 200), 2.0f);
  }
  if (s->has_result) {
    std::vector<ImVec2> tp;
    for (auto& st : s->result.states) tp.push_back(W(st(0), st(1)));
    DrawPoly(dl, tp, IM_COL32(255, 80, 80, 230), 2.5f);
  }
  if (s->has_result && s->show_pp_overlay && s->result.used_optimizer &&
      !s->result.pp_states.empty()) {
    std::vector<ImVec2> pp;
    for (auto& st : s->result.pp_states) pp.push_back(W(st(0), st(1)));
    DrawPoly(dl, pp, IM_COL32(0, 255, 120, 220), 2.0f);
  }
  char buf[8];
  for (size_t i = 0; i < s->waypoints.size(); ++i) {
    ImVec2 sp = W(s->waypoints[i].x, s->waypoints[i].y);
    dl->AddCircleFilled(sp, 5, IM_COL32(255, 220, 60, 255));
    dl->AddCircle(sp, 5, IM_COL32(200, 180, 40, 255), 0, 1.5f);
    std::snprintf(buf, sizeof(buf), "%d", int(i));
    dl->AddText(ImVec2(sp.x + 7, sp.y - 7), IM_COL32(255, 255, 255, 200), buf);
  }
  dl->PopClipRect();

  dl->AddText(ImVec2(po.x + 2, po.y - 16), IM_COL32(160, 170, 190, 200),
              "Left-click: add | Right-drag: pan | Scroll: zoom");
  dl->AddText(ImVec2(po.x + 2, po.y + 2), IM_COL32(80, 140, 255, 220),
              "Guidance");
  dl->AddText(ImVec2(po.x + 78, po.y + 2), IM_COL32(255, 80, 80, 220),
              s->result.used_optimizer ? "PP+iLQR" : "PP");
  if (s->has_result && s->result.used_optimizer) {
    dl->AddText(ImVec2(po.x + 152, po.y + 2), IM_COL32(0, 255, 120, 220),
                "PP Seed");
  }
  if (s->has_result) {
    char info[160];
    if (s->result.used_optimizer) {
      std::snprintf(info, sizeof(info),
                    "time=%.1fms  conv=%d  outer=%d inner=%d  viol=%.4f",
                    s->result.solve_time_ms, s->result.converged ? 1 : 0,
                    s->result.outer_iters, s->result.total_inner, s->result.max_violation);
    } else {
      std::snprintf(info, sizeof(info), "PP only  no optimizer");
    }
    dl->AddText(ImVec2(po.x + 2, po.y + ps.y - 16), IM_COL32(100, 255, 120, 230), info);
  }
  ImGui::Dummy(cs);
}

static void DrawPlot(const char* label, const std::vector<double>& xs,
                     const std::vector<double>& ys, ImU32 color,
                     const std::vector<double>* ys2 = nullptr, ImU32 c2 = 0) {
  if (xs.empty() || ys.empty()) { ImGui::TextDisabled("%s: no data", label); return; }
  ImVec2 pos = ImGui::GetCursorScreenPos();
  ImVec2 full(std::max(100.0f, ImGui::GetContentRegionAvail().x), 120.0f);
  ImDrawList* dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(pos, ImVec2(pos.x + full.x, pos.y + full.y), IM_COL32(20, 24, 32, 255), 4.0f);

  const float ml = 38, mr = 6, mt = 16, mb = 18;
  ImVec2 po(pos.x + ml, pos.y + mt), ps(full.x - ml - mr, full.y - mt - mb);
  if (ps.x < 10 || ps.y < 10) { ImGui::Dummy(full); return; }

  double xn = *std::min_element(xs.begin(), xs.end());
  double xx = *std::max_element(xs.begin(), xs.end());
  double yn = *std::min_element(ys.begin(), ys.end());
  double yx = *std::max_element(ys.begin(), ys.end());
  if (ys2) {
    yn = std::min(yn, *std::min_element(ys2->begin(), ys2->end()));
    yx = std::max(yx, *std::max_element(ys2->begin(), ys2->end()));
  }
  double ym = (yx - yn) * 0.1 + 1e-6; yn -= ym; yx += ym;
  if (xx - xn < 1e-9) xx = xn + 1.0;

  auto Pt = [&](double x, double y) -> ImVec2 {
    return {po.x + float((x - xn) / (xx - xn)) * ps.x,
            po.y + ps.y - float((y - yn) / (yx - yn)) * ps.y};
  };

  DrawGridTicks(dl, po, ps, xn, xx, yn, yx);
  dl->AddRect(po, ImVec2(po.x + ps.x, po.y + ps.y), IM_COL32(70, 78, 95, 200));

  dl->PushClipRect(po, ImVec2(po.x + ps.x, po.y + ps.y), true);
  int n = int(std::min(xs.size(), ys.size()));
  for (int i = 0; i + 1 < n; ++i)
    dl->AddLine(Pt(xs[i], ys[i]), Pt(xs[i + 1], ys[i + 1]), color, 1.5f);
  if (ys2) {
    int n2 = int(std::min(xs.size(), ys2->size()));
    for (int i = 0; i + 1 < n2; ++i)
      dl->AddLine(Pt(xs[i], (*ys2)[i]), Pt(xs[i + 1], (*ys2)[i + 1]), c2, 1.5f);
  }
  dl->PopClipRect();

  dl->AddText(ImVec2(po.x + 2, pos.y + 1), IM_COL32(200, 210, 225, 220), label);
  ImGui::Dummy(full);
}

// ─── Extract speed from state depending on model ─────────────────────────
static double ExtractSpeed(const Vector& state, const Vector& ctrl, DynModel m) {
  switch (m) {
    case DynModel::kPointMass2D: return std::sqrt(state(2)*state(2) + state(3)*state(3));
    case DynModel::kUnicycle:    return ctrl.size() >= 1 ? ctrl(0) : 0.0;
    case DynModel::kBicycle:     return state(3);
  }
  return 0.0;
}

// ─── Main ────────────────────────────────────────────────────────────────
int main() {
  TrackerTestState state;

  if (!glfwInit()) return 1;
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  GLFWwindow* window = glfwCreateWindow(1280, 800, "Guidance Tracker Test", nullptr, nullptr);
  if (!window) { glfwTerminate(); return 1; }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 130");

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float pw = 280;

    ImGui::SetNextWindowPos(vp->Pos, ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(pw, vp->Size.y), ImGuiCond_Always);
    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

    ImGui::SeparatorText("Model");
    ImGui::Combo("Type", &state.model_idx, kModelNames, 3);
    ImGui::Combo("Tracker", &state.tracker_mode_idx, kTrackerModeNames, 2);
    ImGui::SliderFloat("Speed", &state.target_speed, 0.5f, 6.0f, "%.1f m/s");

    DynModel dm = static_cast<DynModel>(state.model_idx);

    ImGui::SeparatorText("Model Params");
    switch (dm) {
      case DynModel::kPointMass2D:
        ImGui::SliderFloat("max accel", &state.pm_max_accel, 0.5f, 10.0f, "%.1f");
        ImGui::TextWrapped("state=[x,y,vx,vy]  ctrl=[ax,ay]");
        break;
      case DynModel::kUnicycle:
        ImGui::SliderFloat("max speed", &state.uni_max_speed, 1.0f, 15.0f, "%.1f");
        ImGui::SliderFloat("max omega", &state.uni_max_omega, 0.3f, 5.0f, "%.2f rad/s");
        ImGui::TextWrapped("state=[x,y,theta]  ctrl=[v,omega]");
        break;
      case DynModel::kBicycle:
        ImGui::SliderFloat("wheelbase", &state.bic_wheelbase, 1.0f, 5.0f, "%.1f m");
        ImGui::SliderFloat("max accel", &state.bic_max_accel, 0.5f, 5.0f, "%.1f");
        ImGui::SliderFloat("max steer", &state.bic_max_steering, 0.1f, 1.0f, "%.2f rad");
        ImGui::SliderFloat("max speed", &state.bic_max_speed, 1.0f, 15.0f, "%.1f");
        ImGui::SliderFloat("road half width", &state.road_half_width, 1.5f, 10.0f, "%.1f m");
        {
          float r = state.bic_wheelbase / std::tan(state.bic_max_steering);
          ImGui::Text("min turn R: %.1f m", r);
        }
        ImGui::TextWrapped("state=[x,y,yaw,v]  ctrl=[a,delta]");
        break;
    }

    ImGui::SeparatorText("Tracker Weights");
    ImGui::SliderFloat("pos_w", &state.weights_ui.pos, 0.5f, 50.0f, "%.1f");
    if (dm != DynModel::kPointMass2D)
      ImGui::SliderFloat("hdg_w", &state.weights_ui.hdg, 0.0f, 20.0f, "%.1f");
    ImGui::SliderFloat("spd_w", &state.weights_ui.spd, 0.0f, 20.0f, "%.1f");
    ImGui::SliderFloat("ctrl0_reg", &state.weights_ui.ctrl0, 0.01f, 5.0f, "%.2f");
    ImGui::SliderFloat("ctrl1_reg", &state.weights_ui.ctrl1, 0.01f, 5.0f, "%.2f");
    ImGui::SliderFloat("term_pos", &state.weights_ui.term_pos, 0.0f, 50.0f, "%.1f");
    if (dm != DynModel::kPointMass2D)
      ImGui::SliderFloat("term_hdg", &state.weights_ui.term_hdg, 0.0f, 20.0f, "%.1f");
    ImGui::SliderFloat("term_spd", &state.weights_ui.term_spd, 0.0f, 20.0f, "%.1f");

    ImGui::SeparatorText("Solver");
    if (state.tracker_mode_idx == static_cast<int>(TrackerMode::kPP)) {
      ImGui::BeginDisabled();
    }
    ImGui::SliderInt("outer", &state.max_outer, 1, 20);
    ImGui::SliderInt("inner", &state.max_inner, 5, 50);
    if (state.tracker_mode_idx == static_cast<int>(TrackerMode::kPP)) {
      ImGui::EndDisabled();
      ImGui::TextWrapped("PP mode does not run AL-iLQR. Solver settings are ignored.");
    }

    ImGui::Separator();
    if (ImGui::Button("Track", ImVec2(-1, 30))) {
      state.weights_ui.SyncTo(state.weights);
      RunTracker(&state);
    }
    float half = (ImGui::GetContentRegionAvail().x - 8) * 0.5f;
    if (ImGui::Button("Clear", ImVec2(half, 0))) {
      state.waypoints.clear(); state.guidance.clear(); state.has_result = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("Undo", ImVec2(half, 0))) {
      if (!state.waypoints.empty()) state.waypoints.pop_back();
    }

    ImGui::Separator();
    ImGui::Checkbox("Show PP Overlay", &state.show_pp_overlay);
    ImGui::Text("Waypoints: %d", int(state.waypoints.size()));
    if (state.has_result) {
      ImGui::Text("Mode: %s", state.result.used_optimizer ? "PP+iLQR" : "PP");
      if (dm == DynModel::kBicycle && state.result.used_optimizer) {
        ImGui::TextDisabled("Backend: project GuidanceTracker");
      }
      if (state.result.used_optimizer) {
        ImGui::TextColored(state.result.converged ? ImVec4(.3f,1,.4f,1) : ImVec4(1,.3f,.3f,1),
                           "Conv: %s", state.result.converged ? "YES" : "NO");
        ImGui::Text("Time: %.1f ms", state.result.solve_time_ms);
        ImGui::Text("Outer: %d  Inner: %d", state.result.outer_iters, state.result.total_inner);
        ImGui::Text("Violation: %.4f", state.result.max_violation);
      } else {
        ImGui::TextDisabled("Optimizer not run in PP mode.");
      }
      ImGui::Text("Horizon: %d", int(state.result.controls.size()));
      ImGui::Text("Final max pos err: %.3f m", state.result.final_max_err);
      ImGui::TextColored(ImVec4(.2f,.8f,.5f,1), "PP max pos err: %.3f m",
                         state.result.pp_max_err);
    }
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(vp->Pos.x + pw, vp->Pos.y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(vp->Size.x - pw, vp->Size.y * 0.58f), ImGuiCond_Always);
    ImGui::Begin("Scene", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
    DrawCanvas(&state);
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(vp->Pos.x + pw, vp->Pos.y + vp->Size.y * 0.58f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(vp->Size.x - pw, vp->Size.y * 0.42f), ImGuiCond_Always);
    ImGui::Begin("Plots", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

    if (state.has_result && !state.result.controls.empty()) {
      int H = int(state.result.controls.size());
      int gs = int(state.guidance.size());
      std::vector<double> ta, c0v, c1v, spd_g, spd_t, err_v, pp_err_v, ts;
      for (int k = 0; k < H; ++k) {
        ta.push_back(k * kDt);
        c0v.push_back(state.result.controls[k](0));
        c1v.push_back(state.result.controls[k].size() > 1 ? state.result.controls[k](1) : 0);
      }
      for (int k = 0; k <= H; ++k) {
        ts.push_back(k * kDt);
        spd_g.push_back(state.guidance[std::min(k, gs - 1)].speed);
        Vector u_k = k < H ? state.result.controls[k] : state.result.controls[H - 1];
        spd_t.push_back(ExtractSpeed(state.result.states[k], u_k, dm));
        auto& pt = state.guidance[std::min(k, gs - 1)];
        double dx = state.result.states[k](0) - pt.x, dy = state.result.states[k](1) - pt.y;
        err_v.push_back(std::sqrt(dx * dx + dy * dy));
        if (!state.result.pp_states.empty() && k < static_cast<int>(state.result.pp_states.size())) {
          const double pdx = state.result.pp_states[k](0) - pt.x;
          const double pdy = state.result.pp_states[k](1) - pt.y;
          pp_err_v.push_back(std::sqrt(pdx * pdx + pdy * pdy));
        }
      }

      const char* c0_name = dm == DynModel::kUnicycle ? "Speed / ctrl[0]" : "Accel / ctrl[0]";
      const char* c1_name = dm == DynModel::kPointMass2D ? "Accel Y / ctrl[1]"
                          : dm == DynModel::kUnicycle ? "Omega / ctrl[1]"
                          : "Steering / ctrl[1]";

      float w = ImGui::GetContentRegionAvail().x;
      float h = ImGui::GetContentRegionAvail().y;
      ImGui::BeginChild("p0", ImVec2(w * 0.25f, h));
      DrawPlot(c0_name, ta, c0v, IM_COL32(80, 220, 100, 255));
      ImGui::Spacing();
      DrawPlot(c1_name, ta, c1v, IM_COL32(200, 100, 255, 255));
      ImGui::EndChild();

      ImGui::SameLine();
      ImGui::BeginChild("p1", ImVec2(w * 0.25f, h));
      DrawPlot("Speed (blue=guide red=track)", ts, spd_g,
               IM_COL32(80, 140, 255, 255), &spd_t, IM_COL32(255, 80, 80, 255));
      ImGui::EndChild();

      ImGui::SameLine();
      ImGui::BeginChild("p2", ImVec2(0, h));
      DrawPlot(state.result.used_optimizer
                   ? "Position Error (orange=PP+iLQR green=PP)"
                   : "Position Error (m)",
               ts, err_v, IM_COL32(255, 180, 50, 255),
               state.result.used_optimizer ? &pp_err_v : nullptr,
               IM_COL32(0, 255, 120, 255));
      ImGui::EndChild();
    } else {
      ImGui::TextDisabled("Click points on canvas, then press 'Track'.");
    }
    ImGui::End();

    ImGui::Render();
    int dw, dh;
    glfwGetFramebufferSize(window, &dw, &dh);
    glViewport(0, 0, dw, dh);
    glClearColor(0.06f, 0.06f, 0.08f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
