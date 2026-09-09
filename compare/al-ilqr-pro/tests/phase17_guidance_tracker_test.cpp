#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "matplotlibcpp.h"

#include "autodrive/global_guidance_planner.hpp"
#include "autodrive/guidance_tracker.hpp"
#include "autodrive/vehicle_bicycle_config.hpp"

namespace plt = matplotlibcpp;
using namespace my_al_ilqr;

static constexpr int kHorizon = 55;
static constexpr double kDt = 0.1;
static constexpr double kRoadHalfWidth = 3.5;

VehicleBicycleConfig DefaultVehicle() {
  VehicleBicycleConfig cfg;
  cfg.model.wheelbase = 2.8;
  cfg.model.min_acceleration = -1.0;
  cfg.model.max_acceleration = 1.0;
  cfg.model.min_steering = -0.45;
  cfg.model.max_steering = 0.45;
  cfg.model.min_speed = 0.0;
  cfg.model.max_speed = 6.0;
  cfg.body.length = 4.6;
  cfg.body.width = 1.9;
  cfg.body.rear_axle_to_rear = 1.0;
  cfg.body.collision_circle_count = 2;
  return cfg;
}

std::vector<GuidancePoint> MakeStraightGuidance(double speed, int n) {
  std::vector<GuidancePoint> pts(n);
  double s = 0.0;
  for (int k = 0; k < n; ++k) {
    pts[k].x = s;
    pts[k].y = 0.0;
    pts[k].yaw = 0.0;
    pts[k].speed = speed;
    pts[k].t = k * kDt;
    pts[k].s = s;
    pts[k].d = 0.0;
    s += speed * kDt;
  }
  return pts;
}

std::vector<GuidancePoint> MakeLaneChangeGuidance(
    double speed, double lateral_offset, int n) {
  std::vector<GuidancePoint> pts(n);
  double s = 0.0;
  const double total_s = speed * (n - 1) * kDt;
  for (int k = 0; k < n; ++k) {
    pts[k].x = s;
    const double ratio = s / total_s;
    const double smooth = 3.0 * ratio * ratio - 2.0 * ratio * ratio * ratio;
    pts[k].y = lateral_offset * smooth;
    const double dsmooth_ds = (6.0 * ratio - 6.0 * ratio * ratio) / total_s;
    pts[k].yaw = std::atan(lateral_offset * dsmooth_ds);
    pts[k].speed = speed;
    pts[k].t = k * kDt;
    pts[k].s = s;
    pts[k].d = pts[k].y;
    s += speed * kDt;
  }
  return pts;
}

std::vector<GuidancePoint> MakeAccelGuidance(
    double init_speed, double final_speed, int n) {
  std::vector<GuidancePoint> pts(n);
  double x = 0.0;
  for (int k = 0; k < n; ++k) {
    const double ratio = static_cast<double>(k) / (n - 1);
    const double v = init_speed + (final_speed - init_speed) * ratio;
    pts[k].x = x;
    pts[k].y = 0.0;
    pts[k].yaw = 0.0;
    pts[k].speed = v;
    pts[k].t = k * kDt;
    pts[k].s = x;
    pts[k].d = 0.0;
    x += v * kDt;
  }
  return pts;
}

std::vector<GuidancePoint> MakeCurvedGuidance(
    double speed, double curvature, int n) {
  std::vector<GuidancePoint> pts(n);
  double x = 0.0, y = 0.0, yaw = 0.0;
  for (int k = 0; k < n; ++k) {
    pts[k].x = x;
    pts[k].y = y;
    pts[k].yaw = yaw;
    pts[k].speed = speed;
    pts[k].t = k * kDt;
    pts[k].s = k * speed * kDt;
    pts[k].d = 0.0;
    const double ds = speed * kDt;
    x += ds * std::cos(yaw);
    y += ds * std::sin(yaw);
    yaw += curvature * ds;
  }
  return pts;
}

struct TestCase {
  std::string name;
  Vector initial_state;
  std::vector<GuidancePoint> guidance;
  GuidanceTrackerConfig config;
};

void SaveCasePlot(const TestCase& tc, const GuidanceTrackerResult& result,
                  int case_idx) {
  std::vector<double> gx, gy, tx, ty;
  for (const auto& pt : tc.guidance) {
    gx.push_back(pt.x);
    gy.push_back(pt.y);
  }
  for (const auto& st : result.states) {
    tx.push_back(st(0));
    ty.push_back(st(1));
  }

  std::vector<double> t_axis, accel, steer, speed_g, speed_t;
  for (int k = 0; k < static_cast<int>(result.controls.size()); ++k) {
    t_axis.push_back(k * kDt);
    accel.push_back(result.controls[k](0));
    steer.push_back(result.controls[k](1));
  }
  for (const auto& pt : tc.guidance) speed_g.push_back(pt.speed);
  for (const auto& st : result.states) speed_t.push_back(st(3));

  plt::figure_size(1400, 400);

  plt::subplot2grid(1, 3, 0, 0);
  plt::named_plot("Guidance", gx, gy, "b--");
  plt::named_plot("Tracked", tx, ty, "r-");
  plt::title(tc.name + " - XY");
  plt::xlabel("x [m]");
  plt::ylabel("y [m]");
  plt::legend();
  plt::grid(true);

  plt::subplot2grid(1, 3, 0, 1);
  plt::named_plot("accel", t_axis, accel, "g-");
  plt::named_plot("steer", t_axis, steer, "m-");
  plt::title("Controls");
  plt::xlabel("t [s]");
  plt::ylabel("u");
  plt::legend();
  plt::grid(true);

  plt::subplot2grid(1, 3, 0, 2);
  std::vector<double> ts_full;
  for (int k = 0; k <= kHorizon; ++k) ts_full.push_back(k * kDt);
  plt::named_plot("Guidance", ts_full, speed_g, "b--");
  plt::named_plot("Tracked", ts_full, speed_t, "r-");
  plt::title("Speed");
  plt::xlabel("t [s]");
  plt::ylabel("v [m/s]");
  plt::legend();
  plt::grid(true);

  plt::tight_layout();
  const std::string path =
      "build/tracker_case" + std::to_string(case_idx) + ".png";
  plt::save(path);
  plt::close();
  std::cout << "  -> saved " << path << std::endl;
}

int main() {
  const auto vehicle = DefaultVehicle();
  const VehicleCircleApproximation geom(vehicle.body);
  GuidanceTracker tracker(vehicle, -kRoadHalfWidth, kRoadHalfWidth, geom);

  std::vector<TestCase> cases;

  {
    TestCase tc;
    tc.name = "Straight line (v=3.0)";
    tc.initial_state = Vector(4);
    tc.initial_state << 0.0, 0.0, 0.0, 3.0;
    tc.guidance = MakeStraightGuidance(3.0, kHorizon + 1);
    cases.push_back(std::move(tc));
  }

  {
    TestCase tc;
    tc.name = "Lane change (d=2.5)";
    tc.initial_state = Vector(4);
    tc.initial_state << 0.0, 0.0, 0.0, 2.5;
    tc.guidance = MakeLaneChangeGuidance(2.5, 2.5, kHorizon + 1);
    cases.push_back(std::move(tc));
  }

  {
    TestCase tc;
    tc.name = "Acceleration (2->5)";
    tc.initial_state = Vector(4);
    tc.initial_state << 0.0, 0.0, 0.0, 2.0;
    tc.guidance = MakeAccelGuidance(2.0, 5.0, kHorizon + 1);
    cases.push_back(std::move(tc));
  }

  {
    TestCase tc;
    tc.name = "Curve (kappa=0.02)";
    tc.initial_state = Vector(4);
    tc.initial_state << 0.0, 0.0, 0.0, 2.5;
    tc.guidance = MakeCurvedGuidance(2.5, 0.02, kHorizon + 1);
    cases.push_back(std::move(tc));
  }

  plt::backend("Agg");

  const int num_cases = static_cast<int>(cases.size());
  for (int i = 0; i < num_cases; ++i) {
    auto& tc = cases[i];
    std::vector<Vector> zero_controls(kHorizon, Vector::Zero(2));

    auto result = tracker.Track(
        tc.initial_state, tc.guidance, zero_controls,
        kHorizon, kDt, tc.config);

    double max_pos_err = 0.0;
    for (int k = 0; k <= kHorizon; ++k) {
      const auto& pt = tc.guidance[std::min(k,
          static_cast<int>(tc.guidance.size()) - 1)];
      const double dx = result.states[k](0) - pt.x;
      const double dy = result.states[k](1) - pt.y;
      max_pos_err = std::max(max_pos_err, std::sqrt(dx * dx + dy * dy));
    }

    std::cout << "[" << tc.name << "]"
              << "  converged=" << result.converged
              << "  time=" << result.solve_time_ms << " ms"
              << "  outer=" << result.outer_iterations
              << "  inner=" << result.total_inner_iterations
              << "  violation=" << result.max_violation
              << "  max_pos_err=" << max_pos_err << " m"
              << std::endl;

    assert(result.solve_time_ms < 50.0 &&
           "Tracking must finish within 50 ms");

    SaveCasePlot(tc, result, i);
  }

  std::cout << "\nAll " << num_cases << " test cases passed." << std::endl;
  return 0;
}
