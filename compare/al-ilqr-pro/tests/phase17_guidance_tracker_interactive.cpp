#include <Python.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "matplotlibcpp.h"

#include "autodrive/guidance_tracker.hpp"
#include "autodrive/vehicle_bicycle_config.hpp"

namespace plt = matplotlibcpp;
using namespace my_al_ilqr;

static constexpr double kDt = 0.1;
static constexpr double kRoadHalfWidth = 4.0;
static constexpr double kDefaultSpeed = 3.0;

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

std::vector<GuidancePoint> InterpolateWaypoints(
    const std::vector<std::array<double, 2>>& waypoints, double speed) {
  if (waypoints.size() < 2) return {};

  std::vector<double> seg_len;
  double total_len = 0.0;
  for (size_t i = 1; i < waypoints.size(); ++i) {
    double dx = waypoints[i][0] - waypoints[i - 1][0];
    double dy = waypoints[i][1] - waypoints[i - 1][1];
    double d = std::sqrt(dx * dx + dy * dy);
    seg_len.push_back(d);
    total_len += d;
  }
  if (total_len < 0.01) return {};

  double total_time = total_len / std::max(speed, 0.5);
  int horizon = std::max(10, static_cast<int>(total_time / kDt));
  horizon = std::min(horizon, 80);

  std::vector<double> cum_s(waypoints.size(), 0.0);
  for (size_t i = 1; i < waypoints.size(); ++i) {
    cum_s[i] = cum_s[i - 1] + seg_len[i - 1];
  }

  std::vector<GuidancePoint> pts;
  pts.reserve(horizon + 1);
  for (int k = 0; k <= horizon; ++k) {
    double s = (static_cast<double>(k) / horizon) * total_len;
    size_t seg = 0;
    for (size_t i = 1; i < waypoints.size(); ++i) {
      if (cum_s[i] >= s) { seg = i - 1; break; }
      seg = i - 1;
    }
    double local_ratio = 0.0;
    if (seg_len[seg] > 1e-6) {
      local_ratio = (s - cum_s[seg]) / seg_len[seg];
    }
    local_ratio = std::clamp(local_ratio, 0.0, 1.0);

    GuidancePoint pt;
    pt.x = waypoints[seg][0] +
           local_ratio * (waypoints[seg + 1][0] - waypoints[seg][0]);
    pt.y = waypoints[seg][1] +
           local_ratio * (waypoints[seg + 1][1] - waypoints[seg][1]);
    pt.speed = speed;
    pt.t = k * kDt;
    pt.s = s;
    pts.push_back(pt);
  }

  for (size_t i = 0; i < pts.size(); ++i) {
    double dx, dy;
    if (i + 1 < pts.size()) {
      dx = pts[i + 1].x - pts[i].x;
      dy = pts[i + 1].y - pts[i].y;
    } else if (i > 0) {
      dx = pts[i].x - pts[i - 1].x;
      dy = pts[i].y - pts[i - 1].y;
    } else {
      dx = 1.0; dy = 0.0;
    }
    pts[i].yaw = std::atan2(dy, dx);
  }

  return pts;
}

void DrawResult(const std::vector<std::array<double, 2>>& waypoints,
                const std::vector<GuidancePoint>& guidance,
                const GuidanceTrackerResult& result) {
  plt::clf();

  std::vector<double> wx, wy;
  for (const auto& w : waypoints) {
    wx.push_back(w[0]);
    wy.push_back(w[1]);
  }

  std::vector<double> gx, gy, tx, ty;
  for (const auto& pt : guidance) {
    gx.push_back(pt.x);
    gy.push_back(pt.y);
  }
  for (const auto& st : result.states) {
    tx.push_back(st(0));
    ty.push_back(st(1));
  }

  int horizon = static_cast<int>(result.controls.size());
  std::vector<double> t_axis, accel_v, steer_v, speed_g, speed_t;
  for (int k = 0; k < horizon; ++k) {
    t_axis.push_back(k * kDt);
    accel_v.push_back(result.controls[k](0));
    steer_v.push_back(result.controls[k](1));
  }
  for (const auto& pt : guidance) speed_g.push_back(pt.speed);
  for (const auto& st : result.states) speed_t.push_back(st(3));
  std::vector<double> ts_full;
  for (int k = 0; k <= horizon; ++k) ts_full.push_back(k * kDt);

  double max_pos_err = 0.0;
  int gs = static_cast<int>(guidance.size());
  for (int k = 0; k <= horizon; ++k) {
    const auto& pt = guidance[std::min(k, gs - 1)];
    double dx = result.states[k](0) - pt.x;
    double dy = result.states[k](1) - pt.y;
    max_pos_err = std::max(max_pos_err, std::sqrt(dx * dx + dy * dy));
  }

  plt::subplot2grid(2, 3, 0, 0, 1, 2);
  plt::named_plot("Waypoints", wx, wy, "ko-");
  plt::named_plot("Guidance", gx, gy, "b--");
  plt::named_plot("Tracked", tx, ty, "r-");
  char title_buf[128];
  std::snprintf(title_buf, sizeof(title_buf),
                "XY  |  time=%.1fms  err=%.3fm  conv=%d  outer=%d inner=%d",
                result.solve_time_ms, max_pos_err,
                result.converged ? 1 : 0,
                result.outer_iterations, result.total_inner_iterations);
  plt::title(title_buf);
  plt::xlabel("x [m]");
  plt::ylabel("y [m]");
  plt::legend();
  plt::grid(true);
  plt::set_aspect_equal();

  plt::subplot2grid(2, 3, 0, 2);
  plt::named_plot("Guidance", ts_full, speed_g, "b--");
  plt::named_plot("Tracked", ts_full, speed_t, "r-");
  plt::title("Speed");
  plt::xlabel("t [s]");
  plt::ylabel("v [m/s]");
  plt::legend();
  plt::grid(true);

  plt::subplot2grid(2, 3, 1, 0, 1, 2);
  plt::named_plot("accel", t_axis, accel_v, "g-");
  plt::named_plot("steer", t_axis, steer_v, "m-");
  plt::title("Controls");
  plt::xlabel("t [s]");
  plt::ylabel("u");
  plt::legend();
  plt::grid(true);

  std::vector<double> err_v;
  for (int k = 0; k <= horizon; ++k) {
    const auto& pt = guidance[std::min(k, gs - 1)];
    double dx = result.states[k](0) - pt.x;
    double dy = result.states[k](1) - pt.y;
    err_v.push_back(std::sqrt(dx * dx + dy * dy));
  }
  plt::subplot2grid(2, 3, 1, 2);
  plt::plot(ts_full, err_v, "r-");
  plt::title("Position Error");
  plt::xlabel("t [s]");
  plt::ylabel("error [m]");
  plt::grid(true);

  plt::tight_layout();
}

int main() {
  const auto vehicle = DefaultVehicle();
  const VehicleCircleApproximation geom(vehicle.body);
  GuidanceTracker tracker(vehicle, -kRoadHalfWidth, kRoadHalfWidth, geom);

  auto CollectViaGinput = [&]() -> std::vector<std::array<double, 2>> {
    plt::figure_size(1400, 800);
    plt::title("Left-click: add point | Right-click/Enter: finish");
    plt::xlabel("x [m]");
    plt::ylabel("y [m]");
    plt::xlim(-2.0, 30.0);
    plt::ylim(-kRoadHalfWidth - 1, kRoadHalfWidth + 1);
    plt::grid(true);
    std::vector<double> rx = {-2, 30},
        ru = {kRoadHalfWidth, kRoadHalfWidth},
        rl = {-kRoadHalfWidth, -kRoadHalfWidth};
    plt::plot(rx, ru, "k--");
    plt::plot(rx, rl, "k--");
    auto pts = plt::ginput(20);
    plt::close();
    return pts;
  };

  auto CollectViaTerminal = [&]() -> std::vector<std::array<double, 2>> {
    std::vector<std::array<double, 2>> pts;
    std::cout << "Enter waypoints as 'x y' per line (empty line to finish):\n";
    std::string line;
    while (true) {
      std::cout << "  pt" << pts.size() << "> ";
      std::getline(std::cin, line);
      if (line.empty() || line == "q") break;
      std::istringstream iss(line);
      double x, y;
      if (iss >> x >> y) {
        pts.push_back({x, y});
      } else {
        std::cout << "    (invalid, use 'x y' format)\n";
      }
    }
    return pts;
  };

  bool use_ginput = false;
  try {
    PyErr_Clear();
    plt::figure_size(400, 300);
    plt::title("Click once to test, then close window");
    auto test = plt::ginput(1);
    plt::close();
    use_ginput = true;
  } catch (...) {
    PyErr_Clear();
    try { plt::close(); } catch (...) {}
  }

  if (use_ginput) {
    std::cout << "Interactive mode: click waypoints on the plot.\n";
  } else {
    std::cout << "ginput unavailable, using terminal input.\n"
              << "Hint: enter points like '0 0', '5 1', '10 -1', '15 0'\n";
  }

  while (true) {
    std::cout << "\n=== New test (enter 'q' to quit) ===\n";

    std::vector<std::array<double, 2>> clicks;
    if (use_ginput) {
      clicks = CollectViaGinput();
    } else {
      clicks = CollectViaTerminal();
    }

    if (clicks.empty()) {
      std::cout << "No points. Try again? (Enter=yes, q=quit): ";
      std::string s; std::getline(std::cin, s);
      if (s == "q" || s == "Q") break;
      continue;
    }

    std::cout << "Waypoints (" << clicks.size() << "):";
    for (const auto& c : clicks)
      std::cout << " (" << c[0] << "," << c[1] << ")";
    std::cout << std::endl;

    auto guidance = InterpolateWaypoints(clicks, kDefaultSpeed);
    if (guidance.empty()) {
      std::cout << "Could not build guidance.\n";
      continue;
    }

    int horizon = static_cast<int>(guidance.size()) - 1;
    Vector init_state(4);
    init_state << clicks[0][0], clicks[0][1], guidance[0].yaw, kDefaultSpeed;

    std::vector<Vector> empty_controls;
    auto result = tracker.Track(init_state, guidance, empty_controls,
                                horizon, kDt);

    std::cout << "Tracker: time=" << result.solve_time_ms << "ms"
              << "  converged=" << result.converged
              << "  outer=" << result.outer_iterations
              << "  inner=" << result.total_inner_iterations
              << std::endl;

    plt::figure_size(1400, 800);
    DrawResult(clicks, guidance, result);
    plt::show();

    std::cout << "\nEnter=again, q=quit: ";
    std::string s; std::getline(std::cin, s);
    if (s == "q" || s == "Q") break;
  }

  std::cout << "Done.\n";
  return 0;
}
