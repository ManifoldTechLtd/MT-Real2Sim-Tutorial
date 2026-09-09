#include "autodrive/lattice_initial_guess.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

#include <Eigen/Dense>

namespace my_al_ilqr {

namespace {

struct FrenetState {
  double s;
  double d;
  double ds;
  double dd;
};

struct QuinticPoly {
  double a0, a1, a2, a3, a4, a5;

  double Eval(double t) const {
    return a0 + t * (a1 + t * (a2 + t * (a3 + t * (a4 + t * a5))));
  }
  double EvalDeriv(double t) const {
    return a1 + t * (2 * a2 + t * (3 * a3 + t * (4 * a4 + t * 5 * a5)));
  }
  double EvalDeriv2(double t) const {
    return 2 * a2 + t * (6 * a3 + t * (12 * a4 + t * 20 * a5));
  }
};

struct QuarticPoly {
  double b0, b1, b2, b3, b4;

  double Eval(double t) const {
    return b0 + t * (b1 + t * (b2 + t * (b3 + t * b4)));
  }
  double EvalDeriv(double t) const {
    return b1 + t * (2 * b2 + t * (3 * b3 + t * 4 * b4));
  }
  double EvalDeriv2(double t) const {
    return 2 * b2 + t * (6 * b3 + t * 12 * b4);
  }
};

// Quintic polynomial: d(0)=d0, d'(0)=dd0, d''(0)=ddd0,
//                     d(T)=df, d'(T)=ddf, d''(T)=dddf
QuinticPoly SolveQuintic(double d0, double dd0, double ddd0,
                         double df, double ddf, double dddf,
                         double T) {
  QuinticPoly p;
  p.a0 = d0;
  p.a1 = dd0;
  p.a2 = ddd0 / 2.0;

  const double T2 = T * T;
  const double T3 = T2 * T;
  const double T4 = T3 * T;
  const double T5 = T4 * T;

  Eigen::Matrix3d A;
  A << T3, T4, T5,
       3 * T2, 4 * T3, 5 * T4,
       6 * T, 12 * T2, 20 * T3;

  Eigen::Vector3d b;
  b << df - p.a0 - p.a1 * T - p.a2 * T2,
       ddf - p.a1 - 2 * p.a2 * T,
       dddf - 2 * p.a2;

  Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
  p.a3 = x(0);
  p.a4 = x(1);
  p.a5 = x(2);
  return p;
}

// Quartic polynomial: s(0)=s0, s'(0)=v0, s''(0)=a0, s'(T)=vf, s''(T)=af
QuarticPoly SolveQuartic(double s0, double v0, double a0,
                         double vf, double af, double T) {
  QuarticPoly p;
  p.b0 = s0;
  p.b1 = v0;
  p.b2 = a0 / 2.0;

  const double T2 = T * T;
  const double T3 = T2 * T;

  Eigen::Matrix2d A;
  A << 3 * T2, 4 * T3,
       6 * T, 12 * T2;

  Eigen::Vector2d b;
  b << vf - p.b1 - 2 * p.b2 * T,
       af - 2 * p.b2;

  Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
  p.b3 = x(0);
  p.b4 = x(1);
  return p;
}

// A lattice node at a layer boundary
struct LatticeNode {
  double s;
  double d;
  double v;
};

// Sampled Cartesian trajectory from a polynomial segment
struct SegmentTrajectory {
  std::vector<double> x_pts;
  std::vector<double> y_pts;
  std::vector<double> yaw_pts;
  std::vector<double> speed_pts;
  int num_steps;
};

SegmentTrajectory SampleSegment(const QuinticPoly& lat_poly,
                                const QuarticPoly& lon_poly,
                                double ref_heading,
                                double ref_ox, double ref_oy,
                                int num_steps, double dt) {
  SegmentTrajectory seg;
  seg.num_steps = num_steps;
  seg.x_pts.resize(num_steps + 1);
  seg.y_pts.resize(num_steps + 1);
  seg.yaw_pts.resize(num_steps + 1);
  seg.speed_pts.resize(num_steps + 1);

  const double cos_h = std::cos(ref_heading);
  const double sin_h = std::sin(ref_heading);

  for (int k = 0; k <= num_steps; ++k) {
    const double t = k * dt;
    const double s = lon_poly.Eval(t);
    const double d = lat_poly.Eval(t);
    const double ds = lon_poly.EvalDeriv(t);
    const double dd = lat_poly.EvalDeriv(t);

    seg.x_pts[k] = ref_ox + s * cos_h - d * sin_h;
    seg.y_pts[k] = ref_oy + s * sin_h + d * cos_h;
    seg.speed_pts[k] = std::max(std::sqrt(ds * ds + dd * dd), 0.0);
    seg.yaw_pts[k] = std::atan2(ds * sin_h + dd * cos_h,
                                 ds * cos_h - dd * sin_h);
  }
  return seg;
}

struct PathCandidate {
  std::vector<double> x_pts;
  std::vector<double> y_pts;
  std::vector<double> yaw_pts;
  std::vector<double> speed_pts;
  double cost = 1.0e18;
  bool collision_free = true;
};

// Convert a Cartesian trajectory to control sequence [accel, steering]
std::vector<Vector> TrajectoryToControls(const PathCandidate& path,
                                         double dt, double wheelbase,
                                         int horizon) {
  std::vector<Vector> controls(horizon, Vector::Zero(2));
  const int n = static_cast<int>(path.speed_pts.size());

  for (int k = 0; k < horizon; ++k) {
    if (k + 1 < n) {
      double accel = (path.speed_pts[k + 1] - path.speed_pts[k]) / dt;
      controls[k](0) = std::clamp(accel, -1.0, 1.0);
    }
    if (k + 1 < n && path.speed_pts[k] > 0.01) {
      double dyaw = path.yaw_pts[k + 1] - path.yaw_pts[k];
      while (dyaw > M_PI) dyaw -= 2 * M_PI;
      while (dyaw < -M_PI) dyaw += 2 * M_PI;
      double kappa = dyaw / (path.speed_pts[k] * dt);
      double steer = std::atan(kappa * wheelbase);
      controls[k](1) = std::clamp(steer, -0.45, 0.45);
    }
  }
  return controls;
}

double EvaluateCandidate(PathCandidate& path,
                         double target_speed,
                         double road_half_width,
                         double vehicle_radius,
                         const LatticeObstacleInfo& obstacle,
                         double dt,
                         const LatticeConfig& config) {
  const int n = static_cast<int>(path.x_pts.size());
  double cost = 0.0;
  path.collision_free = true;

  for (int k = 0; k < n; ++k) {
    const double d = path.y_pts[k];
    cost += config.lateral_weight * d * d;

    const double dv = path.speed_pts[k] - target_speed;
    cost += config.speed_weight * dv * dv;

    if (std::abs(d) + vehicle_radius > road_half_width) {
      cost += config.collision_penalty;
      path.collision_free = false;
    }

    const double obs_x = obstacle.center_x + obstacle.speed_x * k * dt;
    const double obs_y = obstacle.center_y;
    const double dx = path.x_pts[k] - obs_x;
    const double dy = path.y_pts[k] - obs_y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    const double min_dist = obstacle.radius + vehicle_radius + config.safety_clearance;
    if (dist < min_dist) {
      cost += config.collision_penalty * (min_dist - dist) / min_dist;
      path.collision_free = false;
    }
  }

  // Control smoothness: penalize accel and curvature changes
  for (int k = 0; k + 1 < n; ++k) {
    const double accel = (path.speed_pts[k + 1] - path.speed_pts[k]) / dt;
    cost += config.accel_weight * accel * accel;

    if (k + 2 < n) {
      const double dyaw1 = path.yaw_pts[k + 1] - path.yaw_pts[k];
      const double dyaw2 = path.yaw_pts[k + 2] - path.yaw_pts[k + 1];
      const double steer_rate = (dyaw2 - dyaw1) / (dt * dt);
      cost += config.steering_weight * steer_rate * steer_rate;
    }
  }

  path.cost = cost;
  return cost;
}

}  // namespace

LatticeInitialGuess::LatticeInitialGuess(
    std::shared_ptr<StraightReferenceLine> ref_line, double wheelbase)
    : ref_line_(std::move(ref_line)), wheelbase_(wheelbase) {}

LatticeResult LatticeInitialGuess::Generate(
    const Vector& initial_state,
    double target_speed,
    double road_half_width,
    double vehicle_radius,
    const LatticeObstacleInfo& obstacle,
    int horizon,
    double dt,
    const LatticeConfig& config) const {

  LatticeResult result;
  const double total_time = horizon * dt;
  const int num_layers = std::max(config.num_layers, 1);
  const double ref_heading = ref_line_->Heading();
  const double ref_ox = ref_line_->OriginX();
  const double ref_oy = ref_line_->OriginY();

  const double s0 = ref_line_->LongitudinalPosition(initial_state);
  const double d0 = ref_line_->LateralError(initial_state);
  const double yaw0 = initial_state(2);
  const double v0 = initial_state(3);

  const double dd0 = v0 * std::sin(yaw0 - ref_heading);
  const double ds0 = v0 * std::cos(yaw0 - ref_heading);

  // Compute layer boundaries (timestep indices)
  std::vector<int> layer_steps(num_layers + 1);
  layer_steps[0] = 0;
  for (int l = 1; l <= num_layers; ++l) {
    layer_steps[l] = horizon * l / num_layers;
  }

  const auto& lat_offsets = config.lateral_offsets;
  const auto& spd_ratios = config.speed_ratios;
  const int n_lat = static_cast<int>(lat_offsets.size());
  const int n_spd = static_cast<int>(spd_ratios.size());
  const int n_nodes = n_lat * n_spd;

  // DP tables: cost_dp[node_idx] = min cost to reach this node from start
  // prev_dp[node_idx] = predecessor node index at previous layer (-1 for start)
  // seg_cache: segment trajectories for path reconstruction
  struct DPEntry {
    double cost = 1.0e18;
    int prev = -1;
    FrenetState state = {0, 0, 0, 0};
  };

  auto node_idx = [&](int lat, int spd) { return lat * n_spd + spd; };

  // Layer 0 (start) → Layer 1
  std::vector<DPEntry> prev_layer_dp(n_nodes);
  // Store all segment trajectories for reconstruction
  // segments[layer][to_node] = the segment trajectory taken to reach to_node
  std::vector<std::vector<SegmentTrajectory>> segments(num_layers);
  for (int l = 0; l < num_layers; ++l) {
    segments[l].resize(n_nodes);
  }
  // Also need prev index per layer for reconstruction
  std::vector<std::vector<int>> prev_indices(num_layers);
  for (int l = 0; l < num_layers; ++l) {
    prev_indices[l].resize(n_nodes, -1);
  }

  // Process layer 0 → layer 1
  {
    const int steps = layer_steps[1] - layer_steps[0];
    const double seg_T = steps * dt;

    for (int j = 0; j < n_nodes; ++j) {
      const int lat_j = j / n_spd;
      const int spd_j = j % n_spd;
      const double d_target = lat_offsets[lat_j];
      const double v_target = spd_ratios[spd_j] * target_speed;

      if (v_target < 0) continue;

      const double s_target = s0 + (ds0 + v_target) / 2.0 * seg_T;

      QuinticPoly lat_poly = SolveQuintic(d0, dd0, 0.0, d_target, 0.0, 0.0, seg_T);
      QuarticPoly lon_poly = SolveQuartic(s0, ds0, 0.0, v_target, 0.0, seg_T);

      auto seg = SampleSegment(lat_poly, lon_poly, ref_heading,
                                ref_ox, ref_oy, steps, dt);

      PathCandidate candidate;
      candidate.x_pts = seg.x_pts;
      candidate.y_pts = seg.y_pts;
      candidate.yaw_pts = seg.yaw_pts;
      candidate.speed_pts = seg.speed_pts;

      EvaluateCandidate(candidate, target_speed, road_half_width,
                        vehicle_radius, obstacle, dt, config);

      if (candidate.cost < prev_layer_dp[j].cost) {
        prev_layer_dp[j].cost = candidate.cost;
        prev_layer_dp[j].prev = -1;
        prev_layer_dp[j].state = {lon_poly.Eval(seg_T), lat_poly.Eval(seg_T),
                                  lon_poly.EvalDeriv(seg_T), lat_poly.EvalDeriv(seg_T)};
        segments[0][j] = std::move(seg);
        prev_indices[0][j] = -1;
      }
    }
  }

  // Process layer l → layer l+1
  for (int l = 1; l < num_layers; ++l) {
    std::vector<DPEntry> curr_dp(n_nodes);
    const int steps = layer_steps[l + 1] - layer_steps[l];
    const double seg_T = steps * dt;
    const double t_offset = layer_steps[l] * dt;

    for (int j = 0; j < n_nodes; ++j) {
      const int lat_j = j / n_spd;
      const int spd_j = j % n_spd;
      const double d_target = lat_offsets[lat_j];
      const double v_target = spd_ratios[spd_j] * target_speed;

      if (v_target < 0) continue;

      for (int i = 0; i < n_nodes; ++i) {
        if (prev_layer_dp[i].cost >= 1.0e17) continue;

        const auto& from = prev_layer_dp[i].state;

        QuinticPoly lat_poly = SolveQuintic(from.d, from.dd, 0.0,
                                            d_target, 0.0, 0.0, seg_T);
        QuarticPoly lon_poly = SolveQuartic(from.s, from.ds, 0.0,
                                            v_target, 0.0, seg_T);

        auto seg = SampleSegment(lat_poly, lon_poly, ref_heading,
                                  ref_ox, ref_oy, steps, dt);

        // Adjust obstacle time offset for this segment
        PathCandidate candidate;
        candidate.x_pts = seg.x_pts;
        candidate.y_pts = seg.y_pts;
        candidate.yaw_pts = seg.yaw_pts;
        candidate.speed_pts = seg.speed_pts;

        // Shift obstacle evaluation to correct time
        LatticeObstacleInfo shifted_obs = obstacle;
        shifted_obs.center_x = obstacle.center_x + obstacle.speed_x * t_offset;

        EvaluateCandidate(candidate, target_speed, road_half_width,
                          vehicle_radius, shifted_obs, dt, config);

        const double total_cost = prev_layer_dp[i].cost + candidate.cost;
        if (total_cost < curr_dp[j].cost) {
          curr_dp[j].cost = total_cost;
          curr_dp[j].prev = i;
          curr_dp[j].state = {lon_poly.Eval(seg_T), lat_poly.Eval(seg_T),
                              lon_poly.EvalDeriv(seg_T), lat_poly.EvalDeriv(seg_T)};
          segments[l][j] = std::move(seg);
          prev_indices[l][j] = i;
        }
      }
    }

    prev_layer_dp = std::move(curr_dp);
  }

  // Find best terminal node
  int best_node = -1;
  double best_cost = 1.0e18;
  for (int j = 0; j < n_nodes; ++j) {
    if (prev_layer_dp[j].cost < best_cost) {
      best_cost = prev_layer_dp[j].cost;
      best_node = j;
    }
  }

  result.num_candidates = n_nodes;
  result.num_collision_free = 0;

  if (best_node < 0) {
    result.controls.assign(horizon, Vector::Zero(2));
    return result;
  }

  // Helper: reconstruct full path for a given terminal node index
  auto ReconstructPath = [&](int terminal_node) -> PathCandidate {
    std::vector<int> pnodes(num_layers);
    pnodes[num_layers - 1] = terminal_node;
    for (int l = num_layers - 1; l > 0; --l) {
      pnodes[l - 1] = prev_indices[l][pnodes[l]];
    }

    PathCandidate path;
    path.x_pts.reserve(horizon + 1);
    path.y_pts.reserve(horizon + 1);
    path.yaw_pts.reserve(horizon + 1);
    path.speed_pts.reserve(horizon + 1);

    for (int l = 0; l < num_layers; ++l) {
      const auto& seg = segments[l][pnodes[l]];
      const int start_k = (l == 0) ? 0 : 1;
      for (int k = start_k; k <= seg.num_steps; ++k) {
        path.x_pts.push_back(seg.x_pts[k]);
        path.y_pts.push_back(seg.y_pts[k]);
        path.yaw_pts.push_back(seg.yaw_pts[k]);
        path.speed_pts.push_back(seg.speed_pts[k]);
      }
    }

    while (static_cast<int>(path.x_pts.size()) < horizon + 1) {
      path.x_pts.push_back(path.x_pts.back());
      path.y_pts.push_back(path.y_pts.back());
      path.yaw_pts.push_back(path.yaw_pts.back());
      path.speed_pts.push_back(path.speed_pts.back());
    }
    if (static_cast<int>(path.x_pts.size()) > horizon + 1) {
      path.x_pts.resize(horizon + 1);
      path.y_pts.resize(horizon + 1);
      path.yaw_pts.resize(horizon + 1);
      path.speed_pts.resize(horizon + 1);
    }
    return path;
  };

  // Reconstruct paths for ALL reachable terminal nodes → candidates for visualization
  for (int j = 0; j < n_nodes; ++j) {
    if (prev_layer_dp[j].cost >= 1.0e17) continue;

    auto path = ReconstructPath(j);
    result.num_candidates++;

    LatticeCandidate cand;
    cand.x = std::move(path.x_pts);
    cand.y = std::move(path.y_pts);
    cand.cost = prev_layer_dp[j].cost;
    cand.collision_free = (prev_layer_dp[j].cost < config.collision_penalty);
    cand.is_best = (j == best_node);
    if (cand.collision_free) result.num_collision_free++;
    result.candidates.push_back(std::move(cand));
  }

  // Best path → controls
  auto best_path = ReconstructPath(best_node);
  result.controls = TrajectoryToControls(best_path, dt, wheelbase_, horizon);
  result.best_cost = best_cost;

  return result;
}

}  // namespace my_al_ilqr
