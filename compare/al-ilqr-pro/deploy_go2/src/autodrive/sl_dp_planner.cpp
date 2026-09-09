#include "autodrive/sl_dp_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Dense>

namespace my_al_ilqr {

namespace {

struct FrenetState {
  double s = 0.0;
  double d = 0.0;
  double ds = 0.0;
  double dd = 0.0;
};

struct QuinticPoly {
  double a0 = 0.0, a1 = 0.0, a2 = 0.0, a3 = 0.0, a4 = 0.0, a5 = 0.0;

  double Eval(double t) const {
    return a0 + t * (a1 + t * (a2 + t * (a3 + t * (a4 + t * a5))));
  }
  double EvalDeriv(double t) const {
    return a1 + t * (2.0 * a2 + t * (3.0 * a3 + t * (4.0 * a4 + t * 5.0 * a5)));
  }
};

struct QuarticPoly {
  double b0 = 0.0, b1 = 0.0, b2 = 0.0, b3 = 0.0, b4 = 0.0;

  double Eval(double t) const {
    return b0 + t * (b1 + t * (b2 + t * (b3 + t * b4)));
  }
  double EvalDeriv(double t) const {
    return b1 + t * (2.0 * b2 + t * (3.0 * b3 + t * 4.0 * b4));
  }
};

struct SegmentTrajectory {
  std::vector<GuidancePoint> points;
  int num_steps = 0;
};

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
  A << T3,  T4,  T5,
       3*T2, 4*T3, 5*T4,
       6*T, 12*T2, 20*T3;

  Eigen::Vector3d b;
  b << df - p.a0 - p.a1 * T - p.a2 * T2,
       ddf - p.a1 - 2.0 * p.a2 * T,
       dddf - 2.0 * p.a2;

  const Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
  p.a3 = x(0);
  p.a4 = x(1);
  p.a5 = x(2);
  return p;
}

QuarticPoly SolveQuartic(double s0, double v0, double a0,
                         double vf, double af, double T) {
  QuarticPoly p;
  p.b0 = s0;
  p.b1 = v0;
  p.b2 = a0 / 2.0;

  const double T2 = T * T;
  const double T3 = T2 * T;

  Eigen::Matrix2d A;
  A << 3.0 * T2, 4.0 * T3,
       6.0 * T, 12.0 * T2;

  Eigen::Vector2d b;
  b << vf - p.b1 - 2.0 * p.b2 * T,
       af - 2.0 * p.b2;

  const Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
  p.b3 = x(0);
  p.b4 = x(1);
  return p;
}

SegmentTrajectory SampleSegment(const QuinticPoly& lat_poly,
                                const QuarticPoly& lon_poly,
                                double ref_heading,
                                double ref_ox,
                                double ref_oy,
                                int num_steps,
                                int start_step,
                                double dt) {
  SegmentTrajectory seg;
  seg.num_steps = num_steps;
  seg.points.resize(num_steps + 1);

  const double cos_h = std::cos(ref_heading);
  const double sin_h = std::sin(ref_heading);

  for (int k = 0; k <= num_steps; ++k) {
    const double local_t = k * dt;
    const double s = lon_poly.Eval(local_t);
    const double d = lat_poly.Eval(local_t);
    const double ds = lon_poly.EvalDeriv(local_t);
    const double dd = lat_poly.EvalDeriv(local_t);

    GuidancePoint point;
    point.s = s;
    point.d = d;
    point.x = ref_ox + s * cos_h - d * sin_h;
    point.y = ref_oy + s * sin_h + d * cos_h;
    point.speed = std::max(std::sqrt(ds * ds + dd * dd), 0.0);
    point.yaw = std::atan2(ds * sin_h + dd * cos_h,
                           ds * cos_h - dd * sin_h);
    point.t = (start_step + k) * dt;
    seg.points[k] = point;
  }
  return seg;
}

double NormalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

GuidancePoint InterpolatePoint(const GuidancePoint& a,
                               const GuidancePoint& b,
                               double alpha) {
  GuidancePoint p;
  p.x = (1.0 - alpha) * a.x + alpha * b.x;
  p.y = (1.0 - alpha) * a.y + alpha * b.y;
  p.s = (1.0 - alpha) * a.s + alpha * b.s;
  p.d = (1.0 - alpha) * a.d + alpha * b.d;
  p.speed = (1.0 - alpha) * a.speed + alpha * b.speed;
  p.t = (1.0 - alpha) * a.t + alpha * b.t;
  p.yaw = NormalizeAngle(a.yaw + alpha * NormalizeAngle(b.yaw - a.yaw));
  return p;
}

Vector ObstacleCenterAt(const PredictedObstacle& obs, int step, double alpha) {
  if (obs.centers.empty()) {
    return Vector::Zero(2);
  }
  const int i0 = std::clamp(step, 0, static_cast<int>(obs.centers.size()) - 1);
  const int i1 = std::min(i0 + 1, static_cast<int>(obs.centers.size()) - 1);
  if (i0 == i1 || alpha <= 1e-6) return obs.centers[i0];
  Vector c(2);
  c << (1.0 - alpha) * obs.centers[i0](0) + alpha * obs.centers[i1](0),
       (1.0 - alpha) * obs.centers[i0](1) + alpha * obs.centers[i1](1);
  return c;
}

double SegmentLength(const std::vector<GuidancePoint>& points) {
  double len = 0.0;
  for (size_t k = 0; k + 1 < points.size(); ++k) {
    const double dx = points[k + 1].x - points[k].x;
    const double dy = points[k + 1].y - points[k].y;
    len += std::sqrt(dx * dx + dy * dy);
  }
  return len;
}

double EvaluateSegmentCost(const std::vector<GuidancePoint>& points,
                           int global_start_step,
                           double target_speed,
                           double road_half_width,
                           const StraightReferenceLine& ref_line,
                           const VehicleCircleApproximation& vehicle_geometry,
                           const std::vector<PredictedObstacle>& obstacles,
                           double dt,
                           const SLDPConfig& config,
                           bool* collision_free) {
  double cost = 0.0;
  bool safe = true;
  const int substeps = std::max(1, config.collision_check_substeps);
  const double ref_heading = ref_line.Heading();

  auto evaluate_point = [&](const GuidancePoint& point,
                            int base_step, double alpha, double w) {
    cost += w * config.lateral_weight * point.d * point.d;

    const double dv = point.speed - target_speed;
    cost += w * config.speed_weight * dv * dv;

    const double heading_err = NormalizeAngle(point.yaw - ref_heading);
    cost += w * config.heading_weight * heading_err * heading_err;

    Vector state(4);
    state << point.x, point.y, point.yaw, point.speed;
    const auto centers = vehicle_geometry.EvaluateWorldCenters(state);
    const int global_step = global_start_step + base_step;

    for (int ci = 0; ci < vehicle_geometry.CircleCount(); ++ci) {
      Vector cs(2);
      cs << centers[ci](0), centers[ci](1);
      const double lat = ref_line.LateralError(cs);
      const double cr = vehicle_geometry.Circles()[ci].radius;

      if (std::abs(lat) + cr > road_half_width) {
        cost += config.collision_penalty;
        safe = false;
      }

      for (const auto& obs : obstacles) {
        const Vector oc = ObstacleCenterAt(obs, global_step, alpha);
        double signed_dist = 0.0;
        if (obs.shape == PredictedObstacleShape::kCircle) {
          const double dx = centers[ci](0) - oc(0);
          const double dy = centers[ci](1) - oc(1);
          signed_dist = std::sqrt(dx * dx + dy * dy) - obs.radius;
        } else {
          Vector cc(2);
          cc << centers[ci](0), centers[ci](1);
          signed_dist = MakeRectangle(oc(0), oc(1),
                                      obs.half_length, obs.half_width,
                                      obs.yaw).SignedDistance(cc);
        }

        const double min_dist = cr + config.safety_clearance;
        if (signed_dist < min_dist) {
          cost += config.collision_penalty *
                  (min_dist - signed_dist) / std::max(min_dist, 1e-6);
          safe = false;
          continue;
        }

        const double prox_limit = min_dist + std::max(0.0, config.proximity_buffer);
        if (signed_dist < prox_limit) {
          const double ratio = (prox_limit - signed_dist) /
                               std::max(config.proximity_buffer, 1e-6);
          cost += config.proximity_weight * ratio * ratio;
        }

        const double obs_s = ref_line.LongitudinalPosition(oc);
        const double obs_d = ref_line.LateralError(oc);
        const double obs_half_span =
            (obs.shape == PredictedObstacleShape::kCircle)
                ? obs.radius
                : std::abs(obs.half_length * std::cos(obs.yaw))
                  + std::abs(obs.half_width * std::sin(obs.yaw));
        const double gap = obs_s - point.s - obs_half_span;
        const double ant_dist = config.anticipation_distance;
        if (ant_dist > 1e-6 && gap > 0.0 && gap < ant_dist) {
          const double approach_ratio = (ant_dist - gap) / ant_dist;
          const double lat_sep = std::abs(point.d - obs_d);
          const double sep_deficit =
              std::max(0.0, config.anticipation_lateral_clearance - lat_sep);
          cost += w * config.anticipation_weight *
                  approach_ratio * approach_ratio *
                  sep_deficit * sep_deficit;
        }
      }
    }
  };

  if (points.empty()) {
    if (collision_free) *collision_free = safe;
    return cost;
  }

  if (points.size() == 1) {
    evaluate_point(points.front(), 0, 0.0, 1.0);
  } else {
    for (size_t k = 0; k + 1 < points.size(); ++k) {
      for (int sub = 0; sub < substeps; ++sub) {
        const double alpha = static_cast<double>(sub) / static_cast<double>(substeps);
        evaluate_point(InterpolatePoint(points[k], points[k + 1], alpha),
                       static_cast<int>(k), alpha,
                       1.0 / static_cast<double>(substeps));
      }
    }
    evaluate_point(points.back(), static_cast<int>(points.size()) - 1, 0.0, 1.0);
  }

  for (size_t k = 0; k + 1 < points.size(); ++k) {
    const double accel = (points[k + 1].speed - points[k].speed) / dt;
    cost += config.accel_weight * accel * accel;

    const double dd = (points[k + 1].d - points[k].d) / dt;
    cost += config.lateral_change_weight * dd * dd;
  }

  for (size_t k = 0; k + 2 < points.size(); ++k) {
    if (points[k].speed > 1e-3) {
      const double dyaw1 = NormalizeAngle(points[k + 1].yaw - points[k].yaw);
      const double dyaw2 = NormalizeAngle(points[k + 2].yaw - points[k + 1].yaw);
      const double steer_rate = (dyaw2 - dyaw1) / (dt * dt);
      cost += config.steering_weight * steer_rate * steer_rate;
    }
  }

  cost += config.path_length_weight * SegmentLength(points);

  if (collision_free) *collision_free = safe;
  return cost;
}

std::vector<GuidancePoint> ReconstructPath(
    int terminal_node, int num_layers,
    const std::vector<std::vector<SegmentTrajectory>>& segments,
    const std::vector<std::vector<int>>& prev_indices,
    int horizon) {
  std::vector<int> nodes(num_layers, -1);
  nodes[num_layers - 1] = terminal_node;
  for (int l = num_layers - 1; l > 0; --l) {
    nodes[l - 1] = prev_indices[l][nodes[l]];
  }

  std::vector<GuidancePoint> points;
  points.reserve(horizon + 1);
  for (int l = 0; l < num_layers; ++l) {
    const auto& seg = segments[l][nodes[l]];
    const int start_k = (l == 0) ? 0 : 1;
    for (int k = start_k; k <= seg.num_steps; ++k) {
      points.push_back(seg.points[k]);
    }
  }

  while (static_cast<int>(points.size()) < horizon + 1 && !points.empty()) {
    points.push_back(points.back());
  }
  if (static_cast<int>(points.size()) > horizon + 1) {
    points.resize(horizon + 1);
  }
  return points;
}

std::vector<Vector> PointsToControls(const std::vector<GuidancePoint>& points,
                                     double dt, double wheelbase,
                                     const SLDPConfig& config, int horizon) {
  std::vector<Vector> controls(horizon, Vector::Zero(2));
  if (points.size() < 2) return controls;

  double cx = points[0].x;
  double cy = points[0].y;
  double cyaw = points[0].yaw;
  double cv = points[0].speed;

  for (int k = 0; k < horizon; ++k) {
    if (k + 1 >= static_cast<int>(points.size())) break;
    const auto& target = points[k + 1];

    const double accel = (target.speed - cv) / dt;
    controls[k](0) = std::clamp(accel, config.min_acceleration, config.max_acceleration);

    if (cv > 1e-3) {
      const double dx = target.x - cx;
      const double dy = target.y - cy;

      const double cross_track = -std::sin(cyaw) * dx + std::cos(cyaw) * dy;

      const double heading_err = NormalizeAngle(target.yaw - cyaw);
      const double ct_gain = 2.0;
      const double ct_correction = std::atan2(ct_gain * cross_track,
                                              std::max(cv, 0.5));

      const double total_dyaw = heading_err + ct_correction;
      const double curvature = total_dyaw / std::max(cv * dt, 1e-3);
      const double steer = std::atan(curvature * wheelbase);
      controls[k](1) = std::clamp(steer, config.min_steering, config.max_steering);
    }

    cx += cv * std::cos(cyaw) * dt;
    cy += cv * std::sin(cyaw) * dt;
    cyaw += cv * std::tan(controls[k](1)) / wheelbase * dt;
    cyaw = NormalizeAngle(cyaw);
    cv += controls[k](0) * dt;
    cv = std::max(0.0, cv);
  }
  return controls;
}

}  // namespace

SLDPPlanner::SLDPPlanner(std::shared_ptr<StraightReferenceLine> ref_line,
                         double wheelbase)
    : ref_line_(std::move(ref_line)), wheelbase_(wheelbase) {}

GlobalGuidanceResult SLDPPlanner::Generate(
    const Vector& initial_state,
    double target_speed,
    double road_half_width,
    const VehicleCircleApproximation& vehicle_geometry,
    const std::vector<PredictedObstacle>& obstacles,
    int horizon,
    double dt,
    const SLDPConfig& config) const {
  if (!ref_line_) {
    throw std::invalid_argument("SLDPPlanner requires a valid reference line.");
  }
  if (initial_state.size() < 4) {
    throw std::invalid_argument("SLDPPlanner requires a 4D initial state.");
  }
  if (horizon <= 0 || dt <= 0.0) {
    throw std::invalid_argument("SLDPPlanner requires positive horizon and dt.");
  }

  GlobalGuidanceResult result;

  const int num_layers = std::max(1, config.num_layers);
  const int n_lat = std::max(1, config.lateral_samples);
  const int n_spd = std::max(1, config.speed_samples);
  const int n_nodes = n_lat * n_spd;

  // Build lateral offsets: uniform in [-lateral_span, lateral_span]
  std::vector<double> lat_offsets(n_lat);
  for (int i = 0; i < n_lat; ++i) {
    lat_offsets[i] = (n_lat == 1)
        ? 0.0
        : -config.lateral_span + 2.0 * config.lateral_span * i / (n_lat - 1);
  }

  // Build speed targets: uniform in [0, max_speed_ratio * target_speed]
  std::vector<double> speed_targets(n_spd);
  for (int i = 0; i < n_spd; ++i) {
    speed_targets[i] = (n_spd == 1)
        ? target_speed
        : config.max_speed_ratio * target_speed * i / (n_spd - 1);
  }

  // Layer step boundaries
  std::vector<int> layer_steps(num_layers + 1);
  for (int l = 0; l <= num_layers; ++l) {
    layer_steps[l] = l * horizon / num_layers;
  }
  layer_steps[num_layers] = horizon;

  // Frenet initial state
  const double ref_heading = ref_line_->Heading();
  const double ref_ox = ref_line_->OriginX();
  const double ref_oy = ref_line_->OriginY();

  Vector init_pos(2);
  init_pos << initial_state(0), initial_state(1);
  const double s0 = ref_line_->LongitudinalPosition(init_pos);
  const double d0 = ref_line_->LateralError(init_pos);
  const double yaw0 = initial_state(2);
  const double v0 = initial_state(3);
  const double ds0 = v0 * std::cos(yaw0 - ref_heading);
  const double dd0 = v0 * std::sin(yaw0 - ref_heading);

  struct DPEntry {
    double cost = 1.0e18;
    FrenetState state;
    bool reachable = false;
  };

  std::vector<DPEntry> prev_dp(n_nodes);
  std::vector<std::vector<SegmentTrajectory>> segments(
      num_layers, std::vector<SegmentTrajectory>(n_nodes));
  std::vector<std::vector<int>> prev_indices(
      num_layers, std::vector<int>(n_nodes, -1));

  // Layer 0: connect start → each node
  {
    const int steps = layer_steps[1] - layer_steps[0];
    const double seg_T = steps * dt;
    if (seg_T < 1e-9) {
      result.controls.assign(horizon, Vector::Zero(2));
      return result;
    }

    for (int j = 0; j < n_nodes; ++j) {
      const int lat_j = j / n_spd;
      const int spd_j = j % n_spd;
      const double d_target = lat_offsets[lat_j];
      const double v_target = speed_targets[spd_j];

      const QuinticPoly lat_poly = SolveQuintic(d0, dd0, 0.0, d_target, 0.0, 0.0, seg_T);
      const QuarticPoly lon_poly = SolveQuartic(s0, ds0, 0.0, v_target, 0.0, seg_T);
      SegmentTrajectory seg = SampleSegment(lat_poly, lon_poly, ref_heading,
                                            ref_ox, ref_oy, steps, layer_steps[0], dt);

      bool col_free = true;
      const double cost = EvaluateSegmentCost(
          seg.points, layer_steps[0], target_speed, road_half_width,
          *ref_line_, vehicle_geometry, obstacles, dt, config, &col_free);

      prev_dp[j].cost = cost;
      prev_dp[j].state = FrenetState{
          lon_poly.Eval(seg_T), lat_poly.Eval(seg_T),
          lon_poly.EvalDeriv(seg_T), lat_poly.EvalDeriv(seg_T)};
      prev_dp[j].reachable = true;
      segments[0][j] = std::move(seg);
      prev_indices[0][j] = -1;
    }
  }

  // Subsequent layers
  for (int l = 1; l < num_layers; ++l) {
    std::vector<DPEntry> curr_dp(n_nodes);
    const int steps = layer_steps[l + 1] - layer_steps[l];
    const double seg_T = steps * dt;
    if (seg_T < 1e-9) continue;

    for (int j = 0; j < n_nodes; ++j) {
      const int lat_j = j / n_spd;
      const int spd_j = j % n_spd;
      const double d_target = lat_offsets[lat_j];
      const double v_target = speed_targets[spd_j];

      for (int i = 0; i < n_nodes; ++i) {
        if (!prev_dp[i].reachable) continue;

        const int lat_i = i / n_spd;
        const int spd_i = i % n_spd;
        if (std::abs(lat_j - lat_i) > std::max(0, config.max_lateral_index_jump)) continue;
        if (std::abs(spd_j - spd_i) > std::max(0, config.max_speed_index_jump)) continue;

        const auto& from = prev_dp[i].state;
        const QuinticPoly lat_poly = SolveQuintic(from.d, from.dd, 0.0,
                                                   d_target, 0.0, 0.0, seg_T);
        const QuarticPoly lon_poly = SolveQuartic(from.s, from.ds, 0.0,
                                                   v_target, 0.0, seg_T);
        SegmentTrajectory seg = SampleSegment(lat_poly, lon_poly, ref_heading,
                                              ref_ox, ref_oy, steps, layer_steps[l], dt);

        bool col_free = true;
        const double edge_cost = EvaluateSegmentCost(
            seg.points, layer_steps[l], target_speed, road_half_width,
            *ref_line_, vehicle_geometry, obstacles, dt, config, &col_free);

        const double total_cost = prev_dp[i].cost + edge_cost;
        if (total_cost < curr_dp[j].cost) {
          curr_dp[j].cost = total_cost;
          curr_dp[j].state = FrenetState{
              lon_poly.Eval(seg_T), lat_poly.Eval(seg_T),
              lon_poly.EvalDeriv(seg_T), lat_poly.EvalDeriv(seg_T)};
          curr_dp[j].reachable = true;
          segments[l][j] = std::move(seg);
          prev_indices[l][j] = i;
        }
      }
    }
    prev_dp = std::move(curr_dp);
  }

  // Collect DP lattice nodes for visualization
  result.dp_lattice_nodes.reserve(num_layers * n_nodes);
  for (int l = 0; l < num_layers; ++l) {
    for (int j = 0; j < n_nodes; ++j) {
      DPLatticeNode node;
      node.layer = l;
      node.lat_index = j / n_spd;
      node.spd_index = j % n_spd;
      const auto& seg = segments[l][j];
      if (!seg.points.empty()) {
        node.x = seg.points.back().x;
        node.y = seg.points.back().y;
        node.reachable = true;
      } else {
        const double approx_s = s0 + target_speed * layer_steps[l + 1] * dt;
        const double d_target = lat_offsets[node.lat_index];
        node.x = ref_ox + approx_s * std::cos(ref_heading) - d_target * std::sin(ref_heading);
        node.y = ref_oy + approx_s * std::sin(ref_heading) + d_target * std::cos(ref_heading);
        node.reachable = false;
      }
      result.dp_lattice_nodes.push_back(node);
    }
  }

  // Collect all reachable terminal candidates
  int best_j = -1;
  double best_cost = std::numeric_limits<double>::max();

  for (int j = 0; j < n_nodes; ++j) {
    if (!prev_dp[j].reachable) continue;

    std::vector<GuidancePoint> points =
        ReconstructPath(j, num_layers, segments, prev_indices, horizon);
    if (points.empty()) continue;

    bool col_free = true;
    double total_cost = EvaluateSegmentCost(
        points, 0, target_speed, road_half_width,
        *ref_line_, vehicle_geometry, obstacles, dt, config, &col_free);

    GuidanceCandidate candidate;
    candidate.points = points;
    candidate.controls = PointsToControls(points, dt, wheelbase_, config, horizon);
    candidate.cost = total_cost;
    candidate.collision_free = col_free;
    candidate.topology_id = j;

    if (total_cost < best_cost) {
      best_cost = total_cost;
      best_j = static_cast<int>(result.candidate_bundle.size());
    }
    result.candidate_bundle.push_back(std::move(candidate));
  }

  if (best_j < 0 || result.candidate_bundle.empty()) {
    result.controls.assign(horizon, Vector::Zero(2));
    return result;
  }

  std::sort(result.candidate_bundle.begin(), result.candidate_bundle.end(),
            [](const GuidanceCandidate& a, const GuidanceCandidate& b) {
              return a.cost < b.cost;
            });

  auto& best = result.candidate_bundle.front();
  best.selected = true;
  result.controls = best.controls;
  result.selected_guidance = best.points;
  result.best_cost = best.cost;
  result.selected_topology_id = best.topology_id;
  result.used_fallback = false;

  // Mark nodes on the best path
  const int best_terminal = best.topology_id;
  std::vector<int> best_path_nodes(num_layers, -1);
  best_path_nodes[num_layers - 1] = best_terminal;
  for (int l = num_layers - 1; l > 0; --l) {
    best_path_nodes[l - 1] = prev_indices[l][best_path_nodes[l]];
  }
  for (int l = 0; l < num_layers; ++l) {
    if (best_path_nodes[l] >= 0 && best_path_nodes[l] < n_nodes) {
      result.dp_lattice_nodes[l * n_nodes + best_path_nodes[l]].on_best_path = true;
    }
  }

  return result;
}

}  // namespace my_al_ilqr
