#include "autodrive/global_guidance_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
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
  double a0 = 0.0;
  double a1 = 0.0;
  double a2 = 0.0;
  double a3 = 0.0;
  double a4 = 0.0;
  double a5 = 0.0;

  double Eval(double t) const {
    return a0 + t * (a1 + t * (a2 + t * (a3 + t * (a4 + t * a5))));
  }
  double EvalDeriv(double t) const {
    return a1 + t * (2.0 * a2 + t * (3.0 * a3 + t * (4.0 * a4 + t * 5.0 * a5)));
  }
};

struct QuarticPoly {
  double b0 = 0.0;
  double b1 = 0.0;
  double b2 = 0.0;
  double b3 = 0.0;
  double b4 = 0.0;

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

struct DPEntry {
  double cost = 1.0e18;
  FrenetState state;
  bool reachable = false;
};

double NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

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

double SegmentLength(const std::vector<GuidancePoint>& points) {
  if (points.size() < 2) {
    return 0.0;
  }
  double total = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    const double dx = points[i].x - points[i - 1].x;
    const double dy = points[i].y - points[i - 1].y;
    total += std::sqrt(dx * dx + dy * dy);
  }
  return total;
}

GuidancePoint InterpolateGuidancePoint(const GuidancePoint& a,
                                       const GuidancePoint& b,
                                       double alpha) {
  GuidancePoint point;
  point.x = (1.0 - alpha) * a.x + alpha * b.x;
  point.y = (1.0 - alpha) * a.y + alpha * b.y;
  point.s = (1.0 - alpha) * a.s + alpha * b.s;
  point.d = (1.0 - alpha) * a.d + alpha * b.d;
  point.speed = (1.0 - alpha) * a.speed + alpha * b.speed;
  point.t = (1.0 - alpha) * a.t + alpha * b.t;
  const double yaw_delta = NormalizeAngle(b.yaw - a.yaw);
  point.yaw = NormalizeAngle(a.yaw + alpha * yaw_delta);
  return point;
}

int PredictionIndexForStep(const PredictedObstacle& obstacle, int step) {
  if (obstacle.centers.empty()) {
    return -1;
  }
  return std::clamp(step, 0, static_cast<int>(obstacle.centers.size()) - 1);
}

double ObstacleLongitudinalHalfSpan(const PredictedObstacle& obstacle) {
  if (obstacle.shape == PredictedObstacleShape::kCircle) {
    return obstacle.radius;
  }
  const double c = std::cos(obstacle.yaw);
  const double s = std::sin(obstacle.yaw);
  return std::abs(obstacle.half_length * c) + std::abs(obstacle.half_width * s);
}

double ObstacleLongitudinalSpeedAt(const PredictedObstacle& obstacle,
                                   int step,
                                   const StraightReferenceLine& ref_line,
                                   double dt) {
  if (dt <= 0.0 || obstacle.centers.size() < 2) {
    return 0.0;
  }
  const int i0 = PredictionIndexForStep(obstacle, step);
  if (i0 < 0) {
    return 0.0;
  }
  const int i1 = std::min(i0 + 1, static_cast<int>(obstacle.centers.size()) - 1);
  const int i_prev = std::max(0, i0 - 1);
  if (i1 == i0 && i_prev == i0) {
    return 0.0;
  }
  if (i1 != i0) {
    return (ref_line.LongitudinalPosition(obstacle.centers[i1]) -
            ref_line.LongitudinalPosition(obstacle.centers[i0])) / dt;
  }
  return (ref_line.LongitudinalPosition(obstacle.centers[i0]) -
          ref_line.LongitudinalPosition(obstacle.centers[i_prev])) / dt;
}

ConvexPolygon ObstaclePolygonAt(const PredictedObstacle& obstacle, int step) {
  const int obs_index = PredictionIndexForStep(obstacle, step);
  if (obs_index < 0) {
    throw std::invalid_argument("ObstaclePolygonAt requires a valid prediction step.");
  }
  return MakeRectangle(obstacle.centers[obs_index](0), obstacle.centers[obs_index](1),
                       obstacle.half_length, obstacle.half_width, obstacle.yaw);
}

Vector ObstacleCenterAt(const PredictedObstacle& obstacle, int step, double alpha) {
  const int i0 = PredictionIndexForStep(obstacle, step);
  if (i0 < 0) {
    throw std::invalid_argument("ObstacleCenterAt requires a valid prediction step.");
  }
  const int i1 = std::min(i0 + 1, static_cast<int>(obstacle.centers.size()) - 1);
  if (i0 == i1 || alpha <= 1e-6) {
    return obstacle.centers[i0];
  }
  if (alpha >= 1.0 - 1e-6) {
    return obstacle.centers[i1];
  }
  Vector center(2);
  center << (1.0 - alpha) * obstacle.centers[i0](0) + alpha * obstacle.centers[i1](0),
      (1.0 - alpha) * obstacle.centers[i0](1) + alpha * obstacle.centers[i1](1);
  return center;
}

double EvaluateSegmentCost(const std::vector<GuidancePoint>& points,
                           int global_start_step,
                           double target_speed,
                           double road_half_width,
                           const StraightReferenceLine& ref_line,
                           const VehicleCircleApproximation& vehicle_geometry,
                           const std::vector<PredictedObstacle>& obstacles,
                           double dt,
                           const GlobalGuidanceConfig& config,
                           bool* collision_free) {
  double cost = 0.0;
  bool safe = true;
  const int substeps = std::max(1, config.collision_check_substeps);

  auto evaluate_point = [&](const GuidancePoint& point,
                            int base_step,
                            double alpha,
                            double smooth_weight) {
    cost += smooth_weight * config.lateral_weight * point.d * point.d;

    const double dv = point.speed - target_speed;
    cost += smooth_weight * config.speed_weight * dv * dv;

    Vector state(4);
    state << point.x, point.y, point.yaw, point.speed;
    const auto centers = vehicle_geometry.EvaluateWorldCenters(state);

    const int global_step = global_start_step + base_step;
    for (int circle_index = 0; circle_index < vehicle_geometry.CircleCount(); ++circle_index) {
      Vector center_state(2);
      center_state << centers[circle_index](0), centers[circle_index](1);
      const double lateral = ref_line.LateralError(center_state);
      const double circle_radius = vehicle_geometry.Circles()[circle_index].radius;
      if (std::abs(lateral) + circle_radius > road_half_width) {
        cost += config.collision_penalty;
        safe = false;
      }

      for (const auto& obstacle : obstacles) {
        const Vector obs_center = ObstacleCenterAt(obstacle, global_step, alpha);
        const double obs_s = ref_line.LongitudinalPosition(obs_center);
        const double obs_d = ref_line.LateralError(obs_center);
        const double obstacle_half_span = ObstacleLongitudinalHalfSpan(obstacle);
        double signed_distance = 0.0;
        if (obstacle.shape == PredictedObstacleShape::kCircle) {
          const double dx = centers[circle_index](0) - obs_center(0);
          const double dy = centers[circle_index](1) - obs_center(1);
          signed_distance = std::sqrt(dx * dx + dy * dy) - obstacle.radius;
        } else {
          Vector circle_center(2);
          circle_center << centers[circle_index](0), centers[circle_index](1);
          signed_distance = MakeRectangle(obs_center(0), obs_center(1),
                                         obstacle.half_length, obstacle.half_width,
                                         obstacle.yaw).SignedDistance(circle_center);
        }

        const double min_dist = circle_radius + config.safety_clearance;
        if (signed_distance < min_dist) {
          cost += config.collision_penalty *
                  (min_dist - signed_distance) / std::max(min_dist, 1e-6);
          safe = false;
          continue;
        }

        const double proximity_limit = min_dist + std::max(0.0, config.proximity_buffer);
        if (signed_distance < proximity_limit) {
          const double ratio =
              (proximity_limit - signed_distance) /
              std::max(config.proximity_buffer, 1e-6);
          cost += config.proximity_weight * ratio * ratio;
        }

        const double lateral_delta = point.d - obs_d;
        const double lateral_separation = std::abs(lateral_delta);
        const double anticipation_distance =
            obstacle_half_span +
            std::max(0.0, config.bypass_anticipation_distance) +
            config.follow_headway * std::max(0.0, point.speed);
        const double gap_to_obstacle = obs_s - point.s - obstacle_half_span;
        if (anticipation_distance > 1e-6 &&
            gap_to_obstacle > 0.0 &&
            gap_to_obstacle < anticipation_distance) {
          const double approach_ratio =
              (anticipation_distance - gap_to_obstacle) / anticipation_distance;
          const double separation_deficit =
              std::max(0.0, config.bypass_lateral_clearance - lateral_separation);
          cost += config.bypass_anticipation_weight *
                  smooth_weight *
                  approach_ratio * approach_ratio *
                  separation_deficit * separation_deficit;
        }

        if (std::abs(lateral_delta) <= config.follow_lateral_band) {
          if (gap_to_obstacle > 0.0) {
            const double obstacle_speed =
                ObstacleLongitudinalSpeedAt(obstacle, global_step, ref_line, dt);
            const double closing_speed = std::max(0.0, point.speed - obstacle_speed);
            if (closing_speed > 1e-3) {
              const double desired_gap =
                  config.follow_base_gap + config.follow_headway * closing_speed;
              if (gap_to_obstacle < desired_gap) {
                const double deficit = desired_gap - gap_to_obstacle;
                cost += config.follow_penalty * deficit * deficit;
              }
            }
          }
        }

        const double passed_distance = point.s - obs_s - obstacle_half_span;
        if (config.post_bypass_hold_distance > 1e-6 &&
            passed_distance >= 0.0 &&
            passed_distance < config.post_bypass_hold_distance) {
          const double hold_ratio =
              (config.post_bypass_hold_distance - passed_distance) /
              config.post_bypass_hold_distance;
          const double separation_deficit =
              std::max(0.0, config.bypass_lateral_clearance - lateral_separation);
          cost += config.post_bypass_hold_weight *
                  smooth_weight *
                  hold_ratio * hold_ratio *
                  separation_deficit * separation_deficit;
        }
      }
    }
  };

  if (points.empty()) {
    if (collision_free != nullptr) {
      *collision_free = safe;
    }
    return cost;
  }

  if (points.size() == 1) {
    evaluate_point(points.front(), 0, 0.0, 1.0);
  } else {
    for (size_t k = 0; k + 1 < points.size(); ++k) {
      for (int sub = 0; sub < substeps; ++sub) {
        const double alpha = static_cast<double>(sub) / static_cast<double>(substeps);
        const GuidancePoint point = InterpolateGuidancePoint(points[k], points[k + 1], alpha);
        evaluate_point(point, static_cast<int>(k), alpha, 1.0 / static_cast<double>(substeps));
      }
    }
    evaluate_point(points.back(), static_cast<int>(points.size()) - 1, 0.0, 1.0);
  }

  cost += config.path_length_weight * SegmentLength(points);

  for (size_t k = 0; k + 1 < points.size(); ++k) {
    const double accel = (points[k + 1].speed - points[k].speed) / dt;
    cost += config.accel_weight * accel * accel;

    if (k + 2 < points.size() && points[k].speed > 1e-3) {
      const double dyaw1 = NormalizeAngle(points[k + 1].yaw - points[k].yaw);
      const double dyaw2 = NormalizeAngle(points[k + 2].yaw - points[k + 1].yaw);
      const double steer_rate = (dyaw2 - dyaw1) / (dt * dt);
      cost += config.steering_weight * steer_rate * steer_rate;
    }
  }

  if (collision_free != nullptr) {
    *collision_free = safe;
  }
  return cost;
}

int RelationSign(double value) {
  return value >= 0.0 ? 1 : -1;
}

TopologySignature BuildSignature(const std::vector<GuidancePoint>& points,
                                 const std::vector<PredictedObstacle>& obstacles,
                                 const StraightReferenceLine& ref_line) {
  TopologySignature signature;
  std::ostringstream key;

  for (const auto& obstacle : obstacles) {
    if (obstacle.centers.empty()) {
      continue;
    }

    double best_dist_sq = std::numeric_limits<double>::infinity();
    int best_step = 0;
    for (int k = 0; k < static_cast<int>(points.size()); ++k) {
      const int obs_index = PredictionIndexForStep(obstacle, k);
      if (obs_index < 0) {
        continue;
      }
      const double dx = points[k].x - obstacle.centers[obs_index](0);
      const double dy = points[k].y - obstacle.centers[obs_index](1);
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq < best_dist_sq) {
        best_dist_sq = dist_sq;
        best_step = k;
      }
    }

    const int obs_index = PredictionIndexForStep(obstacle, best_step);
    if (obs_index < 0) {
      continue;
    }
    const double obs_s = ref_line.LongitudinalPosition(obstacle.centers[obs_index]);
    const double obs_d = ref_line.LateralError(obstacle.centers[obs_index]);

    TopologyInteraction interaction;
    interaction.obstacle_id = obstacle.obstacle_id;
    interaction.lateral_relation = RelationSign(points[best_step].d - obs_d);
    interaction.temporal_relation = RelationSign(points[best_step].s - obs_s);
    signature.interactions.push_back(interaction);

    if (!signature.key.empty()) {
      key << "|";
    }
    key << obstacle.obstacle_id << ":"
        << (interaction.lateral_relation > 0 ? "R" : "L")
        << (interaction.temporal_relation > 0 ? "B" : "A");
  }

  signature.key = key.str();
  return signature;
}

std::vector<GuidancePoint> ReconstructPath(
    int terminal_node,
    int num_layers,
    int horizon,
    const std::vector<std::vector<SegmentTrajectory>>& segments,
    const std::vector<std::vector<int>>& prev_indices) {
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
    GuidancePoint point = points.back();
    point.t += 0.0;
    points.push_back(point);
  }
  if (static_cast<int>(points.size()) > horizon + 1) {
    points.resize(horizon + 1);
  }
  return points;
}

std::unordered_map<std::string, int> BuildTopologyMap(const GlobalGuidanceMemory* previous_memory) {
  std::unordered_map<std::string, int> ids;
  if (previous_memory == nullptr) {
    return ids;
  }
  for (const auto& entry : previous_memory->topology_entries) {
    ids[entry.signature_key] = entry.topology_id;
  }
  return ids;
}

double GeometryConsistencyPenalty(const std::vector<GuidancePoint>& current,
                                  const std::vector<GuidancePoint>& previous) {
  if (current.empty() || previous.empty()) {
    return 0.0;
  }
  const int count = std::min(static_cast<int>(current.size()), static_cast<int>(previous.size()));
  if (count <= 0) {
    return 0.0;
  }

  double accum = 0.0;
  for (int k = 0; k < count; ++k) {
    const double dx = current[k].x - previous[k].x;
    const double dy = current[k].y - previous[k].y;
    accum += std::sqrt(dx * dx + dy * dy);
  }
  return accum / static_cast<double>(count);
}

int AssignTopologyId(const std::string& signature_key,
                     std::unordered_map<std::string, int>* ids,
                     int* next_topology_id) {
  const auto it = ids->find(signature_key);
  if (it != ids->end()) {
    return it->second;
  }
  const int topology_id = std::max(1, *next_topology_id);
  (*ids)[signature_key] = topology_id;
  *next_topology_id = topology_id + 1;
  return topology_id;
}

}  // namespace

PredictedObstacle MakeConstantVelocityPrediction(int obstacle_id,
                                                 double radius,
                                                 double start_x,
                                                 double start_y,
                                                 double velocity_x,
                                                 double velocity_y,
                                                 int horizon,
                                                 double dt) {
  PredictedObstacle obstacle;
  obstacle.obstacle_id = obstacle_id;
  obstacle.shape = PredictedObstacleShape::kCircle;
  obstacle.radius = radius;
  obstacle.centers.reserve(horizon + 1);
  for (int k = 0; k <= horizon; ++k) {
    Vector center(2);
    center << start_x + velocity_x * k * dt,
              start_y + velocity_y * k * dt;
    obstacle.centers.push_back(center);
  }
  return obstacle;
}

PredictedObstacle MakeBoxConstantVelocityPrediction(int obstacle_id,
                                                    double half_length,
                                                    double half_width,
                                                    double yaw,
                                                    double start_x,
                                                    double start_y,
                                                    double velocity_x,
                                                    double velocity_y,
                                                    int horizon,
                                                    double dt) {
  PredictedObstacle obstacle;
  obstacle.obstacle_id = obstacle_id;
  obstacle.shape = PredictedObstacleShape::kBox;
  obstacle.half_length = half_length;
  obstacle.half_width = half_width;
  obstacle.yaw = yaw;
  obstacle.centers.reserve(horizon + 1);
  for (int k = 0; k <= horizon; ++k) {
    Vector center(2);
    center << start_x + velocity_x * k * dt,
              start_y + velocity_y * k * dt;
    obstacle.centers.push_back(center);
  }
  return obstacle;
}

std::vector<Vector> GuidancePointsToControls(const std::vector<GuidancePoint>& points,
                                             double dt,
                                             double wheelbase,
                                             const GlobalGuidanceConfig& config,
                                             int horizon) {
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

GlobalGuidancePlanner::GlobalGuidancePlanner(
    std::shared_ptr<StraightReferenceLine> ref_line,
    double wheelbase)
    : ref_line_(std::move(ref_line)),
      wheelbase_(wheelbase) {}

GlobalGuidanceResult GlobalGuidancePlanner::Generate(
    const Vector& initial_state,
    double target_speed,
    double road_half_width,
    const VehicleCircleApproximation& vehicle_geometry,
    const std::vector<PredictedObstacle>& obstacles,
    int horizon,
    double dt,
    const GlobalGuidanceConfig& config,
    const GlobalGuidanceMemory* previous_memory) const {
  if (!ref_line_) {
    throw std::invalid_argument("GlobalGuidancePlanner requires a valid reference line.");
  }
  if (initial_state.size() < 4) {
    throw std::invalid_argument("GlobalGuidancePlanner requires a 4D initial state.");
  }
  if (horizon <= 0 || dt <= 0.0) {
    throw std::invalid_argument("GlobalGuidancePlanner requires a positive horizon and dt.");
  }

  GlobalGuidanceResult result;

  const int num_layers = std::max(config.num_layers, 1);
  std::vector<int> layer_steps(num_layers + 1, 0);
  for (int l = 1; l <= num_layers; ++l) {
    layer_steps[l] = horizon * l / num_layers;
  }

  const int n_lat = static_cast<int>(config.lateral_offsets.size());
  const int n_spd = static_cast<int>(config.speed_ratios.size());
  const int n_nodes = n_lat * n_spd;
  if (n_nodes == 0) {
    result.controls.assign(horizon, Vector::Zero(2));
    return result;
  }

  const double ref_heading = ref_line_->Heading();
  const double ref_ox = ref_line_->OriginX();
  const double ref_oy = ref_line_->OriginY();

  const double s0 = ref_line_->LongitudinalPosition(initial_state);
  const double d0 = ref_line_->LateralError(initial_state);
  const double yaw0 = initial_state(2);
  const double v0 = initial_state(3);
  const double ds0 = v0 * std::cos(yaw0 - ref_heading);
  const double dd0 = v0 * std::sin(yaw0 - ref_heading);

  std::vector<DPEntry> prev_dp(n_nodes);
  std::vector<std::vector<SegmentTrajectory>> segments(num_layers, std::vector<SegmentTrajectory>(n_nodes));
  std::vector<std::vector<int>> prev_indices(num_layers, std::vector<int>(n_nodes, -1));

  for (int j = 0; j < n_nodes; ++j) {
    const int lat_j = j / n_spd;
    const int spd_j = j % n_spd;
    const double d_target = config.lateral_offsets[lat_j];
    const double v_target = std::max(0.0, config.speed_ratios[spd_j] * target_speed);
    const int steps = layer_steps[1] - layer_steps[0];
    const double seg_T = steps * dt;

    const QuinticPoly lat_poly = SolveQuintic(d0, dd0, 0.0, d_target, 0.0, 0.0, seg_T);
    const QuarticPoly lon_poly = SolveQuartic(s0, ds0, 0.0, v_target, 0.0, seg_T);
    SegmentTrajectory seg = SampleSegment(lat_poly, lon_poly, ref_heading, ref_ox, ref_oy,
                                          steps, layer_steps[0], dt);

    bool collision_free = true;
    const double cost = EvaluateSegmentCost(seg.points, layer_steps[0], target_speed,
                                            road_half_width, *ref_line_, vehicle_geometry, obstacles,
                                            dt, config, &collision_free);
    prev_dp[j].cost = cost;
    prev_dp[j].state = FrenetState{
        lon_poly.Eval(seg_T), lat_poly.Eval(seg_T),
        lon_poly.EvalDeriv(seg_T), lat_poly.EvalDeriv(seg_T)};
    prev_dp[j].reachable = true;
    segments[0][j] = std::move(seg);
    prev_indices[0][j] = -1;
  }

  for (int l = 1; l < num_layers; ++l) {
    std::vector<DPEntry> curr_dp(n_nodes);
    const int steps = layer_steps[l + 1] - layer_steps[l];
    const double seg_T = steps * dt;

    for (int j = 0; j < n_nodes; ++j) {
      const int lat_j = j / n_spd;
      const int spd_j = j % n_spd;
      const double d_target = config.lateral_offsets[lat_j];
      const double v_target = std::max(0.0, config.speed_ratios[spd_j] * target_speed);

      for (int i = 0; i < n_nodes; ++i) {
        if (!prev_dp[i].reachable) {
          continue;
        }
        const int lat_i = i / n_spd;
        const int spd_i = i % n_spd;
        if (std::abs(lat_j - lat_i) > std::max(0, config.max_lateral_index_jump)) {
          continue;
        }
        if (std::abs(spd_j - spd_i) > std::max(0, config.max_speed_index_jump)) {
          continue;
        }
        const auto& from = prev_dp[i].state;

        const QuinticPoly lat_poly = SolveQuintic(from.d, from.dd, 0.0, d_target, 0.0, 0.0, seg_T);
        const QuarticPoly lon_poly = SolveQuartic(from.s, from.ds, 0.0, v_target, 0.0, seg_T);
        SegmentTrajectory seg = SampleSegment(lat_poly, lon_poly, ref_heading, ref_ox, ref_oy,
                                              steps, layer_steps[l], dt);

        bool collision_free = true;
        const double edge_cost = EvaluateSegmentCost(seg.points, layer_steps[l], target_speed,
                                                     road_half_width, *ref_line_, vehicle_geometry, obstacles,
                                                     dt, config, &collision_free);
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

  std::unordered_map<std::string, GuidanceCandidate> grouped_candidates;
  std::unordered_map<std::string, int> topology_ids = BuildTopologyMap(previous_memory);
  int next_topology_id = previous_memory != nullptr ? previous_memory->next_topology_id : 1;

  for (int j = 0; j < n_nodes; ++j) {
    if (!prev_dp[j].reachable) {
      continue;
    }
    std::vector<GuidancePoint> points = ReconstructPath(j, num_layers, horizon, segments, prev_indices);
    if (points.empty()) {
      continue;
    }

    bool collision_free = true;
    double total_cost = EvaluateSegmentCost(points, 0, target_speed, road_half_width,
                                            *ref_line_, vehicle_geometry, obstacles, dt, config,
                                            &collision_free);
    const TopologySignature signature = BuildSignature(points, obstacles, *ref_line_);
    if (previous_memory != nullptr &&
        previous_memory->valid &&
        !previous_memory->selected_signature_key.empty() &&
        signature.key != previous_memory->selected_signature_key) {
      total_cost += config.consistency_penalty;
    }
    if (previous_memory != nullptr &&
        previous_memory->valid &&
        !previous_memory->previous_selected_guidance.empty()) {
      total_cost += config.consistency_geometry_weight *
          GeometryConsistencyPenalty(points, previous_memory->previous_selected_guidance);
    }

    GuidanceCandidate candidate;
    candidate.points = points;
    candidate.controls = GuidancePointsToControls(candidate.points, dt, wheelbase_, config, horizon);
    candidate.signature = signature;
    candidate.cost = total_cost;
    candidate.collision_free = collision_free;
    candidate.topology_id = AssignTopologyId(signature.key, &topology_ids, &next_topology_id);

    auto it = grouped_candidates.find(signature.key);
    if (it == grouped_candidates.end() || candidate.cost < it->second.cost) {
      grouped_candidates[signature.key] = std::move(candidate);
    }
  }

  if (grouped_candidates.empty()) {
    result.controls.assign(horizon, Vector::Zero(2));
    result.planner_memory.valid = previous_memory != nullptr && previous_memory->valid;
    result.planner_memory.selected_topology_id =
        previous_memory != nullptr ? previous_memory->selected_topology_id : -1;
    result.planner_memory.selected_signature_key =
        previous_memory != nullptr ? previous_memory->selected_signature_key : "";
    result.planner_memory.next_topology_id = next_topology_id;
    return result;
  }

  result.candidate_bundle.reserve(grouped_candidates.size());
  GuidanceCandidate* best_candidate = nullptr;
  for (auto& [signature_key, candidate] : grouped_candidates) {
    result.candidate_bundle.push_back(std::move(candidate));
  }
  std::sort(result.candidate_bundle.begin(), result.candidate_bundle.end(),
            [](const GuidanceCandidate& lhs, const GuidanceCandidate& rhs) {
              return lhs.cost < rhs.cost;
            });

  best_candidate = &result.candidate_bundle.front();
  best_candidate->selected = true;
  result.controls = best_candidate->controls;
  result.selected_guidance = best_candidate->points;
  result.best_cost = best_candidate->cost;
  result.selected_topology_id = best_candidate->topology_id;
  result.used_fallback = false;

  result.planner_memory.valid = true;
  result.planner_memory.selected_topology_id = best_candidate->topology_id;
  result.planner_memory.selected_signature_key = best_candidate->signature.key;
  result.planner_memory.next_topology_id = next_topology_id;
  result.planner_memory.previous_selected_guidance = best_candidate->points;
  result.planner_memory.topology_entries.reserve(topology_ids.size());
  for (const auto& [key, topology_id] : topology_ids) {
    result.planner_memory.topology_entries.push_back(TopologyMemoryEntry{topology_id, key});
  }
  std::sort(result.planner_memory.topology_entries.begin(),
            result.planner_memory.topology_entries.end(),
            [](const TopologyMemoryEntry& lhs, const TopologyMemoryEntry& rhs) {
              return lhs.topology_id < rhs.topology_id;
            });

  return result;
}

}  // namespace my_al_ilqr
