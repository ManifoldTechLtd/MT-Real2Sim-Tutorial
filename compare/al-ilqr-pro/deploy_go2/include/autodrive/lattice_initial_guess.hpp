#pragma once

#include <memory>
#include <vector>

#include "autodrive/straight_reference_line.hpp"
#include "core/types.hpp"

namespace my_al_ilqr {

struct LatticeConfig {
  // Lateral offsets from reference line (meters).
  // Negative = left of reference, Positive = right.
  // Represents: 左极限, 左绕, 直行, 右绕, 右极限
  std::vector<double> lateral_offsets = {-2.5, -1.5, 0.0, 1.5, 2.5};

  // Speed multipliers relative to target_speed.
  // 0.0 = stop, 0.5 = decelerate, 1.0 = cruise, 1.3 = accelerate
  std::vector<double> speed_ratios = {0.0, 0.5, 1.0, 1.3};

  // Number of intermediate layers (excluding start).
  // 1 = single-layer, 2 = two-layer (default), etc.
  int num_layers = 2;

  // Obstacle collision penalty weight (large = strictly avoid)
  double collision_penalty = 1.0e4;

  // Cost weights for candidate ranking
  double lateral_weight = 2.0;
  double speed_weight = 1.0;
  double accel_weight = 0.3;
  double steering_weight = 0.5;

  // Safety clearance beyond obstacle/vehicle radius for collision check
  double safety_clearance = 0.3;
};

struct LatticeObstacleInfo {
  double center_x;
  double center_y;
  double radius;
  double speed_x = 0.0;
};

struct LatticeCandidate {
  std::vector<double> x;
  std::vector<double> y;
  double cost = 1.0e18;
  bool collision_free = true;
  bool is_best = false;
};

struct LatticeResult {
  std::vector<Vector> controls;
  double best_cost = 1.0e18;
  int num_candidates = 0;
  int num_collision_free = 0;
  std::vector<LatticeCandidate> candidates;
};

class LatticeInitialGuess {
 public:
  LatticeInitialGuess(std::shared_ptr<StraightReferenceLine> ref_line,
                      double wheelbase);

  LatticeResult Generate(const Vector& initial_state,
                         double target_speed,
                         double road_half_width,
                         double vehicle_radius,
                         const LatticeObstacleInfo& obstacle,
                         int horizon,
                         double dt,
                         const LatticeConfig& config = {}) const;

 private:
  std::shared_ptr<StraightReferenceLine> ref_line_;
  double wheelbase_;
};

}  // namespace my_al_ilqr
