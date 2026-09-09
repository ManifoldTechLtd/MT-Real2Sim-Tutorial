#pragma once

#include <memory>
#include <vector>

#include "autodrive/global_guidance_planner.hpp"
#include "autodrive/straight_reference_line.hpp"
#include "autodrive/vehicle_circle_approximation.hpp"
#include "core/types.hpp"

namespace my_al_ilqr {

struct SLDPConfig {
  int num_layers = 4;
  int lateral_samples = 7;
  double lateral_span = 2.5;
  int speed_samples = 3;
  double max_speed_ratio = 1.2;
  int max_lateral_index_jump = 2;
  int max_speed_index_jump = 2;

  double lateral_weight = 2.0;
  double lateral_change_weight = 3.0;
  double heading_weight = 2.0;
  double speed_weight = 1.0;
  double accel_weight = 0.5;
  double steering_weight = 0.5;
  double path_length_weight = 0.2;
  double collision_penalty = 1.0e4;
  int collision_check_substeps = 2;
  double safety_clearance = 0.3;
  double proximity_buffer = 2.0;
  double proximity_weight = 15.0;

  double anticipation_distance = 15.0;
  double anticipation_lateral_clearance = 1.8;
  double anticipation_weight = 12.0;

  double min_acceleration = -1.0;
  double max_acceleration = 1.0;
  double min_steering = -0.45;
  double max_steering = 0.45;
};

class SLDPPlanner {
 public:
  SLDPPlanner(std::shared_ptr<StraightReferenceLine> ref_line,
              double wheelbase);

  GlobalGuidanceResult Generate(const Vector& initial_state,
                                double target_speed,
                                double road_half_width,
                                const VehicleCircleApproximation& vehicle_geometry,
                                const std::vector<PredictedObstacle>& obstacles,
                                int horizon,
                                double dt,
                                const SLDPConfig& config = {}) const;

 private:
  std::shared_ptr<StraightReferenceLine> ref_line_;
  double wheelbase_ = 0.0;
};

}  // namespace my_al_ilqr
