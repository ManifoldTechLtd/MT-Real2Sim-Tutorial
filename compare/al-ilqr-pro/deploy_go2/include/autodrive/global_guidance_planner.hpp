#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autodrive/convex_polygon.hpp"
#include "autodrive/straight_reference_line.hpp"
#include "autodrive/vehicle_circle_approximation.hpp"
#include "core/types.hpp"

namespace my_al_ilqr {

enum class PredictedObstacleShape {
  kCircle = 0,
  kBox = 1,
};

struct PredictedObstacle {
  int obstacle_id = -1;
  PredictedObstacleShape shape = PredictedObstacleShape::kCircle;
  double radius = 0.0;
  double half_length = 0.0;
  double half_width = 0.0;
  double yaw = 0.0;
  std::vector<Vector> centers;
};

struct GuidancePoint {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double speed = 0.0;
  double t = 0.0;
  double s = 0.0;
  double d = 0.0;
};

struct TopologyInteraction {
  int obstacle_id = -1;
  int lateral_relation = 0;   // -1 = obstacle on left, +1 = obstacle on right
  int temporal_relation = 0;  // -1 = pass after, +1 = pass before
};

struct TopologySignature {
  std::string key;
  std::vector<TopologyInteraction> interactions;
};

struct GuidanceCandidate {
  std::vector<GuidancePoint> points;
  std::vector<Vector> controls;
  TopologySignature signature;
  double cost = 1.0e18;
  bool collision_free = true;
  bool selected = false;
  int topology_id = -1;
};

struct TopologyMemoryEntry {
  int topology_id = -1;
  std::string signature_key;
};

struct GlobalGuidanceMemory {
  bool valid = false;
  int selected_topology_id = -1;
  std::string selected_signature_key;
  int next_topology_id = 1;
  std::vector<TopologyMemoryEntry> topology_entries;
  std::vector<GuidancePoint> previous_selected_guidance;
};

struct GlobalGuidanceConfig {
  std::vector<double> lateral_offsets = {-2.5, -1.5, 0.0, 1.5, 2.5};
  std::vector<double> speed_ratios = {0.0, 0.5, 1.0, 1.2};
  int num_layers = 2;

  double lateral_weight = 2.0;
  double speed_weight = 1.0;
  double accel_weight = 0.3;
  double steering_weight = 0.5;
  double path_length_weight = 0.2;
  double collision_penalty = 1.0e4;
  int collision_check_substeps = 2;
  double safety_clearance = 0.3;
  double proximity_buffer = 1.2;
  double proximity_weight = 12.0;
  double follow_lateral_band = 0.9;
  double follow_base_gap = 6.0;
  double follow_headway = 1.2;
  double follow_penalty = 16.0;
  double bypass_lateral_clearance = 1.6;
  double bypass_anticipation_distance = 16.0;
  double bypass_anticipation_weight = 18.0;
  double post_bypass_hold_distance = 12.0;
  double post_bypass_hold_weight = 12.0;
  double consistency_penalty = 8.0;
  double consistency_geometry_weight = 0.15;
  int max_lateral_index_jump = 2;
  int max_speed_index_jump = 1;

  double min_acceleration = -1.0;
  double max_acceleration = 1.0;
  double min_steering = -0.45;
  double max_steering = 0.45;
};

struct DPLatticeNode {
  double x = 0.0;
  double y = 0.0;
  int layer = 0;
  int lat_index = 0;
  int spd_index = 0;
  bool reachable = false;
  bool on_best_path = false;
};

struct GlobalGuidanceResult {
  std::vector<Vector> controls;
  std::vector<GuidancePoint> selected_guidance;
  std::vector<GuidanceCandidate> candidate_bundle;
  GlobalGuidanceMemory planner_memory;
  std::vector<DPLatticeNode> dp_lattice_nodes;
  double best_cost = 1.0e18;
  int selected_topology_id = -1;
  bool used_fallback = true;
};

PredictedObstacle MakeConstantVelocityPrediction(int obstacle_id,
                                                 double radius,
                                                 double start_x,
                                                 double start_y,
                                                 double velocity_x,
                                                 double velocity_y,
                                                 int horizon,
                                                 double dt);

PredictedObstacle MakeBoxConstantVelocityPrediction(int obstacle_id,
                                                    double half_length,
                                                    double half_width,
                                                    double yaw,
                                                    double start_x,
                                                    double start_y,
                                                    double velocity_x,
                                                    double velocity_y,
                                                    int horizon,
                                                    double dt);

std::vector<Vector> GuidancePointsToControls(const std::vector<GuidancePoint>& points,
                                             double dt,
                                             double wheelbase,
                                             const GlobalGuidanceConfig& config,
                                             int horizon);

class GlobalGuidancePlanner {
 public:
  GlobalGuidancePlanner(std::shared_ptr<StraightReferenceLine> ref_line,
                        double wheelbase);

  GlobalGuidanceResult Generate(const Vector& initial_state,
                                double target_speed,
                                double road_half_width,
                                const VehicleCircleApproximation& vehicle_geometry,
                                const std::vector<PredictedObstacle>& obstacles,
                                int horizon,
                                double dt,
                                const GlobalGuidanceConfig& config = {},
                                const GlobalGuidanceMemory* previous_memory = nullptr) const;

 private:
  std::shared_ptr<StraightReferenceLine> ref_line_;
  double wheelbase_ = 0.0;
};

}  // namespace my_al_ilqr
