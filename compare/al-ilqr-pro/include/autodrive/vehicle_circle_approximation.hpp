#pragma once

#include <vector>

#include "autodrive/vehicle_bicycle_config.hpp"
#include "core/types.hpp"

namespace my_al_ilqr {

struct VehicleCircle {
  double center_x_body = 0.0;
  double center_y_body = 0.0;
  double radius = 0.0;
};

class VehicleCircleApproximation {
 public:
  explicit VehicleCircleApproximation(VehicleBodyConfig config);

  const VehicleBodyConfig& Config() const { return config_; }
  const std::vector<VehicleCircle>& Circles() const { return circles_; }
  int CircleCount() const { return static_cast<int>(circles_.size()); }

  std::vector<Vector> EvaluateWorldCenters(const Vector& state) const;

 private:
  VehicleBodyConfig config_;
  std::vector<VehicleCircle> circles_;
};

}  // namespace my_al_ilqr
