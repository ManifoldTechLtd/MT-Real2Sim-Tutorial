#include "autodrive/vehicle_circle_approximation.hpp"

#include <cmath>
#include <stdexcept>

namespace my_al_ilqr {

VehicleCircleApproximation::VehicleCircleApproximation(VehicleBodyConfig config)
    : config_(std::move(config)) {
  if (config_.length <= 0.0 || config_.width <= 0.0) {
    throw std::invalid_argument("VehicleCircleApproximation requires positive body dimensions.");
  }
  if (config_.rear_axle_to_rear < 0.0 || config_.rear_axle_to_rear >= config_.length) {
    throw std::invalid_argument("VehicleCircleApproximation rear axle position is invalid.");
  }
  if (config_.collision_circle_count <= 0) {
    throw std::invalid_argument("VehicleCircleApproximation requires a positive circle count.");
  }

  const double radius = 0.5 * config_.width;
  const double rear_limit = -config_.rear_axle_to_rear + radius;
  const double front_limit = (config_.length - config_.rear_axle_to_rear) - radius;

  circles_.reserve(config_.collision_circle_count);
  if (config_.collision_circle_count == 1) {
    circles_.push_back(VehicleCircle{0.5 * (rear_limit + front_limit), 0.0, radius});
    return;
  }

  const double span = front_limit - rear_limit;
  const double step = span / static_cast<double>(config_.collision_circle_count - 1);
  for (int i = 0; i < config_.collision_circle_count; ++i) {
    circles_.push_back(VehicleCircle{rear_limit + step * i, 0.0, radius});
  }
}

std::vector<Vector> VehicleCircleApproximation::EvaluateWorldCenters(const Vector& state) const {
  if (state.size() < 3) {
    throw std::invalid_argument(
        "VehicleCircleApproximation requires a state containing x, y, and yaw.");
  }

  const double x = state(0);
  const double y = state(1);
  const double yaw = state(2);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  std::vector<Vector> centers;
  centers.reserve(circles_.size());
  for (const auto& circle : circles_) {
    Vector center(2);
    center(0) = x + cos_yaw * circle.center_x_body - sin_yaw * circle.center_y_body;
    center(1) = y + sin_yaw * circle.center_x_body + cos_yaw * circle.center_y_body;
    centers.push_back(center);
  }
  return centers;
}

}  // namespace my_al_ilqr
