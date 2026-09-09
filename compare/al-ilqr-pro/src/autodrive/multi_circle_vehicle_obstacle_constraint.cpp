#include "autodrive/multi_circle_vehicle_obstacle_constraint.hpp"

#include <cmath>
#include <stdexcept>

namespace my_al_ilqr {

MultiCircleVehicleObstacleConstraint::MultiCircleVehicleObstacleConstraint(
    VehicleCircleApproximation vehicle_geometry,
    double obstacle_center_x,
    double obstacle_center_y,
    double obstacle_radius)
    : vehicle_geometry_(std::move(vehicle_geometry)),
      obstacle_center_x_(obstacle_center_x),
      obstacle_center_y_(obstacle_center_y),
      obstacle_radius_(obstacle_radius) {
  if (obstacle_radius_ <= 0.0) {
    throw std::invalid_argument(
        "MultiCircleVehicleObstacleConstraint obstacle radius must be positive.");
  }
}

Vector MultiCircleVehicleObstacleConstraint::Evaluate(const Vector& state,
                                                      const Vector& control) const {
  if (state.size() != StateDim() || control.size() != ControlDim()) {
    throw std::invalid_argument(
        "MultiCircleVehicleObstacleConstraint received an input with invalid dimensions.");
  }

  const auto centers = vehicle_geometry_.EvaluateWorldCenters(state);
  Vector values(OutputDim());
  for (int i = 0; i < OutputDim(); ++i) {
    const double dx = centers[i](0) - obstacle_center_x_;
    const double dy = centers[i](1) - obstacle_center_y_;
    const double safe_distance = obstacle_radius_ + vehicle_geometry_.Circles()[i].radius;
    values(i) = safe_distance * safe_distance - (dx * dx + dy * dy);
  }
  return values;
}

Matrix MultiCircleVehicleObstacleConstraint::JacobianState(const Vector& state,
                                                            const Vector& /*control*/) const {
  const double yaw = state(2);
  const double sin_yaw = std::sin(yaw);
  const double cos_yaw = std::cos(yaw);

  const auto centers = vehicle_geometry_.EvaluateWorldCenters(state);
  const auto& circles = vehicle_geometry_.Circles();
  Matrix jac = Matrix::Zero(OutputDim(), 4);

  for (int i = 0; i < OutputDim(); ++i) {
    const double dx = centers[i](0) - obstacle_center_x_;
    const double dy = centers[i](1) - obstacle_center_y_;
    const double bx = circles[i].center_x_body;
    // ∂c/∂x = [-2dx, -2dy, 2*bx*(dx*sin(yaw) - dy*cos(yaw)), 0]
    jac(i, 0) = -2.0 * dx;
    jac(i, 1) = -2.0 * dy;
    jac(i, 2) = 2.0 * bx * (dx * sin_yaw - dy * cos_yaw);
  }
  return jac;
}

Matrix MultiCircleVehicleObstacleConstraint::JacobianControl(const Vector& /*state*/,
                                                              const Vector& /*control*/) const {
  return Matrix::Zero(OutputDim(), 2);
}

}  // namespace my_al_ilqr
