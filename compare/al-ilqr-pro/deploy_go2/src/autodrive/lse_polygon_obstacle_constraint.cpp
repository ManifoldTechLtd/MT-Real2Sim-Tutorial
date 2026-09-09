#include "autodrive/lse_polygon_obstacle_constraint.hpp"

#include <cmath>
#include <stdexcept>

namespace my_al_ilqr {

LSEPolygonObstacleConstraint::LSEPolygonObstacleConstraint(
    VehicleCircleApproximation vehicle_geometry, ConvexPolygon obstacle_polygon,
    double alpha)
    : vehicle_geometry_(std::move(vehicle_geometry)),
      obstacle_polygon_(std::move(obstacle_polygon)),
      alpha_(alpha) {
  if (alpha_ <= 0.0) {
    throw std::invalid_argument(
        "LSEPolygonObstacleConstraint requires positive alpha.");
  }
}

Vector LSEPolygonObstacleConstraint::Evaluate(const Vector& state,
                                              const Vector& control) const {
  if (state.size() != StateDim() || control.size() != ControlDim()) {
    throw std::invalid_argument(
        "LSEPolygonObstacleConstraint: invalid input dimensions.");
  }

  const auto centers = vehicle_geometry_.EvaluateWorldCenters(state);
  const auto& circles = vehicle_geometry_.Circles();
  Vector values(OutputDim());
  for (int i = 0; i < OutputDim(); ++i) {
    const double lse =
        obstacle_polygon_.LogSumExpDistance(centers[i], alpha_);
    values(i) = circles[i].radius - lse;
  }
  return values;
}

Matrix LSEPolygonObstacleConstraint::JacobianState(const Vector& state,
                                                    const Vector& /*control*/) const {
  const double yaw = state(2);
  const double sin_yaw = std::sin(yaw);
  const double cos_yaw = std::cos(yaw);

  const auto centers = vehicle_geometry_.EvaluateWorldCenters(state);
  const auto& circles = vehicle_geometry_.Circles();
  Matrix jac = Matrix::Zero(OutputDim(), 4);

  for (int i = 0; i < OutputDim(); ++i) {
    const double bx = circles[i].center_x_body;
    // ∂LSE/∂p (2×1)
    const Vector dlse_dp = obstacle_polygon_.LogSumExpGradient(centers[i], alpha_);

    // ∂center/∂x (2×4):
    //   [1  0  -sin(yaw)*bx  0]
    //   [0  1   cos(yaw)*bx  0]
    // ∂c_j/∂x = -dlse_dpᵀ · ∂center/∂x  (1×4)
    jac(i, 0) = -dlse_dp(0);
    jac(i, 1) = -dlse_dp(1);
    jac(i, 2) = -dlse_dp(0) * (-sin_yaw * bx) - dlse_dp(1) * (cos_yaw * bx);
    // jac(i, 3) = 0 (no velocity dependence)
  }
  return jac;
}

Matrix LSEPolygonObstacleConstraint::JacobianControl(const Vector& /*state*/,
                                                      const Vector& /*control*/) const {
  return Matrix::Zero(OutputDim(), 2);
}

}  // namespace my_al_ilqr
