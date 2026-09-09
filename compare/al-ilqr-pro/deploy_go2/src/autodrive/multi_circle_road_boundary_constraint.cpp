#include "autodrive/multi_circle_road_boundary_constraint.hpp"

#include <cmath>
#include <stdexcept>

namespace my_al_ilqr {

MultiCircleRoadBoundaryConstraint::MultiCircleRoadBoundaryConstraint(
    std::shared_ptr<StraightReferenceLine> reference_line,
    VehicleCircleApproximation vehicle_geometry,
    double road_lower_bound,
    double road_upper_bound)
    : reference_line_(std::move(reference_line)),
      vehicle_geometry_(std::move(vehicle_geometry)),
      road_lower_bound_(road_lower_bound),
      road_upper_bound_(road_upper_bound) {
  if (!reference_line_) {
    throw std::invalid_argument(
        "MultiCircleRoadBoundaryConstraint requires a valid reference line.");
  }
  if (road_lower_bound_ >= road_upper_bound_) {
    throw std::invalid_argument(
        "MultiCircleRoadBoundaryConstraint requires lower_bound < upper_bound.");
  }
}

Vector MultiCircleRoadBoundaryConstraint::Evaluate(const Vector& state,
                                                   const Vector& control) const {
  if (state.size() != StateDim() || control.size() != ControlDim()) {
    throw std::invalid_argument(
        "MultiCircleRoadBoundaryConstraint received an input with invalid dimensions.");
  }

  const auto world_centers = vehicle_geometry_.EvaluateWorldCenters(state);
  const auto& circles = vehicle_geometry_.Circles();
  const int n = vehicle_geometry_.CircleCount();

  Vector values(OutputDim());
  for (int i = 0; i < n; ++i) {
    Vector center_state(2);
    center_state(0) = world_centers[i](0);
    center_state(1) = world_centers[i](1);
    const double lateral = reference_line_->LateralError(center_state);
    const double r = circles[i].radius;
    values(2 * i)     = (road_lower_bound_ + r) - lateral;
    values(2 * i + 1) = lateral - (road_upper_bound_ - r);
  }
  return values;
}

Matrix MultiCircleRoadBoundaryConstraint::JacobianState(const Vector& state,
                                                         const Vector& /*control*/) const {
  const double h = reference_line_->Heading();
  const double sin_h = std::sin(h);
  const double cos_h = std::cos(h);
  const double yaw = state(2);
  const double sin_yaw = std::sin(yaw);
  const double cos_yaw = std::cos(yaw);

  const auto& circles = vehicle_geometry_.Circles();
  const int n = vehicle_geometry_.CircleCount();
  Matrix jac = Matrix::Zero(OutputDim(), 4);

  for (int i = 0; i < n; ++i) {
    const double bx = circles[i].center_x_body;
    // ∂lat_j/∂x = [-sin(h), cos(h), bx*cos(yaw-h), 0]
    const double dlat_dtheta = bx * std::cos(yaw - h);
    // c_{2i} = ... - lat → ∂c/∂x = -∂lat/∂x
    jac(2 * i, 0) = sin_h;
    jac(2 * i, 1) = -cos_h;
    jac(2 * i, 2) = -dlat_dtheta;
    // c_{2i+1} = lat - ... → ∂c/∂x = ∂lat/∂x
    jac(2 * i + 1, 0) = -sin_h;
    jac(2 * i + 1, 1) = cos_h;
    jac(2 * i + 1, 2) = dlat_dtheta;
  }
  return jac;
}

Matrix MultiCircleRoadBoundaryConstraint::JacobianControl(const Vector& /*state*/,
                                                           const Vector& /*control*/) const {
  return Matrix::Zero(OutputDim(), 2);
}

}  // namespace my_al_ilqr
