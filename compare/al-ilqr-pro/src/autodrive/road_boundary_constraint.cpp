#include "autodrive/road_boundary_constraint.hpp"

#include <stdexcept>

namespace my_al_ilqr {

RoadBoundaryConstraint::RoadBoundaryConstraint(std::shared_ptr<StraightReferenceLine> reference_line,
                                               double lateral_lower_bound,
                                               double lateral_upper_bound)
    : reference_line_(std::move(reference_line)),
      lateral_lower_bound_(lateral_lower_bound),
      lateral_upper_bound_(lateral_upper_bound) {
  if (!reference_line_) {
    throw std::invalid_argument("RoadBoundaryConstraint requires a valid reference line.");
  }
  if (lateral_lower_bound_ >= lateral_upper_bound_) {
    throw std::invalid_argument("RoadBoundaryConstraint requires lower_bound < upper_bound.");
  }
}

Vector RoadBoundaryConstraint::Evaluate(const Vector& state, const Vector& control) const {
  if (state.size() != StateDim() || control.size() != ControlDim()) {
    throw std::invalid_argument(
        "RoadBoundaryConstraint received an input with invalid dimensions.");
  }

  const double lateral_error = reference_line_->LateralError(state);
  Vector values(OutputDim());
  values(0) = lateral_lower_bound_ - lateral_error;
  values(1) = lateral_error - lateral_upper_bound_;
  return values;
}

Matrix RoadBoundaryConstraint::JacobianState(const Vector& /*state*/,
                                              const Vector& /*control*/) const {
  const double h = reference_line_->Heading();
  const double sin_h = std::sin(h);
  const double cos_h = std::cos(h);
  // ∂e_lat/∂x = [-sin(h), cos(h), 0, 0]
  Matrix jac = Matrix::Zero(2, 4);
  jac(0, 0) = sin_h;   // -∂e_lat/∂px
  jac(0, 1) = -cos_h;  // -∂e_lat/∂py
  jac(1, 0) = -sin_h;  //  ∂e_lat/∂px
  jac(1, 1) = cos_h;   //  ∂e_lat/∂py
  return jac;
}

Matrix RoadBoundaryConstraint::JacobianControl(const Vector& /*state*/,
                                                const Vector& /*control*/) const {
  return Matrix::Zero(2, 2);
}

}  // namespace my_al_ilqr
