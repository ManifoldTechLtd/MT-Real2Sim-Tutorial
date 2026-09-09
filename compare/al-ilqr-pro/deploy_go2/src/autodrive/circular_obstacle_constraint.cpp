#include "autodrive/circular_obstacle_constraint.hpp"

#include <stdexcept>

namespace my_al_ilqr {

CircularObstacleConstraint::CircularObstacleConstraint(double center_x,
                                                       double center_y,
                                                       double safe_radius)
    : center_x_(center_x), center_y_(center_y), safe_radius_(safe_radius) {
  if (safe_radius_ <= 0.0) {
    throw std::invalid_argument("CircularObstacleConstraint safe_radius must be positive.");
  }
}

Vector CircularObstacleConstraint::Evaluate(const Vector& state, const Vector& control) const {
  if (state.size() != StateDim() || control.size() != ControlDim()) {
    throw std::invalid_argument(
        "CircularObstacleConstraint received an input with invalid dimensions.");
  }

  const double dx = state(0) - center_x_;
  const double dy = state(1) - center_y_;
  Vector values(OutputDim());
  values(0) = safe_radius_ * safe_radius_ - (dx * dx + dy * dy);
  return values;
}

Matrix CircularObstacleConstraint::JacobianState(const Vector& state,
                                                  const Vector& /*control*/) const {
  const double dx = state(0) - center_x_;
  const double dy = state(1) - center_y_;
  Matrix jac = Matrix::Zero(1, 4);
  jac(0, 0) = -2.0 * dx;
  jac(0, 1) = -2.0 * dy;
  return jac;
}

Matrix CircularObstacleConstraint::JacobianControl(const Vector& /*state*/,
                                                    const Vector& /*control*/) const {
  return Matrix::Zero(1, 2);
}

}  // namespace my_al_ilqr
