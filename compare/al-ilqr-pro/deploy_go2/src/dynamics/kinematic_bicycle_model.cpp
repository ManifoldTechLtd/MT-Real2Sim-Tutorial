#include "dynamics/kinematic_bicycle_model.hpp"

#include <cmath>
#include <stdexcept>

namespace my_al_ilqr {

Vector KinematicBicycleModel::NextState(const Vector& state,
                                        const Vector& control,
                                        double dt) const {
  if (state.size() != StateDim() || control.size() != ControlDim()) {
    throw std::invalid_argument("KinematicBicycleModel received an input with invalid dimensions.");
  }
  if (dt <= 0.0) {
    throw std::invalid_argument("Time step must be positive.");
  }
  if (wheelbase_ <= 0.0) {
    throw std::invalid_argument("Wheelbase must be positive.");
  }

  const double x = state(0);
  const double y = state(1);
  const double yaw = state(2);
  const double velocity = state(3);
  const double acceleration = control(0);
  const double steering = control(1);

  Vector next_state(StateDim());
  next_state(0) = x + dt * velocity * std::cos(yaw);
  next_state(1) = y + dt * velocity * std::sin(yaw);
  next_state(2) = yaw + dt * velocity * std::tan(steering) / wheelbase_;
  next_state(3) = velocity + dt * acceleration;
  return next_state;
}

Matrix KinematicBicycleModel::JacobianState(const Vector& state, const Vector& control,
                                             double dt) const {
  const double yaw = state(2);
  const double v = state(3);
  const double steering = control(1);

  Matrix A = Matrix::Identity(4, 4);
  A(0, 2) = -dt * v * std::sin(yaw);
  A(0, 3) = dt * std::cos(yaw);
  A(1, 2) = dt * v * std::cos(yaw);
  A(1, 3) = dt * std::sin(yaw);
  A(2, 3) = dt * std::tan(steering) / wheelbase_;
  return A;
}

Matrix KinematicBicycleModel::JacobianControl(const Vector& state, const Vector& control,
                                               double dt) const {
  const double v = state(3);
  const double steering = control(1);
  const double cos_s = std::cos(steering);

  Matrix B = Matrix::Zero(4, 2);
  B(2, 1) = dt * v / (wheelbase_ * cos_s * cos_s);
  B(3, 0) = dt;
  return B;
}

}  // namespace my_al_ilqr
