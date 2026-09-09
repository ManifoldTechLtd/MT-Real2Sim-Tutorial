#pragma once

#include "constraints/constraint_function.hpp"

namespace my_al_ilqr {

class CircularObstacleConstraint final : public ConstraintFunction {
 public:
  CircularObstacleConstraint(double center_x, double center_y, double safe_radius);

  ConstraintType Type() const override { return ConstraintType::kInequality; }
  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }
  int OutputDim() const override { return 1; }
  std::string Name() const override { return "circular_obstacle"; }
  Vector Evaluate(const Vector& state, const Vector& control) const override;

  bool HasAnalyticalJacobian() const override { return true; }
  Matrix JacobianState(const Vector& state, const Vector& control) const override;
  Matrix JacobianControl(const Vector& state, const Vector& control) const override;

  double CenterX() const { return center_x_; }
  double CenterY() const { return center_y_; }
  double SafeRadius() const { return safe_radius_; }

 private:
  double center_x_ = 0.0;
  double center_y_ = 0.0;
  double safe_radius_ = 0.0;
};

}  // namespace my_al_ilqr
