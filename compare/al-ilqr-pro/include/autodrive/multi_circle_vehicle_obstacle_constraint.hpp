#pragma once

#include "autodrive/vehicle_circle_approximation.hpp"
#include "constraints/constraint_function.hpp"

namespace my_al_ilqr {

class MultiCircleVehicleObstacleConstraint final : public ConstraintFunction {
 public:
  MultiCircleVehicleObstacleConstraint(VehicleCircleApproximation vehicle_geometry,
                                       double obstacle_center_x,
                                       double obstacle_center_y,
                                       double obstacle_radius);

  ConstraintType Type() const override { return ConstraintType::kInequality; }
  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }
  int OutputDim() const override { return vehicle_geometry_.CircleCount(); }
  std::string Name() const override { return "multi_circle_vehicle_obstacle"; }
  Vector Evaluate(const Vector& state, const Vector& control) const override;

  bool HasAnalyticalJacobian() const override { return true; }
  Matrix JacobianState(const Vector& state, const Vector& control) const override;
  Matrix JacobianControl(const Vector& state, const Vector& control) const override;

  const VehicleCircleApproximation& VehicleGeometry() const { return vehicle_geometry_; }
  double ObstacleCenterX() const { return obstacle_center_x_; }
  double ObstacleCenterY() const { return obstacle_center_y_; }
  double ObstacleRadius() const { return obstacle_radius_; }

 private:
  VehicleCircleApproximation vehicle_geometry_;
  double obstacle_center_x_ = 0.0;
  double obstacle_center_y_ = 0.0;
  double obstacle_radius_ = 0.0;
};

}  // namespace my_al_ilqr
