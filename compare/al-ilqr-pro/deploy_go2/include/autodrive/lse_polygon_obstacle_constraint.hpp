#pragma once

#include "autodrive/convex_polygon.hpp"
#include "autodrive/vehicle_circle_approximation.hpp"
#include "constraints/constraint_function.hpp"

namespace my_al_ilqr {

// Multi-circle vehicle vs convex polygon obstacle, using Log-Sum-Exp
// smooth distance. Each vehicle circle produces one inequality constraint.
class LSEPolygonObstacleConstraint final : public ConstraintFunction {
 public:
  LSEPolygonObstacleConstraint(VehicleCircleApproximation vehicle_geometry,
                               ConvexPolygon obstacle_polygon,
                               double alpha = 20.0);

  ConstraintType Type() const override { return ConstraintType::kInequality; }
  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }
  int OutputDim() const override { return vehicle_geometry_.CircleCount(); }
  std::string Name() const override { return "lse_polygon_obstacle"; }
  Vector Evaluate(const Vector& state, const Vector& control) const override;

  bool HasAnalyticalJacobian() const override { return true; }
  Matrix JacobianState(const Vector& state, const Vector& control) const override;
  Matrix JacobianControl(const Vector& state, const Vector& control) const override;

 private:
  VehicleCircleApproximation vehicle_geometry_;
  ConvexPolygon obstacle_polygon_;
  double alpha_;
};

}  // namespace my_al_ilqr
