#pragma once

#include "autodrive/convex_polygon.hpp"
#include "constraints/constraint_function.hpp"

namespace my_al_ilqr {

// Polygon-to-polygon collision constraint. Represents the vehicle as its true
// rectangular body (no circle approximation) and computes the exact minimum
// distance to a convex polygon obstacle. Inspired by Tracy & Manchester,
// "Differentiable Collision Detection for a Set of Convex Primitives" (2023).
class PolygonCollisionConstraint final : public ConstraintFunction {
 public:
  PolygonCollisionConstraint(double vehicle_length, double vehicle_width,
                             double rear_axle_to_rear,
                             ConvexPolygon obstacle_polygon,
                             double safety_margin = 0.1);

  ConstraintType Type() const override { return ConstraintType::kInequality; }
  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }
  int OutputDim() const override { return 1; }
  std::string Name() const override { return "polygon_collision"; }
  Vector Evaluate(const Vector& state, const Vector& control) const override;

 private:
  double vehicle_length_;
  double vehicle_width_;
  double rear_axle_to_rear_;
  ConvexPolygon obstacle_polygon_;
  double safety_margin_;
};

}  // namespace my_al_ilqr
