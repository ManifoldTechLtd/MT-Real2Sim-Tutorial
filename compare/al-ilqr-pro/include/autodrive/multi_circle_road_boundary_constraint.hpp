#pragma once

#include <memory>

#include "autodrive/straight_reference_line.hpp"
#include "autodrive/vehicle_circle_approximation.hpp"
#include "constraints/constraint_function.hpp"

namespace my_al_ilqr {

class MultiCircleRoadBoundaryConstraint final : public ConstraintFunction {
 public:
  MultiCircleRoadBoundaryConstraint(std::shared_ptr<StraightReferenceLine> reference_line,
                                    VehicleCircleApproximation vehicle_geometry,
                                    double road_lower_bound,
                                    double road_upper_bound);

  ConstraintType Type() const override { return ConstraintType::kInequality; }
  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }
  int OutputDim() const override { return 2 * vehicle_geometry_.CircleCount(); }
  std::string Name() const override { return "multi_circle_road_boundary"; }
  Vector Evaluate(const Vector& state, const Vector& control) const override;

  bool HasAnalyticalJacobian() const override { return true; }
  Matrix JacobianState(const Vector& state, const Vector& control) const override;
  Matrix JacobianControl(const Vector& state, const Vector& control) const override;

 private:
  std::shared_ptr<StraightReferenceLine> reference_line_;
  VehicleCircleApproximation vehicle_geometry_;
  double road_lower_bound_ = 0.0;
  double road_upper_bound_ = 0.0;
};

}  // namespace my_al_ilqr
