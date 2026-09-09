#pragma once

#include <memory>

#include "autodrive/straight_reference_line.hpp"
#include "constraints/constraint_function.hpp"

namespace my_al_ilqr {

class RoadBoundaryConstraint final : public ConstraintFunction {
 public:
  RoadBoundaryConstraint(std::shared_ptr<StraightReferenceLine> reference_line,
                         double lateral_lower_bound,
                         double lateral_upper_bound);

  ConstraintType Type() const override { return ConstraintType::kInequality; }
  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }
  int OutputDim() const override { return 2; }
  std::string Name() const override { return "road_boundary"; }
  Vector Evaluate(const Vector& state, const Vector& control) const override;

  bool HasAnalyticalJacobian() const override { return true; }
  Matrix JacobianState(const Vector& state, const Vector& control) const override;
  Matrix JacobianControl(const Vector& state, const Vector& control) const override;

 private:
  std::shared_ptr<StraightReferenceLine> reference_line_;
  double lateral_lower_bound_ = 0.0;
  double lateral_upper_bound_ = 0.0;
};

}  // namespace my_al_ilqr
