#pragma once

#include "cost/cost_function.hpp"

namespace my_al_ilqr {

class ControlRateCost final : public CostFunction {
 public:
  ControlRateCost(double jerk_weight, double steering_rate_weight);

  void SetPreviousControl(const Vector& prev_control) { prev_control_ = prev_control; }
  const Vector& PreviousControl() const { return prev_control_; }

  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }
  double StageCost(const Vector& state, const Vector& control) const override;
  double TerminalCost(const Vector& state) const override { return 0.0; }

  bool HasAnalyticalDerivatives() const override { return true; }
  CostExpansion AnalyticalStageCostExpansion(const Vector& state,
                                             const Vector& control) const override;
  std::pair<Vector, Matrix> AnalyticalTerminalCostExpansion(const Vector& state) const override;

 private:
  double jerk_weight_;
  double steering_rate_weight_;
  Vector prev_control_;
};

}  // namespace my_al_ilqr
