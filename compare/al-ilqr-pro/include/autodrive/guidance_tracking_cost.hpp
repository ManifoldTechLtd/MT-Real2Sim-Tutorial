#pragma once

#include "autodrive/global_guidance_planner.hpp"
#include "autodrive/straight_reference_line.hpp"
#include "cost/cost_function.hpp"

namespace my_al_ilqr {

class GuidanceTrackingCost final : public CostFunction {
 public:
  GuidanceTrackingCost(const GuidancePoint& reference,
                       double position_weight,
                       double heading_weight,
                       double speed_weight,
                       bool terminal_mode = false,
                       std::shared_ptr<StraightReferenceLine> reference_line = nullptr,
                       bool terminal_longitudinal_only = false);

  int StateDim() const override { return 4; }
  int ControlDim() const override { return 2; }

  double StageCost(const Vector& state, const Vector& control) const override;
  double TerminalCost(const Vector& state) const override;

  bool HasAnalyticalDerivatives() const override { return true; }
  CostExpansion AnalyticalStageCostExpansion(const Vector& state,
                                             const Vector& control) const override;
  std::pair<Vector, Matrix> AnalyticalTerminalCostExpansion(const Vector& state) const override;

 private:
  GuidancePoint reference_;
  double position_weight_ = 0.0;
  double heading_weight_ = 0.0;
  double speed_weight_ = 0.0;
  bool terminal_mode_ = false;
  std::shared_ptr<StraightReferenceLine> reference_line_;
  bool terminal_longitudinal_only_ = false;
};

}  // namespace my_al_ilqr
