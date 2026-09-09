#pragma once

#include <memory>
#include <vector>

#include "cost/cost_function.hpp"

namespace my_al_ilqr {

class CompositeCostFunction final : public CostFunction {
 public:
  explicit CompositeCostFunction(std::vector<std::shared_ptr<CostFunction>> terms);

  int StateDim() const override;
  int ControlDim() const override;
  double StageCost(const Vector& state, const Vector& control) const override;
  double TerminalCost(const Vector& state) const override;

  bool HasAnalyticalDerivatives() const override;
  CostExpansion AnalyticalStageCostExpansion(const Vector& state,
                                             const Vector& control) const override;
  std::pair<Vector, Matrix> AnalyticalTerminalCostExpansion(const Vector& state) const override;

 private:
  std::vector<std::shared_ptr<CostFunction>> terms_;
};

}  // namespace my_al_ilqr
