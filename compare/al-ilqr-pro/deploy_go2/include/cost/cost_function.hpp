#pragma once

#include "core/types.hpp"

namespace my_al_ilqr {

struct CostExpansion {
  Vector lx;
  Vector lu;
  Matrix lxx;
  Matrix lux;
  Matrix luu;
};

class CostFunction {
 public:
  virtual ~CostFunction() = default;

  virtual int StateDim() const = 0;
  virtual int ControlDim() const = 0;
  virtual double StageCost(const Vector& state, const Vector& control) const = 0;
  virtual double TerminalCost(const Vector& state) const = 0;

  // Analytical derivatives (optional). Override in subclasses to enable.
  virtual bool HasAnalyticalDerivatives() const { return false; }
  virtual CostExpansion AnalyticalStageCostExpansion(const Vector& state,
                                                     const Vector& control) const;
  virtual std::pair<Vector, Matrix> AnalyticalTerminalCostExpansion(const Vector& state) const;
};

}  // namespace my_al_ilqr
