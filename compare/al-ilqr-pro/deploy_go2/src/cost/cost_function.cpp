#include "cost/cost_function.hpp"

#include <stdexcept>

namespace my_al_ilqr {

CostExpansion CostFunction::AnalyticalStageCostExpansion(const Vector& /*state*/,
                                                         const Vector& /*control*/) const {
  throw std::logic_error("CostFunction::AnalyticalStageCostExpansion not implemented.");
}

std::pair<Vector, Matrix> CostFunction::AnalyticalTerminalCostExpansion(
    const Vector& /*state*/) const {
  throw std::logic_error("CostFunction::AnalyticalTerminalCostExpansion not implemented.");
}

}  // namespace my_al_ilqr
