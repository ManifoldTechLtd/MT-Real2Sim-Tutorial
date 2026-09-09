#include "cost/composite_cost_function.hpp"

#include <stdexcept>

namespace my_al_ilqr {

CompositeCostFunction::CompositeCostFunction(std::vector<std::shared_ptr<CostFunction>> terms)
    : terms_(std::move(terms)) {
  if (terms_.empty()) {
    throw std::invalid_argument("CompositeCostFunction requires at least one cost term.");
  }
  for (const auto& term : terms_) {
    if (!term) {
      throw std::invalid_argument("CompositeCostFunction received a null cost term.");
    }
  }
  for (size_t i = 1; i < terms_.size(); ++i) {
    if (terms_[i]->StateDim() != terms_[0]->StateDim() ||
        terms_[i]->ControlDim() != terms_[0]->ControlDim()) {
      throw std::invalid_argument("CompositeCostFunction terms must share dimensions.");
    }
  }
}

int CompositeCostFunction::StateDim() const {
  return terms_.front()->StateDim();
}

int CompositeCostFunction::ControlDim() const {
  return terms_.front()->ControlDim();
}

double CompositeCostFunction::StageCost(const Vector& state, const Vector& control) const {
  double total = 0.0;
  for (const auto& term : terms_) {
    total += term->StageCost(state, control);
  }
  return total;
}

double CompositeCostFunction::TerminalCost(const Vector& state) const {
  double total = 0.0;
  for (const auto& term : terms_) {
    total += term->TerminalCost(state);
  }
  return total;
}

bool CompositeCostFunction::HasAnalyticalDerivatives() const {
  for (const auto& term : terms_) {
    if (!term->HasAnalyticalDerivatives()) {
      return false;
    }
  }
  return true;
}

CostExpansion CompositeCostFunction::AnalyticalStageCostExpansion(const Vector& state,
                                                                  const Vector& control) const {
  auto result = terms_[0]->AnalyticalStageCostExpansion(state, control);
  for (size_t i = 1; i < terms_.size(); ++i) {
    auto term_exp = terms_[i]->AnalyticalStageCostExpansion(state, control);
    result.lx += term_exp.lx;
    result.lu += term_exp.lu;
    result.lxx += term_exp.lxx;
    result.lux += term_exp.lux;
    result.luu += term_exp.luu;
  }
  return result;
}

std::pair<Vector, Matrix> CompositeCostFunction::AnalyticalTerminalCostExpansion(
    const Vector& state) const {
  auto [lx, lxx] = terms_[0]->AnalyticalTerminalCostExpansion(state);
  for (size_t i = 1; i < terms_.size(); ++i) {
    auto [lx_i, lxx_i] = terms_[i]->AnalyticalTerminalCostExpansion(state);
    lx += lx_i;
    lxx += lxx_i;
  }
  return {lx, lxx};
}

}  // namespace my_al_ilqr
