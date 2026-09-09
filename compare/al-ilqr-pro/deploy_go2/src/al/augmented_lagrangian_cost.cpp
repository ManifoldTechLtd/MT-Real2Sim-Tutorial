#include "al/augmented_lagrangian_cost.hpp"

#include <algorithm>
#include <stdexcept>

namespace my_al_ilqr {

namespace {

Vector EmptyControl() {
  return Vector::Zero(0);
}

AugmentedLagrangianConstraintData MakeConstraintData(const ConstraintPtr& constraint,
                                                     double initial_penalty,
                                                     double penalty_scaling) {
  if (!constraint) {
    throw std::invalid_argument("Augmented Lagrangian constraint cannot be null.");
  }
  if (initial_penalty <= 0.0 || penalty_scaling <= 1.0) {
    throw std::invalid_argument("Penalty parameters must satisfy mu > 0 and scaling > 1.");
  }
  return AugmentedLagrangianConstraintData{
      constraint,
      Vector::Zero(constraint->OutputDim()),
      Vector::Constant(constraint->OutputDim(), initial_penalty),
      penalty_scaling};
}

double MaxViolationForValues(ConstraintType type, const Vector& values) {
  if (type == ConstraintType::kEquality) {
    return values.cwiseAbs().maxCoeff();
  }
  double max_violation = 0.0;
  for (int i = 0; i < values.size(); ++i) {
    max_violation = std::max(max_violation, values(i));
  }
  return std::max(0.0, max_violation);
}

}  // namespace

AugmentedLagrangianKnotCost::AugmentedLagrangianKnotCost(
    std::shared_ptr<CostFunction> base_cost,
    std::vector<AugmentedLagrangianConstraintData> constraints)
    : base_cost_(std::move(base_cost)), constraints_(std::move(constraints)) {
  if (!base_cost_) {
    throw std::invalid_argument("AugmentedLagrangianKnotCost requires a valid base cost.");
  }
}

double AugmentedLagrangianKnotCost::ConstraintAugmentedTerm(
    const AugmentedLagrangianConstraintData& data,
    const Vector& state,
    const Vector& control) const {
  const Vector values = data.constraint->Evaluate(state, control);
  double total = 0.0;
  for (int i = 0; i < values.size(); ++i) {
    if (data.constraint->Type() == ConstraintType::kEquality) {
      total += data.lambda(i) * values(i) + 0.5 * data.mu(i) * values(i) * values(i);
    } else {
      const double projected = std::max(0.0, data.lambda(i) + data.mu(i) * values(i));
      total += (projected * projected - data.lambda(i) * data.lambda(i)) / (2.0 * data.mu(i));
    }
  }
  return total;
}

double AugmentedLagrangianKnotCost::TerminalAugmentedTerm(
    const AugmentedLagrangianConstraintData& data,
    const Vector& state) const {
  return ConstraintAugmentedTerm(data, state, EmptyControl());
}

double AugmentedLagrangianKnotCost::StageCost(const Vector& state, const Vector& control) const {
  double cost = base_cost_->StageCost(state, control);
  for (const auto& data : constraints_) {
    cost += ConstraintAugmentedTerm(data, state, control);
  }
  return cost;
}

double AugmentedLagrangianKnotCost::TerminalCost(const Vector& state) const {
  double cost = base_cost_->TerminalCost(state);
  for (const auto& data : constraints_) {
    cost += TerminalAugmentedTerm(data, state);
  }
  return cost;
}

bool AugmentedLagrangianKnotCost::HasAnalyticalDerivatives() const {
  if (!base_cost_->HasAnalyticalDerivatives()) {
    return false;
  }
  for (const auto& data : constraints_) {
    if (!data.constraint->HasAnalyticalJacobian()) {
      return false;
    }
  }
  return true;
}

namespace {

// Compute the AL penalty expansion for a single constraint.
void AccumulateConstraintExpansion(const AugmentedLagrangianConstraintData& data,
                                   const Vector& state, const Vector& control,
                                   int nx, int nu,
                                   Vector& lx, Vector& lu,
                                   Matrix& lxx, Matrix& lux, Matrix& luu) {
  const Vector c = data.constraint->Evaluate(state, control);
  const Matrix Jx = data.constraint->JacobianState(state, control);
  const Matrix Ju = data.constraint->JacobianControl(state, control);
  const int p = c.size();

  if (data.constraint->Type() == ConstraintType::kEquality) {
    // φ = Σ [λⱼcⱼ + μⱼ/2·cⱼ²]
    // ∂φ/∂x = Jxᵀ·(λ + μ⊙c), ∂²φ/∂x² ≈ Jxᵀ·diag(μ)·Jx
    const Vector w = data.lambda + data.mu.cwiseProduct(c);
    lx += Jx.transpose() * w;
    if (nu > 0) lu += Ju.transpose() * w;
    // Gauss-Newton Hessian
    const Matrix Jx_scaled = data.mu.asDiagonal() * Jx;
    lxx += Jx.transpose() * Jx_scaled;
    if (nu > 0) {
      const Matrix Ju_scaled = data.mu.asDiagonal() * Ju;
      luu += Ju.transpose() * Ju_scaled;
      lux += Ju.transpose() * Jx_scaled;
    }
  } else {
    // Inequality: φ = Σ [max(0, λⱼ+μⱼcⱼ)² - λⱼ²]/(2μⱼ)
    Vector active = Vector::Zero(p);
    Vector projected = Vector::Zero(p);
    for (int j = 0; j < p; ++j) {
      const double s = data.lambda(j) + data.mu(j) * c(j);
      if (s > 0.0) {
        active(j) = 1.0;
        projected(j) = s;
      }
    }
    // ∂φ/∂x = Jxᵀ·(I⊙p)
    const Vector w = active.cwiseProduct(projected);
    lx += Jx.transpose() * w;
    if (nu > 0) lu += Ju.transpose() * w;
    // ∂²φ/∂x² ≈ Jxᵀ·diag(I⊙μ)·Jx
    const Vector mu_active = active.cwiseProduct(data.mu);
    const Matrix Jx_scaled = mu_active.asDiagonal() * Jx;
    lxx += Jx.transpose() * Jx_scaled;
    if (nu > 0) {
      const Matrix Ju_scaled = mu_active.asDiagonal() * Ju;
      luu += Ju.transpose() * Ju_scaled;
      lux += Ju.transpose() * Jx_scaled;
    }
  }
}

}  // namespace

CostExpansion AugmentedLagrangianKnotCost::AnalyticalStageCostExpansion(
    const Vector& state, const Vector& control) const {
  auto result = base_cost_->AnalyticalStageCostExpansion(state, control);
  const int nx = StateDim();
  const int nu = ControlDim();
  for (const auto& data : constraints_) {
    AccumulateConstraintExpansion(data, state, control, nx, nu,
                                  result.lx, result.lu, result.lxx, result.lux, result.luu);
  }
  return result;
}

std::pair<Vector, Matrix> AugmentedLagrangianKnotCost::AnalyticalTerminalCostExpansion(
    const Vector& state) const {
  auto [lx, lxx] = base_cost_->AnalyticalTerminalCostExpansion(state);
  const int nx = StateDim();
  const int nu = 0;
  Vector lu_dummy;
  Matrix lux_dummy, luu_dummy;
  for (const auto& data : constraints_) {
    AccumulateConstraintExpansion(data, state, EmptyControl(), nx, nu,
                                  lx, lu_dummy, lxx, lux_dummy, luu_dummy);
  }
  return {lx, lxx};
}

void AugmentedLagrangianKnotCost::UpdateDuals(const Vector& state, const Vector& control) {
  for (auto& data : constraints_) {
    const Vector values = data.constraint->Evaluate(state, control);
    if (data.constraint->Type() == ConstraintType::kEquality) {
      data.lambda += data.mu.cwiseProduct(values);
    } else {
      data.lambda = (data.lambda + data.mu.cwiseProduct(values)).cwiseMax(0.0);
    }
  }
}

void AugmentedLagrangianKnotCost::UpdatePenalties() {
  for (auto& data : constraints_) {
    data.mu *= data.penalty_scaling;
  }
}

double AugmentedLagrangianKnotCost::MaxViolation(const Vector& state, const Vector& control) const {
  double max_violation = 0.0;
  for (const auto& data : constraints_) {
    const Vector values = data.constraint->Evaluate(state, control);
    max_violation = std::max(max_violation, MaxViolationForValues(data.constraint->Type(), values));
  }
  return max_violation;
}

double AugmentedLagrangianKnotCost::MaxPenalty() const {
  double max_penalty = 0.0;
  for (const auto& data : constraints_) {
    if (data.mu.size() > 0) {
      max_penalty = std::max(max_penalty, data.mu.maxCoeff());
    }
  }
  return max_penalty;
}

AugmentedLagrangianProblem::AugmentedLagrangianProblem(
    const ConstrainedOptimalControlProblem& constrained_problem,
    double initial_penalty,
    double penalty_scaling) {
  auto base_problem = constrained_problem.SharedBaseProblem();

  std::vector<std::shared_ptr<CostFunction>> stage_costs;
  stage_costs.reserve(base_problem->Horizon());
  stage_costs_.reserve(base_problem->Horizon());
  for (int k = 0; k < base_problem->Horizon(); ++k) {
    std::vector<AugmentedLagrangianConstraintData> constraints;
    for (const auto& constraint : constrained_problem.StageConstraints(k)) {
      constraints.push_back(MakeConstraintData(constraint, initial_penalty, penalty_scaling));
    }
    auto knot_cost =
        std::make_shared<AugmentedLagrangianKnotCost>(base_problem->SharedStageCostFunction(k),
                                                      constraints);
    stage_costs_.push_back(knot_cost);
    stage_costs.push_back(knot_cost);
  }

  std::vector<AugmentedLagrangianConstraintData> terminal_constraints;
  for (const auto& constraint : constrained_problem.TerminalConstraints()) {
    terminal_constraints.push_back(MakeConstraintData(constraint, initial_penalty, penalty_scaling));
  }
  terminal_cost_ =
      std::make_shared<AugmentedLagrangianKnotCost>(base_problem->SharedTerminalCostFunction(),
                                                    terminal_constraints);

  subproblem_ = std::make_shared<OptimalControlProblem>(base_problem->SharedDynamics(),
                                                        std::move(stage_costs),
                                                        terminal_cost_,
                                                        base_problem->TimeStep());
}

void AugmentedLagrangianProblem::UpdateDuals(const Trajectory& trajectory) {
  for (int k = 0; k < static_cast<int>(stage_costs_.size()); ++k) {
    stage_costs_[k]->UpdateDuals(trajectory.State(k), trajectory.Control(k));
  }
  terminal_cost_->UpdateDuals(trajectory.State(trajectory.Horizon()), EmptyControl());
}

void AugmentedLagrangianProblem::UpdatePenalties() {
  for (auto& cost : stage_costs_) {
    cost->UpdatePenalties();
  }
  terminal_cost_->UpdatePenalties();
}

double AugmentedLagrangianProblem::MaxViolation(const Trajectory& trajectory) const {
  double max_violation = 0.0;
  for (int k = 0; k < static_cast<int>(stage_costs_.size()); ++k) {
    max_violation = std::max(max_violation,
                             stage_costs_[k]->MaxViolation(trajectory.State(k), trajectory.Control(k)));
  }
  max_violation = std::max(max_violation,
                           terminal_cost_->MaxViolation(trajectory.State(trajectory.Horizon()), EmptyControl()));
  return max_violation;
}

double AugmentedLagrangianProblem::MaxPenalty() const {
  double max_penalty = terminal_cost_->MaxPenalty();
  for (const auto& cost : stage_costs_) {
    max_penalty = std::max(max_penalty, cost->MaxPenalty());
  }
  return max_penalty;
}

}  // namespace my_al_ilqr
