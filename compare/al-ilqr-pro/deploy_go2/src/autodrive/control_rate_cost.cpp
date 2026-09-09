#include "autodrive/control_rate_cost.hpp"

namespace my_al_ilqr {

ControlRateCost::ControlRateCost(double jerk_weight, double steering_rate_weight)
    : jerk_weight_(jerk_weight),
      steering_rate_weight_(steering_rate_weight),
      prev_control_(Vector::Zero(2)) {}

double ControlRateCost::StageCost(const Vector& state, const Vector& control) const {
  (void)state;
  const double da = control(0) - prev_control_(0);
  const double ds = control(1) - prev_control_(1);
  return 0.5 * (jerk_weight_ * da * da + steering_rate_weight_ * ds * ds);
}

CostExpansion ControlRateCost::AnalyticalStageCostExpansion(const Vector& /*state*/,
                                                            const Vector& control) const {
  const double da = control(0) - prev_control_(0);
  const double ds = control(1) - prev_control_(1);

  Vector lu(2);
  lu << jerk_weight_ * da, steering_rate_weight_ * ds;

  Matrix luu = Matrix::Zero(2, 2);
  luu(0, 0) = jerk_weight_;
  luu(1, 1) = steering_rate_weight_;

  return CostExpansion{Vector::Zero(4), lu, Matrix::Zero(4, 4), Matrix::Zero(2, 4), luu};
}

std::pair<Vector, Matrix> ControlRateCost::AnalyticalTerminalCostExpansion(
    const Vector& /*state*/) const {
  return {Vector::Zero(4), Matrix::Zero(4, 4)};
}

}  // namespace my_al_ilqr
