#include "autodrive/guidance_tracking_cost.hpp"

#include <cmath>
#include <stdexcept>

namespace my_al_ilqr {

namespace {

double NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double EvaluateTrackingCost(const Vector& state,
                           const GuidancePoint& reference,
                           double position_weight,
                           double heading_weight,
                           double speed_weight) {
  const double dx = state(0) - reference.x;
  const double dy = state(1) - reference.y;
  const double heading_error = NormalizeAngle(state(2) - reference.yaw);
  const double speed_error = state(3) - reference.speed;
  return 0.5 * position_weight * (dx * dx + dy * dy) +
         0.5 * heading_weight * heading_error * heading_error +
         0.5 * speed_weight * speed_error * speed_error;
}

}  // namespace

GuidanceTrackingCost::GuidanceTrackingCost(const GuidancePoint& reference,
                                           double position_weight,
                                           double heading_weight,
                                           double speed_weight,
                                           bool terminal_mode,
                                           std::shared_ptr<StraightReferenceLine> reference_line,
                                           bool terminal_longitudinal_only)
    : reference_(reference),
      position_weight_(position_weight),
      heading_weight_(heading_weight),
      speed_weight_(speed_weight),
      terminal_mode_(terminal_mode),
      reference_line_(std::move(reference_line)),
      terminal_longitudinal_only_(terminal_longitudinal_only) {
  if (terminal_longitudinal_only_ && !reference_line_) {
    throw std::invalid_argument(
        "GuidanceTrackingCost terminal_longitudinal_only requires a valid reference line.");
  }
}

double GuidanceTrackingCost::StageCost(const Vector& state, const Vector& control) const {
  if (state.size() != StateDim() || control.size() != ControlDim()) {
    throw std::invalid_argument("GuidanceTrackingCost::StageCost received an input with invalid dimensions.");
  }
  if (terminal_mode_) {
    return 0.0;
  }
  return EvaluateTrackingCost(state, reference_, position_weight_, heading_weight_, speed_weight_);
}

double GuidanceTrackingCost::TerminalCost(const Vector& state) const {
  if (state.size() != StateDim()) {
    throw std::invalid_argument(
        "GuidanceTrackingCost::TerminalCost received a state with invalid dimensions.");
  }
  if (!terminal_mode_) {
    return 0.0;
  }
  if (terminal_longitudinal_only_) {
    Vector point(2);
    point << reference_.x, reference_.y;
    const double reference_s = reference_line_->LongitudinalPosition(point);
    const double state_s = reference_line_->LongitudinalPosition(state);
    const double longitudinal_error = state_s - reference_s;
    return 0.5 * position_weight_ * longitudinal_error * longitudinal_error;
  }
  return EvaluateTrackingCost(state, reference_, position_weight_, heading_weight_, speed_weight_);
}

CostExpansion GuidanceTrackingCost::AnalyticalStageCostExpansion(const Vector& state,
                                                                 const Vector& control) const {
  (void)control;
  const int nx = StateDim();
  const int nu = ControlDim();

  if (terminal_mode_) {
    return CostExpansion{Vector::Zero(nx), Vector::Zero(nu), Matrix::Zero(nx, nx),
                         Matrix::Zero(nu, nx), Matrix::Zero(nu, nu)};
  }

  const double dx = state(0) - reference_.x;
  const double dy = state(1) - reference_.y;
  const double e_hdg = NormalizeAngle(state(2) - reference_.yaw);
  const double e_spd = state(3) - reference_.speed;

  Vector lx(nx);
  lx << position_weight_ * dx, position_weight_ * dy, heading_weight_ * e_hdg,
      speed_weight_ * e_spd;

  Matrix lxx = Matrix::Zero(nx, nx);
  lxx(0, 0) = position_weight_;
  lxx(1, 1) = position_weight_;
  lxx(2, 2) = heading_weight_;
  lxx(3, 3) = speed_weight_;

  return CostExpansion{lx, Vector::Zero(nu), lxx, Matrix::Zero(nu, nx), Matrix::Zero(nu, nu)};
}

std::pair<Vector, Matrix> GuidanceTrackingCost::AnalyticalTerminalCostExpansion(
    const Vector& state) const {
  const int nx = StateDim();

  if (!terminal_mode_) {
    return {Vector::Zero(nx), Matrix::Zero(nx, nx)};
  }

  if (terminal_longitudinal_only_) {
    const double h = reference_line_->Heading();
    const double cos_h = std::cos(h);
    const double sin_h = std::sin(h);

    Vector point(2);
    point << reference_.x, reference_.y;
    const double ref_s = reference_line_->LongitudinalPosition(point);
    const double state_s = reference_line_->LongitudinalPosition(state);
    const double e_lon = state_s - ref_s;

    Vector de_lon(nx);
    de_lon << cos_h, sin_h, 0.0, 0.0;

    return {position_weight_ * e_lon * de_lon,
            position_weight_ * de_lon * de_lon.transpose()};
  }

  const double dx = state(0) - reference_.x;
  const double dy = state(1) - reference_.y;
  const double e_hdg = NormalizeAngle(state(2) - reference_.yaw);
  const double e_spd = state(3) - reference_.speed;

  Vector lx(nx);
  lx << position_weight_ * dx, position_weight_ * dy, heading_weight_ * e_hdg,
      speed_weight_ * e_spd;

  Matrix lxx = Matrix::Zero(nx, nx);
  lxx(0, 0) = position_weight_;
  lxx(1, 1) = position_weight_;
  lxx(2, 2) = heading_weight_;
  lxx(3, 3) = speed_weight_;

  return {lx, lxx};
}

}  // namespace my_al_ilqr
